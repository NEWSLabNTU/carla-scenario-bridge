//! Reading traffic light positions out of a Lanelet2 map.
//!
//! Lanelet2 maps are OSM XML. A traffic light is a two-node way tagged with a bulb-layout
//! subtype, and a `regulatory_element` relation refers to it:
//!
//! ```xml
//! <way id="43733">
//!   <nd ref="43734"/>
//!   <nd ref="43735"/>
//!   <tag k="subtype" v="red_redYellow_green_yellow"/>
//!   <tag k="height" v="1.2"/>
//! </way>
//! <relation id="43842">
//!   <member type="way" ref="43733" role="refers"/>
//!   <tag k="type" v="regulatory_element"/>
//! </relation>
//! ```
//!
//! `TrafficSignal.id` in the SSv2 protocol is the **way** ID (`43733` above), which is what
//! this module keys on.
//!
//! # Coordinates
//!
//! Node positions come from the `local_x` / `local_y` / `ele` tags, not from `lat`/`lon`.
//! Those local coordinates are in the ROS frame, so converting to CARLA is the usual Y-flip.
//! [`TrafficLightElement::carla_position`] does that.
//!
//! Verified against the TUM-converted Town01 map: 36 traffic light ways, each with two
//! nodes at `ele=5.0`, spanning x 75..350 and y -333..4.5.

use eyre::{Result, WrapErr};
use std::collections::HashMap;
use std::path::Path;

/// Identifying a traffic light way by its tags does not work on real data.
///
/// The obvious approach is the `subtype` tag, which reads `red_redYellow_green_yellow` in
/// Town01. But the TUM converter also writes an empty `type` tag instead of `traffic_light`,
/// and in Town03 the subtype is outright corrupted:
///
/// ```text
/// _OLYeaShicfaTNEGeaShicfaTWLE_E.tttgLifr_E.tttgLifr
/// ```
///
/// That is mangled text where a bulb layout should be, and it silently hid all 38 of
/// Town03's traffic lights behind a subtype allowlist.
///
/// So identification is structural instead: in Lanelet2 a `regulatory_element` relation
/// points at the element it controls with `role="refers"`, and that is the traffic light.
/// Tags are ignored entirely, which is why this survives the corruption.
const REFERS_ROLE: &str = "refers";

/// WGS84 semi-major axis, the radius the TUM converter projected with.
const EARTH_RADIUS_M: f64 = 6_378_137.0;

/// Local coordinates derived from a node's `lat`/`lon`.
///
/// Most nodes carry `local_x`/`local_y` directly, but Town03's traffic light nodes carry
/// only `lat`/`lon` -- which is why all 38 of its lights went missing when local coordinates
/// were treated as required.
///
/// The maps declare `projector_type: local` and were produced about the origin, so the
/// projection is a plain equirectangular one. Checked against the 20,000 Town01 nodes that
/// carry both forms: maximum error 0.0000 m in both axes, i.e. this is exactly how the
/// converter generated `local_x`/`local_y`.
fn local_from_lat_lon(lat_deg: f64, lon_deg: f64) -> (f64, f64) {
    (
        lon_deg.to_radians() * EARTH_RADIUS_M,
        lat_deg.to_radians() * EARTH_RADIUS_M,
    )
}

/// A traffic light found in a Lanelet2 map.
#[derive(Debug, Clone, PartialEq)]
pub struct TrafficLightElement {
    /// Lanelet2 way ID. This is what SSv2 sends as `TrafficSignal.id`.
    pub way_id: i32,
    /// Centre of the light bar, in the map's local ROS frame.
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl TrafficLightElement {
    /// Position in CARLA's left-handed frame.
    ///
    /// Lanelet2 local coordinates are ROS-frame, so only the Y sign differs.
    pub fn carla_position(&self) -> (f64, f64, f64) {
        (self.x, -self.y, self.z)
    }

    /// Planar distance to a CARLA-frame point.
    ///
    /// Height is deliberately excluded: the lanelet element sits at the mapped bulb height
    /// while CARLA reports the actor's own origin, and the two need not agree. Traffic
    /// lights are far enough apart horizontally that x/y alone identifies them.
    pub fn planar_distance_to_carla(&self, cx: f64, cy: f64) -> f64 {
        let (sx, sy, _) = self.carla_position();
        ((sx - cx).powi(2) + (sy - cy).powi(2)).sqrt()
    }
}

/// Parse traffic light elements out of Lanelet2 OSM XML.
pub fn parse_traffic_lights(xml: &str) -> Result<Vec<TrafficLightElement>> {
    let doc = roxmltree::Document::parse(xml).wrap_err("parse lanelet2 OSM")?;

    // Node ID to local position. Lanelet2 carries both lat/lon and local_x/local_y; the
    // local pair is the one that shares a frame with the scenario's poses.
    let mut nodes: HashMap<&str, (f64, f64, f64)> = HashMap::new();
    for node in doc.descendants().filter(|n| n.has_tag_name("node")) {
        let Some(id) = node.attribute("id") else {
            continue;
        };
        let tags = collect_tags(node);

        // Prefer the explicit local coordinates; fall back to projecting lat/lon, which is
        // all Town03's traffic light nodes carry. See local_from_lat_lon.
        let position = match (parse_tag(&tags, "local_x"), parse_tag(&tags, "local_y")) {
            (Some(x), Some(y)) => Some((x, y)),
            _ => match (
                node.attribute("lat").and_then(|v| v.parse::<f64>().ok()),
                node.attribute("lon").and_then(|v| v.parse::<f64>().ok()),
            ) {
                (Some(lat), Some(lon)) => Some(local_from_lat_lon(lat, lon)),
                _ => None,
            },
        };

        let Some((x, y)) = position else {
            continue;
        };

        // `ele` is optional; a light with no elevation is still locatable in plan.
        let z = parse_tag(&tags, "ele").unwrap_or(0.0);
        nodes.insert(id, (x, y, z));
    }

    // Ways that a regulatory element controls. See REFERS_ROLE for why tags are not used.
    let mut controlled: std::collections::HashSet<&str> = std::collections::HashSet::new();
    for relation in doc.descendants().filter(|n| n.has_tag_name("relation")) {
        let tags = collect_tags(relation);
        if tags.get("type").map(String::as_str) != Some("regulatory_element") {
            continue;
        }
        for member in relation.children().filter(|c| c.has_tag_name("member")) {
            if member.attribute("type") == Some("way")
                && member.attribute("role") == Some(REFERS_ROLE)
            {
                if let Some(r) = member.attribute("ref") {
                    controlled.insert(r);
                }
            }
        }
    }

    let mut lights = Vec::new();
    for way in doc.descendants().filter(|n| n.has_tag_name("way")) {
        let Some(way_id_str) = way.attribute("id") else {
            continue;
        };
        if !controlled.contains(way_id_str) {
            continue;
        }

        let Some(way_id) = way_id_str.parse::<i32>().ok() else {
            continue;
        };

        let points: Vec<(f64, f64, f64)> = way
            .children()
            .filter(|c| c.has_tag_name("nd"))
            .filter_map(|c| c.attribute("ref"))
            .filter_map(|r| nodes.get(r).copied())
            .collect();

        if points.is_empty() {
            tracing::warn!("Traffic light way {way_id} has no resolvable nodes; skipping");
            continue;
        }

        let n = points.len() as f64;
        lights.push(TrafficLightElement {
            way_id,
            x: points.iter().map(|p| p.0).sum::<f64>() / n,
            y: points.iter().map(|p| p.1).sum::<f64>() / n,
            z: points.iter().map(|p| p.2).sum::<f64>() / n,
        });
    }

    Ok(lights)
}

/// Parse traffic light elements from a Lanelet2 map file.
pub fn load_traffic_lights(path: &Path) -> Result<Vec<TrafficLightElement>> {
    let xml = std::fs::read_to_string(path).wrap_err_with(|| format!("read {}", path.display()))?;
    let lights = parse_traffic_lights(&xml)?;
    tracing::info!(
        "Found {} traffic light element(s) in {}",
        lights.len(),
        path.display()
    );
    Ok(lights)
}

fn collect_tags<'a>(node: roxmltree::Node<'a, 'a>) -> HashMap<String, String> {
    node.children()
        .filter(|c| c.has_tag_name("tag"))
        .filter_map(|c| Some((c.attribute("k")?.to_string(), c.attribute("v")?.to_string())))
        .collect()
}

fn parse_tag(tags: &HashMap<String, String>, key: &str) -> Option<f64> {
    tags.get(key)?.parse().ok()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Trimmed from the TUM-converted Town01 map, with real IDs and coordinates.
    const TOWN01_EXCERPT: &str = r#"<?xml version='1.0' encoding='UTF-8'?>
<osm version="0.6" generator="commonroad-scenario-designer">
  <node id="43734" lat="4.0e-05" lon="0.0031">
    <tag k="local_x" v="348.7519"/>
    <tag k="local_y" v="4.4788"/>
    <tag k="ele" v="5"/>
  </node>
  <node id="43735" lat="4.0e-05" lon="0.0031">
    <tag k="local_x" v="348.6481"/>
    <tag k="local_y" v="4.5812"/>
    <tag k="ele" v="5"/>
  </node>
  <node id="43737" lat="3.0e-05" lon="0.0029">
    <tag k="local_x" v="323.4300"/>
    <tag k="local_y" v="-4.9300"/>
    <tag k="ele" v="5"/>
  </node>
  <node id="43738" lat="3.0e-05" lon="0.0029">
    <tag k="local_x" v="323.4300"/>
    <tag k="local_y" v="-4.9300"/>
    <tag k="ele" v="5"/>
  </node>
  <way id="43733">
    <nd ref="43734"/>
    <nd ref="43735"/>
    <tag k="subtype" v="red_redYellow_green_yellow"/>
    <tag k="type" v=""/>
    <tag k="height" v="1.2"/>
  </way>
  <way id="43736">
    <nd ref="43737"/>
    <nd ref="43738"/>
    <tag k="subtype" v="red_redYellow_green_yellow"/>
    <tag k="type" v=""/>
    <tag k="height" v="1.2"/>
  </way>
  <way id="148">
    <nd ref="43734"/>
    <nd ref="43735"/>
    <tag k="type" v="line_thin"/>
    <tag k="subtype" v="dashed"/>
  </way>
  <relation id="43842">
    <member type="way" ref="43733" role="refers"/>
    <tag k="type" v="regulatory_element"/>
  </relation>
  <relation id="43843">
    <member type="way" ref="43736" role="refers"/>
    <tag k="type" v="regulatory_element"/>
  </relation>
</osm>
"#;

    #[test]
    fn finds_traffic_light_ways_and_ignores_other_ways() {
        let lights = parse_traffic_lights(TOWN01_EXCERPT).expect("parses");
        assert_eq!(lights.len(), 2, "the line_thin way must not be picked up");

        let ids: Vec<i32> = lights.iter().map(|l| l.way_id).collect();
        assert!(ids.contains(&43733));
        assert!(ids.contains(&43736));
        assert!(!ids.contains(&148));
    }

    /// Position is the centre of the light bar, from local_x/local_y -- not lat/lon.
    #[test]
    fn position_is_the_centre_of_the_bar() {
        let lights = parse_traffic_lights(TOWN01_EXCERPT).expect("parses");
        let first = lights.iter().find(|l| l.way_id == 43733).expect("43733");

        assert!((first.x - 348.70).abs() < 0.01, "x was {}", first.x);
        assert!((first.y - 4.53).abs() < 0.01, "y was {}", first.y);
        assert!((first.z - 5.0).abs() < 1e-9, "z was {}", first.z);
    }

    /// Lanelet2 local coordinates are ROS-frame; CARLA flips Y.
    #[test]
    fn carla_position_flips_y_only() {
        let e = TrafficLightElement {
            way_id: 1,
            x: 348.70,
            y: 4.53,
            z: 5.0,
        };
        let (x, y, z) = e.carla_position();
        assert!((x - 348.70).abs() < 1e-9);
        assert!((y + 4.53).abs() < 1e-9, "Y must be negated, got {y}");
        assert!((z - 5.0).abs() < 1e-9);
    }

    #[test]
    fn planar_distance_ignores_height() {
        let e = TrafficLightElement {
            way_id: 1,
            x: 10.0,
            y: -20.0,
            z: 5.0,
        };
        // CARLA-frame position is (10, 20, 5). A point 3m away in x, at any height.
        assert!((e.planar_distance_to_carla(13.0, 20.0) - 3.0).abs() < 1e-9);
    }

    #[test]
    fn a_way_with_unresolvable_nodes_is_skipped() {
        let xml = r#"<osm>
          <way id="99">
            <nd ref="does-not-exist"/>
          </way>
          <relation id="1">
            <member type="way" ref="99" role="refers"/>
            <tag k="type" v="regulatory_element"/>
          </relation>
        </osm>"#;
        let lights = parse_traffic_lights(xml).expect("parses");
        assert!(lights.is_empty());
    }

    /// A node with neither local coordinates nor lat/lon cannot be placed, and must not
    /// silently default to the origin.
    #[test]
    fn nodes_without_any_coordinates_are_not_placed_at_the_origin() {
        let xml = r#"<osm>
          <node id="1"/>
          <way id="99">
            <nd ref="1"/>
          </way>
          <relation id="2">
            <member type="way" ref="99" role="refers"/>
            <tag k="type" v="regulatory_element"/>
          </relation>
        </osm>"#;
        let lights = parse_traffic_lights(xml).expect("parses");
        assert!(
            lights.is_empty(),
            "a light with no usable node must be skipped, not placed at (0,0)"
        );
    }

    /// Town03's traffic light nodes carry only lat/lon. Validated against Town01, where
    /// both forms are present, to 0.0000 m.
    #[test]
    fn lat_lon_is_used_when_local_coordinates_are_absent() {
        let xml = r#"<osm>
          <node id="1" lat="0.0" lon="0.003454829852879983">
            <tag k="ele" v="5"/>
          </node>
          <way id="99"><nd ref="1"/></way>
          <relation id="100">
            <member type="way" ref="99" role="refers"/>
            <tag k="type" v="regulatory_element"/>
          </relation>
        </osm>"#;
        let lights = parse_traffic_lights(xml).expect("parses");
        assert_eq!(lights.len(), 1);
        // Town01 node 1 carries exactly this lon alongside local_x=384.5899.
        assert!(
            (lights[0].x - 384.5899).abs() < 0.001,
            "x was {}",
            lights[0].x
        );
    }

    #[test]
    fn projection_matches_the_converters_local_coordinates() {
        // Town01 node 1: lat=-1.805613721080238e-07 lon=0.003454829852879983
        //                local_x=384.5899 local_y=-0.0201
        let (x, y) = local_from_lat_lon(-1.805613721080238e-07, 0.003454829852879983);
        assert!((x - 384.5899).abs() < 0.001, "x was {x}");
        assert!((y + 0.0201).abs() < 0.001, "y was {y}");
    }

    /// Explicit local coordinates win over lat/lon when both are present.
    #[test]
    fn local_coordinates_take_precedence_over_lat_lon() {
        let xml = r#"<osm>
          <node id="1" lat="0.5" lon="0.5">
            <tag k="local_x" v="11.0"/>
            <tag k="local_y" v="22.0"/>
          </node>
          <way id="99"><nd ref="1"/></way>
          <relation id="100">
            <member type="way" ref="99" role="refers"/>
            <tag k="type" v="regulatory_element"/>
          </relation>
        </osm>"#;
        let lights = parse_traffic_lights(xml).expect("parses");
        assert!((lights[0].x - 11.0).abs() < 1e-9);
        assert!((lights[0].y - 22.0).abs() < 1e-9);
    }

    #[test]
    fn malformed_xml_is_an_error() {
        assert!(parse_traffic_lights("<osm><way>").is_err());
    }

    #[test]
    fn a_map_with_no_traffic_lights_parses_to_nothing() {
        let xml = r#"<osm><way id="1"><tag k="type" v="line_thin"/></way></osm>"#;
        assert!(parse_traffic_lights(xml).expect("parses").is_empty());
    }

    /// Town03's subtype tag is corrupted in the source data, so identification must not
    /// depend on it. The regulatory element is what makes a way a traffic light.
    #[test]
    fn a_corrupted_subtype_does_not_hide_a_light() {
        let xml = r#"<osm>
          <node id="10">
            <tag k="local_x" v="1.0"/>
            <tag k="local_y" v="2.0"/>
            <tag k="ele" v="5"/>
          </node>
          <way id="99">
            <nd ref="10"/>
            <tag k="subtype" v="_OLYeaShicfaTNEGeaShicfaTWLE_E.tttgLifr_E.tttgLifr"/>
            <tag k="type" v=""/>
          </way>
          <relation id="100">
            <member type="way" ref="99" role="refers"/>
            <tag k="type" v="regulatory_element"/>
          </relation>
        </osm>"#;
        let lights = parse_traffic_lights(xml).expect("parses");
        assert_eq!(lights.len(), 1, "corrupted subtype must not hide the light");
        assert_eq!(lights[0].way_id, 99);
    }

    /// A ref_line (stop line) is a member of the same relation but a different role, and is
    /// not the light itself.
    #[test]
    fn only_the_refers_member_is_the_light() {
        let xml = r#"<osm>
          <node id="10">
            <tag k="local_x" v="1.0"/>
            <tag k="local_y" v="2.0"/>
          </node>
          <way id="99"><nd ref="10"/></way>
          <way id="88"><nd ref="10"/></way>
          <relation id="100">
            <member type="way" ref="99" role="refers"/>
            <member type="way" ref="88" role="ref_line"/>
            <tag k="type" v="regulatory_element"/>
          </relation>
        </osm>"#;
        let lights = parse_traffic_lights(xml).expect("parses");
        assert_eq!(lights.len(), 1);
        assert_eq!(
            lights[0].way_id, 99,
            "the ref_line must not be treated as a light"
        );
    }
}

#[cfg(test)]
mod real_map_tests {
    use super::*;

    /// Parse the actual TUM-converted maps when they are available locally.
    ///
    /// Ignored by default: the maps live on a NAS mount that is not present everywhere.
    /// Run with `cargo test -- --ignored --nocapture` where they are.
    #[test]
    #[ignore = "requires the TUM map pack on a local/NAS path"]
    fn parses_the_real_tum_maps() {
        let base = format!(
            "{}/nas/autoveh/dataset/CARLA-map-from-TUM",
            std::env::var("HOME").expect("HOME")
        );

        for town in ["Town01", "Town02", "Town03", "Town05", "Town10"] {
            let path = std::path::Path::new(&base)
                .join(town)
                .join("lanelet2_map.osm");
            if !path.exists() {
                eprintln!("{town}: not present, skipping");
                continue;
            }
            let lights = load_traffic_lights(&path).expect("real map parses");
            eprintln!("{town}: {} traffic lights", lights.len());

            for l in &lights {
                assert!(l.z.is_finite() && l.x.is_finite() && l.y.is_finite());
                let (cx, cy, _) = l.carla_position();
                assert!((cy + l.y).abs() < 1e-9, "Y must be flipped");
                assert!((cx - l.x).abs() < 1e-9, "X must be unchanged");
            }
        }
    }
}
