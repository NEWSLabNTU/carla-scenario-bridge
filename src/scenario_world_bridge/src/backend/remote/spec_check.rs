//! The `spec_hash` start check (roadmap 051): an ego is one car only if every part of it
//! was built from the same vehicle spec (sim_vehicle_spec). Before an ego starts, the bridge
//! collects the `spec_hash` each part loaded -- the world server's body (the rig or the
//! physics it applied), the vehicle agent's registration (the autopilot's vehicle files), the
//! installed vehicle catalog's entry for the ego's body (what SSv2 declared it from) -- and
//! refuses the ego when two disagree, naming who loaded which. A part that reports none is
//! left out: it catches a process running an old install, which the build checks cannot see.

use std::collections::HashMap;
use std::path::PathBuf;

/// The catalog's property that carries an entry's spec_hash.
const PROPERTY: &str = "swb.spec_hash";

/// The vehicle catalog as installed beside the bridge (sim_vehicle_spec's share, found on
/// `AMENT_PREFIX_PATH`): the one SSv2 reads when it shares the bridge's workspace.
pub fn installed_catalog() -> Option<PathBuf> {
    std::env::var("AMENT_PREFIX_PATH")
        .ok()?
        .split(':')
        .map(|prefix| PathBuf::from(prefix).join("share/sim_vehicle_spec/catalogs/vehicles.xosc"))
        .find(|path| path.is_file())
}

/// `{model3d: spec_hash}` of a vehicle catalog's entries.
pub fn catalog_hashes(text: &str) -> HashMap<String, String> {
    let mut out = HashMap::new();
    for entry in text.split("<Vehicle ").skip(1) {
        let entry = entry.split("</Vehicle>").next().unwrap_or_default();
        let head = entry.split('>').next().unwrap_or_default();
        let (Some(model), Some(hash)) = (attribute(head, "model3d"), property(entry, PROPERTY))
        else {
            continue;
        };
        out.insert(model, hash);
    }
    out
}

fn attribute(tag: &str, name: &str) -> Option<String> {
    let start = tag.find(&format!(" {name}=\""))? + name.len() + 3;
    let len = tag[start..].find('"')?;
    Some(tag[start..start + len].to_string())
}

fn property(entry: &str, name: &str) -> Option<String> {
    entry
        .split("<Property ")
        .skip(1)
        .find(|p| attribute(&format!(" {p}"), "name").as_deref() == Some(name))
        .and_then(|p| attribute(&format!(" {p}"), "value"))
}

/// Why the parts disagree, `None` when every part that reported a hash reported the same.
pub fn disagreement(parts: &[(&str, Option<&str>)]) -> Option<String> {
    let known: Vec<(&str, &str)> = parts
        .iter()
        .filter_map(|(who, hash)| hash.filter(|h| !h.is_empty()).map(|h| (*who, h)))
        .collect();
    let first = known.first()?.1;
    if known.iter().all(|(_, h)| *h == first) {
        return None;
    }
    Some(
        known
            .iter()
            .map(|(who, h)| format!("{who} {}", &h[..h.len().min(12)]))
            .collect::<Vec<_>>()
            .join(", "),
    )
}

/// One line of who reported what, for the log.
pub fn summary(parts: &[(&str, Option<&str>)]) -> String {
    parts
        .iter()
        .map(|(who, hash)| match hash.filter(|h| !h.is_empty()) {
            Some(h) => format!("{who} {}", &h[..h.len().min(12)]),
            None => format!("{who} none"),
        })
        .collect::<Vec<_>>()
        .join(", ")
}

#[cfg(test)]
mod tests {
    use super::*;

    const CATALOG: &str = r#"<?xml version="1.0"?>
<OpenSCENARIO><Catalog name="vehicles">
    <Vehicle name="tesla_model3" vehicleCategory="car" model3d="vehicle.tesla.model3">
      <BoundingBox><Center x="1.416" y="0.0" z="0.744" /></BoundingBox>
      <Properties>
        <Property name="swb.spec_hash" value="d5d4b266c7fd1fb4" />
      </Properties>
    </Vehicle>
    <Vehicle name="box" vehicleCategory="car">
      <Properties><Property name="swb.spec_hash" value="ffff" /></Properties>
    </Vehicle>
</Catalog></OpenSCENARIO>"#;

    #[test]
    fn the_catalog_maps_each_body_to_its_spec() {
        let h = catalog_hashes(CATALOG);
        assert_eq!(h.len(), 1, "{h:?}");
        assert_eq!(h["vehicle.tesla.model3"], "d5d4b266c7fd1fb4");
    }

    #[test]
    fn the_shipped_catalog_has_the_model3() {
        let text = include_str!("../../../../sim_vehicle_spec/catalogs/vehicles.xosc");
        assert_eq!(catalog_hashes(text)["vehicle.tesla.model3"].len(), 64);
    }

    #[test]
    fn parts_agree_unless_two_known_hashes_differ() {
        let a = "d5d4b266c7fd1fb46630";
        assert_eq!(disagreement(&[("world", Some(a)), ("agent", None)]), None);
        assert_eq!(
            disagreement(&[("world", Some(a)), ("agent", Some(a))]),
            None
        );
        assert_eq!(disagreement(&[("world", Some("")), ("agent", None)]), None);
        assert_eq!(
            disagreement(&[
                ("the world server", Some(a)),
                ("the vehicle agent", Some("0123456789abcdef")),
                ("the catalog", None),
            ])
            .as_deref(),
            Some("the world server d5d4b266c7fd, the vehicle agent 0123456789ab")
        );
        assert_eq!(
            summary(&[("world", Some(a)), ("agent", None)]),
            "world d5d4b266c7fd, agent none"
        );
    }
}
