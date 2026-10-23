//! OpenSCENARIO's road condition on CARLA (roadmap 047, S2): what CARLA 0.9.16 can do with it.
//!
//! CARLA has one friction per wheel (`WheelPhysicsControl::tire_friction`) and no road
//! surface model, so the condition becomes a factor on every vehicle's tire friction:
//! `frictionScaleFactor` times the wet factor of asphalt, the class CARLA's towns are paved
//! with (the same numbers as `isaac_world/config/surface_classes.yaml`, which the tests
//! check). The wetness also changes the weather's look: `wetness` and
//! `precipitation_deposits` (puddles), which on 0.9.16 are rendering only. Standing water,
//! hydroplaning and the map's zones (`swb.zone.*`) are the Isaac world server's surface
//! layer; their properties are refused here.

use swb_core::sim::RoadCondition;
use swb_core::weather::Weather;

/// Asphalt's friction factor per wetness level, dry first: `surface_classes.yaml`'s
/// `classes.asphalt.wet`.
pub const WET_FRICTION: [(&str, f64); 5] = [
    ("dry", 1.0),
    ("moist", 0.9),
    ("wetWithPuddles", 0.75),
    ("lowFlooded", 0.65),
    ("highFlooded", 0.55),
];

/// The weather's `wetness` and `precipitation_deposits` (0..100) per wetness level: how wet
/// the road looks. Chosen by eye, not measured.
pub const WET_LOOK: [(&str, f32, f32); 5] = [
    ("dry", 0.0, 0.0),
    ("moist", 40.0, 0.0),
    ("wetWithPuddles", 80.0, 40.0),
    ("lowFlooded", 100.0, 80.0),
    ("highFlooded", 100.0, 100.0),
];

/// What CARLA makes of a road condition.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Applied {
    /// Every vehicle's tire friction is its own times this.
    pub friction_factor: f64,
    /// The weather's `wetness`, `precipitation_deposits`.
    pub look: (f32, f32),
}

/// The road condition as CARLA applies it, or why not (every problem, one per line).
pub fn applied(rc: &RoadCondition) -> Result<Applied, String> {
    let mut problems = Vec::new();
    let refused: Vec<&str> = rc.swb_properties().map(|(k, _)| k.as_str()).collect();
    if !refused.is_empty() {
        problems.push(format!(
            "the CARLA world server has no surface map: zones and other swb properties are \
             the Isaac world server's (refused: {})",
            refused.join(", ")
        ));
    }
    let wetness = if rc.wetness.is_empty() {
        "dry"
    } else {
        rc.wetness.as_str()
    };
    let wet = WET_FRICTION.iter().find(|(w, _)| *w == wetness);
    let look = WET_LOOK.iter().find(|(w, _, _)| *w == wetness);
    let (Some((_, wet)), Some((_, wetness_look, deposits))) = (wet, look) else {
        problems.push(format!("unknown wetness {wetness:?}"));
        return Err(problems.join("\n"));
    };
    let f = rc.friction_scale_factor;
    if !(f.is_finite() && f >= 0.0) {
        problems.push(format!("frictionScaleFactor {f} is not in [0, inf)"));
    }
    if !problems.is_empty() {
        return Err(problems.join("\n"));
    }
    Ok(Applied {
        friction_factor: f * wet,
        look: (*wetness_look, *deposits),
    })
}

/// `weather` with the road's wet look.
pub fn with_look(mut weather: Weather, look: (f32, f32)) -> Weather {
    weather.wetness = look.0;
    weather.precipitation_deposits = look.1;
    weather
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;

    fn rc(f: f64, wetness: &str, props: &[(&str, &str)]) -> RoadCondition {
        RoadCondition {
            friction_scale_factor: f,
            wetness: wetness.into(),
            properties: props
                .iter()
                .map(|(k, v)| (k.to_string(), v.to_string()))
                .collect::<BTreeMap<_, _>>(),
        }
    }

    #[test]
    fn friction_is_the_scale_times_wet_asphalt() {
        let a = applied(&rc(0.8, "wetWithPuddles", &[])).unwrap();
        assert!((a.friction_factor - 0.6).abs() < 1e-12);
        assert_eq!(a.look, (80.0, 40.0));
        let a = applied(&RoadCondition::default()).unwrap();
        assert_eq!(a.friction_factor, 1.0);
        assert_eq!(a.look, (0.0, 0.0));
        // Another tool's property is not ours to refuse.
        assert!(applied(&rc(1.0, "", &[("esmini.x", "1")])).is_ok());
    }

    #[test]
    fn zones_are_refused_with_every_problem() {
        let e = applied(&rc(
            -1.0,
            "dry",
            &[
                ("swb.zone.ford.water_depth", "0.1"),
                ("swb.zone.pit.class", "mud_wet"),
            ],
        ))
        .unwrap_err();
        assert!(
            e.contains("swb.zone.ford.water_depth, swb.zone.pit.class"),
            "{e}"
        );
        assert!(e.contains("frictionScaleFactor -1"), "{e}");
        assert!(applied(&rc(1.0, "soaked", &[]))
            .unwrap_err()
            .contains("soaked"));
    }

    /// The wet factors are asphalt's in the Isaac surface layer's class table, so one
    /// scenario brakes alike on both simulators.
    #[test]
    fn wet_factors_match_the_isaac_class_table() {
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../isaac_world/config/surface_classes.yaml"
        );
        let table: serde_yaml::Value =
            serde_yaml::from_str(&std::fs::read_to_string(path).unwrap()).unwrap();
        let wet = &table["classes"]["asphalt"]["wet"];
        for (level, factor) in WET_FRICTION.iter().skip(1) {
            assert_eq!(wet[*level].as_f64(), Some(*factor), "{level}");
        }
    }
}
