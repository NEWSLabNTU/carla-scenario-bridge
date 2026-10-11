//! Which CARLA body fits the box a scenario declares (roadmap 022).
//!
//! CARLA gives a blueprint no bounding box; only a spawned actor has one. So the boxes come
//! from a table measured once per CARLA version by `scripts/measure_blueprint_boxes.py` and
//! installed as `config/blueprint_boxes_<version>.yaml`. This module reads it and picks, from
//! an entity's class, the blueprint whose box is closest to the declared one.
//!
//! The choice is a fallback below the operator's `blueprint_map` and the scenario's own
//! blueprint name; it replaces only "every vehicle is a Model 3". And it keeps the kind's
//! default whenever the default already [`fits`](Dimensions::fits): the starter scenarios'
//! 4.5 x 2.1 x 1.8 m car box is closer to a Mini Cooper S than to a Model 3, but within
//! tolerance of both, and swapping every NPC's body for that would change behaviour where
//! no gap is being closed.

// Wired into spawn_entity in 022 step 4; until then only the tests use it.
#![allow(dead_code)]

use std::collections::BTreeMap;
use std::path::Path;

use eyre::{Result, WrapErr};
use serde::Deserialize;

use crate::proto::traffic_simulator_msgs::entity_subtype::Enum as EntitySubtype;

/// Largest in-plane (length, width) difference at which a body still fits its declared
/// box, m. SSv2 judges collisions on the 2D footprint, so this is the axis pair that
/// decides whether SSv2 and CARLA can disagree about a contact.
pub const PLANE_TOLERANCE: f64 = 0.3;
/// Largest height difference at which a body still fits, m. Looser: no verdict is
/// computed from height, only what a sensor sees.
pub const HEIGHT_TOLERANCE: f64 = 0.5;

/// Weight of the length error in the size score: longitudinal scenarios (following
/// distance, time to collision) measure along it.
const LENGTH_WEIGHT: f64 = 2.0;

/// An entity's box, m.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Dimensions {
    pub length: f64,
    pub width: f64,
    pub height: f64,
}

impl Dimensions {
    /// Usable as a declared box: every side finite and positive.
    pub fn is_valid(&self) -> bool {
        [self.length, self.width, self.height]
            .iter()
            .all(|d| d.is_finite() && *d > 0.0)
    }

    /// `actual - self` per axis.
    pub fn residual(&self, actual: &Dimensions) -> Dimensions {
        Dimensions {
            length: actual.length - self.length,
            width: actual.width - self.width,
            height: actual.height - self.height,
        }
    }

    /// Whether a body of `actual` size is within tolerance of this declared box.
    pub fn fits(&self, actual: &Dimensions) -> bool {
        let r = self.residual(actual);
        r.length.abs() <= PLANE_TOLERANCE
            && r.width.abs() <= PLANE_TOLERANCE
            && r.height.abs() <= HEIGHT_TOLERANCE
    }

    /// Summed relative error of `actual` against this declared box, length weighted.
    fn score(&self, actual: &Dimensions) -> f64 {
        let r = self.residual(actual);
        LENGTH_WEIGHT * (r.length / self.length).abs()
            + (r.width / self.width).abs()
            + (r.height / self.height).abs()
    }
}

/// One blueprint's measured box.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct BlueprintBox {
    /// `vehicle`, `pedestrian` or `misc_object`.
    pub kind: String,
    /// CARLA's `base_type`, lower-cased; absent on a few cars.
    #[serde(default)]
    pub base_type: Option<String>,
    #[serde(default)]
    pub wheels: Option<u32>,
    pub length: f64,
    pub width: f64,
    pub height: f64,
    /// Box centre relative to the actor origin, CARLA frame, m.
    #[serde(default)]
    pub center: [f64; 3],
}

impl BlueprintBox {
    pub fn dimensions(&self) -> Dimensions {
        Dimensions {
            length: self.length,
            width: self.width,
            height: self.height,
        }
    }
}

/// The measured table for one CARLA version.
#[derive(Debug, Clone, Default, Deserialize)]
pub struct BoxTable {
    pub carla_version: String,
    pub blueprints: BTreeMap<String, BlueprintBox>,
}

impl BoxTable {
    pub fn from_yaml(text: &str) -> Result<Self> {
        serde_yaml::from_str(text).wrap_err("parse blueprint box table")
    }

    pub fn load(path: &Path) -> Result<Self> {
        let text =
            std::fs::read_to_string(path).wrap_err_with(|| format!("read {}", path.display()))?;
        Self::from_yaml(&text).wrap_err_with(|| path.display().to_string())
    }

    pub fn get(&self, blueprint: &str) -> Option<&BlueprintBox> {
        self.blueprints.get(blueprint)
    }
}

/// The blueprints an entity may be size-matched among.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SizeClass {
    /// `car` and `van`; also a four-wheeler CARLA gave no `base_type`.
    Car,
    Truck,
    Bus,
    Motorcycle,
    Bicycle,
    /// Subtype unknown: anything on four or more wheels.
    FourWheeled,
    /// `static.prop.*`.
    MiscObject,
}

impl SizeClass {
    /// A vehicle's class from SSv2's subtype. A trailer has no CARLA body of its own, so it
    /// gets the closest truck. `None` for a pedestrian, which is never size-matched.
    pub fn of_vehicle(subtype: EntitySubtype) -> Option<SizeClass> {
        match subtype {
            EntitySubtype::Car => Some(SizeClass::Car),
            EntitySubtype::Truck | EntitySubtype::Trailer => Some(SizeClass::Truck),
            EntitySubtype::Bus => Some(SizeClass::Bus),
            EntitySubtype::Motorcycle => Some(SizeClass::Motorcycle),
            EntitySubtype::Bicycle => Some(SizeClass::Bicycle),
            EntitySubtype::Unknown => Some(SizeClass::FourWheeled),
            EntitySubtype::Pedestrian => None,
        }
    }

    fn admits(self, bp: &BlueprintBox) -> bool {
        let base = bp.base_type.as_deref();
        let four_wheels = bp.wheels.is_some_and(|w| w >= 4);
        match self {
            SizeClass::MiscObject => bp.kind == "misc_object",
            _ if bp.kind != "vehicle" => false,
            SizeClass::Car => {
                matches!(base, Some("car" | "van")) || (base.is_none() && four_wheels)
            }
            SizeClass::Truck => base == Some("truck"),
            SizeClass::Bus => base == Some("bus"),
            SizeClass::Motorcycle => base == Some("motorcycle"),
            SizeClass::Bicycle => base == Some("bicycle"),
            SizeClass::FourWheeled => four_wheels,
        }
    }
}

/// The blueprint of `class` whose box is closest to `declared`, or `None` when nothing
/// can be chosen (no valid declared box, no candidate in the table).
///
/// `default` -- the blueprint the kind would get without this -- wins whenever it is in
/// the class and fits the declared box, so a scenario whose box was already served well
/// keeps its body.
pub fn choose_by_size(
    class: SizeClass,
    declared: &Dimensions,
    table: &BoxTable,
    default: &str,
) -> Option<String> {
    if !declared.is_valid() {
        return None;
    }
    if let Some(bp) = table.get(default) {
        if class.admits(bp) && declared.fits(&bp.dimensions()) {
            return Some(default.to_string());
        }
    }
    table
        .blueprints
        .iter()
        .filter(|(_, bp)| class.admits(bp))
        .map(|(id, bp)| (id, declared.score(&bp.dimensions())))
        // BTreeMap order breaks exact ties by name, so the choice is reproducible.
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(id, _)| id.clone())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The table the package ships, so the cases below are the choices a run makes.
    fn shipped() -> BoxTable {
        BoxTable::from_yaml(include_str!("../config/blueprint_boxes_0.9.16.yaml")).unwrap()
    }

    fn dims(length: f64, width: f64, height: f64) -> Dimensions {
        Dimensions {
            length,
            width,
            height,
        }
    }

    const MODEL3: &str = "vehicle.tesla.model3";

    #[test]
    fn the_shipped_table_parses_and_has_every_class() {
        let t = shipped();
        assert_eq!(t.carla_version, "0.9.16");
        for class in [
            SizeClass::Car,
            SizeClass::Truck,
            SizeClass::Bus,
            SizeClass::Motorcycle,
            SizeClass::Bicycle,
            SizeClass::FourWheeled,
            SizeClass::MiscObject,
        ] {
            assert!(
                t.blueprints.values().any(|bp| class.admits(bp)),
                "{class:?} has no blueprint"
            );
        }
        let m3 = t.get(MODEL3).unwrap();
        assert!((m3.length - 4.792).abs() < 0.01, "{m3:?}");
    }

    #[test]
    fn the_starter_car_box_keeps_its_model3() {
        // 4.5 x 2.1 x 1.8: a Mini Cooper S 2021 scores closer, but the Model 3 fits.
        let got = choose_by_size(SizeClass::Car, &dims(4.5, 2.1, 1.8), &shipped(), MODEL3);
        assert_eq!(got.as_deref(), Some(MODEL3));
    }

    #[test]
    fn a_car_box_the_model3_does_not_fit_gets_the_closest_car() {
        // A city car: 3.6 m long.
        let got = choose_by_size(SizeClass::Car, &dims(3.6, 1.8, 1.5), &shipped(), MODEL3);
        assert_eq!(got.as_deref(), Some("vehicle.nissan.micra"));
    }

    #[test]
    fn a_12m_truck_gets_the_longest_truck() {
        let got = choose_by_size(SizeClass::Truck, &dims(12.0, 2.5, 3.5), &shipped(), MODEL3);
        assert_eq!(got.as_deref(), Some("vehicle.carlamotors.firetruck"));
    }

    #[test]
    fn a_truck_never_keeps_a_car_default_even_one_that_fits() {
        // A pickup-sized truck: the Model 3 would fit, but it is not a truck.
        let got = choose_by_size(SizeClass::Truck, &dims(4.8, 2.1, 1.6), &shipped(), MODEL3);
        let t = shipped();
        assert_eq!(
            t.get(got.as_deref().unwrap()).unwrap().base_type.as_deref(),
            Some("truck")
        );
    }

    #[test]
    fn trailer_and_bus_map_to_their_bodies() {
        assert_eq!(
            SizeClass::of_vehicle(EntitySubtype::Trailer),
            Some(SizeClass::Truck)
        );
        let got = choose_by_size(SizeClass::Bus, &dims(12.0, 2.5, 3.2), &shipped(), MODEL3);
        assert_eq!(got.as_deref(), Some("vehicle.mitsubishi.fusorosa"));
    }

    #[test]
    fn a_motorcycle_gets_a_motorcycle() {
        let got = choose_by_size(
            SizeClass::Motorcycle,
            &dims(2.2, 0.8, 1.5),
            &shipped(),
            MODEL3,
        );
        assert_eq!(got.as_deref(), Some("vehicle.yamaha.yzf"));
        let got = choose_by_size(SizeClass::Bicycle, &dims(1.8, 0.6, 1.7), &shipped(), MODEL3);
        assert_eq!(got.as_deref(), Some("vehicle.gazelle.omafiets"));
    }

    #[test]
    fn an_unknown_subtype_picks_among_four_wheelers() {
        assert_eq!(
            SizeClass::of_vehicle(EntitySubtype::Unknown),
            Some(SizeClass::FourWheeled)
        );
        // Bicycle-sized, but with no subtype it cannot become a bicycle.
        let got = choose_by_size(
            SizeClass::FourWheeled,
            &dims(1.8, 0.6, 1.7),
            &shipped(),
            MODEL3,
        )
        .unwrap();
        assert!(shipped().get(&got).unwrap().wheels.unwrap() >= 4, "{got}");
        // And the starter box still keeps its Model 3.
        let got = choose_by_size(
            SizeClass::FourWheeled,
            &dims(4.5, 2.1, 1.8),
            &shipped(),
            MODEL3,
        );
        assert_eq!(got.as_deref(), Some(MODEL3));
    }

    #[test]
    fn a_pedestrian_is_never_size_matched() {
        assert_eq!(SizeClass::of_vehicle(EntitySubtype::Pedestrian), None);
        assert!(!shipped()
            .blueprints
            .iter()
            .any(|(id, bp)| id.starts_with("walker.") && SizeClass::FourWheeled.admits(bp)));
    }

    #[test]
    fn a_misc_object_gets_the_closest_prop() {
        let barrier = "static.prop.streetbarrier";
        // Its own size keeps the default...
        let got = choose_by_size(
            SizeClass::MiscObject,
            &dims(1.2, 0.4, 1.0),
            &shipped(),
            barrier,
        );
        assert_eq!(got.as_deref(), Some(barrier));
        // ...a cone-sized box gets a cone, and never a vehicle.
        let got = choose_by_size(
            SizeClass::MiscObject,
            &dims(0.45, 0.4, 1.2),
            &shipped(),
            barrier,
        )
        .unwrap();
        assert_eq!(got, "static.prop.trafficcone02");
    }

    #[test]
    fn nothing_to_choose_from_is_none() {
        let empty = BoxTable::default();
        assert_eq!(
            choose_by_size(SizeClass::Car, &dims(4.5, 2.1, 1.8), &empty, MODEL3),
            None
        );
        for bad in [
            dims(0.0, 2.0, 1.5),
            dims(4.5, f64::NAN, 1.5),
            dims(4.5, 2.0, -1.0),
        ] {
            assert_eq!(
                choose_by_size(SizeClass::Car, &bad, &shipped(), MODEL3),
                None
            );
        }
    }

    #[test]
    fn fit_is_within_tolerance_per_axis() {
        let declared = dims(4.5, 2.1, 1.8);
        assert!(declared.fits(&dims(4.8, 2.4, 2.3)));
        assert!(!declared.fits(&dims(4.81, 2.1, 1.8)));
        assert!(!declared.fits(&dims(4.5, 1.79, 1.8)));
        assert!(!declared.fits(&dims(4.5, 2.1, 2.31)));
        let r = declared.residual(&dims(4.792, 2.163, 1.488));
        assert!((r.length - 0.292).abs() < 1e-9 && (r.height + 0.312).abs() < 1e-9);
    }
}
