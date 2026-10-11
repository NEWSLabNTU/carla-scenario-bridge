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

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

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

/// Largest difference, per axis, between the table and the actor CARLA spawned before the
/// table counts as stale, m. The table is CARLA's own numbers rounded to 1 mm, so anything
/// visible at 5 cm is a different asset.
pub const STALE_TOLERANCE: f64 = 0.05;

/// The script that regenerates the table, named in the stale-table warning.
pub const MEASURE_SCRIPT: &str = "scripts/measure_blueprint_boxes.py";

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

    /// Whether `actual` and this box differ by more than [`STALE_TOLERANCE`] on any axis.
    pub fn differs_from(&self, actual: &Dimensions) -> bool {
        let r = self.residual(actual);
        [r.length, r.width, r.height]
            .iter()
            .any(|d| d.abs() > STALE_TOLERANCE)
    }

    /// The same box turned 90° about the vertical: length and width swapped.
    pub fn turned(&self) -> Dimensions {
        Dimensions {
            length: self.width,
            width: self.length,
            height: self.height,
        }
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

    /// The table for `version`, looked up beside the config file first (like the per-map
    /// traffic-light files), then in each installed share. `None` with a log line when there
    /// is none or it does not parse: the bridge then spawns as before 022.
    pub fn load_for(config_dir: &Path, version: &str) -> Option<Self> {
        let Some(path) = table_path(config_dir, version, installed_shares()) else {
            tracing::info!(
                "No blueprint box table for CARLA {version} ({}); entities get the default \
                 blueprint for their kind, and no spawn-time size check",
                table_file(version)
            );
            return None;
        };
        match Self::load(&path) {
            Ok(table) => {
                if table.carla_version != version {
                    tracing::warn!(
                        "{} says CARLA {}, this bridge is built for {version}; using it anyway",
                        path.display(),
                        table.carla_version
                    );
                }
                tracing::info!(
                    "Blueprint box table: {} blueprints from {}",
                    table.blueprints.len(),
                    path.display()
                );
                Some(table)
            }
            Err(e) => {
                tracing::warn!("Ignoring the blueprint box table: {e:#}");
                None
            }
        }
    }
}

/// `blueprint_boxes_<version>.yaml`.
pub fn table_file(version: &str) -> String {
    format!("blueprint_boxes_{version}.yaml")
}

/// `<prefix>/share/carla_scenario_bridge/config` for each prefix in `AMENT_PREFIX_PATH`.
fn installed_shares() -> Vec<PathBuf> {
    std::env::var("AMENT_PREFIX_PATH")
        .unwrap_or_default()
        .split(':')
        .filter(|p| !p.is_empty())
        .map(|p| Path::new(p).join("share/carla_scenario_bridge/config"))
        .collect()
}

fn table_path(config_dir: &Path, version: &str, shares: Vec<PathBuf>) -> Option<PathBuf> {
    let file = table_file(version);
    std::iter::once(config_dir.to_path_buf())
        .chain(shares)
        .map(|dir| dir.join(&file))
        .find(|p| p.is_file())
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

    /// Whether a body may be turned 90° to fit: props have no driving direction, a
    /// vehicle's length is its direction of travel.
    pub fn may_turn(self) -> bool {
        self == SizeClass::MiscObject
    }

    /// `actual` in the orientation that suits `declared` best: turned only when the class
    /// may turn and turning scores strictly better. `(dimensions, turned)`.
    fn orient(self, declared: &Dimensions, actual: Dimensions) -> (Dimensions, bool) {
        if self.may_turn() && declared.score(&actual.turned()) < declared.score(&actual) {
            (actual.turned(), true)
        } else {
            (actual, false)
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

/// A size match: the blueprint, and whether it is to be spawned turned 90° about the
/// vertical (see [`SizeClass::may_turn`]).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SizeChoice {
    pub blueprint: String,
    pub turned: bool,
}

/// The blueprint of `class` whose box is closest to `declared`, or `None` when nothing
/// can be chosen (no valid declared box, no candidate in the table). A class that may turn
/// tries each candidate both ways and keeps the better orientation.
///
/// `default` -- the blueprint the kind would get without this -- wins whenever it is in
/// the class and fits the declared box, so a scenario whose box was already served well
/// keeps its body.
pub fn choose_by_size(
    class: SizeClass,
    declared: &Dimensions,
    table: &BoxTable,
    default: &str,
) -> Option<SizeChoice> {
    if !declared.is_valid() {
        return None;
    }
    if let Some(bp) = table.get(default) {
        let (oriented, turned) = class.orient(declared, bp.dimensions());
        if class.admits(bp) && declared.fits(&oriented) {
            return Some(SizeChoice {
                blueprint: default.to_string(),
                turned,
            });
        }
    }
    table
        .blueprints
        .iter()
        .filter(|(_, bp)| class.admits(bp))
        .map(|(id, bp)| {
            let (oriented, turned) = class.orient(declared, bp.dimensions());
            (id, turned, declared.score(&oriented))
        })
        // BTreeMap order breaks exact ties by name, so the choice is reproducible.
        .min_by(|a, b| a.2.total_cmp(&b.2))
        .map(|(id, turned, _)| SizeChoice {
            blueprint: id.clone(),
            turned,
        })
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

    /// The chosen blueprint alone, for the classes that never turn.
    fn pick(
        class: SizeClass,
        declared: &Dimensions,
        table: &BoxTable,
        default: &str,
    ) -> Option<String> {
        let choice = choose_by_size(class, declared, table, default)?;
        assert!(!choice.turned || class.may_turn(), "{class:?} turned");
        Some(choice.blueprint)
    }

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
        let got = pick(SizeClass::Car, &dims(4.5, 2.1, 1.8), &shipped(), MODEL3);
        assert_eq!(got.as_deref(), Some(MODEL3));
    }

    #[test]
    fn a_car_box_the_model3_does_not_fit_gets_the_closest_car() {
        // A city car: 3.6 m long.
        let got = pick(SizeClass::Car, &dims(3.6, 1.8, 1.5), &shipped(), MODEL3);
        assert_eq!(got.as_deref(), Some("vehicle.nissan.micra"));
    }

    #[test]
    fn a_12m_truck_gets_the_longest_truck() {
        let got = pick(SizeClass::Truck, &dims(12.0, 2.5, 3.5), &shipped(), MODEL3);
        assert_eq!(got.as_deref(), Some("vehicle.carlamotors.firetruck"));
    }

    #[test]
    fn a_truck_never_keeps_a_car_default_even_one_that_fits() {
        // A pickup-sized truck: the Model 3 would fit, but it is not a truck.
        let got = pick(SizeClass::Truck, &dims(4.8, 2.1, 1.6), &shipped(), MODEL3);
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
        let got = pick(SizeClass::Bus, &dims(12.0, 2.5, 3.2), &shipped(), MODEL3);
        assert_eq!(got.as_deref(), Some("vehicle.mitsubishi.fusorosa"));
    }

    #[test]
    fn a_motorcycle_gets_a_motorcycle() {
        let got = pick(
            SizeClass::Motorcycle,
            &dims(2.2, 0.8, 1.5),
            &shipped(),
            MODEL3,
        );
        assert_eq!(got.as_deref(), Some("vehicle.yamaha.yzf"));
        let got = pick(SizeClass::Bicycle, &dims(1.8, 0.6, 1.7), &shipped(), MODEL3);
        assert_eq!(got.as_deref(), Some("vehicle.gazelle.omafiets"));
    }

    #[test]
    fn an_unknown_subtype_picks_among_four_wheelers() {
        assert_eq!(
            SizeClass::of_vehicle(EntitySubtype::Unknown),
            Some(SizeClass::FourWheeled)
        );
        // Bicycle-sized, but with no subtype it cannot become a bicycle.
        let got = pick(
            SizeClass::FourWheeled,
            &dims(1.8, 0.6, 1.7),
            &shipped(),
            MODEL3,
        )
        .unwrap();
        assert!(shipped().get(&got).unwrap().wheels.unwrap() >= 4, "{got}");
        // And the starter box still keeps its Model 3.
        let got = pick(
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
        let got = pick(
            SizeClass::MiscObject,
            &dims(1.2, 0.4, 1.0),
            &shipped(),
            barrier,
        );
        assert_eq!(got.as_deref(), Some(barrier));
        // ...a cone-sized box gets a cone, and never a vehicle.
        let got = pick(
            SizeClass::MiscObject,
            &dims(0.45, 0.4, 1.2),
            &shipped(),
            barrier,
        )
        .unwrap();
        assert_eq!(got, "static.prop.trafficcone02");
    }

    #[test]
    fn a_prop_declared_across_keeps_its_body_turned() {
        // town01_pedestrian's barriers: the 1.22 x 0.37 m street barrier, across the road.
        // Unturned, the closest prop was a garden gnome.
        let barrier = "static.prop.streetbarrier";
        let across = dims(0.4, 1.2, 1.0);
        let got = choose_by_size(SizeClass::MiscObject, &across, &shipped(), barrier).unwrap();
        assert_eq!(
            got,
            SizeChoice {
                blueprint: barrier.to_string(),
                turned: true
            }
        );
        // Along the road it is not turned.
        let got = choose_by_size(SizeClass::MiscObject, &across.turned(), &shipped(), barrier);
        assert_eq!(got.map(|c| c.turned), Some(false));
    }

    #[test]
    fn a_vehicle_never_turns() {
        // A car box declared sideways: no car is 1.9 m long and 4.5 m wide, and it stays so.
        let got = choose_by_size(SizeClass::Car, &dims(1.9, 4.5, 1.5), &shipped(), MODEL3).unwrap();
        assert!(!got.turned);
    }

    #[test]
    fn nothing_to_choose_from_is_none() {
        let empty = BoxTable::default();
        assert_eq!(
            pick(SizeClass::Car, &dims(4.5, 2.1, 1.8), &empty, MODEL3),
            None
        );
        for bad in [
            dims(0.0, 2.0, 1.5),
            dims(4.5, f64::NAN, 1.5),
            dims(4.5, 2.0, -1.0),
        ] {
            assert_eq!(pick(SizeClass::Car, &bad, &shipped(), MODEL3), None);
        }
    }

    #[test]
    fn the_table_is_found_beside_the_config_then_in_a_share() {
        let root = std::env::temp_dir().join(format!("csb-boxes-{}", std::process::id()));
        let (config, share) = (root.join("config"), root.join("share"));
        std::fs::create_dir_all(&config).unwrap();
        std::fs::create_dir_all(&share).unwrap();
        let file = table_file("0.9.16");
        assert_eq!(table_path(&config, "0.9.16", vec![share.clone()]), None);
        std::fs::write(share.join(&file), "").unwrap();
        assert_eq!(
            table_path(&config, "0.9.16", vec![share.clone()]),
            Some(share.join(&file))
        );
        std::fs::write(config.join(&file), "").unwrap();
        assert_eq!(
            table_path(&config, "0.9.16", vec![share.clone()]),
            Some(config.join(&file))
        );
        assert_eq!(table_path(&config, "0.10.0", vec![share]), None);
        let _ = std::fs::remove_dir_all(&root);
    }

    #[test]
    fn a_table_off_by_more_than_5cm_is_stale() {
        let m3 = shipped().get(MODEL3).unwrap().dimensions();
        assert!(!m3.differs_from(&dims(4.792, 2.163, 1.488)));
        assert!(!m3.differs_from(&dims(4.84, 2.163, 1.488)));
        assert!(m3.differs_from(&dims(4.85, 2.163, 1.488)));
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
