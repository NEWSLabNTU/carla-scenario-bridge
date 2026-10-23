//! OpenSCENARIO's weather and time of day on CARLA (roadmap 047): CARLA's `WeatherParameters`
//! show all of it but snow, on cameras only (CARLA 0.9.16's lidar and radar ignore the weather).
//!
//! The mappings are engineered, not measured: cloud cover in oktas to `cloudiness` (8 oktas =
//! 100), rain in mm/h to `precipitation` (25 mm/h, heavy rain, = 100), a fog's visual range to
//! `fog_density` (1000 m or more = none, falling linearly to 100 at 0 m, from 0 m out), wind
//! speed to `wind_intensity` (20 m/s = 100). The sun's elevation is CARLA's altitude; its
//! azimuth (OpenSCENARIO: clockwise from north) goes to CARLA's measured from east, which is
//! UE's +x on CARLA's maps, also clockwise seen from above -- assumed, not checked against
//! CARLA's renderer. A sun's illuminance has no CARLA counterpart and is ignored.

use swb_core::environment::{PrecipitationType, ScenarioWeather};
use swb_core::weather::Weather;

/// The weather fields the scenario has set so far; a part not given keeps its last value.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Sky {
    pub cloudiness: Option<f32>,
    pub precipitation: Option<f32>,
    pub fog_density: Option<f32>,
    pub wind_intensity: Option<f32>,
    /// A sun the scenario placed (deg: CARLA azimuth, altitude); it wins over a time of day.
    pub sun: Option<(f32, f32)>,
}

impl Sky {
    pub fn is_empty(&self) -> bool {
        *self == Self::default()
    }

    /// The parts `w` gives, laid over these, or every part CARLA cannot show, one per line.
    pub fn with(&self, w: &ScenarioWeather) -> Result<Sky, String> {
        let mut out = *self;
        if let Some(p) = w.precipitation {
            match p.kind {
                PrecipitationType::Snow => {
                    return Err("snow: CARLA 0.9.16 has none (rain and dry only)".into())
                }
                PrecipitationType::Dry => out.precipitation = Some(0.0),
                PrecipitationType::Rain => {
                    out.precipitation = Some((p.intensity / 25.0 * 100.0).clamp(0.0, 100.0) as f32)
                }
            }
        }
        if let Some(oktas) = w.cloud_cover_oktas {
            out.cloudiness = Some((f32::from(oktas.min(8)) / 8.0 * 100.0).min(100.0));
        }
        if let Some(v) = w.fog_visual_range {
            out.fog_density = Some(((1.0 - v / 1000.0) * 100.0).clamp(0.0, 100.0) as f32);
        }
        if let Some(wind) = w.wind {
            out.wind_intensity = Some((wind.speed / 20.0 * 100.0).clamp(0.0, 100.0) as f32);
        }
        if let Some(sun) = w.sun {
            out.sun = Some(carla_sun(sun.azimuth, sun.elevation));
        }
        Ok(out)
    }

    /// `weather` with these parts, and `sun` (CARLA deg) where the scenario placed none.
    pub fn apply(&self, mut weather: Weather, sun: Option<(f32, f32)>) -> Weather {
        if let Some(c) = self.cloudiness {
            weather.cloudiness = c;
        }
        if let Some(p) = self.precipitation {
            weather.precipitation = p;
        }
        if let Some(d) = self.fog_density {
            weather.fog_density = d;
            weather.fog_distance = 0.0;
        }
        if let Some(w) = self.wind_intensity {
            weather.wind_intensity = w;
        }
        if let Some((azimuth, altitude)) = self.sun.or(sun) {
            weather.sun_azimuth_angle = azimuth;
            weather.sun_altitude_angle = altitude;
        }
        weather
    }
}

/// An OpenSCENARIO sun (rad: azimuth clockwise from north, elevation) as CARLA's angles, deg.
pub fn carla_sun(azimuth: f64, elevation: f64) -> (f32, f32) {
    (
        (azimuth.to_degrees() - 90.0).rem_euclid(360.0) as f32,
        elevation.to_degrees() as f32,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use swb_core::environment::{Precipitation, Sun, Wind};

    fn clear_noon() -> Weather {
        swb_core::weather::preset("ClearNoon").unwrap().1
    }

    #[test]
    fn parts_map_to_carla_and_parts_not_given_stay() {
        let sky = Sky::default()
            .with(&ScenarioWeather {
                cloud_cover_oktas: Some(4),
                precipitation: Some(Precipitation {
                    kind: PrecipitationType::Rain,
                    intensity: 12.5,
                }),
                fog_visual_range: Some(250.0),
                ..Default::default()
            })
            .unwrap();
        assert_eq!(sky.cloudiness, Some(50.0));
        assert_eq!(sky.precipitation, Some(50.0));
        assert_eq!(sky.fog_density, Some(75.0));
        // Only the wind changes; the rest stays.
        let sky = sky
            .with(&ScenarioWeather {
                wind: Some(Wind {
                    direction: 0.0,
                    speed: 40.0,
                }),
                ..Default::default()
            })
            .unwrap();
        assert_eq!(sky.wind_intensity, Some(100.0));
        assert_eq!(sky.cloudiness, Some(50.0));
        let w = sky.apply(clear_noon(), Some((10.0, 20.0)));
        assert_eq!(
            (
                w.cloudiness,
                w.precipitation,
                w.fog_density,
                w.wind_intensity
            ),
            (50.0, 50.0, 75.0, 100.0)
        );
        assert_eq!((w.sun_azimuth_angle, w.sun_altitude_angle), (10.0, 20.0));
        // Fog beyond a kilometre is none; nine oktas is overcast.
        let s = Sky::default()
            .with(&ScenarioWeather {
                fog_visual_range: Some(100_000.0),
                cloud_cover_oktas: Some(9),
                ..Default::default()
            })
            .unwrap();
        assert_eq!((s.fog_density, s.cloudiness), (Some(0.0), Some(100.0)));
    }

    #[test]
    fn a_placed_sun_wins_over_a_time_of_day() {
        let sky = Sky::default()
            .with(&ScenarioWeather {
                sun: Some(Sun {
                    azimuth: std::f64::consts::PI, // south
                    elevation: 0.5_f64.to_radians() * 60.0,
                    illuminance: 0.0,
                }),
                ..Default::default()
            })
            .unwrap();
        let w = sky.apply(clear_noon(), Some((1.0, 2.0)));
        assert!((w.sun_azimuth_angle - 90.0).abs() < 1e-3);
        assert!((w.sun_altitude_angle - 30.0).abs() < 1e-3);
        // North is 270 from east, east 0.
        assert!((carla_sun(0.0, 0.0).0 - 270.0).abs() < 1e-3);
        assert!(carla_sun(std::f64::consts::FRAC_PI_2, 0.0).0.abs() < 1e-3);
    }

    #[test]
    fn snow_is_refused() {
        let e = Sky::default()
            .with(&ScenarioWeather {
                precipitation: Some(Precipitation {
                    kind: PrecipitationType::Snow,
                    intensity: 2.0,
                }),
                ..Default::default()
            })
            .unwrap_err();
        assert!(e.contains("snow"), "{e}");
    }
}
