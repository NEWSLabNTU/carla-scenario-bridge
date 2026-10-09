//! CARLA weather as the bridge configures it: presets by name, and the values the
//! `/carla/set_weather` and `/carla/get_weather` services carry (roadmap 018).
//!
//! Weather is world state, not scenario state: SSv2 does not command it (its
//! `EnvironmentAction` is parse-only), and every `load_world` resets it to the new town's
//! default. So the bridge holds the weather the user asked for -- the `weather` parameter,
//! then the last `set_weather` -- and applies it again after every map load and reconnect.

use carla::rpc::{weather as presets, WeatherParameters};

/// Every field of CARLA 0.9.16's `WeatherParameters`, as plain data.
///
/// The services expose the first nine; the rest ride along so that explicit parameters
/// leave them at CARLA's current values and preset matching can see them.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Weather {
    pub cloudiness: f32,
    pub precipitation: f32,
    pub precipitation_deposits: f32,
    pub wind_intensity: f32,
    pub fog_density: f32,
    pub fog_distance: f32,
    pub wetness: f32,
    pub sun_azimuth_angle: f32,
    pub sun_altitude_angle: f32,
    pub fog_falloff: f32,
    pub scattering_intensity: f32,
    pub mie_scattering_scale: f32,
    pub rayleigh_scattering_scale: f32,
    pub dust_storm: f32,
}

impl From<&WeatherParameters> for Weather {
    fn from(w: &WeatherParameters) -> Self {
        Self {
            cloudiness: w.cloudiness,
            precipitation: w.precipitation,
            precipitation_deposits: w.precipitation_deposits,
            wind_intensity: w.wind_intensity,
            fog_density: w.fog_density,
            fog_distance: w.fog_distance,
            wetness: w.wetness,
            sun_azimuth_angle: w.sun_azimuth_angle,
            sun_altitude_angle: w.sun_altitude_angle,
            fog_falloff: w.fog_falloff,
            scattering_intensity: w.scattering_intensity,
            mie_scattering_scale: w.mie_scattering_scale,
            rayleigh_scattering_scale: w.rayleigh_scattering_scale,
            dust_storm: w.dust_storm,
        }
    }
}

impl Weather {
    pub fn to_carla(self) -> WeatherParameters {
        WeatherParameters {
            cloudiness: self.cloudiness,
            precipitation: self.precipitation,
            precipitation_deposits: self.precipitation_deposits,
            wind_intensity: self.wind_intensity,
            sun_azimuth_angle: self.sun_azimuth_angle,
            sun_altitude_angle: self.sun_altitude_angle,
            fog_density: self.fog_density,
            fog_distance: self.fog_distance,
            fog_falloff: self.fog_falloff,
            wetness: self.wetness,
            scattering_intensity: self.scattering_intensity,
            mie_scattering_scale: self.mie_scattering_scale,
            rayleigh_scattering_scale: self.rayleigh_scattering_scale,
            dust_storm: self.dust_storm,
        }
    }

    /// The nine user-settable fields, in `csb_interfaces/msg/Weather` order.
    pub fn user_fields(&self) -> [f32; 9] {
        [
            self.cloudiness,
            self.precipitation,
            self.precipitation_deposits,
            self.wind_intensity,
            self.fog_density,
            self.fog_distance,
            self.wetness,
            self.sun_azimuth_angle,
            self.sun_altitude_angle,
        ]
    }

    /// `self` with the nine user-settable fields replaced (msg order), the rest kept.
    pub fn with_user_fields(mut self, f: [f32; 9]) -> Self {
        self.cloudiness = f[0];
        self.precipitation = f[1];
        self.precipitation_deposits = f[2];
        self.wind_intensity = f[3];
        self.fog_density = f[4];
        self.fog_distance = f[5];
        self.wetness = f[6];
        self.sun_azimuth_angle = f[7];
        self.sun_altitude_angle = f[8];
        self
    }

    /// Equal within what a round trip through CARLA's server preserves.
    fn approx_eq(&self, other: &Self) -> bool {
        let a = self.all();
        let b = other.all();
        a.iter().zip(b.iter()).all(|(x, y)| (x - y).abs() <= 1e-3)
    }

    fn all(&self) -> [f32; 14] {
        let u = self.user_fields();
        [
            u[0],
            u[1],
            u[2],
            u[3],
            u[4],
            u[5],
            u[6],
            u[7],
            u[8],
            self.fog_falloff,
            self.scattering_intensity,
            self.mie_scattering_scale,
            self.rayleigh_scattering_scale,
            self.dust_storm,
        ]
    }
}

/// CARLA's weather presets, under their Python API names (`carla.WeatherParameters.<Name>`).
///
/// Values from carla-rust's `rpc::weather`, which reproduces the static presets of CARLA's
/// C++ `WeatherParameters` (LibCarla/source/carla/rpc/WeatherParameters.cpp). The names keep
/// CARLA's own inconsistencies (`MidRainyNoon` but `MidRainSunset`). `Default` is left out:
/// its all-`-1` values mean "the map's own", which is what an empty `weather` already does.
const PRESETS: &[(&str, WeatherParameters)] = &[
    ("ClearNoon", presets::CLEAR_NOON),
    ("CloudyNoon", presets::CLOUDY_NOON),
    ("WetNoon", presets::WET_NOON),
    ("WetCloudyNoon", presets::WET_CLOUDY_NOON),
    ("SoftRainNoon", presets::SOFT_RAIN_NOON),
    ("MidRainyNoon", presets::MID_RAIN_NOON),
    ("HardRainNoon", presets::HARD_RAIN_NOON),
    ("ClearSunset", presets::CLEAR_SUNSET),
    ("CloudySunset", presets::CLOUDY_SUNSET),
    ("WetSunset", presets::WET_SUNSET),
    ("WetCloudySunset", presets::WET_CLOUDY_SUNSET),
    ("SoftRainSunset", presets::SOFT_RAIN_SUNSET),
    ("MidRainSunset", presets::MID_RAIN_SUNSET),
    ("HardRainSunset", presets::HARD_RAIN_SUNSET),
    ("ClearNight", presets::CLEAR_NIGHT),
    ("CloudyNight", presets::CLOUDY_NIGHT),
    ("WetNight", presets::WET_NIGHT),
    ("WetCloudyNight", presets::WET_CLOUDY_NIGHT),
    ("SoftRainNight", presets::SOFT_RAIN_NIGHT),
    ("MidRainyNight", presets::MID_RAIN_NIGHT),
    ("HardRainNight", presets::HARD_RAIN_NIGHT),
    ("DustStorm", presets::DUST_STORM),
];

/// Every preset name, in CARLA's order.
pub fn preset_names() -> impl Iterator<Item = &'static str> {
    PRESETS.iter().map(|(n, _)| *n)
}

/// A preset by name, ignoring case and `_`/`-`/spaces (`clear_noon` finds `ClearNoon`).
/// Returns the canonical name with it.
pub fn preset(name: &str) -> Option<(&'static str, Weather)> {
    let key = normalise(name);
    PRESETS
        .iter()
        .find(|(n, _)| normalise(n) == key)
        .map(|(n, w)| (*n, Weather::from(w)))
}

/// [`preset`], or an error listing the valid names.
pub fn preset_or_err(name: &str) -> Result<(&'static str, Weather), String> {
    preset(name).ok_or_else(|| {
        format!(
            "unknown weather preset '{name}'; one of: {}",
            preset_names().collect::<Vec<_>>().join(", ")
        )
    })
}

/// The preset these values are, if any.
pub fn matching_preset(w: &Weather) -> Option<&'static str> {
    PRESETS
        .iter()
        .find(|(_, p)| Weather::from(p).approx_eq(w))
        .map(|(n, _)| *n)
}

fn normalise(s: &str) -> String {
    s.chars()
        .filter(|c| !matches!(c, '_' | '-' | ' '))
        .flat_map(char::to_lowercase)
        .collect()
}

/// What a `set_weather` request asks for, decoded from the srv fields.
#[derive(Debug, Clone, PartialEq)]
pub enum WeatherRequest {
    Preset(&'static str, Weather),
    /// The nine user-settable fields; the rest are taken from CARLA's current weather.
    Parameters([f32; 9]),
}

/// Decode a request: a non-empty `preset` wins; otherwise `use_parameters` must be set.
pub fn decode_request(
    preset_name: &str,
    use_parameters: bool,
    fields: [f32; 9],
) -> Result<WeatherRequest, String> {
    let preset_name = preset_name.trim();
    if !preset_name.is_empty() {
        let (name, w) = preset_or_err(preset_name)?;
        return Ok(WeatherRequest::Preset(name, w));
    }
    if use_parameters {
        return Ok(WeatherRequest::Parameters(fields));
    }
    Err(format!(
        "give a preset ({}) or use_parameters: true with weather values",
        preset_names().collect::<Vec<_>>().join(", ")
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn presets_are_found_by_python_name_and_loosely() {
        let (name, w) = preset("ClearNoon").unwrap();
        assert_eq!(name, "ClearNoon");
        assert_eq!(w.cloudiness, 5.0);
        assert_eq!(w.sun_altitude_angle, 45.0);
        assert_eq!(preset("clearnoon").unwrap().0, "ClearNoon");
        assert_eq!(preset("clear_noon").unwrap().0, "ClearNoon");
        assert_eq!(preset(" Hard-Rain Night").unwrap().0, "HardRainNight");
        assert_eq!(preset("MidRainyNoon").unwrap().0, "MidRainyNoon");
        assert!(preset("Default").is_none());
        assert!(preset("Sunny").is_none());
    }

    #[test]
    fn every_preset_matches_itself_and_only_itself() {
        for name in preset_names() {
            let (_, w) = preset(name).unwrap();
            assert_eq!(matching_preset(&w), Some(name), "{name}");
        }
        assert_eq!(preset_names().count(), 22);
    }

    #[test]
    fn matching_tolerates_float_noise_but_not_a_change() {
        let (_, mut w) = preset("CloudySunset").unwrap();
        w.cloudiness += 1e-5;
        assert_eq!(matching_preset(&w), Some("CloudySunset"));
        w.cloudiness += 1.0;
        assert_eq!(matching_preset(&w), None);
    }

    #[test]
    fn carla_round_trip_keeps_every_field() {
        let (_, w) = preset("WetCloudyNight").unwrap();
        assert_eq!(Weather::from(&w.to_carla()), w);
    }

    #[test]
    fn user_fields_replace_only_the_nine() {
        let (_, base) = preset("DustStorm").unwrap();
        let f = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0];
        let w = base.with_user_fields(f);
        assert_eq!(w.user_fields(), f);
        assert_eq!(w.dust_storm, base.dust_storm);
        assert_eq!(w.fog_falloff, base.fog_falloff);
        assert_eq!(w.fog_distance, 6.0);
        assert_eq!(w.sun_altitude_angle, 9.0);
    }

    #[test]
    fn request_decoding() {
        let zero = [0.0; 9];
        assert_eq!(
            decode_request("clearsunset", false, zero),
            Ok(WeatherRequest::Preset(
                "ClearSunset",
                preset("ClearSunset").unwrap().1
            ))
        );
        // A preset wins over parameters.
        assert!(matches!(
            decode_request("ClearNoon", true, zero),
            Ok(WeatherRequest::Preset("ClearNoon", _))
        ));
        assert_eq!(
            decode_request("", true, [1.0; 9]),
            Ok(WeatherRequest::Parameters([1.0; 9]))
        );
        // An empty request must not set all-zero weather.
        let e = decode_request("", false, zero).unwrap_err();
        assert!(e.contains("use_parameters"), "{e}");
        let e = decode_request("Sunny", false, zero).unwrap_err();
        assert!(
            e.contains("unknown weather preset 'Sunny'") && e.contains("ClearNoon"),
            "{e}"
        );
    }
}
