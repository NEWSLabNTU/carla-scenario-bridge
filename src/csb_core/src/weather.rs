//! CARLA weather as the bridge configures it: presets by name, and the values the
//! `/carla/set_weather` and `/carla/get_weather` services carry (roadmap 018). Plain data:
//! the conversion to and from carla-rust's `WeatherParameters` is the CARLA backend's
//! (`backend::carla::weather`), which also checks the presets below against carla-rust's.
//!
//! Weather is world state, not scenario state: SSv2 does not command it (its
//! `EnvironmentAction` is parse-only), and every `load_world` resets it to the new town's
//! default. So the bridge holds the weather the user asked for -- the `weather` parameter,
//! then the last `set_weather` -- and applies it again after every map load and reconnect.

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

impl Weather {
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

/// One preset's values in carla-rust's field order (`rpc::weather::wp`), so the rows below
/// read like its table.
#[allow(clippy::too_many_arguments)]
const fn w(
    cloudiness: f32,
    precipitation: f32,
    precipitation_deposits: f32,
    wind_intensity: f32,
    sun_azimuth_angle: f32,
    sun_altitude_angle: f32,
    fog_density: f32,
    fog_distance: f32,
    fog_falloff: f32,
    wetness: f32,
    scattering_intensity: f32,
    mie_scattering_scale: f32,
    rayleigh_scattering_scale: f32,
    dust_storm: f32,
) -> Weather {
    Weather {
        cloudiness,
        precipitation,
        precipitation_deposits,
        wind_intensity,
        fog_density,
        fog_distance,
        wetness,
        sun_azimuth_angle,
        sun_altitude_angle,
        fog_falloff,
        scattering_intensity,
        mie_scattering_scale,
        rayleigh_scattering_scale,
        dust_storm,
    }
}

/// CARLA's weather presets, under their Python API names (`carla.WeatherParameters.<Name>`).
///
/// Values from carla-rust's `rpc::weather`, which reproduces the static presets of CARLA's
/// C++ `WeatherParameters` (LibCarla/source/carla/rpc/WeatherParameters.cpp); the CARLA
/// backend's tests compare every row with carla-rust's. The names keep CARLA's own
/// inconsistencies (`MidRainyNoon` but `MidRainSunset`). `Default` is left out: its all-`-1`
/// values mean "the map's own", which is what an empty `weather` already does.
#[rustfmt::skip]
pub const PRESETS: &[(&str, Weather)] = &[
    //                                cloud  precip dep   wind   azim  alt    fogD   fogDist fogF wet    scatI mieS  raylS   dust
    ("ClearNoon",        w(  5.0,   0.0,  0.0,  10.0, -1.0,  45.0,   2.0,  0.75, 0.1,   0.0, 1.0, 0.03, 0.0331,   0.0)),
    ("CloudyNoon",       w( 60.0,   0.0,  0.0,  10.0, -1.0,  45.0,   3.0,  0.75, 0.1,   0.0, 1.0, 0.03, 0.0331,   0.0)),
    ("WetNoon",          w(  5.0,   0.0, 50.0,  10.0, -1.0,  45.0,   3.0,  0.75, 0.1,   0.0, 1.0, 0.03, 0.0331,   0.0)),
    ("WetCloudyNoon",    w( 60.0,   0.0, 50.0,  10.0, -1.0,  45.0,   3.0,  0.75, 0.1,   0.0, 1.0, 0.03, 0.0331,   0.0)),
    ("SoftRainNoon",     w( 20.0,  30.0, 50.0,  30.0, -1.0,  45.0,   3.0,  0.75, 0.1,   0.0, 1.0, 0.03, 0.0331,   0.0)),
    ("MidRainyNoon",     w( 60.0,  60.0, 60.0,  60.0, -1.0,  45.0,   3.0,  0.75, 0.1,   0.0, 1.0, 0.03, 0.0331,   0.0)),
    ("HardRainNoon",     w(100.0, 100.0, 90.0, 100.0, -1.0,  45.0,   7.0,  0.75, 0.1,   0.0, 1.0, 0.03, 0.0331,   0.0)),
    ("ClearSunset",      w(  5.0,   0.0,  0.0,  10.0, -1.0,  15.0,   2.0,  0.75, 0.1,   0.0, 1.0, 0.03, 0.0331,   0.0)),
    ("CloudySunset",     w( 60.0,   0.0,  0.0,  10.0, -1.0,  15.0,   3.0,  0.75, 0.1,   0.0, 1.0, 0.03, 0.0331,   0.0)),
    ("WetSunset",        w(  5.0,   0.0, 50.0,  10.0, -1.0,  15.0,   2.0,  0.75, 0.1,   0.0, 1.0, 0.03, 0.0331,   0.0)),
    ("WetCloudySunset",  w( 60.0,   0.0, 50.0,  10.0, -1.0,  15.0,   2.0,  0.75, 0.1,   0.0, 1.0, 0.03, 0.0331,   0.0)),
    ("SoftRainSunset",   w( 20.0,  30.0, 50.0,  30.0, -1.0,  15.0,   2.0,  0.75, 0.1,   0.0, 1.0, 0.03, 0.0331,   0.0)),
    ("MidRainSunset",    w( 60.0,  60.0, 60.0,  60.0, -1.0,  15.0,   3.0,  0.75, 0.1,   0.0, 1.0, 0.03, 0.0331,   0.0)),
    ("HardRainSunset",   w(100.0, 100.0, 90.0, 100.0, -1.0,  15.0,   7.0,  0.75, 0.1,   0.0, 1.0, 0.03, 0.0331,   0.0)),
    ("ClearNight",       w(  5.0,   0.0,  0.0,  10.0, -1.0, -90.0,  60.0, 75.0,  1.0,   0.0, 1.0, 0.03, 0.0331,   0.0)),
    ("CloudyNight",      w( 60.0,   0.0,  0.0,  10.0, -1.0, -90.0,  60.0,  0.75, 0.1,   0.0, 1.0, 0.03, 0.0331,   0.0)),
    ("WetNight",         w(  5.0,   0.0, 50.0,  10.0, -1.0, -90.0,  60.0, 75.0,  1.0,  60.0, 1.0, 0.03, 0.0331,   0.0)),
    ("WetCloudyNight",   w( 60.0,   0.0, 50.0,  10.0, -1.0, -90.0,  60.0,  0.75, 0.1,  60.0, 1.0, 0.03, 0.0331,   0.0)),
    ("SoftRainNight",    w( 60.0,  30.0, 50.0,  30.0, -1.0, -90.0,  60.0,  0.75, 0.1,  60.0, 1.0, 0.03, 0.0331,   0.0)),
    ("MidRainyNight",    w( 80.0,  60.0, 60.0,  60.0, -1.0, -90.0,  60.0,  0.75, 0.1,  80.0, 1.0, 0.03, 0.0331,   0.0)),
    ("HardRainNight",    w(100.0, 100.0, 90.0, 100.0, -1.0, -90.0, 100.0,  0.75, 0.1, 100.0, 1.0, 0.03, 0.0331,   0.0)),
    ("DustStorm",        w(100.0,   0.0,  0.0, 100.0, -1.0,  45.0,   2.0,  0.75, 0.1,   0.0, 1.0, 0.03, 0.0331, 100.0)),
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
        .map(|(n, w)| (*n, *w))
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
        .find(|(_, p)| p.approx_eq(w))
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

/// The answer to either weather service (the world protocol's `WeatherReply`).
#[derive(Debug, Clone, PartialEq)]
pub struct WeatherReply {
    pub success: bool,
    pub message: String,
    pub weather: Option<Weather>,
}

impl WeatherReply {
    pub fn ok(weather: Weather, message: impl Into<String>) -> Self {
        Self {
            success: true,
            message: message.into(),
            weather: Some(weather),
        }
    }

    pub fn failed(message: impl Into<String>, weather: Option<Weather>) -> Self {
        Self {
            success: false,
            message: message.into(),
            weather,
        }
    }
}

/// The world protocol's names for [`Weather`]'s fields, in its order: the nine a request may
/// set first (`csb_interfaces/msg/Weather` order), then the rest.
pub const FIELD_NAMES: [&str; 14] = [
    "cloudiness",
    "precipitation",
    "precipitation_deposits",
    "wind_intensity",
    "fog_density",
    "fog_distance",
    "wetness",
    "sun_azimuth_angle",
    "sun_altitude_angle",
    "fog_falloff",
    "scattering_intensity",
    "mie_scattering_scale",
    "rayleigh_scattering_scale",
    "dust_storm",
];

impl Weather {
    /// Every field, in [`FIELD_NAMES`] order.
    pub fn fields(&self) -> [f32; 14] {
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

    /// From every field, in [`FIELD_NAMES`] order.
    pub fn from_fields(f: [f32; 14]) -> Self {
        Self {
            cloudiness: f[0],
            precipitation: f[1],
            precipitation_deposits: f[2],
            wind_intensity: f[3],
            fog_density: f[4],
            fog_distance: f[5],
            wetness: f[6],
            sun_azimuth_angle: f[7],
            sun_altitude_angle: f[8],
            fog_falloff: f[9],
            scattering_intensity: f[10],
            mie_scattering_scale: f[11],
            rayleigh_scattering_scale: f[12],
            dust_storm: f[13],
        }
    }
}

/// The world protocol's form of a weather and of the requests and replies about it.
pub mod wire_form {
    use super::*;
    use crate::proto::csb_world as wire;

    fn parameters(names: &[&str], values: &[f32]) -> std::collections::HashMap<String, f64> {
        names
            .iter()
            .zip(values)
            .map(|(n, v)| ((*n).to_string(), f64::from(*v)))
            .collect()
    }

    /// A weather as the protocol carries it: every field, and the preset it matches.
    pub fn weather(w: &Weather) -> wire::Weather {
        wire::Weather {
            preset: matching_preset(w).unwrap_or_default().to_string(),
            parameters: parameters(&FIELD_NAMES, &w.fields()),
            use_parameters: true,
        }
    }

    /// The weather a protocol `Weather` describes: its preset, else its parameters (fields
    /// it does not name are 0). `None` when it names neither.
    pub fn weather_of(w: &wire::Weather) -> Option<Weather> {
        if !w.use_parameters {
            return preset(&w.preset).map(|(_, w)| w);
        }
        let mut f = [0.0_f32; 14];
        for (i, name) in FIELD_NAMES.iter().enumerate() {
            f[i] = w.parameters.get(*name).copied().unwrap_or_default() as f32;
        }
        Some(Weather::from_fields(f))
    }

    pub fn request(r: &WeatherRequest) -> wire::Weather {
        match r {
            WeatherRequest::Preset(name, _) => wire::Weather {
                preset: (*name).to_string(),
                parameters: Default::default(),
                use_parameters: false,
            },
            WeatherRequest::Parameters(f) => wire::Weather {
                preset: String::new(),
                parameters: parameters(&FIELD_NAMES[..9], f),
                use_parameters: true,
            },
        }
    }

    /// The request a protocol `Weather` makes, validated as the ROS service validates one.
    pub fn request_of(w: &wire::Weather) -> Result<WeatherRequest, String> {
        let mut f = [0.0_f32; 9];
        for (i, name) in FIELD_NAMES[..9].iter().enumerate() {
            f[i] = w.parameters.get(*name).copied().unwrap_or_default() as f32;
        }
        decode_request(&w.preset, w.use_parameters, f)
    }

    pub fn reply(r: &WeatherReply) -> wire::WeatherReply {
        wire::WeatherReply {
            success: r.success,
            message: r.message.clone(),
            weather: r.weather.as_ref().map(weather),
        }
    }

    pub fn reply_of(r: &wire::WeatherReply) -> WeatherReply {
        WeatherReply {
            success: r.success,
            message: r.message.clone(),
            weather: r.weather.as_ref().and_then(weather_of),
        }
    }
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

    #[test]
    fn the_wire_form_round_trips() {
        let (_, w) = preset("WetCloudyNoon").unwrap();
        assert_eq!(wire_form::weather_of(&wire_form::weather(&w)), Some(w));
        let r = WeatherRequest::Parameters([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]);
        assert_eq!(wire_form::request_of(&wire_form::request(&r)), Ok(r));
        let p = decode_request("ClearNoon", false, [0.0; 9]).unwrap();
        assert_eq!(wire_form::request_of(&wire_form::request(&p)), Ok(p));
        let reply = WeatherReply::ok(w, "set");
        assert_eq!(wire_form::reply_of(&wire_form::reply(&reply)), reply);
    }
}
