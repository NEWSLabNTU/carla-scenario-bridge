//! [`Weather`] to and from carla-rust's `WeatherParameters`.

use carla::rpc::WeatherParameters;

use crate::weather::Weather;

/// CARLA's weather as the shared [`Weather`].
pub fn from_carla(w: &WeatherParameters) -> Weather {
    {
        Weather {
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

/// The shared [`Weather`] as CARLA's.
pub fn to_carla(w: Weather) -> WeatherParameters {
    let s = w;
    {
        WeatherParameters {
            cloudiness: s.cloudiness,
            precipitation: s.precipitation,
            precipitation_deposits: s.precipitation_deposits,
            wind_intensity: s.wind_intensity,
            sun_azimuth_angle: s.sun_azimuth_angle,
            sun_altitude_angle: s.sun_altitude_angle,
            fog_density: s.fog_density,
            fog_distance: s.fog_distance,
            fog_falloff: s.fog_falloff,
            wetness: s.wetness,
            scattering_intensity: s.scattering_intensity,
            mie_scattering_scale: s.mie_scattering_scale,
            rayleigh_scattering_scale: s.rayleigh_scattering_scale,
            dust_storm: s.dust_storm,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use carla::rpc::weather as presets;

    #[test]
    fn carla_round_trip_keeps_every_field() {
        let (_, w) = crate::weather::preset("WetCloudyNight").unwrap();
        assert_eq!(from_carla(&to_carla(w)), w);
    }

    /// The shared table is a copy of carla-rust's presets; a carla-rust release that changes
    /// one fails here.
    #[test]
    fn every_preset_is_carla_rusts() {
        let carla: &[(&str, WeatherParameters)] = &[
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
        let ours = crate::weather::PRESETS;
        assert_eq!(ours.len(), carla.len());
        for ((name, w), (carla_name, p)) in ours.iter().zip(carla) {
            assert_eq!(name, carla_name);
            assert_eq!(*w, from_carla(p), "{name}");
        }
    }
}
