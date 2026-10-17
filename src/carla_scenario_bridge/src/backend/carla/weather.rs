//! [`Weather`] to and from carla-rust's `WeatherParameters`.

use carla::rpc::WeatherParameters;

use crate::weather::Weather;

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
}

#[cfg(test)]
mod tests {
    use super::*;
    use carla::rpc::weather as presets;

    #[test]
    fn carla_round_trip_keeps_every_field() {
        let (_, w) = crate::weather::preset("WetCloudyNight").unwrap();
        assert_eq!(Weather::from(&w.to_carla()), w);
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
            assert_eq!(*w, Weather::from(p), "{name}");
        }
    }
}
