//! OpenSCENARIO's weather and time of day (roadmap 047), as SSv2's `UpdateEnvironment` brings
//! them: the parts a scenario gives, checked, and where the sun stands at a time of day.
//!
//! Every part is optional, and one not given leaves that part of the world as it was. A world
//! server shows what it can and refuses the rest; temperature and air pressure are not carried
//! (no simulator here renders them).

/// The parts of a `Weather` the scenario gave.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ScenarioWeather {
    /// 0 (clear) to 8 (overcast); 9: the sky cannot be seen.
    pub cloud_cover_oktas: Option<u8>,
    pub sun: Option<Sun>,
    /// m.
    pub fog_visual_range: Option<f64>,
    pub precipitation: Option<Precipitation>,
    pub wind: Option<Wind>,
}

impl ScenarioWeather {
    pub fn is_empty(&self) -> bool {
        *self == Self::default()
    }
}

/// Map frame: azimuth clockwise from north (+y), elevation above the horizon, both rad;
/// illuminance lx (0: not given).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sun {
    pub azimuth: f64,
    pub elevation: f64,
    pub illuminance: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PrecipitationType {
    Dry,
    Rain,
    Snow,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Precipitation {
    pub kind: PrecipitationType,
    /// mm/h.
    pub intensity: f64,
}

/// Map frame: the direction it blows to, rad counterclockwise from +x; speed m/s.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Wind {
    pub direction: f64,
    pub speed: f64,
}

/// A `TimeOfDay`: the scenario's local clock time and day of the year.
#[derive(Debug, Clone, PartialEq)]
pub struct TimeOfDay {
    /// h, 0-24.
    pub hours: f64,
    /// 1-366.
    pub day_of_year: u32,
    /// The time runs on with the simulation.
    pub animation: bool,
    /// As written, for the log.
    pub date_time: String,
}

/// OpenSCENARIO's `FractionalCloudCover` literal (`zeroOktas` .. `nineOktas`) in oktas.
pub fn fractional_cloud_cover(literal: &str) -> Option<u8> {
    const NAMES: [&str; 10] = [
        "zeroOktas",
        "oneOktas",
        "twoOktas",
        "threeOktas",
        "fourOktas",
        "fiveOktas",
        "sixOktas",
        "sevenOktas",
        "eightOktas",
        "nineOktas",
    ];
    NAMES.iter().position(|n| *n == literal).map(|i| i as u8)
}

/// The deprecated `cloudState` in oktas: free 0, cloudy 4, overcast and rainy 8, skyOff 9.
pub fn cloud_state(literal: &str) -> Option<u8> {
    match literal {
        "free" => Some(0),
        "cloudy" => Some(4),
        "overcast" | "rainy" => Some(8),
        "skyOff" => Some(9),
        _ => None,
    }
}

pub fn precipitation_type(literal: &str) -> Option<PrecipitationType> {
    match literal {
        "dry" => Some(PrecipitationType::Dry),
        "rain" => Some(PrecipitationType::Rain),
        "snow" => Some(PrecipitationType::Snow),
        _ => None,
    }
}

/// An `xsd:dateTime` (`2026-10-23T14:30:00`, a fraction and a zone allowed) as the clock time
/// written, h, and the day of the year. The zone is not applied: the time as written is taken
/// for the local solar time.
pub fn date_time(s: &str) -> Result<(f64, u32), String> {
    let bad = || format!("dateTime {s:?} is not YYYY-MM-DDThh:mm:ss");
    let (date, time) = s.trim().split_once('T').ok_or_else(bad)?;
    let mut d = date.splitn(3, '-');
    let (y, m, day): (i64, u32, u32) = match (d.next(), d.next(), d.next()) {
        (Some(y), Some(m), Some(day)) => (
            y.parse().map_err(|_| bad())?,
            m.parse().map_err(|_| bad())?,
            day.parse().map_err(|_| bad())?,
        ),
        _ => return Err(bad()),
    };
    // The zone (Z, +hh:mm, -hh:mm) is dropped; a '-' in it comes after the seconds.
    let clock = time
        .split(['Z', '+'])
        .next()
        .unwrap_or("")
        .split('-')
        .next()
        .unwrap_or("");
    let mut t = clock.splitn(3, ':');
    let (hh, mm, ss): (u32, u32, f64) = match (t.next(), t.next(), t.next()) {
        (Some(h), Some(m), Some(s)) => (
            h.parse().map_err(|_| bad())?,
            m.parse().map_err(|_| bad())?,
            s.parse().map_err(|_| bad())?,
        ),
        _ => return Err(bad()),
    };
    let leap = (y % 4 == 0 && y % 100 != 0) || y % 400 == 0;
    let days = [
        31,
        if leap { 29 } else { 28 },
        31,
        30,
        31,
        30,
        31,
        31,
        30,
        31,
        30,
        31,
    ];
    if !(1..=12).contains(&m) || day == 0 || day > days[m as usize - 1] {
        return Err(bad());
    }
    if hh > 24 || mm > 59 || !(0.0..61.0).contains(&ss) {
        return Err(bad());
    }
    let doy = days[..m as usize - 1].iter().sum::<u32>() + day;
    Ok((f64::from(hh) + f64::from(mm) / 60.0 + ss / 3600.0, doy))
}

/// Where the sun stands at local solar time `hours` on `day_of_year` at `latitude` (deg):
/// azimuth clockwise from north and elevation, rad. A simple declination model (within about
/// a degree), enough for light and shadow.
pub fn sun_position(hours: f64, day_of_year: u32, latitude: f64) -> (f64, f64) {
    let decl = 23.44_f64.to_radians()
        * (2.0 * std::f64::consts::PI * (284.0 + f64::from(day_of_year)) / 365.0).sin();
    let h = (15.0 * (hours - 12.0)).to_radians();
    let phi = latitude.to_radians();
    let elevation = (phi.sin() * decl.sin() + phi.cos() * decl.cos() * h.cos())
        .clamp(-1.0, 1.0)
        .asin();
    let azimuth = (-decl.cos() * h.sin())
        .atan2(decl.sin() * phi.cos() - decl.cos() * phi.sin() * h.cos())
        .rem_euclid(2.0 * std::f64::consts::PI);
    (azimuth, elevation)
}

/// The latitude of a map (deg): `<map dir>/map_projector_info.yaml`'s `map_origin.latitude`;
/// 0 where there is none (CARLA's towns are at 0, 0).
pub fn map_latitude(map_dir: &std::path::Path) -> f64 {
    std::fs::read_to_string(map_dir.join("map_projector_info.yaml"))
        .ok()
        .and_then(|s| serde_yaml::from_str::<serde_yaml::Value>(&s).ok())
        .and_then(|v| v["map_origin"]["latitude"].as_f64())
        .unwrap_or(0.0)
}

/// A scenario's time of day on a server's clock: where it stands `elapsed` s after it was
/// set, when it animates.
pub fn hours_after(time: &TimeOfDay, elapsed: f64) -> (f64, u32) {
    if !time.animation {
        return (time.hours, time.day_of_year);
    }
    let h = time.hours + elapsed.max(0.0) / 3600.0;
    let days = (h / 24.0).floor();
    (
        h - days * 24.0,
        ((f64::from(time.day_of_year) - 1.0 + days).rem_euclid(365.0) + 1.0) as u32,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn deg(x: f64) -> f64 {
        x.to_degrees()
    }

    /// The same cases as isaac_world's test_environment: both servers put the sun alike.
    #[test]
    fn the_sun_stands_where_it_should() {
        // Equator, June solstice, noon: high in the north.
        let (az, el) = sun_position(12.0, 172, 0.0);
        assert!((deg(el) - 66.56).abs() < 0.1, "{}", deg(el));
        assert!(
            deg(az).abs() < 0.5 || (deg(az) - 360.0).abs() < 0.5,
            "{}",
            deg(az)
        );
        // Equinox, 06:00 at the equator: rising in the east.
        let (az, el) = sun_position(6.0, 81, 0.0);
        assert!(deg(el).abs() < 1.0, "{}", deg(el));
        assert!((deg(az) - 90.0).abs() < 1.0, "{}", deg(az));
        // 45 N, equinox noon: 45 degrees up, due south.
        let (az, el) = sun_position(12.0, 81, 45.0);
        assert!((deg(el) - 45.0).abs() < 1.0, "{}", deg(el));
        assert!((deg(az) - 180.0).abs() < 0.5, "{}", deg(az));
        // 18:00 at the equinox: setting in the west; midnight: down.
        let (az, _) = sun_position(18.0, 81, 0.0);
        assert!((deg(az) - 270.0).abs() < 1.0, "{}", deg(az));
        assert!(sun_position(0.0, 81, 45.0).1 < 0.0);
    }

    #[test]
    fn a_date_time_is_its_clock_time_and_day() {
        assert_eq!(date_time("2026-10-23T14:30:00").unwrap(), (14.5, 296));
        assert_eq!(date_time("2024-03-01T06:00:00+09:00").unwrap(), (6.0, 61));
        assert_eq!(date_time("2023-03-01T06:00:00Z").unwrap(), (6.0, 60));
        assert_eq!(date_time("2026-01-01T00:00:30.0-05:00").unwrap().1, 1);
        for bad in [
            "14:30",
            "2026-13-01T00:00:00",
            "2026-02-30T00:00:00",
            "2026-01-01T25:00:00",
        ] {
            assert!(date_time(bad).is_err(), "{bad}");
        }
    }

    #[test]
    fn an_animated_time_runs_on_and_a_fixed_one_stays() {
        let mut t = TimeOfDay {
            hours: 23.5,
            day_of_year: 10,
            animation: true,
            date_time: String::new(),
        };
        assert_eq!(hours_after(&t, 3600.0), (0.5, 11));
        t.animation = false;
        assert_eq!(hours_after(&t, 3600.0), (23.5, 10));
    }

    #[test]
    fn cloud_literals_are_oktas() {
        assert_eq!(fractional_cloud_cover("zeroOktas"), Some(0));
        assert_eq!(fractional_cloud_cover("nineOktas"), Some(9));
        assert_eq!(fractional_cloud_cover("tenOktas"), None);
        assert_eq!(cloud_state("free"), Some(0));
        assert_eq!(cloud_state("skyOff"), Some(9));
        assert_eq!(precipitation_type("snow"), Some(PrecipitationType::Snow));
        assert_eq!(precipitation_type("hail"), None);
    }
}
