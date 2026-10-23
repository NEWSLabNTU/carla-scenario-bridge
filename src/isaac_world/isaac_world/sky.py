"""The scenario's weather and time of day on Isaac (roadmap 047), pure Python: what Isaac Sim
6.1 can show of OpenSCENARIO's `Weather` and `TimeOfDay` (measured: design doc, "Weather and
time of day on Isaac"), and the settings for it. `world.IsaacWorld` applies them.

  time of day, a placed sun  the world's sun (a distant light) turned to where the sun stands
                             over the map's latitude, dimmed toward the horizon, off below it;
                             the sky's dome lights dimmed with it (night: 3 %)
  cloud cover                no clouds to draw: the sun dimmed by up to 80 % at 8 oktas, off at 9
                             (the sky obscured), the dome left as it is -- a lighting
                             approximation, cameras only
  fog                        RTX fog (cameras only): distance-based, fully fogged at the visual
                             range; none from 1000 m
  rain                       the RTX lidar's rain model, `rainRate` in mm/h (lidar only; cameras
                             see no rain). Measured: the lidar does not fully recover when the
                             rate goes back to 0
  snow                       refused: nothing in Isaac simulates it
  wind                       nothing to move: taken and ignored

Every part is optional; one not given keeps its last value. `LoadWorld` and a release go back to
the world's own lights, no fog, no rain. Engineered numbers throughout, not calibrated.
"""
import math
from dataclasses import dataclass, field, replace
from pathlib import Path

PRECIPITATION = ("dry", "rain", "snow")
# Fog thinner than this is none, m.
FOG_NONE = 1000.0
# The dome's share of its own intensity at night (the sun 6 deg or more below the horizon).
NIGHT_DOME = 0.03
RAIN_RATE = "/app/sensors/nv/atmospherics/rainRate"
FOG = "/rtx/fog"


def sun_position(hours, day_of_year, latitude):
    """Azimuth clockwise from north and elevation (rad) at local solar time `hours` on
    `day_of_year` at `latitude` (deg). The same model as swb_core::environment::sun_position."""
    decl = math.radians(23.44) * math.sin(2 * math.pi * (284.0 + day_of_year) / 365.0)
    h = math.radians(15.0 * (hours - 12.0))
    phi = math.radians(latitude)
    s = math.sin(phi) * math.sin(decl) + math.cos(phi) * math.cos(decl) * math.cos(h)
    elevation = math.asin(max(-1.0, min(1.0, s)))
    azimuth = math.atan2(-math.cos(decl) * math.sin(h),
                         math.sin(decl) * math.cos(phi) - math.cos(decl) * math.sin(phi)
                         * math.cos(h)) % (2 * math.pi)
    return azimuth, elevation


def hours_after(time, elapsed):
    """(hours, day of year) of a time of day `elapsed` s after it was set, if it animates."""
    if not time["animation"]:
        return time["hours"], time["day_of_year"]
    h = time["hours"] + max(0.0, elapsed) / 3600.0
    days = math.floor(h / 24.0)
    return h - 24.0 * days, int((time["day_of_year"] - 1 + days) % 365 + 1)


def map_latitude(map_dir):
    """`<map dir>/map_projector_info.yaml`'s map_origin.latitude, deg; 0 without one."""
    import yaml

    try:
        with open(Path(map_dir) / "map_projector_info.yaml") as f:
            return float((yaml.safe_load(f) or {}).get("map_origin", {}).get("latitude", 0.0))
    except (OSError, TypeError, ValueError, AttributeError):
        return 0.0


def toward_sun(azimuth, elevation):
    """The unit vector toward the sun in the map frame (x east, y north, z up)."""
    return (math.sin(azimuth) * math.cos(elevation), math.cos(azimuth) * math.cos(elevation),
            math.sin(elevation))


@dataclass(frozen=True)
class Sky:
    """The scenario's weather parts so far, and its time of day."""

    cloud_oktas: int = None
    sun: tuple = None  # (azimuth, elevation, illuminance): a placed sun
    fog_visual_range: float = None
    rain_rate: float = None  # mm/h
    wind: tuple = None
    time: dict = None  # {hours, day_of_year, animation, date_time}
    time_set_at: float = 0.0  # the server's clock when the time was set, s
    changed: tuple = field(default=(), compare=False)  # parts the last `with_` set

    def is_default(self):
        return replace(self, changed=()) == Sky()

    def with_(self, weather=None, time_of_day=None, now=0.0):
        """These parts with the scenario's new ones laid over, or ValueError listing what
        Isaac cannot show."""
        weather = weather or {}
        problems = []
        out, changed = self, []
        p = weather.get("precipitation")
        if p is not None:
            kind, intensity = p
            if kind == "snow":
                problems.append("snow: Isaac Sim simulates none (rain reaches the lidar only)")
            elif kind not in PRECIPITATION:
                problems.append(f"precipitation {kind!r} is not dry, rain or snow")
            elif not (math.isfinite(intensity) and intensity >= 0):
                problems.append(f"precipitation intensity {intensity} is not in [0, inf) mm/h")
            else:
                out = replace(out, rain_rate=float(intensity) if kind == "rain" else 0.0)
                changed.append("rain")
        if problems:
            raise ValueError("\n".join(problems))
        if weather.get("cloud_cover_oktas") is not None:
            out = replace(out, cloud_oktas=min(9, int(weather["cloud_cover_oktas"])))
            changed.append("clouds")
        if weather.get("sun") is not None:
            out = replace(out, sun=tuple(weather["sun"]))
            changed.append("sun")
        if weather.get("fog_visual_range") is not None:
            out = replace(out, fog_visual_range=float(weather["fog_visual_range"]))
            changed.append("fog")
        if weather.get("wind") is not None:
            out = replace(out, wind=tuple(weather["wind"]))
            changed.append("wind")
        if time_of_day is not None:
            out = replace(out, time=dict(time_of_day), time_set_at=now)
            changed.append("time")
        return replace(out, changed=tuple(changed))

    def sun_at(self, latitude, now=0.0):
        """(azimuth, elevation, illuminance or 0) of the scenario's sun: placed, or from the time
        of day; None when it set neither."""
        if self.sun is not None:
            return self.sun
        if self.time is not None:
            hours, day = hours_after(self.time, now - self.time_set_at)
            return (*sun_position(hours, day, latitude), 0.0)
        return None

    def lights(self, latitude, now=0.0):
        """(sun direction toward the sun or None, sun intensity factor, dome factor) for the
        world's lights; None when the scenario has set nothing they show."""
        sun = self.sun_at(latitude, now)
        if sun is None and self.cloud_oktas is None:
            return None
        if sun is None:
            direction, sun_f, dome_f = None, 1.0, 1.0
        else:
            azimuth, elevation, lux = sun
            direction = toward_sun(azimuth, elevation)
            el = math.degrees(elevation)
            # Full sun from 30 deg up, fading to nothing at the horizon.
            sun_f = max(0.0, min(1.0, math.sin(elevation) / math.sin(math.radians(30.0))))
            if lux > 0:
                sun_f = min(sun_f, lux / 100000.0)
            dome_f = max(NIGHT_DOME, min(1.0, (el + 6.0) / 18.0))
        if self.cloud_oktas is not None:
            sun_f *= 0.0 if self.cloud_oktas >= 9 else 1.0 - 0.8 * self.cloud_oktas / 8.0
        return direction, sun_f, dome_f

    def fog_settings(self):
        """The RTX fog settings ({path: value}), or None for no fog."""
        v = self.fog_visual_range
        if v is None or v >= FOG_NONE:
            return None
        return {f"{FOG}/enabled": True, f"{FOG}/fogDistanceBased/enabled": True,
                f"{FOG}/fogStartDist": 0.0, f"{FOG}/fogEndDist": max(1.0, v),
                f"{FOG}/fogDistanceDensity": 1.0, f"{FOG}/fogColorIntensity": 1.0}
