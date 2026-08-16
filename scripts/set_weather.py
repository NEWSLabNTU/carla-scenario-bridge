#!/usr/bin/env python3
"""Set CARLA's weather, because the default is not a neutral choice.

Town01 comes up at cloudiness 60, precipitation 40 and a sun 20 degrees above the
horizon: overcast and raining, in every frame of every run. Nothing in this stack sets
it, so that is what Autoware's cameras have always seen -- and a traffic light classifier
trained on real daylight footage is being asked to read a wet, dim, low-contrast scene.

Weather is not scenario state. SSv2 does not command it and csb does not touch it, so it
persists in the CARLA server across runs until something changes it. Run this once after
starting the server, or whenever a run needs a known sky.

    scripts/set_weather.py             # ClearNoon
    scripts/set_weather.py ClearSunset
    scripts/set_weather.py --list
"""
import sys

try:
    import carla
except ImportError:
    sys.exit("carla PythonAPI not importable; put the unpacked wheel on PYTHONPATH")


def presets():
    return sorted(n for n in dir(carla.WeatherParameters)
                  if not n.startswith("_")
                  and isinstance(getattr(carla.WeatherParameters, n), carla.WeatherParameters))


def describe(w):
    return (f"cloudiness={w.cloudiness:.0f} precipitation={w.precipitation:.0f} "
            f"puddles={w.precipitation_deposits:.0f} fog={w.fog_density:.0f} "
            f"sun_altitude={w.sun_altitude_angle:.0f}")


def main():
    args = [a for a in sys.argv[1:] if a != "--list"]
    if "--list" in sys.argv[1:]:
        print("\n".join(presets()))
        return
    name = args[0] if args else "ClearNoon"
    if name not in presets():
        sys.exit(f"unknown preset {name!r}; --list to see them")

    client = carla.Client("localhost", 2000)
    client.set_timeout(30.0)
    world = client.get_world()

    before = world.get_weather()
    world.set_weather(getattr(carla.WeatherParameters, name))
    after = world.get_weather()
    print(f"before: {describe(before)}")
    print(f"after:  {describe(after)}   [{name}]")


if __name__ == "__main__":
    main()
