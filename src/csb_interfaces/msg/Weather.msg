# CARLA weather, the subset of carla.WeatherParameters a user sets
# (https://carla.readthedocs.io/en/0.9.16/python_api/#carla.WeatherParameters).
# Fields not listed here (fog_falloff, scattering, dust_storm) keep CARLA's current values
# when explicit parameters are applied.

float32 cloudiness              # 0-100
float32 precipitation           # rain, 0-100
float32 precipitation_deposits  # puddles, 0-100
float32 wind_intensity          # 0-100
float32 fog_density             # 0-100
float32 fog_distance            # metres where fog starts
float32 wetness                 # 0-100
float32 sun_azimuth_angle       # degrees, 0-360
float32 sun_altitude_angle      # degrees, -90 (midnight) to 90 (midday)
