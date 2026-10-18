"""Shared by the CARLA and Isaac step-response runs (roadmap 043): the command sequences.

A command is Autoware's: (acceleration m/s^2, velocity m/s, tire angle rad). Both runs call
`command(test, t, v)` once per 0.05 s frame with the frame's start time and the measured
forward speed, and record (t, v, yaw rate) after it.
"""
DT = 0.05
TESTS = {
    # hold 1 s; +1.5 m/s^2 for 5 s; -3 m/s^2 to a stop; hold
    "accel_brake": 11.0,
    # speed to 5 m/s (a P loop, the same on both), steer step 0.2 rad at t = 8 s for 4 s
    "steer_step": 12.0,
    # the same at 10 m/s, steer 0.1 rad for 3 s
    "steer_step_10": 11.0,
}
STEER_AT = 8.0


def speed_hold(target, v, gain=1.0, limit=1.5):
    return max(-limit, min(limit, gain * (target - v)))


def command(test, t, v):
    if test == "accel_brake":
        if t < 1.0:
            return 0.0, 0.0, 0.0
        if t < 6.0:
            return 1.5, 10.0, 0.0
        if v > 0.05 and t < 10.0:
            return -3.0, 0.0, 0.0
        return 0.0, 0.0, 0.0
    if test in ("steer_step", "steer_step_10"):
        target, steer, hold = (5.0, 0.2, 4.0) if test == "steer_step" else (10.0, 0.1, 3.0)
        a = speed_hold(target, v) if t >= 1.0 else 0.0
        vel = target if t >= 1.0 else 0.0
        return a, vel, (steer if STEER_AT <= t < STEER_AT + hold else 0.0)
    raise KeyError(test)
