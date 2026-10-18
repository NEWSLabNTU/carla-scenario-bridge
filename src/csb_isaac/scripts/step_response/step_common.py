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
    # AEB-like: to 15 m/s, then -8 m/s^2 to a stop
    "hard_brake": 16.0,
    # the tires' limit: -12 m/s^2 asked from 15 m/s, more than ~1 g can give
    "limit_brake": 16.0,
    # speed dependence: 15 m/s, steer 0.05 rad for 3 s (lateral ~3.7 m/s^2)
    "steer_step_15": 13.0,
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
    if test in ("hard_brake", "limit_brake"):
        if t < 1.0:
            return 0.0, 0.0, 0.0
        if t < 11.0:
            return speed_hold(15.0, v, limit=2.5), 15.0, 0.0
        if v > 0.05:
            return (-8.0 if test == "hard_brake" else -12.0), 0.0, 0.0
        return 0.0, 0.0, 0.0
    if test in ("steer_step", "steer_step_10", "steer_step_15"):
        target, steer, hold = {"steer_step": (5.0, 0.2, 4.0), "steer_step_10": (10.0, 0.1, 3.0),
                               "steer_step_15": (15.0, 0.05, 3.0)}[test]
        if test == "steer_step_15" and t < STEER_AT:
            a = speed_hold(target, v, limit=2.5) if t >= 1.0 else 0.0
            return a, (target if t >= 1.0 else 0.0), 0.0
        a = speed_hold(target, v) if t >= 1.0 else 0.0
        vel = target if t >= 1.0 else 0.0
        return a, vel, (steer if STEER_AT <= t < STEER_AT + hold else 0.0)
    raise KeyError(test)
