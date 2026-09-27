import carla, csv, math, sys
c = carla.Client('localhost', 2000); c.set_timeout(20)
w = c.get_world()
orig = w.get_settings()
print("map", w.get_map().name, "sync", orig.synchronous_mode, orig.fixed_delta_seconds)
bp = w.get_blueprint_library().find('vehicle.tesla.model3')
bp.set_attribute('role_name', 'steer_probe')
T0 = carla.Transform(carla.Location(320, 129.8, 0.3), carla.Rotation(yaw=180))
FL, FR = carla.VehicleWheelLocation.FL_Wheel, carla.VehicleWheelLocation.FR_Wheel
DT = 0.05
v = None
rows = []
def spd(): 
    vv = v.get_velocity(); return math.sqrt(vv.x**2+vv.y**2+vv.z**2)
def tick(n=1):
    for _ in range(n): w.tick()
def angles():
    return v.get_wheel_steer_angle(FL), v.get_wheel_steer_angle(FR)
def reset():
    v.set_target_velocity(carla.Vector3D()); v.set_target_angular_velocity(carla.Vector3D())
    v.set_transform(T0); v.apply_control(carla.VehicleControl(brake=1.0)); tick(40)
try:
    s = w.get_settings(); s.synchronous_mode = True; s.fixed_delta_seconds = DT; w.apply_settings(s)
    v = w.spawn_actor(bp, T0); tick(40)
    pc = v.get_physics_control()
    print("mass", pc.mass, "autobox", pc.use_gear_autobox)
    print("steering_curve", [(p.x, p.y) for p in pc.steering_curve])
    for i, wh in enumerate(pc.wheels):
        print("wheel", i, "max_steer", wh.max_steer_angle, "pos", wh.position)
    # wheelbase/track
    p = [wh.position for wh in pc.wheels]
    track = p[0].distance(p[1])/100; wb = ((p[0].x+p[1].x)/2 - (p[2].x+p[3].x)/2)
    wb = math.hypot((p[0].x+p[1].x-p[2].x-p[3].x)/2, (p[0].y+p[1].y-p[2].y-p[3].y)/2)/100
    print("track", track, "wheelbase", wb)
    # standstill
    for cmd in [0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0,-0.5,-1.0]:
        v.apply_control(carla.VehicleControl(steer=cmd, brake=1.0)); tick(20)
        fl, fr = angles()
        rows.append(dict(trial='standstill', target_kmh=0, speed_kmh=round(spd()*3.6,2), cmd=cmd, t=1.0, fl=fl, fr=fr))
        print('stand', cmd, fl, fr, spd())
    v.apply_control(carla.VehicleControl(steer=0, brake=1.0)); tick(20)
    # speed sweep
    for tgt in [10, 20, 30, 40, 60, 80]:
        for cmd in [0.2, 0.5]:
            reset()
            ok = False
            for k in range(600):
                sp = spd()*3.6
                if abs(sp - tgt) < 1.5 and k > 20: ok = True; break
                if v.get_location().x < 150: break
                thr = 1.0 if sp < tgt-3 else 0.2
                v.apply_control(carla.VehicleControl(throttle=thr, brake=0 if sp < tgt+1 else 0.3, steer=0)); tick()
            loc = v.get_location()
            if not ok: print("did not reach", tgt, spd()*3.6, loc); 
            for j in range(1, 11):  # 0.5 s
                sp = spd()*3.6
                thr = 0.5 if sp < tgt else 0.0
                v.apply_control(carla.VehicleControl(throttle=thr, steer=cmd)); tick()
                fl, fr = angles()
                rows.append(dict(trial='speed', target_kmh=tgt, speed_kmh=round(spd()*3.6,2), cmd=cmd, t=round(j*DT,2), fl=fl, fr=fr))
            print('speed', tgt, cmd, round(spd()*3.6,1), fl, fr, 'start x', round(loc.x,1))
            v.apply_control(carla.VehicleControl(steer=0, brake=1.0)); tick(10)
finally:
    if v is not None: v.destroy()
    s = w.get_settings(); s.synchronous_mode = False; s.fixed_delta_seconds = None; w.apply_settings(s)
    print("restored", w.get_settings().synchronous_mode, w.get_settings().fixed_delta_seconds)
with open('steer_raw.csv','w',newline='') as f:
    wr = csv.DictWriter(f, fieldnames=list(rows[0].keys())); wr.writeheader(); wr.writerows(rows)
