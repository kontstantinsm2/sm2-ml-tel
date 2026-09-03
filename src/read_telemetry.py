from pymavlink import mavutil

m = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
print("waiting for heartbeat...")
m.wait_heartbeat()
print("connected, sys", m.target_system)

while True:
    msg = m.recv_match(type=['ATTITUDE', 'RC_CHANNELS', 'GLOBAL_POSITION_INT'], blocking=True)
    t = msg.get_type()
    if t == 'ATTITUDE':
        print(f"roll={msg.roll:+.3f} pitch={msg.pitch:+.3f} yaw={msg.yaw:+.3f}")
    elif t == 'RC_CHANNELS':
        print(f"ch1={msg.chan1_raw} ch2={msg.chan2_raw} ch3={msg.chan3_raw} ch4={msg.chan4_raw}")
    else:
        print(f"alt={msg.relative_alt/1000:.1f} m")
