from pymavlink import mavutil

m = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
print("waiting for heartbeat...")
m.wait_heartbeat()
print("connected, sys", m.target_system)

while True:
    msg = m.recv_match(type=['ATTITUDE', 'RC_CHANNELS'], blocking=True)
    if msg.get_type() == 'ATTITUDE':
        print(f"roll={msg.roll:.2f} pitch={msg.pitch:.2f} yaw={msg.yaw:.2f}")
    else:
        print(f"ch1={msg.chan1_raw} ch2={msg.chan2_raw} ch3={msg.chan3_raw} ch4={msg.chan4_raw}")
