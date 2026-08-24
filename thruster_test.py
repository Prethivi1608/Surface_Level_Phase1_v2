from middleware import Middleware
import time

middle = Middleware()

print("[TEST] Initialized Middleware. Waiting 2s for Teensy serial & ESC stabilization...")
time.sleep(2.0)

print("[TEST] Sending pulse to Teensy (fwd=1550, lat=1550)...")
middle.teensy.thruster_run(1550, 1550)
time.sleep(3.0)

print("[TEST] Sending neutral pulse to stop thrusters (1500, 1500)...")
middle.teensy.thruster_run(1500, 1500)
time.sleep(1.0)
print("[TEST] Test completed.")