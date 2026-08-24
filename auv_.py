"""
auv_.py  –  Trinetra TS100 AUV  |  Startup entry point
===========================================================
Run this file at boot.  It:
  1. Boots the Middleware (sensors, RF TX/RX, teensy) in the background.
  2. Continuously reads GCS commands from the Middleware.
  3. Dispatches to Manual or Automatic mode based on operating_mode.

GCS → AUV command list  (11 elements):
  [0]  operating_mode       : 0 = Manual, 1 = Automatic, 2 = Config
  [1]  start_run_mode       : 0 = Stop,   1 = Run
  [2]  no_wp                : number of waypoints
  [3]  waypoints            : list of [lat, lon] pairs
  [4]  linear_pid_values    : [kp, kd, ki]
  [5]  angular_pid_values   : [kp, kd, ki]
  [6]  manual_fwd_thruster  : PWM  (1000–2000, neutral = 1500)
  [7]  manual_lat_thruster  : PWM
  [8]  manual_vert_thruster : PWM
  [9]  light_command        : 0 = OFF, 1 = ON
  [10] camera_command       : 0 = OFF, 1 = ON
"""

import time
import params
from middleware import Middleware
from algorithms import PID
from controllers import Teensy


class AUV():
    def __init__(self):
        # ----------------------------------------------------------------
        #  Middleware is the SINGLE owner of all hardware (GPS, AHRS,
        #  Teensy, RF).  Do NOT open any of those ports again here.
        # ----------------------------------------------------------------
        self.mw = Middleware()      # boots all sensor + RF threads
        self.pid = PID()            # one PID instance for auto mode

        print("[AUV] Middleware initialised. Waiting for GCS link...")

    # ------------------------------------------------------------------
    #  Main dispatcher – called continuously from main()
    # ------------------------------------------------------------------
    def run(self):
        """
        Reads the latest GCS commands and dispatches to the correct mode.
        Returns immediately each cycle so mode changes are picked up fast.
        """
        # ── GCS Link check — MUST be first ───────────────────────────
        if not self.mw.gcs_link_alive:
            self.vehicle_stop()
            if self.mw.last_gcs_time is None:
                print("[AUV] Waiting for first GCS packet...")
            else:
                print("[AUV] GCS link lost. Thrusters held neutral. Waiting to reconnect...")
            time.sleep(0.5)
            return

        mode     = self.mw.operating_mode    # 0=Manual, 1=Auto, 2=Config
        running  = self.mw.start_run_mode    # 0=Stop,   1=Run

        # ── STOP command overrides everything ────────────────────────
        if running == 0:
            self.vehicle_stop()
            time.sleep(0.05)
            return

        # ── Manual Mode ──────────────────────────────────────────────
        if mode == 0:
            self.manual_mode()

        # ── Automatic Mode ───────────────────────────────────────────
        elif mode == 1:
            if not self.mw.data_ready:
                print("[AUV] Waiting for GPS fix...")
                time.sleep(0.5)
                return
            self.auto_mode()    # runs the auto mission

        # ── Config Mode ───────────────────────────────────────────────
        elif mode == 2:
            self.config_mode()


    # ------------------------------------------------------------------
    #  Manual Mode  –  thrusters driven directly by GCS joystick values
    # ------------------------------------------------------------------
    def manual_mode(self):
        fwd  = self.mw.manual_fwd_thruster    # already int from middleware
        lat  = self.mw.manual_lat_thruster
        vert = self.mw.manual_vert_thruster

        print(f"[MANUAL] Fwd={fwd}  Lat={lat}  Vert={vert}")
        self.mw.teensy.thruster_run(fwd, lat)

    # ------------------------------------------------------------------
    #  Config Mode  –  Set PID gains, control lights & camera.
    #                  No navigation. Thrusters stay at neutral (1500).
    #
    #  GCS sends these fields every packet:
    #    [4] linear_pid_values   = [kp, kd, ki]
    #    [5] angular_pid_values  = [kp, kd, ki]
    #    [9] light_command       = 0 (OFF) | 1 (ON)
    #    [10] camera_command     = 0 (OFF) | 1 (ON)
    # ------------------------------------------------------------------
    def config_mode(self):
        teensy = self.mw.teensy

        # ── Apply PID gains sent from GCS ────────────────────────────
        lin  = self.mw.linear_pid_values
        ang  = self.mw.angular_pid_values

        if lin and len(lin) == 3:
            self.pid.set_gains(angular=None, linear=lin)
            print(f"[CONFIG] Linear  PID set → kp={lin[0]}  kd={lin[1]}  ki={lin[2]}")

        if ang and len(ang) == 3:
            self.pid.set_gains(angular=ang, linear=None)
            print(f"[CONFIG] Angular PID set → kp={ang[0]}  kd={ang[1]}  ki={ang[2]}")

        # ── Light control ─────────────────────────────────────────────
        if self.mw.light_command == 1:
            teensy.light_on()
            print("[CONFIG] Lights → ON")
        else:
            teensy.light_off()
            print("[CONFIG] Lights → OFF")

        # ── Camera control ────────────────────────────────────────────
        if self.mw.camera_command == 1:
            teensy.camera_on()
            print("[CONFIG] Camera → ON")
        else:
            teensy.camera_off()
            print("[CONFIG] Camera → OFF")

        # ── Thrusters stay neutral – no movement in config mode ───────
        teensy.thruster_run(1500, 1500)

        time.sleep(0.2)    # Config mode runs at 5 Hz – no need to spam

    # ------------------------------------------------------------------
    #  Automatic Mode  –  GPS waypoint following via PID
    # ------------------------------------------------------------------
    def auto_mode(self):
        profile = self.mw.waypoints

        if not profile:
            print("[AUTO] No waypoints received yet.")
            time.sleep(0.2)
            return

        # Update PID gains from GCS if provided
        self.pid.set_gains(angular=self.mw.angular_pid_values, linear=self.mw.linear_pid_values)

        # Iterate through all waypoints
        for i, destination in enumerate(profile):

            self.pid.target_reached = False


            while not self.pid.target_reached:

                # ── Abort if GCS sends Stop or mode change ──────────
                if self.mw.start_run_mode == 0 or self.mw.operating_mode != 1:
                    print("[AUTO] Mission aborted by GCS.")
                    self.vehicle_stop()
                    return

                # ── Read current state from Middleware ───────────────
                auv = self.mw.auv_data_
                if not auv or None in auv[5:7]:
                    time.sleep(0.05)
                    continue

                current_angle = ((auv[2] + 180) % 360 - 180) + 190   # heading normalised
                current_pos   = [auv[5], auv[6]]               # [lat, lon]

                params.coordinates_[0] = current_pos
                params.coordinates_[1] = destination
                print(f"[AUTO] WP {i+1}/{len(profile)}  →  {destination}  "
                      f"| Heading: {auv[2]:.1f}°")

                # ── PID ──────────────────────────────────────────────
                linear_u_t, angular_u_t = self.pid.move_to_target(
                    params.pid_constants,
                    params.coordinates_,
                    params.limits,
                    current_angle
                )

                fwd_pwm = int(params.T_500_BaseSpeed + linear_u_t)
                lat_pwm = int(params.T_500_BaseSpeed + angular_u_t)
                print(fwd_pwm,lat_pwm)

                self.mw.teensy.send_command(
                    fwd=fwd_pwm,
                    lat=lat_pwm,
                    vert=1500,
                    light=self.mw.light_command,
                    camera=self.mw.camera_command
                )

                time.sleep(0.05)    # ~20 Hz control loop

            print(f"[AUTO] Reached waypoint {i+1}: {destination}")

        print("[AUTO] Mission complete. All waypoints reached.")
        self.vehicle_stop()

    # Alias for backward compatibility
    auto_step = auto_mode

    # ------------------------------------------------------------------
    #  Stop all thrusters (neutral PWM)
    # ------------------------------------------------------------------
    def vehicle_stop(self):
        self.mw.teensy.thruster_run(1500, 1500)


# ======================================================================
#  Entry point
# ======================================================================
def main():
    auv = AUV()
    print("[AUV] System ready. Entering command dispatch loop.")
    while True:
        auv.run()
        time.sleep(0.02)    # 50 Hz outer loop


if __name__ == '__main__':
    main()

