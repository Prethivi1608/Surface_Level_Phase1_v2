from sensors import AHRS_Sensor, GPS
import threading
import params
from controllers import Teensy
import time
from telemetry import RF
import ast

GCS_TIMEOUT = 3.0   # seconds — if no valid GCS packet received, link is considered lost

class Middleware():
    def __init__(self):
        self.gps = GPS(params.gps_)
        self.ahrs = AHRS_Sensor(params.ahrs_)
        self.teensy = Teensy(params.teensy_)
        self.rf = RF(params.rf_)

        # --- AUV → GCS telemetry list (18 elements) ---
        self.auv_data_ = []

        # --- GCS → AUV command list (11 elements) ---
        self.gcs_data_ = []

        # GCS command fields (with safe defaults)
        self.operating_mode       = 0
        self.start_run_mode       = 0
        self.no_wp                = 0
        self.waypoints            = []
        self.linear_pid_values    = []
        self.angular_pid_values   = []
        self.manual_fwd_thruster  = 1500
        self.manual_lat_thruster  = 1500
        self.manual_vert_thruster = 1500
        self.light_command        = 0
        self.camera_command       = 0

        self.data_ready = False

        # --- RF Link health ---
        self.gcs_link_alive = False   # True only when GCS packets arrive regularly
        self.last_gcs_time  = None    # timestamp of last valid GCS packet

        # Start all background threads
        threading.Thread(target=self.gps.read_data,                  daemon=True).start()
        threading.Thread(target=self.ahrs.process_data,              daemon=True).start()
        threading.Thread(target=self.teensy.receieve_pressure_data,  daemon=True).start()
        threading.Thread(target=self.auv_data_update,                daemon=True).start()
        threading.Thread(target=self.send_data_rf,                   daemon=True).start()
        threading.Thread(target=self.recieve_data_rf,                daemon=True).start()
        threading.Thread(target=self.gcs_data_update,                daemon=True).start()
        threading.Thread(target=self._gcs_watchdog,                  daemon=True).start()

    # ------------------------------------------------------------------
    #  AUV → GCS  :  Pack sensor readings into auv_data_ every 50 ms
    # ------------------------------------------------------------------
    def auv_data_update(self):
        while True:
            self.auv_data_ = [
                self.ahrs.pitch,
                self.ahrs.roll,
                self.ahrs.heading,
                self.teensy.chamber_temperature,
                self.teensy.chamber_humidity,
                self.gps.latitude,
                self.gps.longitude,
                self.gps.no_satellites,
                0, 0, 0, 0, 0,
                0, 0, 0,
                0, 0
            ]
            # data_ready is True only when GPS has a fix AND sensors are fresh
            is_gps_fresh = (time.time() - self.gps.last_update) < 2.0
            is_ahrs_fresh = (time.time() - self.ahrs.last_update) < 2.0

            if None in self.auv_data_[5:7] or not is_gps_fresh or not is_ahrs_fresh:
                self.data_ready = False
            else:
                self.data_ready = True

            time.sleep(0.05)

    # ------------------------------------------------------------------
    #  TX thread: sends AUV telemetry to GCS at ~10 Hz.
    #  UART TX/RX are full-duplex – safe to send while RX thread reads.
    # ------------------------------------------------------------------
    def send_data_rf(self):
        while True:
            if self.auv_data_:          # Only send once the list is populated
                self.rf.send_data(self.auv_data_)
            time.sleep(0.1)             # 10 Hz – keeps RF link from saturating

    # ------------------------------------------------------------------
    #  RX thread: blocking read – waits here until a GCS packet arrives.
    #  Stores result in rf.rf_data (protected by _lock inside RF class).
    # ------------------------------------------------------------------
    def recieve_data_rf(self):
        while True:
            self.rf.recieve_data()      # Blocks until one full line arrives

    # ------------------------------------------------------------------
    #  GCS data parser: reads rf_data and unpacks the 11-element list
    # ------------------------------------------------------------------
    def gcs_data_update(self):
        print("GCS_Data Reception thread started")
        while True:
            s = self.rf.get_rf_data()   # Thread-safe read of latest RF string

            if not s:
                time.sleep(0.05)
                continue

            try:
                parsed = ast.literal_eval(s)
            except (ValueError, SyntaxError) as e:
                print(f"[GCS] Bad packet ignored: {e!r} | raw: {s!r}")
                time.sleep(0.05)
                continue

            if not isinstance(parsed, list) or len(parsed) < 11:
                print(f"[GCS] Packet too short ({len(parsed)} fields), skipping.")
                time.sleep(0.05)
                continue

            self.gcs_data_ = parsed

            # Unpack GCS → AUV commands
            self.operating_mode       = parsed[0]
            self.start_run_mode       = parsed[1]
            self.no_wp                = parsed[2]
            self.waypoints            = parsed[3]
            self.linear_pid_values    = parsed[4]
            self.angular_pid_values   = parsed[5]
            self.manual_fwd_thruster  = parsed[6]
            self.manual_lat_thruster  = parsed[7]
            self.manual_vert_thruster = parsed[8]
            self.light_command        = parsed[9]
            self.camera_command       = parsed[10]

            # ── Mark RF link as alive ─────────────────────────────────
            self.last_gcs_time  = time.time()
            self.gcs_link_alive = True

            time.sleep(0.01)    # faster GCS command polling (~100 Hz)

    # ------------------------------------------------------------------
    #  GCS Watchdog: detects RF link loss.
    #  If no valid packet has arrived in GCS_TIMEOUT seconds, sets
    #  gcs_link_alive = False so auv_.py can trigger a safe stop.
    # ------------------------------------------------------------------
    def _gcs_watchdog(self):
        while True:
            if self.last_gcs_time is not None:
                silence = time.time() - self.last_gcs_time
                if silence > GCS_TIMEOUT:
                    if self.gcs_link_alive:   # print only on the transition
                        print(f"[MW] ⚠ GCS link LOST — no packet for {silence:.1f}s. "
                              f"Thrusters will be stopped by AUV dispatcher.")
                    self.gcs_link_alive = False
            time.sleep(0.5)   # check at 2 Hz

