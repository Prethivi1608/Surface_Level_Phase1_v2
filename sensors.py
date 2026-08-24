from communication import SerialComm
from controllers import Teensy
from telemetry import RF
from ultralytics import YOLO
import params
import threading
import time


class SensorData():
    def __init__(self):
        self.gps = GPS(params.gps_)
        self.ahrs = AHRS_Sensor(params.ahrs_)
        self.teensy = Teensy(params.teensy_)
        self.rf = RF(params.rf_)

        threading.Thread(target=self.gps.read_data,daemon=True).start()
        threading.Thread(target=self.ahrs.process_data,daemon=True).start()
        threading.Thread(target=self.data_update,daemon=True).start()
        threading.Thread(target=self.teensy.receieve_pressure_data,daemon=True).start()
        threading.Thread(target=self.send_data_rf,daemon=True).start()

    def data_update(self):
        while True:
            self.data_ = [self.ahrs.pitch,
                          self.ahrs.roll,
                          self.ahrs.heading,
                          self.teensy.chamber_temperature,
                          self.teensy.chamber_humidity,
                          self.gps.latitude,
                          self.gps.longitude,
                          self.gps.no_satellites,
                          0,0,0,0,0,
                          0,0,0,
                          0,0
                          ]
            if None in self.data_[6:8]:  # Check if latitude or longitude is None
                self.data_ready = False
            else:
                self.data_ready = True 
            
            time.sleep(0.05)

    def send_data_rf(self):
        while True:
            self.rf.send_data(self.data_)
    


class AHRS_Sensor():
    def __init__(self,ahrs_com):
        self.ahrs_com = ahrs_com
        self.ser = SerialComm()
        self.ser.connect_to(self.ahrs_com)
        self.pitch = None
        self.roll = None
        self.heading = None
        self.last_update = 0

    def process_data(self):
        while True:
            data = self.ser.recieve_data()
            if not data:
                continue

            data_str = data.strip()
            if not data_str.startswith('$PRDID'):
                continue

            try:
                parts = data_str.split(',')
                if len(parts) < 4:
                    print(f"Invalid Packet: {data_str!r}")
                    continue

                self.pitch = float(parts[1])
                self.roll = float(parts[2])
                self.heading = float(parts[3])
                self.last_update = time.time()  # Mark data as fresh

            except ValueError as e:
                print(f"Invalid AHRS data: {data_str!r}")
                print(f"Error: {e}")
                continue


class GPS():
    def __init__(self,gps_com):
        self.ser = SerialComm()
        self.ser.connect_to(gps_com)
        self.latitude = None
        self.longitude = None   
        self.no_satellites = None
        self.last_update = 0

    def read_data(self):
        while True:
            data_str = self.ser.recieve_data()
            if not data_str.startswith('$GNGGA'):
                continue    # skip non-position sentences, keep last known lat/lon

            try:
                parts = data_str.split(',')
                if len(parts) < 8 or parts[2] == '' or parts[4] == '':
                    continue    # incomplete fix sentence, skip

                latitude_raw  = float(parts[2])
                degrees_lat   = int(latitude_raw / 100)
                minutes_lat   = (latitude_raw - degrees_lat * 100) / 60
                self.latitude = round(degrees_lat + minutes_lat, 5)

                longitude_raw  = float(parts[4])
                degrees_long   = int(longitude_raw / 100)
                minutes_long   = (longitude_raw - degrees_long * 100) / 60
                self.longitude = round(degrees_long + minutes_long, 5)

                self.no_satellites = float(parts[7])
                self.last_update = time.time()  # Mark data as fresh

            except (ValueError, IndexError) as e:
                print(f"[GPS] Bad NMEA sentence ignored: {e!r} | raw: {data_str!r}")
                # Keep previous lat/lon — don't reset to None on a single bad packet

                


# class MagneticSensor():
#     def __init__(self,serial_port,baud_rate):
#         self.serial_port = serial_port
#         self.baud_rate = baud_rate
#         self.ser = serial.Serial(self.serial_port,self.baud_rate)

#         self.roll, self.pitch, self.heading = self.read_data()
        

#     def read_data(self):   
#         line = self.ser.readline().decode().rstrip()
#         data= line.split(',')

#         if len(data) == 3:
#             x = float(data[0])
#             y = float(data[1])
#             z = float(data[2])

#             roll = (math.atan2(z,y) * -1*180/math.pi)
#             pitch = (math.atan2(z,x) * 180/math.pi)
#             heading = math.degrees(math.atan2(y,x))

#             return roll, pitch, heading


    