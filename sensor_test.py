from sensors import SensorData

sensors = SensorData()

while True:
    print(sensors.data_)


# from sensors import AHRS_Sensor,GPS
# import threading
# import params

# ahrs = AHRS_Sensor(params.ahrs_)
# gps = GPS(params.gps_)

# threading.Thread(target=gps.read_data,daemon=True).start()
# threading.Thread(target=ahrs.process_data,daemon=True).start()

# while True:

#     print(ahrs.heading)