from sensors import SensorData
from algorithms import PID
from telemetry import RF
from communication import SerialComm
import params
from controllers import Teensy
import time,ast


#Objects Initialisation    

pid = PID()
sensor_data = SensorData()
thrusters = Teensy(params.teensy_)
rf = RF(params.rf_)
re_coor = None


coordinates = [sensor_data.data_[5],sensor_data.data_[6],sensor_data.data_[7]]  # Get GPS_Values

re_coor = coordinates[:2] #Get Lat and Long Values

while True:
    rf.recieve_data()

    s = rf.rf_data

    profile = ast.literal_eval(s)

    if not sensor_data.data_ready: # Wait for Data
        time.sleep(0.01)   # prevent CPU hogging
        continue

    for i in range(len(profile)): #Loop through all the waypoints. 
        destination_coordinates = profile[i]
        # print(destination_coordinates)
        pid.target_reached = False

        while not pid.target_reached:

            # Get Current Heading

            current_angle = ((sensor_data.data_[2] + 180) % 360 -180)

            # Get Current coordinates and Destination coordinates

            coordinates = [sensor_data.data_[5],sensor_data.data_[6],sensor_data.data_[7]]
            re_coor = coordinates[:2]

            # Store the current heading and Destination heading in the params file

            params.coordinates_[0] = re_coor
            params.coordinates_[1] = destination_coordinates
            print(params.coordinates_)

            # PID Control

            linear_u_t, angular_u_t = pid.move_to_target(params.pid_constants,params.coordinates_,params.limits,current_angle)    

            Fwd_Thruster = params.T_500_BaseSpeed + linear_u_t
            Lat_Thruster = params.T_500_BaseSpeed + angular_u_t

            # Run the Thrusters
            
            thrusters.thruster_run(int(Fwd_Thruster),int(Lat_Thruster))
        
        print("Moved to way point :",profile[i])

    print("Mission completed.")

        # Docking Algorithm


    time.sleep(1000)

