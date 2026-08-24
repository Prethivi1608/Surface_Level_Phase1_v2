from sensors import SensorData
from controllers import Teensy
from algorithms import PID
import params
import time


class Navigation():
    def __init__(self):
        self.pid = PID()
        self.thrusters = Teensy(params.teensy_)
        self.re_coor = None
        self.profile = []

        # Status Flags
        self.operating_mode = 0 #0 for Manual and 1 for Automatic
        self.run_stop = 0 #0 for stop and 1 for Run
        self.wp_completed = False

    def get_profile_data(self,profile):
        while True:
            self.profile = profile

    def select_operating_mode(self,operating_mode):
        while True:
            if operating_mode == '0' or 0:
                self.operating_mode = 0 # Manual Mode

            elif operating_mode == '1' or 1:
                self.operating_mode = 1 # Automatic Mode

    def start_run_mode(self,start_run_mode):
        while True:
            if start_run_mode == '0' or 0:
                self.run_stop = 0 # Stop

            elif start_run_mode == '1' or 1:
                self.run_stop = 1 # Run


    def manual_mode(self):
        fwd_thruster = int(self.telem_data[6])
        lat_thruster = int(self.telem_data[7])
        self.thrusters.thruster_run(int(fwd_thruster),int(lat_thruster))

    def automotic_mode(self,gps_data,ahrs_data):
        for i in range(len(self.profile)): #Loop through all the waypoints. 

            print("Total Waypoints:",len(self.profile))

            destination_coordinates = self.profile[i]
            # print(destination_coordinates)
            self.pid.target_reached = False
    
            while not self.pid.target_reached:
    
                # Get Current Heading
    
                current_angle = (ahrs_data[2] + 180) % 360 -180
    
                # Get Current coordinates and Destination coordinates
    
                coordinates = gps_data
                self.re_coor = coordinates[:2]
    
                # Store the current heading and Destination heading in the params file
    
                params.coordinates_[0] = self.re_coor
                params.coordinates_[1] = destination_coordinates
                print("Moving to the point:",i+1)
    
                # PID Control
    
                linear_u_t, angular_u_t = self.pid.move_to_target(params.pid_constants,params.coordinates_,params.limits,current_angle)    
    
                Fwd_Thruster = params.T_500_BaseSpeed + linear_u_t
                Lat_Thruster = params.T_500_BaseSpeed + angular_u_t
    
                # Run the Thrusters
                
                self.thrusters.thruster_run(int(Fwd_Thruster),int(Lat_Thruster))
            
            print("Moved to way point :",i+1)
    
        print("Mission completed.")

        self.wp_completed = True

        self.vehicle_stop()

    def vehicle_stop(self):
        self.thrusters.thruster_run(1500,1500)

