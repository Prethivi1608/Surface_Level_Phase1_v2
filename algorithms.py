import math
import numpy as np
import params
from communication import SerialComm

class PID():
    def __init__(self):
        self.earth_radius = params.earth_radius
        self.target_reached = False
        self.data_points = None
        self.auv_docked = False
        self.lora = SerialComm()
        self.angular_coeffs = params.pid_constants[0]
        self.linear_coeffs = params.pid_constants[1]


    #This function calculates the error in the measurements. e(t)=Target-current
    def calculate_error(self, current, target):
        error = target - current
        error = (error + 180) % 360 - 180
        return error
    
    #Stores the values in the list. 
    def store_to_list(self,value):
        values = []
        if len(values) > 10:
            values.pop(0)
        values.append(value)

        return values

    def set_gains(self, angular=None, linear=None):
        """Update PID gains for angular and linear controllers.
        Parameters may be None or a list of three values [kp, kd, ki]."""
        if angular is not None and len(angular) == 3:
            self.angular_coeffs = angular
        if linear is not None and len(linear) == 3:
            self.linear_coeffs = linear
    #coefficients = [kp,kd,ki]
    def calculate_pid(self,coefficients,error): #calculate u_t
        errors = self.store_to_list(error)
        
        if len(errors) > 0:
            u_t = (coefficients[0]*error)+(coefficients[2]*sum(errors))+(coefficients[1]*(errors[len(errors)-2]-errors[len(errors)-1]))
        else:
            u_t = (coefficients[0]*error)+(coefficients[2]*sum(errors))

        return u_t
    

    def calculate_error_distance(self,current,target):
        phi1 = math.radians(current[0])
        phi2 = math.radians(target[0])
        lambda1 = math.radians(current[1])
        lambda2 = math.radians(target[1])    

        distance = (2*(self.earth_radius))*(np.arcsin(math.sqrt(((((math.sin((phi2-phi1)/2))**2))+((math.cos(phi1)))*(math.cos(phi2))*((math.sin((lambda2-lambda1)/2))**2)))))
        angle = math.degrees(math.atan2(((math.sin(lambda2-lambda1))*(math.cos(phi2))),((math.cos(phi1))*(math.sin(phi2)))-((math.sin(phi1))*(math.cos(phi2))*(math.cos(lambda2-lambda1)))))
        normalised_angle = (angle + 180) % 360 - 180

        return distance, normalised_angle
    

    #Limits the error value to motor pwm
    def limit_pwm(self,min_val,max_val,control_input):
        pwm_lim = max(min_val,min(max_val,control_input))
        
        return pwm_lim


    #checks the movement and status of the vehicle - basic movement
    def move_to_target(self,coefficients,coordinates,limits,current_angle):
        error_distance, target_angle = self.calculate_error_distance(coordinates[0],coordinates[1]) #error distance calculations
        print("target:",target_angle)
        direction = 'LEFT'

        error_angle = self.calculate_error(current_angle,target_angle) #error angle calculations
        print("current angle:",current_angle)
        print("error angle:",error_angle)
        
        orient_u_t = self.calculate_pid(coefficients[0],error_angle) #angular_u_t
        limit_orient_u_t = self.limit_pwm(limits[0][0],limits[0][1],orient_u_t)

        dist_u_t = self.calculate_pid(coefficients[1],error_distance) #linear u_t
        limit_dist_u_t = self.limit_pwm(limits[1][0],limits[1][1],dist_u_t)

        #return limit_dist_u_t, orient_u_t
    
        if abs(error_angle)>params.angle_threshold:
            if error_angle < 0:
                direction = "LEFT"
                return 0, limit_orient_u_t
            else:
                direction = "RIGHT"
                return 0, -(limit_orient_u_t)
         
        elif (error_distance>params.distance_threshold):
            direction = "FORWARD"
            return limit_dist_u_t, 0
        
        else:   
            direction = "STOP"
            self.target_reached = True
            dist_u_t = 0
            return 0, 0
        


    def AUV_move_to_target(self,coefficients,coordinates,limits,current_angle):
        error_distance, target_angle = self.calculate_error_distance(coordinates[0],coordinates[1]) #error distance calculations

        error_angle = self.calculate_error(current_angle,target_angle) #error angle calculations
        # print(error_angle)
        
        orient_u_t = self.calculate_pid(coefficients[0],error_angle) #angular_u_t
        limit_orient_u_t = self.limit_pwm(limits[0][0],limits[0][1],orient_u_t)
        

        dist_u_t = self.calculate_pid(coefficients[1],error_distance) #linear u_t
        limit_dist_u_t = self.limit_pwm(limits[1][0],limits[1][1],dist_u_t)

        if error_distance < params.distance_threshold:
            self.target_reached = True
            return 0, 0
        
        else:
            return limit_dist_u_t, limit_orient_u_t
        
    # def dock_auv(self,current_angle):

    #     error_angle = params.dock_target_angle - current_angle
    #     if abs(error_angle)>params.angle_threshold:
    #         if error_angle < 0:
    #             return [limit_orient_u_t,direction]
    #         else:
    #             direction = "RIGHT"
    #             return [limit_orient_u_t,direction]
         
    #     elif (error_distance>params.distance_threshold):
    #         direction = "FORWARD"
    #         return [float(limit_dist_u_t),direction]
        
    #     else:   
    #         direction = "STOP"
    #         self.target_reached = True
    #         dist_u_t = 0
    #         return [dist_u_t,direction]
        
    