#PID Threshold
angle_threshold = 40 #40 Degrees
distance_threshold = 3 #meters

#Earth Radius
earth_radius = 6371000 #earth radius

#pwm limits
min_linear_pwm = -100
max_linear_pwm = 100

min_angular_velocity = -100
max_angular_velocity = 100

limits = [[min_angular_velocity,max_angular_velocity],[min_linear_pwm,max_linear_pwm]]

dock_angle = 90
dock_turn_angle_speed = 30

#COMM and Baud Rate Details

gps_ = ['GPS','/dev/ttyACM1',115200]
ahrs_ = ['AHRS','/dev/ttyUSB0',115200]
teensy_ = ['Teensy','/dev/ttyACM0',115200]
rf_ = ['RF','/dev/ttyTHS1',57600]


#kp,kd,ki
linear_kp = 7
linear_kd = 0
linear_ki = 0

angular_kp = 2
angular_kd = 0
angular_ki = 0

pid_constants = [[angular_kp,angular_kd,angular_ki],[linear_kp,linear_kd,linear_ki]]


#Coordinates
destination_coordinates = [12.946428,80.212103]
init_coordinates = [12.946456,80.212140]

coordinates_ = [init_coordinates,destination_coordinates]


#Navigation_ Profile
square_profile = [[13.166873,80.251736],[13.167104,80.251834],[13.166847,80.251863]]

wp_profile = []


#T500- Thrusters
T_500_BaseSpeed = 1500

Arm_T500 = 10
Run_T500 = 11
DeArm_T500 = 12

Light_ON = 1
Light_OFF = 0
Camera_ON = 1
Camera_OFF = 0

