from communication import SerialComm
import params

class Thruster():
    def __init__(self):
        self.teensy_ = SerialComm()

    def run(self,fwd_pwm_value,lat_pwm_value):
        self.teensy_.connect_to(params.teensy_)
        #packets = fwd_pwm_value.to_bytes(2,"little") + lat_pwm_value.to_bytes(2,"little") + params.T_500_BaseSpeed.to_bytes(2,"little") + params.Light_OFF.to_bytes(2,"little")+ params.Camera_OFF.to_bytes(2,"little")
        value = f"{fwd_pwm_value},{lat_pwm_value},1500,0,0"
        self.teensy_.send_data(value)
        # self.teensy_.close_port()

    def stop(self):
        self.teensy_.connect_to(params.teensy_)
        self.teensy_.send_data('1500')
        self.teensy_.close_port()