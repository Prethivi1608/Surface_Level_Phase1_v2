from communication import SerialComm
import params
import time
import threading


class Teensy():
    def __init__(self,teensy_com):
        self.teensy = SerialComm()

        self.teensy.connect_to(teensy_com)
        self.rec_data = None
        self.chamber_temperature = None
        self.chamber_humidity = None

    def thruster_run(self, fwd_pwm_value=1500, lat_pwm_value=1500):
        value = f"{fwd_pwm_value},{lat_pwm_value},1500,0,0"
        self.teensy.send_data(value)

    def thrusterrun(self, fwd_pwm_value=1500, lat_pwm_value=1500):
        """Alias for thruster_run."""
        self.thruster_run(fwd_pwm_value, lat_pwm_value)

    def send_command(self, fwd=1500, lat=1500, vert=1500, light=0, camera=0):
        """
        Full-packet command to Teensy.
        Format: fwd, lat, vert, light(0/1), camera(0/1)
        Use this when you need to control lights/camera independently
        from thruster movement (e.g. in config mode).
        """
        value = f"{fwd},{lat},{vert},{light},{camera}"
        self.teensy.send_data(value)

    def light_on(self):
        value = "1500,1500,1500,1,0"
        self.teensy.send_data(value)

    def light_off(self):
        value = "1500,1500,1500,0,0"    # was incorrectly sending 1 (ON)
        self.teensy.send_data(value)

    def camera_on(self):
        value = "1500,1500,1500,0,1"
        self.teensy.send_data(value)

    def camera_off(self):
        value = "1500,1500,1500,0,0"
        self.teensy.send_data(value)

    def receieve_pressure_data(self):
        while True:
            self.rec_data = self.teensy.recieve_data()
            if not self.rec_data:
                continue
            data = self.rec_data.strip()
            
            try:
                parts = data.split(',')
                if len(parts) >= 2:
                    self.chamber_temperature = parts[0]
                    self.chamber_humidity = parts[1]
            except (ValueError, IndexError) as e:
                print(f"[Teensy] Bad pressure data ignored: {e!r} | raw: {data!r}")


def main():
    teensy = Teensy(params.teensy_)
    threading.Thread(target=teensy.receieve_pressure_data,daemon=True).start()


    while True:
        print(teensy.chamber_temperature)
        teensy.thruster_run(1530,1530)
        time.sleep(3)
        teensy.thruster_run(1500,1500)
        time.sleep(3)


if __name__ == "__main__":
    main()



    