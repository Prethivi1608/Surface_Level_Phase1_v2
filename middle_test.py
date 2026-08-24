from middleware import Middleware
from controllers import Teensy
import params


auv_middleware = Middleware()
teensy = Teensy(params.teensy_)

while True:
    print(auv_middleware.manual_fwd_thruster)
    teensy.thruster_run(auv_middleware.manual_fwd_thruster,auv_middleware.manual_lat_thruster)
    