import pid
import time
import krpc
"""
Hover test using PID controller
"""

target_altitude = 250

def main():
    # setup
    conn = krpc.connect()
    vessel = conn.space_center.active_vessel
    telem = vessel.flight(vessel.orbit.body.reference_frame)

    # PID setup
    pid_controller = pid.PID(0.125, 0.0025, 0.125)
    pid_controller.setClamp(20)
    pid_controller.setTarget(target_altitude)

    # lift off
    vessel.control.sas = True
    vessel.control.throttle = 1.0
    vessel.control.activate_next_stage()

    # test loop
    while True:
        pid_output = pid_controller.update(telem.mean_altitude)
        vessel.control.throttle = pid_output
        print('Input: altitude: {:03.2f} m     Output: throttle: {:03.2f}%     Target: {0.2f}% m'
              .format(telem.mean_altitude,
                      vessel.control.throttle*100,
                      target_altitude))
        time.sleep(.1)

if __name__ == '__main__':
    main()