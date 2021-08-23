"""
Telemetry
Class establishes streams telemetry data from KSP through kRPC
This method of streaming telemetry rather than calling the server is more efficient
and uses less processing power

To use it, simply create a telemetry object. All telemetry data can be accessed
through the telemetry object. If more telemetry variables are required, add them
to this file. For more information, please read the kRPC documentation.
"""
class Telemetry(object):
    def __init__(self, conn, vessel, srf_frame):
        """
        Function sets up streams for telemetry data from KSP
        :param conn: krpc server connection object
        :param vessel: active vessel
        :param srf_frame: surface reference frame
        """
        # universal time
        self.ut = conn.add_stream(getattr, conn.space_center, 'ut')
        # mean altitude of vessel (m)
        self.altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
        # apoapsis altitude (m)
        self.apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')
        # vessel speed with respect to the surface of planet (m/s)
        self.srf_speed = conn.add_stream(getattr, vessel.flight(srf_frame), 'speed')
        # dynamic pressure of vessel (Pa)
        self.dynamic_pressure = conn.add_stream(getattr, vessel.flight(srf_frame), 'dynamic_pressure')
        # current stage resources
        self.stage_resources = vessel.resources_in_decouple_stage(stage=vessel.control.current_stage - 1,
                                                                  cumulative=False)
        # time to apoapsis (s)
        self.time_to_apoapsis = conn.add_stream(getattr, vessel.orbit, 'time_to_apoapsis')