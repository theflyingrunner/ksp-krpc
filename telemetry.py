class Telemetry(object):
    def __init__(self, conn, vessel, srf_frame):
        """
        Function sets up streams for telemetry data from KSP
        :param conn: krpc server connection object
        :param vessel: active vessel
        :param srf_frame: surface reference frame
        :return: telemetry object
        """
        self.ut = conn.add_stream(getattr, conn.space_center, 'ut')
        self.altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
        self.apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')
        self.srf_speed = conn.add_stream(getattr, vessel.flight(srf_frame), 'speed')
        self.dynamic_pressure = conn.add_stream(getattr, vessel.flight(srf_frame), 'dynamic_pressure')
        self.stage_resources = vessel.resources_in_decouple_stage(stage=vessel.control.current_stage - 1,
                                                                  cumulative=False)
        self.time_to_apoapsis = conn.add_stream(getattr, vessel.orbit, 'time_to_apoapsis')