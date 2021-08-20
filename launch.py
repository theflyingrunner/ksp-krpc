import math
import time
import krpc
import launch_parameters
import pid

def telemetry(conn, vessel, srf_frame):
    """
    Function sets up streams for telemetry data from KSP
    :param conn: krpc server connection object
    :param vessel: active vessel
    :param srf_frame: surface reference frame
    :return: telemetry object
    """
    ut = conn.add_stream(getattr, conn.space_center, 'ut')
    altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
    apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')
    srf_speed = conn.add_stream(getattr, vessel.flight(srf_frame), 'speed')
    dynamic_pressure = conn.add_stream(getattr, vessel.flight(srf_frame), 'dynamic_pressure')
    stage_resources = vessel.resources_in_decouple_stage(stage=vessel.control.current_stage - 1, cumulative=False)
    return object

def main():
    """
    Main function
    :return: N/A
    """
    conn = krpc.connect(name='launch')
    vessel = conn.space_center.active_vessel
    srf_frame = vessel.orbit.body.reference_frame

    launch_params = launch_parameters.LaunchParameters()

    telemetry_streams = telemetry(conn, vessel, srf_frame)
    ascent(conn, launch_params, telemetry_streams)

def ascent(connection, lp, ts):
    """

    :param connection:
    :param lp:
    :param ts:
    """
    conn = connection
    launch_params = lp
    telemetry_streams = ts
    vessel = connection.space_center.active_vessel
    srf_frame = vessel.orbit.body.reference_frame
    # =====================
    # Countdown
    # =====================

    text_panel = panel_setup()
    panel_write(text_panel, "Countdown")
    time.sleep(1)
    panel_write(text_panel, "3")
    time.sleep(1)
    panel_write(text_panel, "2")
    time.sleep(1)
    panel_write(text_panel, "1")
    time.sleep(1)

    # =====================
    # Activate the first stage
    # =====================

    panel_write(text_panel, "Lift off!")
    vessel.control.throttle = 1.0
    vessel.auto_pilot.engage()
    vessel.control.activate_next_stage()
    vessel.auto_pilot.target_pitch_and_heading(90, 90) # change arguments to launch params

    # =====================
    # Gravity turn loop
    # =====================
    if (launch_params.turn_start_altitude <
            telemetry_streams.altitude() < launch_params.turn_end_altitude):
        gravity_turn(conn, telemetry_streams, launch_params)


def gravity_turn(connection, ts, lp):
    conn = connection
    telemetry_streams = ts
    launch_params = lp
    vessel = conn.space_center.active_vessel
    text_panel = panel_setup()
    panel_write(text_panel, "Executing gravity turn")
    turn_range_covered = ((telemetry_streams.altitude() -
                           launch_params.turn_start_altitude) /
                          (launch_params.turn_end_altitude -
                           launch_params.turn_start_altitude))
    new_turn_angle = turn_range_covered * 90
    vessel.auto_pilot.target_pitch(90 - new_turn_angle)

def autostage():
    

def panel_setup():
    """
    Function creates a new panel to left of screen
    then enables ability to add text
    :return text object
    """
    canvas = conn.ui.stock_canvas
    screen_size = canvas.rect_transform.size
    panel = canvas.add_panel()
    # position panel to left of screen
    rect = panel.rect_transform
    rect.size = (300,200)
    rect.position = (210-(screen_size[0]/2), 0)
    # create text block on panel
    text = panel.add_text("")
    text.rect_transform.position = (-10, 20)
    text.color = (1,1,1)
    text.size = 13
    return text

def panel_write(text_panel, message):
    """
    Function uses given text object to print message to panel
    :rtype: object
    :param text_panel text panel object from panel_setup
    :param message message to be printed on panel
    :return N/A
    """

    text_panel.content = message