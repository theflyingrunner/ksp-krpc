"""
Launch code intended for ascent into orbit. This script should work with any
rocket that has the required delta v to establish orbit. The simulation is intended
for use on Kerbin in Kerbal Space Program, but the launch parameters can be modified
for use on a simulation of Earth. This script utilizes a PID controller to control
thrust by limiting the maximum allowable dynamic pressure. Note that the script uses
the kRPC library and the launch_parameters, telemetry, and pid Python files. Please
use these in your own versions of the script.
"""

import math
import time
import krpc
import launch_parameters
import telemetry
import pid

def main():
    """
    Main function
    :return: N/A
    """
    conn = krpc.connect(name='launch')
    vessel = conn.space_center.active_vessel
    srf_frame = vessel.orbit.body.reference_frame

    launch_params = launch_parameters.LaunchParameters()
    telemetry_streams = telemetry.Telemetry(conn, vessel, srf_frame)

    ascent(conn, launch_params, telemetry_streams)

def ascent(connection, lp, ts):
    """
    Function serves as main ascent loop. Function executes gravity turn, orbit
    circularization, and node execution. In addition, the function also sets up the
    PID controller which controls the thrust by limiting maximum allowable dynamic
    pressure
    :param connection: krpc connection
    :param lp: launch parameters
    :param ts: telemetry streams
    """
    conn = connection
    launch_params = lp
    telemetry_streams = ts
    vessel = connection.space_center.active_vessel
    srf_frame = vessel.orbit.body.reference_frame
    # PID controller setup
    thrust_pid = pid.PID(0.001, 0.0001, 0.01)
    thrust_pid.setClamp(launch_params.q_allow)
    thrust_pid.setTarget(launch_params.q_allow)

    # =====================
    # Countdown
    # =====================
    text_panel = panel_setup(conn)
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
    time.sleep(0.5)
    vessel.auto_pilot.engage()
    vessel.auto_pilot.target_pitch_and_heading(90, 90)  # change arguments to launch params
    vessel.control.activate_next_stage()

    # =====================
    # Gravity turn loop
    # =====================
    while telemetry_streams.altitude() <= launch_params.turn_start_altitude:
        vessel.control.throttle = 1.0

    while (launch_params.turn_start_altitude <
            telemetry_streams.altitude() < launch_params.turn_end_altitude):
        gravity_turn(conn, telemetry_streams, launch_params, text_panel)
        autostage(conn, telemetry_streams, launch_params, text_panel)
        vessel.control.throttle = thrust_pid.update(telemetry_streams.dynamic_pressure())
        time.sleep(0.1)

    panel_write(text_panel, "Target apoapsis reached")
    vessel.control.throttle = 0.0

    # =====================
    # Coasting out of atmosphere
    # =====================
    panel_write(text_panel, "Coasting out of the atmosphere")
    # Atmosphere of Kerbin extends till ~70000 meters altitude
    while telemetry_streams.altitude() < 70000:
        pass

    # =====================
    # Circularization
    # =====================
    circularization(conn, telemetry_streams, text_panel)
    execute_node(conn, telemetry_streams, launch_params, text_panel)

    # =====================
    # Final ascent closing procedures
    # =====================
    vessel.auto_pilot.disengage()
    panel_write(text_panel, "Target orbit reached")

def gravity_turn(connection, ts, lp, tp):
    """
    Function executes gravity turn by pitching 90° eastward based on the progress
    of turn range covered. The turn range can be modified by changing the turn
    start and end altitudes in launch_parameters
    :param connection: krpc connection
    :param ts: telemetry streams
    :param lp: launch parameters
    :param tp: text panel
    :return: N/A
    """
    conn = connection
    telemetry_streams = ts
    launch_params = lp
    text_panel = tp
    vessel = conn.space_center.active_vessel

    panel_write(text_panel, "Executing gravity turn")

    turn_range_covered = ((telemetry_streams.altitude() -
                           launch_params.turn_start_altitude) /
                          (launch_params.turn_end_altitude -
                           launch_params.turn_start_altitude))
    new_turn_angle = turn_range_covered * 90
    vessel.auto_pilot.target_pitch_and_heading(90 - new_turn_angle, 90)

def autostage(connection, ts, lp, tp):
    """
    Function checks if a stage has fuel and activates next stage. This function
    will do nothing if this is the last stage. In addition, if current is fuel-less
    (such as an in-between stage), it will activate next stage.
    :param connection: krpc connection
    :param ts: telemetry streams
    :param lp: launch parameters
    :param tp: text panel
    :return: N/A
    """
    conn = connection
    telemetry_streams = ts
    launch_params = lp
    vessel = conn.space_center.active_vessel
    text_panel = tp

    fuel_types = ('LiquidFuel', 'SolidFuel')
    resources = vessel.resources_in_decouple_stage(
        vessel.control.current_stage - 1, cumulative=False)
    no_fuel_stage = True

    if vessel.control.current_stage <= 0:
        return
    for fuel in fuel_types:
        if resources.max(fuel) > 0 and resources.amount(fuel) == 0:
            vessel.control.activate_next_stage()
            panel_write(text_panel, "Stage separation")
            return
        if resources.has_resource(fuel):
            no_fuel_stage = False
    if no_fuel_stage:
        vessel.control.activate_next_stage()
        panel_write(text_panel, "Stage separation")

def circularization(connection, ts, tp):
    """
    Function calculates required delta v for orbit circularization using the
    vis-viva equation. The function then adds a node in KSP for subsequent
    burn time calculation
    :param connection: krpc connection
    :param ts: telemetry streams
    :param tp: text panel
    :return: N/A
    """
    conn = connection
    telemetry_streams = ts
    vessel = conn.space_center.active_vessel
    text_panel = tp
    panel_write(text_panel, "Planning circularization burn")

    mu = vessel.orbit.body.gravitational_parameter
    ap = vessel.orbit.apoapsis
    a1 = vessel.orbit.semi_major_axis
    a2 = ap
    v1 = math.sqrt(mu * ((2. / ap) - (1. / a1)))
    v2 = math.sqrt(mu * ((2. / ap) - (1. / a2)))
    delta_v = v2 - v1
    vessel.control.add_node(
        telemetry_streams.ut() + vessel.orbit.time_to_apoapsis, prograde=delta_v)

def execute_node(connection, ts, lp, tp):
    """
    Function executes circularization node by
    1. calculating burn time using the rocket equation
    2. orienting vehicle to node vector
    3. executing node such that apoapsis is achieved at the midpoint of burn time
    :param connection: krpc connection
    :param ts: telemetry streams
    :param lp: launch parameters
    :param tp: text panel
    :return: N/A
    """
    conn = connection
    telemetry_streams = ts
    text_panel = tp
    launch_params = lp
    vessel = conn.space_center.active_vessel
    node = vessel.control.nodes[0]
    delta_v = node.delta_v

    # Calculate burn time: rocket equation
    force = vessel.available_thrust
    isp = vessel.specific_impulse * 9.81
    m0 = vessel.mass
    m1 = m0 / math.exp(delta_v / isp)
    flow_rate = force / isp
    burn_time = (m0 - m1) / flow_rate

    # Orient vessel
    vessel.control.rcs = True
    vessel.auto_pilot.reference_frame = node.reference_frame
    vessel.auto_pilot.target_direction = (0, 1, 0)
    vessel.auto_pilot.wait()

    # Wait until burn
    panel_write(text_panel, "Waiting until circularization burn")
    burn_ut = telemetry_streams.ut() + vessel.orbit.time_to_apoapsis - (burn_time / 2.)
    lead_time = 5

    # Execute burn
    while telemetry_streams.time_to_apoapsis() - (burn_time / 2.) > 0:
        autostage(conn, telemetry_streams, launch_params, text_panel)
        pass
    panel_write(text_panel, "Executing node")
    vessel.control.throttle = 1.0
    remaining_burn = conn.add_stream(node.remaining_burn_vector, node.reference_frame)
    # time.sleep(burn_time)
    while node.remaining_delta_v > 10:
        autostage(conn, telemetry_streams, launch_params, text_panel)
        time.sleep(0.1)
    vessel.control.throttle = 0.25
    while node.remaining_delta_v > 1:
        pass
    # Disengage throttle, rcs and remove node
    vessel.control.throttle = 0.0
    node.remove()
    vessel.control.rcs = False

def panel_setup(connection):
    """
    Function creates a new panel to left of screen
    then enables ability to add text
    :return text object
    """
    conn = connection
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

# Executes main function
if __name__ == "__main__":
    main()