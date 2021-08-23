"""
Launch Parameters
Class sets up various launch parameters. Parameters can be modified as
required for mission. In addition, more parameters can be added as necessary.
"""
class LaunchParameters:

    def __init__(self):
        # start altitude of gravity turn (m)
        self.turn_start_altitude = 250
        # end altitude of gravity turn (m)
        self.turn_end_altitude = 69000
        # target altitude of orbit (m)
        self.target_altitude = 100000
        # maximum allowable dynamic pressure (Pa)
        # a value of 15 kPa is selected as it is half that of most modern launchers
        self.q_allow = 15000

