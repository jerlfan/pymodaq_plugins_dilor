"""
Grating drive of the Dilor spectrometers: a stepper motor positioned on the reading of a HEIDENHAIN optical ruler.

DilorDrive moves the motor until the ruler reads the target, within a tolerance. The motor is the Arduino class of
pymodaq_plugins_arduino or Z40MechanicsMock, the ruler is the IK220 class of pymodaq_plugins_heidenhain or IK220Mock.
The moves do not block: move_to starts a move, and update, called regularly (for instance by the polling of PyMoDAQ),
makes it progress and tells when it is finished.

To cancel the backlash, the target is always reached with motor steps of the same sign, the approach direction.
"""


class DilorDrive:

    def __init__(self, motor, ruler, step_mm=0.49e-3, direction=-1, approach=1, tolerance_mm=0.5e-3, max_speed=200):
        self.motor = motor
        self.ruler = ruler
        self.step_mm = step_mm  # displacement of the ruler for one motor step, in mm (absolute value)
        self.direction = direction  # sign of the ruler displacement for positive motor steps
        self.approach = approach  # sign of the motor steps used to reach the target
        self.tolerance_mm = tolerance_mm  # the move is finished when the ruler is within this distance of the target
        self.max_speed = max_speed  # motor speed, in steps per second
        self.target_mm = None

    def get_position(self) -> float:
        """ Return the position read on the ruler, in mm """
        return self.ruler.get_axis_position()

    def move_to(self, target_mm: float):
        """ Start a move to a target position of the ruler, in mm, without waiting for the end """
        pass

    def update(self) -> bool:
        """ Make the current move progress, and return True when it is finished """
        return True

    def stop(self):
        """ Stop the current move """
        pass
