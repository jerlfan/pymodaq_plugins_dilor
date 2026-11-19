"""
Simulated mechanics of the Dilor Z40 grating drive: a stepper motor turning a screw, whose position is read by a
HEIDENHAIN optical ruler.

The class has the same public methods as the stepper motor part of the Arduino class of pymodaq_plugins_arduino, so
that the Dilor actuator can use one or the other. It moves the position of a ruler object given to the constructor,
such as IK220Mock of pymodaq_plugins_heidenhain.

Default values measured on the Z40: one step moves the ruler by about 0.49 µm, positive steps decrease the reading,
and the backlash is about 1.3 µm.
"""


class Z40MechanicsMock:

    def __init__(self, com_port=None, ruler=None, step_mm=0.49e-3, direction=-1, backlash_mm=1.3e-3):
        self.ruler = ruler
        self.step_mm = step_mm
        self.direction = direction
        self.backlash_mm = backlash_mm
        self.stepper_motor = None
        self.position = 0

    def initialize_stepper_motor(self, pulse_pin, direction_pin, enable_pin=7):
        """ Initialize the simulated stepper motor and set its position to 0 """
        self.stepper_motor = 0  # any value but None marks the motor as initialized, as in the Arduino class
        self.position = 0

    def move_stepper_to_position(self, position: float, max_speed=200, acceleration=400):
        """ Start moving the simulated stepper motor to the specified position, without waiting for the end """
        pass

    def stop_stepper(self):
        """ Stop the simulated stepper motor immediately """
        pass

    def get_stepper_position(self):
        """ Return the current position of the simulated stepper motor, in steps """
        return self.position

    def shutdown(self):
        """ Stop the simulated stepper motor, as the Arduino class closes its connection when PyMoDAQ closes """
        pass
