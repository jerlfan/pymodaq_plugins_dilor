"""
Grating drive of the Dilor spectrometers: a stepper motor positioned on the reading of a HEIDENHAIN optical ruler.

DilorDrive moves the motor until the ruler reads the target, within a tolerance. The motor is the Arduino class of
pymodaq_plugins_arduino or Z40MechanicsMock, the ruler is the IK220 class of pymodaq_plugins_heidenhain or IK220Mock.
The moves do not block: move_to starts a move, and update, called regularly (for instance by the polling of PyMoDAQ),
makes it progress and tells when it is finished.

To cancel the backlash, the target is always reached with motor steps of the same sign, the approach direction.
"""

import time


class DilorDrive:

    def __init__(self, motor, ruler, step_mm=0.49e-3, direction=-1, approach=1, tolerance_mm=0.5e-3, max_speed=200,
                 max_corrections=10, overshoot_steps=20):
        self.motor = motor
        self.ruler = ruler
        self.step_mm = step_mm  # displacement of the ruler for one motor step, in mm (absolute value)
        self.direction = direction  # sign of the ruler displacement for positive motor steps
        self.approach = approach  # sign of the motor steps used to reach the target
        self.tolerance_mm = tolerance_mm  # the move is finished when the ruler is within this distance of the target
        self.max_speed = max_speed  # motor speed, in steps per second
        self.max_corrections = max_corrections  # maximum number of corrections of a move
        self.overshoot_steps = overshoot_steps  # steps beyond a target that is behind
        self.target_mm = None
        self.status = 'idle'  # 'idle', 'moving', 'reached', 'stopped', 'resolution limit' or 'too many corrections'
        self.corrections = 0  # number of corrections of the current or last move
        self._motor_target = 0  # target of the motor, in steps

    def get_position(self) -> float:
        """ Return the position read on the ruler, in mm """
        return self.ruler.get_axis_position()

    def _steps_to(self, target_mm: float) -> int:
        """ Number of motor steps to go from the ruler reading to the target, truncated towards zero so that the
        target is never passed """
        error_mm = target_mm - self.get_position()
        return int(error_mm / (self.direction * self.step_mm))

    def _with_overshoot(self, steps: int) -> int:
        """ If the steps go against the approach direction, add the overshoot, so that the target is then reached
        with steps in the approach direction """
        if steps * self.approach < 0:
            return steps - self.approach * self.overshoot_steps
        return steps

    def _send_steps(self, steps: int):
        """ Start a move of the motor by a number of steps from its current position """
        self._motor_target = self.motor.get_stepper_position() + steps
        self.motor.move_stepper_to_position(self._motor_target, max_speed=self.max_speed)

    def take_up_backlash(self, timeout_s=5.):
        """ Do overshoot_steps steps in the approach direction and wait for the end, so that the screw then pushes the
        carriage on the side of the approach direction. To call once, after the initialization of the motor """
        self._send_steps(self.approach * self.overshoot_steps)
        end = time.perf_counter() + timeout_s
        while self.motor.get_stepper_position() != self._motor_target:
            if time.perf_counter() > end:
                self.motor.stop_stepper()
                raise TimeoutError('The motor did not finish taking up the backlash')
            time.sleep(0.01)

    def move_to(self, target_mm: float):
        """ Start a move to a target position of the ruler, in mm, without waiting for the end """
        self.target_mm = target_mm
        self.corrections = 0
        self.status = 'moving'
        self._send_steps(self._with_overshoot(self._steps_to(target_mm)))

    def update(self) -> bool:
        """ Make the current move progress, and return True when it is finished """
        if self.status != 'moving':
            return True
        if self.motor.get_stepper_position() != self._motor_target:
            return False  # the motor is still moving
        if abs(self.target_mm - self.get_position()) <= self.tolerance_mm:
            self.status = 'reached'
            return True
        steps = self._steps_to(self.target_mm)
        if steps == 0:
            self.status = 'resolution limit'
            return True
        if self.corrections >= self.max_corrections:
            self.status = 'too many corrections'
            return True
        self.corrections += 1
        self._send_steps(self._with_overshoot(steps))
        return False

    def stop(self):
        """ Stop the current move: the motor stops at once and no correction is sent """
        self.motor.stop_stepper()
        if self.status == 'moving':
            self.status = 'stopped'
