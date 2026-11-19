"""
Simulated mechanics of the Dilor Z40 grating drive: a stepper motor turning a screw, whose position is read by a
HEIDENHAIN optical ruler.

The class has the same public methods as the stepper motor part of the Arduino class of pymodaq_plugins_arduino, so
that the Dilor actuator can use one or the other. It moves the position of a ruler object given to the constructor,
such as IK220Mock of pymodaq_plugins_heidenhain.

Default values measured on the Z40: one step moves the ruler by about 0.49 µm, positive steps decrease the reading,
and the backlash is about 1.3 µm.
"""

import threading
import time

TICK_S = 0.01  # period of the simulated motion loop, in s


class Z40MechanicsMock:

    def __init__(self, com_port=None, ruler=None, step_mm=0.49e-3, direction=-1, backlash_mm=1.3e-3):
        self.ruler = ruler
        self.step_mm = step_mm
        self.direction = direction
        self.backlash_mm = backlash_mm
        self.stepper_motor = None
        self.position = 0
        self._screw_mm = 0.  # position of the screw, in mm of ruler
        self._carriage_mm = 0.  # position of the carriage, i.e. true position of the ruler, in mm
        self._target = 0  # target position of the move, in steps
        self._max_speed = 200  # speed of the move, in steps per second
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._thread = None

    def initialize_stepper_motor(self, pulse_pin, direction_pin, enable_pin=7):
        """ Initialize the simulated stepper motor and set its position to 0 """
        self.stepper_motor = 0  # any value but None marks the motor as initialized, as in the Arduino class
        self.position = 0
        if self.ruler is not None:
            self._carriage_mm = self.ruler.position_mm
        self._screw_mm = self._carriage_mm  # screw in the middle of the backlash

    def _do_steps(self, steps: int):
        """ Do steps with the simulated motor: turn the screw, which pushes the carriage once the backlash is taken
        up, and give the carriage position to the ruler """
        self.position += steps
        self._screw_mm += self.direction * self.step_mm * steps
        half_backlash = self.backlash_mm / 2
        if self._screw_mm - self._carriage_mm > half_backlash:
            self._carriage_mm = self._screw_mm - half_backlash
        elif self._carriage_mm - self._screw_mm > half_backlash:
            self._carriage_mm = self._screw_mm + half_backlash
        if self.ruler is not None:
            self.ruler.position_mm = self._carriage_mm

    def move_stepper_to_position(self, position: float, max_speed=200, acceleration=400):
        """ Start moving the simulated stepper motor to the specified position, without waiting for the end. During a
        move, change its target and speed, as with AccelStepper. The acceleration is not simulated """
        if self.stepper_motor is None:
            raise ValueError("Stepper motor not initialized. Call initialize_stepper_motor first.")
        with self._lock:
            self._target = int(position)
            self._max_speed = max_speed
            if self._thread is None:
                self._stop_event.clear()
                self._thread = threading.Thread(target=self._run, daemon=True)
                self._thread.start()

    def _run(self):
        """ Loop of the simulated motion, run in a thread: every TICK_S, do the steps the motor would have done at
        max_speed, towards the target, until the target is reached or the motor is stopped. The target and the speed
        are read at each tick, so they can be changed during the move, as with AccelStepper """
        last_time = time.perf_counter()
        steps_to_do = 0.
        while not self._stop_event.is_set():
            time.sleep(TICK_S)
            now = time.perf_counter()
            steps_to_do += self._max_speed * (now - last_time)
            last_time = now
            remaining = self._target - self.position
            steps = min(int(steps_to_do), abs(remaining))
            steps_to_do -= steps
            if steps > 0:
                self._do_steps(steps if remaining > 0 else -steps)
            with self._lock:
                if self.position == self._target:
                    self._thread = None
                    return
        with self._lock:
            self._thread = None

    def stop_stepper(self):
        """ Stop the simulated stepper motor immediately, at its current position """
        self._stop_event.set()
        thread = self._thread
        if thread is not None:
            thread.join()
        with self._lock:
            self._target = self.position

    def get_stepper_position(self):
        """ Return the current position of the simulated stepper motor, in steps """
        return self.position

    def shutdown(self):
        """ Stop the simulated stepper motor, as the Arduino class closes its connection when PyMoDAQ closes """
        self.stop_stepper()
