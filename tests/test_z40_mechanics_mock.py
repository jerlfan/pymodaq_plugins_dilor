import time

import pytest

from pymodaq_plugins_heidenhain.hardware.ik220_mock import IK220Mock
from pymodaq_plugins_dilor.hardware.z40_mechanics_mock import Z40MechanicsMock

STEP_MM = 0.49e-3
BACKLASH_MM = 1.3e-3


@pytest.fixture
def ruler():
    return IK220Mock(noise_mm=0.)


@pytest.fixture
def mechanics(ruler):
    mechanics = Z40MechanicsMock(ruler=ruler, step_mm=STEP_MM, backlash_mm=BACKLASH_MM)
    mechanics.initialize_stepper_motor(pulse_pin=4, direction_pin=3)
    yield mechanics
    mechanics.shutdown()


def move_and_wait(mechanics, position, max_speed=10000, timeout=2.):
    mechanics.move_stepper_to_position(position, max_speed=max_speed)
    end = time.perf_counter() + timeout
    while mechanics.get_stepper_position() != position:
        assert time.perf_counter() < end, 'the move did not finish in time'
        time.sleep(0.01)


def test_move_before_initialization():
    with pytest.raises(ValueError):
        Z40MechanicsMock().move_stepper_to_position(10)


def test_initialization(mechanics):
    assert mechanics.stepper_motor is not None
    assert mechanics.get_stepper_position() == 0


def test_positive_steps_decrease_the_reading(mechanics, ruler):
    start_mm = ruler.position_mm
    move_and_wait(mechanics, 100)
    assert ruler.position_mm == pytest.approx(start_mm - 100 * STEP_MM + BACKLASH_MM / 2, abs=1e-9)


def test_backlash_on_reversal(mechanics, ruler):
    move_and_wait(mechanics, 100)
    reading_mm = ruler.position_mm
    move_and_wait(mechanics, 98)  # 2 steps back, less than the backlash: the ruler does not move
    assert ruler.position_mm == pytest.approx(reading_mm, abs=1e-9)
    move_and_wait(mechanics, 0)
    assert ruler.position_mm == pytest.approx(reading_mm + 100 * STEP_MM - BACKLASH_MM, abs=1e-9)


def test_stop_during_move(mechanics):
    mechanics.move_stepper_to_position(1000, max_speed=200)
    time.sleep(0.2)
    mechanics.stop_stepper()
    position = mechanics.get_stepper_position()
    assert 0 < position < 1000
    time.sleep(0.1)
    assert mechanics.get_stepper_position() == position


def test_new_target_during_move(mechanics):
    mechanics.move_stepper_to_position(1000, max_speed=200)
    time.sleep(0.2)
    move_and_wait(mechanics, -50)
