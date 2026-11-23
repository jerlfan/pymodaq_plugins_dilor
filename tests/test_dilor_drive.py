import time

import pytest

from pymodaq_plugins_heidenhain.hardware.ik220_mock import IK220Mock
from pymodaq_plugins_dilor.hardware.z40_mechanics_mock import Z40MechanicsMock
from pymodaq_plugins_dilor.hardware.dilor_drive import DilorDrive

TRUE_STEP_MM = 0.49e-3
BACKLASH_MM = 1.3e-3


@pytest.fixture
def ruler():
    return IK220Mock(noise_mm=0.)


@pytest.fixture
def mechanics(ruler):
    mechanics = Z40MechanicsMock(ruler=ruler, step_mm=TRUE_STEP_MM, backlash_mm=BACKLASH_MM)
    mechanics.initialize_stepper_motor(pulse_pin=4, direction_pin=3)
    yield mechanics
    mechanics.shutdown()


def make_drive(mechanics, ruler, step_mm=0.5e-3, **kwargs):
    """ Drive with a step 2 % larger than the true one by default, as a not perfectly known calibration """
    return DilorDrive(mechanics, ruler, step_mm=step_mm, max_speed=10000, **kwargs)


def run(drive, target_mm, timeout=5.):
    """ Move to the target and call update, as the polling of PyMoDAQ does, until the move is finished """
    drive.move_to(target_mm)
    end = time.perf_counter() + timeout
    while not drive.update():
        assert time.perf_counter() < end, 'the move did not finish in time'
        time.sleep(0.01)


@pytest.mark.parametrize('shift_mm', [-0.1, 0.05, -1.])
def test_reach_the_target(mechanics, ruler, shift_mm):
    drive = make_drive(mechanics, ruler)
    target_mm = drive.get_position() + shift_mm
    run(drive, target_mm)
    assert drive.status == 'reached'
    assert abs(drive.get_position() - target_mm) <= drive.tolerance_mm


@pytest.mark.parametrize('shift_mm', [-0.1, 0.05])
def test_arrival_in_the_approach_direction(mechanics, ruler, shift_mm):
    """ Positive steps decrease the reading: the target is reached from above, and the drive stops before passing it """
    drive = make_drive(mechanics, ruler)
    target_mm = drive.get_position() + shift_mm
    run(drive, target_mm)
    assert drive.get_position() - target_mm >= -10e-6  # 10 nm: measuring step of the ruler


@pytest.mark.parametrize('step_mm', [0.45e-3, 0.53e-3])
def test_wrong_step_is_corrected(mechanics, ruler, step_mm):
    drive = make_drive(mechanics, ruler, step_mm=step_mm)
    for shift_mm in [-0.2, 0.1]:
        target_mm = drive.get_position() + shift_mm
        run(drive, target_mm)
        assert drive.status == 'reached'
        assert abs(drive.get_position() - target_mm) <= drive.tolerance_mm


@pytest.mark.parametrize('first_steps', [0, -10, 10])
def test_take_up_backlash_whatever_the_start(mechanics, ruler, first_steps):
    """ Whatever the side of the backlash at the start, the screw pushes the carriage on the approach side after the
    take up (internal attributes of the simulation) """
    drive = make_drive(mechanics, ruler)
    drive._send_steps(first_steps)
    time.sleep(0.1)
    drive.take_up_backlash()
    assert mechanics._carriage_mm - mechanics._screw_mm == pytest.approx(BACKLASH_MM / 2, abs=1e-9)


def test_stop(mechanics, ruler):
    drive = make_drive(mechanics, ruler, max_corrections=10)
    drive.max_speed = 200
    drive.move_to(drive.get_position() - 1.)
    time.sleep(0.1)
    drive.stop()
    assert drive.status == 'stopped'
    assert drive.update()


def test_resolution_limit(mechanics, ruler):
    drive = make_drive(mechanics, ruler, tolerance_mm=0.05e-3)  # less than one step
    target_mm = drive.get_position() - 0.1
    run(drive, target_mm)
    assert drive.status in ('reached', 'resolution limit')
    assert abs(drive.get_position() - target_mm) < TRUE_STEP_MM


class MotorThatDoesNotMove:
    def get_stepper_position(self):
        return 0

    def move_stepper_to_position(self, position, max_speed=200, acceleration=400):
        pass

    def stop_stepper(self):
        pass


def test_take_up_backlash_timeout(ruler):
    drive = DilorDrive(MotorThatDoesNotMove(), ruler)
    with pytest.raises(TimeoutError):
        drive.take_up_backlash(timeout_s=0.1)


def test_step_is_learned(mechanics, ruler):
    """ With a starting step twice too large, the drive measures the true step on the ruler """
    drive = make_drive(mechanics, ruler, step_mm=1e-3)
    drive.take_up_backlash()
    for shift_mm in [-0.1, -0.2, -0.3]:
        run(drive, drive.get_position() + shift_mm)
    assert drive.step_mm == pytest.approx(TRUE_STEP_MM, rel=0.05)
    assert drive.status == 'reached'


def test_step_learned_only_without_change_of_direction(mechanics, ruler):
    """ The first move after a change of direction takes up the backlash: it is not used to measure the step. The next
    move in the same direction is (internal methods of the drive) """
    drive = make_drive(mechanics, ruler, step_mm=0.6e-3)
    drive.take_up_backlash()  # positive steps
    for expected_step_mm in [0.6e-3, 0.6e-3 + 0.5 * (TRUE_STEP_MM - 0.6e-3)]:
        drive._send_steps(-100)  # the first time, a change of direction
        while mechanics.get_stepper_position() != drive._motor_target:
            time.sleep(0.01)
        drive._learn_step()
        assert drive.step_mm == pytest.approx(expected_step_mm, abs=1e-9)
