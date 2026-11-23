from pymodaq.control_modules.move_utility_classes import (DAQ_Move_base, comon_parameters_fun,
                                                          main, DataActuatorType, DataActuator)
from pymodaq.control_modules.daq_move_ui.utils import UiType
from pymodaq_utils.config import GlobalConfig
from pymodaq_utils.utils import ThreadCommand
from pymodaq_gui.parameter import Parameter

from pymodaq_plugins_arduino.hardware.arduino_telemetrix import Arduino
from pymodaq_plugins_heidenhain.hardware.ik220 import IK220
from pymodaq_plugins_heidenhain.hardware.ik220_mock import IK220Mock

from pymodaq_plugins_dilor.hardware.dilor_drive import DilorDrive
from pymodaq_plugins_dilor.hardware.z40_mechanics_mock import Z40MechanicsMock


class DAQ_Move_Dilor(DAQ_Move_base):
    """ PyMoDAQ actuator of the grating drive of the Dilor spectrometers: a stepper motor driven by an Arduino,
    positioned on the reading of a HEIDENHAIN optical ruler read by an IK 220 card. The position is the reading of
    the ruler, in mm.

    Attributes:
    -----------
    controller: DilorDrive
        Grating drive, with the motor (Arduino or Z40MechanicsMock) and the ruler (IK220 or IK220Mock).
    """
    is_multiaxes = False
    _axis_names = ['Grating']
    _controller_units = 'mm'
    _epsilons = GlobalConfig()('dilor', 'drive', 'tolerance_mm')

    data_actuator_type = DataActuatorType.DataActuator
    ui_type = UiType.NONE
    has_encoder = True

    params = [
        {'title': 'Mode:', 'name': 'mode', 'type': 'list', 'limits': ['MOCK', 'Z40'],
         'value': GlobalConfig()('dilor', 'actuator', 'mode'),
         'tip': 'MOCK: simulated mechanics and ruler, Z40: Arduino and IK 220 card '
                '(applied at the next initialization)'},
        {'title': 'Arduino', 'name': 'arduino', 'type': 'group', 'children': [
            {'title': 'COM port:', 'name': 'com_port', 'type': 'list', 'limits': Arduino.COM_PORTS,
             'value': GlobalConfig()('dilor', 'actuator', 'com_port')},
            {'title': 'Pulse pin:', 'name': 'pulse_pin', 'type': 'int',
             'value': GlobalConfig()('dilor', 'actuator', 'pulse_pin')},
            {'title': 'Direction pin:', 'name': 'direction_pin', 'type': 'int',
             'value': GlobalConfig()('dilor', 'actuator', 'direction_pin')},
            {'title': 'Enable pin:', 'name': 'enable_pin', 'type': 'int',
             'value': GlobalConfig()('dilor', 'actuator', 'enable_pin')},
        ]},
        {'title': 'Drive', 'name': 'drive', 'type': 'group', 'children': [
            {'title': 'Step (mm):', 'name': 'step_mm', 'type': 'float', 'decimals': 6,
             'value': GlobalConfig()('dilor', 'drive', 'step_mm'),
             'tip': 'Displacement of the ruler for one motor step'},
            {'title': 'Direction:', 'name': 'direction', 'type': 'list', 'limits': [-1, 1],
             'value': GlobalConfig()('dilor', 'drive', 'direction'),
             'tip': 'Sign of the ruler displacement for positive motor steps'},
            {'title': 'Approach:', 'name': 'approach', 'type': 'list', 'limits': [-1, 1],
             'value': GlobalConfig()('dilor', 'drive', 'approach'),
             'tip': 'Sign of the motor steps used to reach the target'},
            {'title': 'Speed (steps/s):', 'name': 'max_speed', 'type': 'int',
             'value': GlobalConfig()('dilor', 'drive', 'max_speed')},
            {'title': 'Max corrections:', 'name': 'max_corrections', 'type': 'int',
             'value': GlobalConfig()('dilor', 'drive', 'max_corrections')},
            {'title': 'Overshoot (steps):', 'name': 'overshoot_steps', 'type': 'int',
             'value': GlobalConfig()('dilor', 'drive', 'overshoot_steps'),
             'tip': 'Steps beyond a target that is behind, and steps of the backlash take up'},
        ]},
        {'title': 'Last move', 'name': 'last_move', 'type': 'group', 'children': [
            {'title': 'Status:', 'name': 'status', 'type': 'str', 'value': '', 'readonly': True},
            {'title': 'Corrections:', 'name': 'corrections', 'type': 'int', 'value': 0, 'readonly': True},
        ]},
    ] + comon_parameters_fun(axis_names=_axis_names)

    def ini_attributes(self):
        self.controller: DilorDrive = None

    def ini_stage(self, controller=None):
        """Actuator communication initialization

        Parameters
        ----------
        controller: (object)
            custom object of a PyMoDAQ plugin (Slave case). None if only one actuator by controller (Master case)

        Returns
        -------
        info: str
        initialized: bool
            False if initialization failed otherwise True
        """
        if self.is_master:
            if self.settings['mode'] == 'MOCK':
                ruler = IK220Mock()
                motor = Z40MechanicsMock(ruler=ruler, step_mm=GlobalConfig()('dilor', 'mock', 'step_mm'),
                                         backlash_mm=GlobalConfig()('dilor', 'mock', 'backlash_mm'))
            else:
                ruler = IK220(dllpath=GlobalConfig()('heidenhain', 'ik220', 'dll_path'),
                              axis=GlobalConfig()('heidenhain', 'ik220', 'axis'))
                motor = Arduino(com_port=self.settings['arduino', 'com_port'])
            motor.initialize_stepper_motor(self.settings['arduino', 'pulse_pin'],
                                           self.settings['arduino', 'direction_pin'],
                                           self.settings['arduino', 'enable_pin'])
            self.controller = DilorDrive(motor, ruler,
                                         step_mm=self.settings['drive', 'step_mm'],
                                         direction=self.settings['drive', 'direction'],
                                         approach=self.settings['drive', 'approach'],
                                         tolerance_mm=self.epsilon,
                                         max_speed=self.settings['drive', 'max_speed'],
                                         max_corrections=self.settings['drive', 'max_corrections'],
                                         overshoot_steps=self.settings['drive', 'overshoot_steps'])
            self.controller.take_up_backlash()
        else:
            self.controller = controller

        info = f"Dilor grating drive initialized in {self.settings['mode']} mode"
        return info, True

    def close(self):
        """Terminate the communication protocol"""
        if self.is_master and self.controller is not None:
            self.controller.stop()
            self.controller.motor.shutdown()
            self.controller = None

    def get_actuator_value(self) -> DataActuator:
        """Get the current value from the hardware with scaling conversion.

        Returns
        -------
        DataActuator: The position read on the ruler, in mm, after scaling conversion.
        """
        pos = DataActuator(data=self.controller.get_position(), units=self.axis_unit)
        pos = self.get_position_with_scaling(pos)
        return pos

    def absolute_difference_condition_to_reach_target(self) -> bool:
        """ The distance to the target is checked by the drive, with its tolerance: see user_condition_to_reach_target

        Returns
        -------
        bool: always True, so that the end of the move is decided by user_condition_to_reach_target only
        """
        return True

    def user_condition_to_reach_target(self) -> bool:
        """ Make the move of the drive progress, and tell PyMoDAQ when it is finished

        Returns
        -------
        bool: if True, PyMoDAQ considers the target value has been reached
        """
        if not self.controller.update():
            return False
        self.settings.child('last_move', 'status').setValue(self.controller.status)
        self.settings.child('last_move', 'corrections').setValue(self.controller.corrections)
        if self.controller.status != 'reached':
            self.emit_status(ThreadCommand('Update_Status', [f'Move finished: {self.controller.status}']))
        return True

    def move_abs(self, value: DataActuator):
        """ Move the actuator to the absolute target defined by value

        Parameters
        ----------
        value: DataActuator
            Absolute target, with units
        """
        value = self.check_bound(value)  # if user checked bounds, the defined bounds are applied here
        self.target_value = value
        value = self.set_position_with_scaling(value)  # apply scaling if the user specified one
        self.controller.tolerance_mm = self.epsilon
        self.controller.move_to(value.value(self.axis_unit))
        self.emit_status(ThreadCommand('Update_Status', [f'Moving to {value.value(self.axis_unit):.5f} mm']))

    def move_rel(self, value: DataActuator):
        """ Move the actuator to the relative target actuator value defined by value

        Parameters
        ----------
        value: DataActuator
            Relative displacement, with units
        """
        value = self.check_bound(self.current_value + value) - self.current_value
        self.target_value = value + self.current_value
        self.move_abs(self.target_value)

    def move_home(self):
        """ No homing: the ruler is absolute, its reading is always the true position """
        self.emit_status(ThreadCommand('Update_Status', ['No home for the Dilor drive: the ruler is absolute']))
        self.move_done()

    def stop_motion(self):
        """ Stop the actuator: the next polling sees the drive stopped and calls move_done """
        self.controller.stop()
        self.emit_status(ThreadCommand('Update_Status', ['Move stopped']))

    def commit_settings(self, param: Parameter):
        """Apply the consequences of a change of value in the detector settings

        Parameters
        ----------
        param: Parameter
            A given parameter (within detector_settings) whose value has been changed by the user
        """
        if param.parent().name() == 'drive' and self.controller is not None:
            setattr(self.controller, param.name(), param.value())
        # the mode and the Arduino settings are applied at the next initialization


if __name__ == '__main__':
    main(__file__)
