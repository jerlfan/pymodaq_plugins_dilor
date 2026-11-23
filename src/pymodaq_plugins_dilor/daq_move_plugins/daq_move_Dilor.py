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
            Absolute target, with units. (With the backcompatibility DataActuatorType.float it would be a plain float,
            already expressed in the axis unit, but new plugins should not use it.)
        """

        value = self.check_bound(value)  #if user checked bounds, the defined bounds are applied here
        self.target_value = value
        value = self.set_position_with_scaling(value)  # apply scaling if the user specified one
        ## TODO for your custom plugin
        # IMPORTANT: the controller only understands its own units. Always send value.value(self.axis_unit), it returns
        # the magnitude expressed in the unit of this axis (the one of _controller_units, 'mm' here). PyMoDAQ already
        # converts the target to the axis unit before calling move_abs, but value.value() alone is just the magnitude
        # in whatever unit `value` carries (1 cm gives 1.0, not 10.0): value.value(self.axis_unit) guards against a
        # mistake, and keeps the plugin correct when move_abs is called from elsewhere with another unit.
        # (axis_unit is the unit of the current axis, axis_units the list/dict of the units of all the axes.)
        raise NotImplementedError  # when writing your own plugin remove this line
        self.controller.your_method_to_set_an_absolute_value(value.value(self.axis_unit))  # when writing your own plugin replace this line
        self.emit_status(ThreadCommand('Update_Status', ['Some info you want to log']))

    def move_rel(self, value: DataActuator):
        """ Move the actuator to the relative target actuator value defined by value

        Parameters
        ----------
        value: DataActuator
            Relative displacement, with units (convert it with value.value(self.axis_unit) before sending it to the
            controller, see move_abs).
        """
        value = self.check_bound(self.current_value + value) - self.current_value
        self.target_value = value + self.current_value
        value = self.set_position_relative_with_scaling(value)

        ## TODO for your custom plugin
        # IMPORTANT: send value.value(self.axis_unit), not value.value(): the controller only understands its own units
        raise NotImplementedError  # when writing your own plugin remove this line
        self.controller.your_method_to_set_a_relative_value(value.value(self.axis_unit))  # when writing your own plugin replace this line
        self.emit_status(ThreadCommand('Update_Status', ['Some info you want to log']))

    def move_home(self):
        """Call the reference method of the controller

        Contrary to move_abs and move_rel, PyMoDAQ does not poll the position to know when the homing is finished.
        Whether this method blocks until the actuator is at home or only starts the homing (asynchronous), the plugin
        has to call self.move_done() when the homing process ends, otherwise PyMoDAQ keeps waiting for it.
        """

        ## TODO for your custom plugin
        raise NotImplementedError  # when writing your own plugin remove this line
        self.controller.your_method_to_get_to_a_known_reference()  # when writing your own plugin replace this line
        self.emit_status(ThreadCommand('Update_Status', ['Some info you want to log']))
        self.move_done()  # blocking homing: call it here. Asynchronous homing: call it once the controller reports the end

    def stop_motion(self):
        """Stop the actuator and emits move_done signal"""

        ## TODO for your custom plugin
        raise NotImplementedError  # when writing your own plugin remove this line
        self.controller.your_method_to_stop_positioning()  # when writing your own plugin replace this line
        self.emit_status(ThreadCommand('Update_Status', ['Some info you want to log']))

    def commit_settings(self, param: Parameter):
        """Apply the consequences of a change of value in the detector settings

        Parameters
        ----------
        param: Parameter
            A given parameter (within detector_settings) whose value has been changed by the user
        """
        ## TODO for your custom plugin
        if param.name() == 'axis':
            self.axis_unit = self.controller.your_method_to_get_correct_axis_unit()
            # do this only if you can and if the units are not known beforehand, for instance
            # if the motors connected to the controller are of different type (mm, µm, nm, , etc...)
            # see BrushlessDCMotor from the thorlabs plugin for an exemple

        elif param.name() == "a_parameter_you've_added_in_self.params":
           self.controller.your_method_to_apply_this_param_change()
        else:
            pass


if __name__ == '__main__':
    main(__file__)
