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
    ] + comon_parameters_fun(axis_names=_axis_names)

    def ini_attributes(self):
        #  TODO declare the type of the wrapper (and assign it to self.controller) you're going to use for easy
        #  autocompletion
        self.controller: PythonWrapperObjectOfYourInstrument = None

        #TODO declare here attributes you want/need to init with a default value
        pass

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
        raise NotImplementedError  # TODO when writing your own plugin remove this line and modify the ones below
        if self.is_master:  # is needed when controller is master
            self.controller = PythonWrapperObjectOfYourInstrument()  # TODO add the arguments for the instantiation
            initialized = self.controller.a_method_or_atttribute_to_check_if_init()  # todo
            # todo: enter here whatever is needed for your controller initialization and eventual
            #  opening of the communication channel
        else:
            self.controller = controller
            initialized = True

        info = "Whatever info you want to log"
        return info, initialized

    def close(self):
        """Terminate the communication protocol"""
        ## TODO for your custom plugin
        raise NotImplementedError  # when writing your own plugin remove this line
        # only the master owns the controller (a slave must not close the shared one) and ini_stage/ini_detector may
        # have failed before creating it, so close can be called while self.controller is still None
        if self.is_master and self.controller is not None:
            #  self.controller.your_method_to_terminate_the_communication()  # when writing your own plugin replace this line
            ...

    def get_actuator_value(self) -> DataActuator:
        """Get the current value from the hardware with scaling conversion.

        Returns
        -------
        float: The position obtained after scaling conversion.
        """
        ## TODO for your custom plugin
        raise NotImplementedError  # when writing your own plugin remove this line
        pos = DataActuator(data=self.controller.your_method_to_get_the_actuator_value(),  # when writing your own plugin replace this line
                           units=self.axis_unit)
        pos = self.get_position_with_scaling(pos)
        return pos

    def user_condition_to_reach_target(self) -> bool:
        """ Implement a condition for exiting the polling mechanism and specifying that the
        target value has been reached

       Returns
        -------
        bool: if True, PyMoDAQ considers the target value has been reached
        """
        # TODO either delete this method if the usual polling is fine with you, but if need you can
        #  add here some other condition to be fullfilled either a completely new one or
        #  using or/and operations between the epsilon_bool and some other custom booleans
        #  for a usage example see DAQ_Move_brushlessMotor from the Thorlabs plugin
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
