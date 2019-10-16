# This file is part of ts_rotator.
#
# Developed for the LSST Data Management System.
# This product includes software developed by the LSST Project
# (https://www.lsst.org).
# See the COPYRIGHT file at the top-level directory of this distribution
# for details of code ownership.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import argparse
import asyncio

from lsst.ts import salobj
from lsst.ts import hexrotcomm
from . import constants
from . import enums
from . import structs
from . import mock_controller


# Dict of controller state: CSC state.
# The names match but the numeric values do not.
StateCscState = {
    enums.ControllerState.OFFLINE: salobj.State.OFFLINE,
    enums.ControllerState.STANDBY: salobj.State.STANDBY,
    enums.ControllerState.DISABLED: salobj.State.DISABLED,
    enums.ControllerState.ENABLED: salobj.State.ENABLED,
    enums.ControllerState.FAULT: salobj.State.FAULT,
}

# Dict of CSC state: controller state.
# The names match but the numeric values do not.
CscStateState = dict((value, key) for key, value in StateCscState.items())


class RotatorCsc(salobj.Controller):
    """MT rotator CSC.

    Parameters
    ----------
    initial_state : `lsst.ts.salobj.State` or `int` (optional)
        The initial state of the CSC. Ignored (other than checking
        that it is a valid value) except in simulation mode,
        because in normal operation the initial state is the current state
        of the controller. This is provided for unit testing.
    simulation_mode : `int` (optional)
        Simulation mode. Allowed values:

        * 0: regular operation.
        * 1: simulation: use a mock low level controller.

    Notes
    -----
    **Error Codes**

    * 1: invalid data read on the telemetry socket

    This CSC is unusual in several respect:

    * It acts as a server (not a client) for a low level controller
      (because that is how the low level controller is written).
    * The low level controller maintains the summary state and detailed state
      (that's why this code inherits from Controller instead of BaseCsc).
    * The simulation mode can only be set at construction time.
    """

    def __init__(self,
                 initial_state=salobj.State.OFFLINE,
                 simulation_mode=0):
        self._initial_state = salobj.State(initial_state)
        if simulation_mode not in (0, 1):
            raise ValueError(f"simulation_mode = {simulation_mode}; must be 0 or 1")
        self.simulation_mode = simulation_mode
        self.server = None
        self.mock_ctrl = None
        # Set this to 2 when trackStart is called, then decrement
        # when telemetry is received. If > 0 or enabled_substate is
        # SLEWING_OR_TRACKING then allow the track command.
        # This solves the problem of allowing the track command
        # immediately after the trackStart, before telemetry is received.
        self._tracking_started_n = 0
        super().__init__(name="Rotator", index=0, do_callbacks=True)

        # Dict of enum.CommandCode: Command
        # with constants set to suitable values.
        self.commands = dict()
        for cmd in enums.CommandCode:
            command = hexrotcomm.Command()
            command.cmd = cmd
            command.sync_pattern = constants.ROTATOR_SYNC_PATTERN
            self.commands[cmd] = command

    @property
    def summary_state(self):
        """Return the current summary state as a salobj.State,
        or OFFLINE if unknown.
        """
        if self.server is None or not self.server.connected:
            return salobj.State.OFFLINE
        return StateCscState.get(int(self.server.telemetry.state), salobj.State.OFFLINE)

    async def start(self):
        await super().start()
        simulating = self.simulation_mode != 0
        host = hexrotcomm.LOCAL_HOST if simulating else None
        self.server = hexrotcomm.Server(host=host,
                                        log=self.log,
                                        ConfigClass=structs.Config,
                                        TelemetryClass=structs.Telemetry,
                                        connect_callback=self.connect_callback,
                                        config_callback=self.config_callback,
                                        telemetry_callback=self.telemetry_callback,
                                        use_random_ports=simulating)
        await self.server.start_task
        if simulating:
            initial_ctrl_state = CscStateState[self._initial_state]
            self.mock_ctrl = mock_controller.MockMTRotatorController(
                log=self.log,
                initial_state=initial_ctrl_state,
                command_port=self.server.command_port,
                telemetry_port=self.server.telemetry_port)
            await self.mock_ctrl.connect_task
        else:
            self.evt_summaryState.set_put(summaryState=salobj.State.OFFLINE)
        self.evt_inPosition.set_put(inPosition=False, force_output=True)

    async def close_tasks(self):
        if self.mock_ctrl is not None:
            await self.mock_ctrl.close()
        if self.server is not None:
            await self.server.close()

    def assert_summary_state(self, *allowed_states):
        """Assert that the current summary state is as specified.

        Used in do_xxx methods to check that a command is allowed.
        """
        if self.summary_state not in allowed_states:
            raise salobj.ExpectedError(f"Must be in state(s) {allowed_states}, not {self.summary_state}")

    async def run_command(self, cmd, **kwargs):
        command = self.commands[cmd]
        for name, value in kwargs.items():
            if hasattr(command, name):
                setattr(command, name, value)
            else:
                raise ValueError(f"Unknown command argument {name}")
        # Note: increment correctly wraps around
        command.counter += 1
        await self.server.put_command(command)

    # Unsupported standard CSC commnands.
    async def do_abort(self, data):
        raise salobj.ExpectedError("Unsupported command")

    async def do_setSimulationMode(self, data):
        raise salobj.ExpectedError("Unsupported command: "
                                   "simulation mode can only be set when starting the CSC.")

    async def do_setValue(self, data):
        raise salobj.ExpectedError("Unsupported command")

    # Standard CSC commnands.
    async def do_enable(self, data):
        """Execute the enable command."""
        self.assert_summary_state(salobj.State.DISABLED)
        await self.run_command(cmd=enums.CommandCode.SET_STATE,
                               param1=enums.SetStateParam.ENABLE)
        await self.server.next_telemetry()
        self.assert_summary_state(salobj.State.ENABLED)

    async def do_disable(self, data):
        """Execute the disable command."""
        self.assert_summary_state(salobj.State.ENABLED)
        await self.run_command(cmd=enums.CommandCode.SET_STATE,
                               param1=enums.SetStateParam.DISABLE)
        await self.server.next_telemetry()
        self.assert_summary_state(salobj.State.DISABLED)

    async def do_enterControl(self, data):
        """Execute the enterControl command.
        """
        self.assert_summary_state(salobj.State.OFFLINE)
        if self.server.telemetry.offline_substate != enums.OfflineSubstate.AVAILABLE:
            raise salobj.ExpectedError(
                "Use the engineering interface to put the controller into state OFFLINE/AVAILABLE")
        await self.run_command(cmd=enums.CommandCode.SET_STATE,
                               param1=enums.SetStateParam.ENTER_CONTROL)
        await self.server.next_telemetry()
        self.assert_summary_state(salobj.State.STANDBY)

    async def do_exitControl(self, data):
        self.assert_summary_state(salobj.State.STANDBY)
        await self.run_command(cmd=enums.CommandCode.SET_STATE,
                               param1=enums.SetStateParam.EXIT)
        await self.server.next_telemetry()
        self.assert_summary_state(salobj.State.OFFLINE)

    async def do_standby(self, data):
        self.assert_summary_state(salobj.State.DISABLED, salobj.State.FAULT)
        if self.server.telemetry.state == enums.ControllerState.DISABLED:
            await self.run_command(cmd=enums.CommandCode.SET_STATE,
                                   param1=enums.SetStateParam.STANDBY)
        else:
            raise salobj.ExpectedError(
                "Use clearError or the engineering interface to set the state to OFFLINE/PUBLISH_ONLY, "
                "then use the engineering interface to set the state to OFFLINE/AVAILABLE.")
        await self.server.next_telemetry()
        self.assert_summary_state(salobj.State.STANDBY)

    async def do_start(self, data):
        """Execute the start command.

        Notes
        -----
        This ignores the data, unlike the vendor's CSC code, which writes the
        supplied file name into a file on an nfs-mounted partition.
        I hope we won't need to do that, as it seems complicated.
        """
        if self.summary_state != salobj.State.STANDBY:
            raise salobj.ExpectedError(f"CSC is in {self.summary_state}; must be STANDBY state to start")
        await self.run_command(cmd=enums.CommandCode.SET_STATE,
                               param1=enums.SetStateParam.START)
        await self.server.next_telemetry()
        self.assert_summary_state(salobj.State.DISABLED)

    # Rotator-specific commands.
    async def do_clearError(self, data):
        """Reset the FAULT state to OFFLINE/PUBLISH_ONLY.

        Unfortunately, after this call you must use the engineering user
        interface to transition the controller from OFFLINE_PUBLISH_ONLY
        to OFFLINE/AVAILABLE before the CSC can control it.
        """
        if self.summary_state != salobj.State.FAULT:
            raise salobj.ExpectedError("Must be in FAULT state.")
        self.assert_enabled_substate(enums.EnabledSubstate.FAULT)
        # Two sequential commands are needed to clear error
        await self.run_command(cmd=enums.CommandCode.SET_STATE,
                               param1=enums.SetStateParam.CLEAR_ERROR)
        await asyncio.sleep(0.9)
        await self.run_command(cmd=enums.CommandCode.SET_STATE,
                               param1=enums.SetStateParam.CLEAR_ERROR)
        await self.server.next_telemetry()
        self.assert_summary_state(salobj.State.OFFLINE)

    async def do_configureAcceleration(self, data):
        """Specify the acceleration limit."""
        self.assert_enabled_substate(enums.EnabledSubstate.STATIONARY)
        if not 0 < data.alimit <= constants.MAX_ACCEL_LIMIT:
            raise salobj.ExpectedError(f"alimit={data.alimit} must be > 0 and <= {constants.MAX_ACCEL_LIMIT}")
        await self.run_command(cmd=enums.CommandCode.CONFIG_ACCEL,
                               param1=data.alimit)

    async def do_configureVelocity(self, data):
        """Specify the velocity limit."""
        self.assert_enabled_substate(enums.EnabledSubstate.STATIONARY)
        if not 0 < data.vlimit <= constants.MAX_VEL_LIMIT:
            raise salobj.ExpectedError(f"vlimit={data.vlimit} must be > 0 and <= {constants.MAX_VEL_LIMIT}")
        await self.run_command(cmd=enums.CommandCode.CONFIG_VEL,
                               param1=data.vlimit)

    async def do_move(self, data):
        """Go to the position specified by the most recent ``positionSet``
        command.
        """
        self.assert_enabled_substate(enums.EnabledSubstate.STATIONARY)
        await self.run_command(cmd=enums.CommandCode.SET_SUBSTATE,
                               param1=enums.EnabledSetSubstateParam.MOVE_POINT_TO_POINT)

    async def do_moveConstantVelocity(self, data):
        """Move at the speed and for the duration specified by the most recent
        ``velocitySet`` command. NOT SUPPORTED.
        """
        raise salobj.ExpectedError("Not implemented")
        # self.assert_enabled_substate(enums.EnabledSubstate.STATIONARY)
        # await self.run_command(cmd=enums.CommandCode.SET_SUBSTATE,
        #                        param1=enums.EnabledSetSubstateParam.CONSTANT_VELOCITY)

    async def do_positionSet(self, data):
        """Specify a position for the ``move`` command.
        """
        self.assert_enabled_substate(enums.EnabledSubstate.STATIONARY)
        if not self.server.config.lower_pos_limit <= data.angle <= self.server.config.upper_pos_limit:
            raise salobj.ExpectedError(f"angle {data.angle} not in range "
                                       f"[{self.server.config.lower_pos_limit}, "
                                       f"{self.server.config.upper_pos_limit}]")
        await self.run_command(cmd=enums.CommandCode.POSITION_SET,
                               param1=data.angle)

    async def do_stop(self, data):
        """Halt tracking or any other motion.
        """
        if self.summary_state != salobj.State.ENABLED:
            raise salobj.ExpectedError("Not enabled")
        await self.run_command(cmd=enums.CommandCode.SET_SUBSTATE,
                               param1=enums.EnabledSetSubstateParam.STOP)

    async def do_test(self, data):
        """Execute the test command. NOT SUPPORTED.
        """
        raise salobj.ExpectedError("Not implemented")
        # self.assert_enabled_substate(enums.EnabledSubstate.STATIONARY)
        # # The test command is unique in that all fields must be left
        # # at their initialized value except sync_pattern
        # # (at least that is what the Vendor's code does).
        # command = structs.Command()
        # command.sync_pattern = structs.ROTATOR_SYNC_PATTERN
        # await self.server.run_command(command)

    async def do_track(self, data):
        """Specify a position, velocity, TAI time tracking update.
        """
        if self.summary_state != salobj.State.ENABLED:
            raise salobj.ExpectedError("Not enabled")
        if self.server.telemetry.enabled_substate != enums.EnabledSubstate.SLEWING_OR_TRACKING:
            if self._tracking_started_n <= 0:
                raise salobj.ExpectedError("Low-level controller in substate "
                                           f"{self.server.telemetry.enabled_substate} "
                                           f"instead of {enums.EnabledSubstate.SLEWING_OR_TRACKING}")
        dt = data.tai - salobj.current_tai()
        curr_pos = data.angle + data.velocity*dt
        if not self.server.config.lower_pos_limit <= curr_pos <= self.server.config.upper_pos_limit:
            raise salobj.ExpectedError(f"current position {curr_pos} not in range "
                                       f"[{self.server.config.lower_pos_limit}, "
                                       f"{self.server.config.upper_pos_limit}]")
        if not abs(data.velocity) <= self.server.config.velocity_limit:
            raise salobj.ExpectedError(f"abs(velocity={data.velocity}) > "
                                       f"[{self.server.config.velocity_limit}")
        await self.run_command(cmd=enums.CommandCode.TRACK_VEL_CMD,
                               param1=data.tai,
                               param2=data.angle,
                               param3=data.velocity)

    async def do_trackStart(self, data):
        """Start tracking.

        Once this is run you must issue ``track`` commands at 10-20Hz
        until you are done tracking, then issue the ``stop`` command.
        """
        self.assert_enabled_substate(enums.EnabledSubstate.STATIONARY)
        await self.run_command(cmd=enums.CommandCode.SET_SUBSTATE,
                               param1=enums.EnabledSetSubstateParam.TRACK)
        self._tracking_started_n = 2

    async def do_velocitySet(self, data):
        """Specify the velocity and duration for the ``moveConstantVelocity``
        command. NOT SUPPORTED.
        """
        raise salobj.ExpectedError("Not implemented")
        # self.assert_enabled_substate(enums.EnabledSubstate.STATIONARY)
        # if abs(data.velocity) > self.server.config.velocity_limit:
        #     raise salobj.ExpectedError(f"Velocity {data.velocity} > "
        #              f"limit {self.server.config.velocity_limit}")
        # await self.run_command(cmd=enums.CommandCode.SET_CONSTANT_VEL,
        #                        param1=data.velocity,
        #                        param2=data.moveDuration)

    def assert_enabled_substate(self, substate):
        """Assert the controller is enabled and in the specified substate.
        """
        if self.summary_state != salobj.State.ENABLED:
            raise salobj.ExpectedError("Not enabled")
        if self.server.telemetry.enabled_substate != substate:
            raise salobj.ExpectedError("Low-level controller in substate "
                                       f"{self.server.telemetry.enabled_substate} "
                                       f"instead of {substate!r}")

    def connect_callback(self, server):
        """Called when the server's command or telemetry sockets
        connect or disconnect.

        Parameters
        ----------
        server : `lsst.ts.hexrotcomm.Server`
            TCP/IP server.
        """
        self.evt_connected.set_put(command=self.server.command_connected,
                                   telemetry=self.server.telemetry_connected)

    def config_callback(self, server):
        """Called when the TCP/IP controller outputs configuration.

        Parameters
        ----------
        server : `lsst.ts.hexrotcomm.Server`
            TCP/IP server.
        """
        self.evt_settingsApplied.set_put(
            positionAngleUpperLimit=server.config.upper_pos_limit,
            velocityLimit=server.config.velocity_limit,
            accelerationLimit=server.config.accel_limit,
            positionAngleLowerLimit=server.config.lower_pos_limit,
            followingErrorThreshold=server.config.following_error_threshold,
            trackingSuccessPositionThreshold=server.config.track_success_pos_threshold,
            trackingLostTimeout=server.config.tracking_lost_timeout,
        )

    def telemetry_callback(self, server):
        """Called when the TCP/IP controller outputs telemetry.

        Parameters
        ----------
        server : `lsst.ts.hexrotcomm.Server`
            TCP/IP server.
        """
        if self._tracking_started_n > 0:
            self._tracking_started_n -= 1
        self.evt_summaryState.set_put(summaryState=self.summary_state)
        # Strangely telemetry.state, offline_substate and enabled_substate
        # are all floats from the controller. But they should only have
        # integer value, so I output them as integers.
        self.evt_controllerState.set_put(controllerState=int(server.telemetry.state + 1),
                                         offlineSubstate=int(server.telemetry.offline_substate + 1),
                                         enabledSubstate=int(server.telemetry.enabled_substate + 1),
                                         applicationStatus=server.telemetry.application_status)

        self.tel_Application.set_put(
            Demand=server.telemetry.commanded_pos,
            Position=server.telemetry.current_pos,
            Error=server.telemetry.commanded_pos - server.telemetry.current_pos,
        )
        self.tel_Electrical.set_put(
            CopleyStatusWordDrive=[server.telemetry.status_word_drive0,
                                   server.telemetry.status_word_drive0_axis_b],
            CopleyLatchingFaultStatus=[server.telemetry.latching_fault_status_register,
                                       server.telemetry.latching_fault_status_register_axis_b],
        )
        self.tel_Motors.set_put(
            Calibrated=[server.telemetry.state_estimation_ch_a_fb,
                        server.telemetry.state_estimation_ch_b_fb],
            Raw=[server.telemetry.state_estimation_ch_a_motor_encoder,
                 server.telemetry.state_estimation_ch_b_motor_encoder],
        )

        self.evt_inPosition.set_put(
            inPosition=server.telemetry.flags_pt2pt_move_complete or server.telemetry.flags_slew_complete,
        )

        self.evt_commandableByDDS.set_put(
            state=bool(server.telemetry.application_status & enums.ApplicationState.DDS_COMMAND_SOURCE),
        )

        device_errors = []
        if server.telemetry.application_status & enums.ApplicationState.DRIVE_FAULT:
            device_errors.append("Drive Error")
        if server.telemetry.flags_following_error:
            device_errors.append("Following Error")
        if server.telemetry.application_status & enums.ApplicationState.EXTEND_LIMIT_SWITCH:
            device_errors.append("Forward Limit Switch")
        if server.telemetry.application_status & enums.ApplicationState.RETRACT_LIMIT_SWITCH:
            device_errors.append("Reverse Limit Switch")
        if server.telemetry.application_status & enums.ApplicationState.ETHERCAT_PROBLEM:
            device_errors.append("Ethercat Error")
        if server.telemetry.application_status & enums.ApplicationState.SIMULINK_FAULT:
            device_errors.append("Simulink Error")
        if server.telemetry.application_status & enums.ApplicationState.ENCODER_FAULT:
            device_errors.append("Encoder Error")
        device_error_code = ",".join(device_errors)
        self.evt_deviceError.set_put(
            code=device_error_code,
            device="Rotator",
            severity=1 if device_error_code else 0,
        )

        if server.telemetry.flags_tracking_success:
            self.evt_tracking.set_put(force_output=True)

        if server.telemetry.flags_tracking_lost:
            self.evt_trackLost.set_put(force_output=True)

        safety_interlock = server.telemetry.application_status & enums.ApplicationState.SAFTEY_INTERLOCK
        self.evt_interlock.set_put(
            detail="Engaged" if safety_interlock else "Disengaged",
        )

    @classmethod
    async def amain(cls):
        """Make a CSC from command-line arguments and run it.
        """
        parser = argparse.ArgumentParser(f"Run {cls.__name__}")
        parser.add_argument("-s", "--simulate", action="store_true",
                            help="Run in simulation mode?")

        args = parser.parse_args()
        csc = cls(simulation_mode=int(args.simulate))
        await csc.done_task
