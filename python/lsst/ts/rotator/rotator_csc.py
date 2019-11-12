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

__all__ = ["RotatorCsc"]

import argparse

from lsst.ts import salobj
from lsst.ts import hexrotcomm
from lsst.ts.idl.enums import Rotator
from . import constants
from . import enums
from . import structs
from . import mock_controller


class RotatorCsc(hexrotcomm.BaseCsc):
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

    def __init__(self, initial_state=salobj.State.OFFLINE, simulation_mode=0):
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
        self._tracking_started_telemetry_counter = 0
        self._prev_flags_tracking_success = False
        self._prev_flags_tracking_lost = False

        super().__init__(name="Rotator",
                         index=0,
                         sync_pattern=constants.ROTATOR_SYNC_PATTERN,
                         CommandCode=enums.CommandCode,
                         ConfigClass=structs.Config,
                         TelemetryClass=structs.Telemetry,
                         initial_state=initial_state,
                         simulation_mode=simulation_mode)

    async def start(self):
        await super().start()
        self.evt_inPosition.set_put(inPosition=False, force_output=True)

    # Rotator-specific commands.
    async def do_configureAcceleration(self, data):
        """Specify the acceleration limit."""
        self.assert_enabled_substate(Rotator.EnabledSubstate.STATIONARY)
        if not 0 < data.alimit <= constants.MAX_ACCEL_LIMIT:
            raise salobj.ExpectedError(f"alimit={data.alimit} must be > 0 and <= {constants.MAX_ACCEL_LIMIT}")
        await self.run_command(cmd=enums.CommandCode.CONFIG_ACCEL,
                               param1=data.alimit)

    async def do_configureVelocity(self, data):
        """Specify the velocity limit."""
        self.assert_enabled_substate(Rotator.EnabledSubstate.STATIONARY)
        if not 0 < data.vlimit <= constants.MAX_VEL_LIMIT:
            raise salobj.ExpectedError(f"vlimit={data.vlimit} must be > 0 and <= {constants.MAX_VEL_LIMIT}")
        await self.run_command(cmd=enums.CommandCode.CONFIG_VEL,
                               param1=data.vlimit)

    async def do_move(self, data):
        """Go to the position specified by the most recent ``positionSet``
        command.
        """
        self.assert_enabled_substate(Rotator.EnabledSubstate.STATIONARY)
        await self.run_command(cmd=enums.CommandCode.SET_ENABLED_SUBSTATE,
                               param1=enums.SetEnabledSubstateParam.MOVE_POINT_TO_POINT)

    async def do_moveConstantVelocity(self, data):
        """Move at the speed and for the duration specified by the most recent
        ``velocitySet`` command. NOT SUPPORTED.
        """
        raise salobj.ExpectedError("Not implemented")
        # self.assert_enabled_substate(Rotator.EnabledSubstate.STATIONARY)
        # await self.run_command(cmd=enums.CommandCode.SET_ENABLED_SUBSTATE,
        #                        param1=enums.SetEnabledSubstateParam.CONSTANT_VELOCITY)

    async def do_positionSet(self, data):
        """Specify a position for the ``move`` command.
        """
        self.assert_enabled_substate(Rotator.EnabledSubstate.STATIONARY)
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
        await self.run_command(cmd=enums.CommandCode.SET_ENABLED_SUBSTATE,
                               param1=enums.SetEnabledSubstateParam.STOP)

    async def do_test(self, data):
        """Execute the test command. NOT SUPPORTED.
        """
        raise salobj.ExpectedError("Not implemented")
        # self.assert_enabled_substate(Rotator.EnabledSubstate.STATIONARY)
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
        if self.server.telemetry.enabled_substate != Rotator.EnabledSubstate.SLEWING_OR_TRACKING:
            if self._tracking_started_telemetry_counter <= 0:
                raise salobj.ExpectedError("Low-level controller in substate "
                                           f"{self.server.telemetry.enabled_substate} "
                                           f"instead of {Rotator.EnabledSubstate.SLEWING_OR_TRACKING}")
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
        self.assert_enabled_substate(Rotator.EnabledSubstate.STATIONARY)
        await self.run_command(cmd=enums.CommandCode.SET_ENABLED_SUBSTATE,
                               param1=enums.SetEnabledSubstateParam.TRACK)
        self._tracking_started_telemetry_counter = 2

    async def do_velocitySet(self, data):
        """Specify the velocity and duration for the ``moveConstantVelocity``
        command. NOT SUPPORTED.
        """
        raise salobj.ExpectedError("Not implemented")
        # self.assert_enabled_substate(Rotator.EnabledSubstate.STATIONARY)
        # if abs(data.velocity) > self.server.config.velocity_limit:
        #     raise salobj.ExpectedError(f"Velocity {data.velocity} > "
        #              f"limit {self.server.config.velocity_limit}")
        # await self.run_command(cmd=enums.CommandCode.SET_CONSTANT_VEL,
        #                        param1=data.velocity,
        #                        param2=data.moveDuration)

    def config_callback(self, server):
        """Called when the TCP/IP controller outputs configuration.

        Parameters
        ----------
        server : `lsst.ts.hexrotcomm.CommandTelemetryServer`
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
        server : `lsst.ts.hexrotcomm.CommandTelemetryServer`
            TCP/IP server.
        """
        if self._tracking_started_telemetry_counter > 0:
            self._tracking_started_telemetry_counter -= 1
        self.evt_summaryState.set_put(summaryState=self.summary_state)
        # Strangely telemetry.state, offline_substate and enabled_substate
        # are all floats from the controller. But they should only have
        # integer value, so I output them as integers.
        self.evt_controllerState.set_put(controllerState=int(server.telemetry.state),
                                         offlineSubstate=int(server.telemetry.offline_substate),
                                         enabledSubstate=int(server.telemetry.enabled_substate),
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
            state=bool(server.telemetry.application_status & Rotator.ApplicationStatus.DDS_COMMAND_SOURCE),
        )

        device_errors = []
        if server.telemetry.application_status & Rotator.ApplicationStatus.DRIVE_FAULT:
            device_errors.append("Drive Error")
        if server.telemetry.flags_following_error:
            device_errors.append("Following Error")
        if server.telemetry.application_status & Rotator.ApplicationStatus.EXTEND_LIMIT_SWITCH:
            device_errors.append("Forward Limit Switch")
        if server.telemetry.application_status & Rotator.ApplicationStatus.RETRACT_LIMIT_SWITCH:
            device_errors.append("Reverse Limit Switch")
        if server.telemetry.application_status & Rotator.ApplicationStatus.ETHERCAT_PROBLEM:
            device_errors.append("Ethercat Error")
        if server.telemetry.application_status & Rotator.ApplicationStatus.SIMULINK_FAULT:
            device_errors.append("Simulink Error")
        if server.telemetry.application_status & Rotator.ApplicationStatus.ENCODER_FAULT:
            device_errors.append("Encoder Error")
        device_error_code = ",".join(device_errors)
        self.evt_deviceError.set_put(
            code=device_error_code,
            device="Rotator",
            severity=1 if device_error_code else 0,
        )

        if server.telemetry.flags_tracking_success != self._prev_flags_tracking_success:
            self._prev_flags_tracking_success = server.telemetry.flags_tracking_success
            if server.telemetry.flags_tracking_success:
                self.evt_tracking.put()

        if server.telemetry.flags_tracking_lost != self._prev_flags_tracking_lost:
            self._prev_flags_tracking_lost = server.telemetry.flags_tracking_lost
            if server.telemetry.flags_tracking_lost:
                self.evt_trackLost.set_put()

        safety_interlock = server.telemetry.application_status & Rotator.ApplicationStatus.SAFTEY_INTERLOCK
        self.evt_interlock.set_put(
            detail="Engaged" if safety_interlock else "Disengaged",
        )

    def make_mock_controller(self, initial_ctrl_state):
        return mock_controller.MockMTRotatorController(
            log=self.log,
            host=self.server.host,
            initial_state=initial_ctrl_state,
            command_port=self.server.command_port,
            telemetry_port=self.server.telemetry_port)

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
