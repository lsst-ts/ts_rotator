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

__all__ = ["MockMTRotatorController"]

import asyncio
import time

from lsst.ts import salobj
from lsst.ts import hexrotcomm
from . import constants
from . import enums
from . import structs

# Maximum time between track commands (seconds)
# The real controller may use 0.15
TRACK_TIMEOUT = 1


class Actuator:
    """Model an actuator that moves between given limits at constant velocity.

    Information is computed on request. This works because the system being
    modeled can only be polled.
    """
    def __init__(self, min_pos, max_pos, pos, speed):
        assert speed > 0
        self.min_pos = min_pos
        self.max_pos = max_pos
        self._start_pos = pos
        self._start_time = time.time()
        self._end_pos = pos
        self._end_time = time.time()
        self.speed = speed

    @property
    def start_pos(self):
        """Start position of move."""
        return self._start_pos

    @property
    def end_pos(self):
        """End position of move."""
        return self._end_pos

    def set_pos(self, pos):
        """Set a new desired position."""
        if pos < self.min_pos or pos > self.max_pos:
            raise ValueError(f"pos={pos} not in range [{self.min_pos}, {self.max_pos}]")
        self._start_pos = self.curr_pos
        self._start_time = time.time()
        self._end_pos = pos
        dtime = self._move_duration()
        self._end_time = self._start_time + dtime

    def _move_duration(self):
        return abs(self.end_pos - self.start_pos) / self.speed

    @property
    def curr_pos(self):
        """Current position."""
        curr_time = time.time()
        if curr_time > self._end_time:
            return self.end_pos
        else:
            dtime = curr_time - self._start_time
            return self.start_pos + self.direction*self.speed*dtime

    @property
    def direction(self):
        """1 if moving or moved to greater position, -1 otherwise."""
        return 1 if self.end_pos >= self.start_pos else -1

    @property
    def moving(self):
        """Is the axis moving?"""
        return time.time() < self._end_time

    def stop(self):
        """Stop motion instantly.

        Set start_pos and end_pos to the current position
        and start_time and end_time to the current time.
        """
        curr_pos = self.curr_pos
        curr_time = time.time()
        self._start_pos = curr_pos
        self._start_time = curr_time
        self._end_pos = curr_pos
        self._end_time = curr_time

    @property
    def remaining_time(self):
        """Remaining time for this move (sec)."""
        duration = self._end_time - time.time()
        return max(duration, 0)


class MockMTRotatorController(hexrotcomm.BaseMockController):
    """Mock MT rotator controller that talks over TCP/IP.

    Parameters
    ----------
    log : `logging.Logger`
        Logger.
    command_port : `int` (optional)
        Command socket port.  This argument is intended for unit tests;
        use the default value for normal operation.
    telemetry_port : `int` (optional)
        Telemetry socket port. This argument is intended for unit tests;
        use the default value for normal operation.
    initial_state : `enums.ControllerState` (optional)
        Initial state of mock controller.

    Notes
    -----
    To start the mock controller:

        ctrl = MockRotatorController(...)
        await ctrl.connect_task

    To stop the server:

        await ctrl.stop()

    *Known Limitations*

    * Constant-velocity motion is not supported.
    * The path generator is very primitive; acceleration is presently
      instantaneous. This could be improved by using code from
      the ATMCS simulator.
    * I am not sure what the real controller does if a track command is late;
      for now the simulator goes into FAULT.
    * I am not sure if the real controller checks that a position was set
      before accepting the MOVE_POINT_TO_POINT command:
      for now the simulator does not check.
    """
    def __init__(self, log, command_port, telemetry_port,
                 initial_state=enums.ControllerState.OFFLINE):
        self.encoder_resolution = 200_000  # counts/deg; arbitrary
        config = structs.Config()
        config.velocity_limit = 3  # deg/sec
        config.accel_limit = 1  # deg/sec^2
        config.pos_error_threshold = 0.1  # deg
        config.upper_pos_limit = 360  # deg
        config.lower_pos_limit = -360  # deg
        config.following_error_threshold = 0.1  # deg
        config.track_success_pos_threshold = 0.01  # deg
        config.tracking_lost_timeout = 5  # sec
        telemetry = structs.Telemetry()
        self.tracking_timer_task = salobj.make_done_future()

        self.rotator = Actuator(min_pos=config.lower_pos_limit,
                                max_pos=config.upper_pos_limit,
                                pos=0,
                                speed=config.velocity_limit)
        # Set True when the first TRACK_VEL_CMD command is received
        # and False when not tracking
        self.track_vel_cmd_seen = False
        super().__init__(log=log, config=config, telemetry=telemetry,
                         command_port=command_port, telemetry_port=telemetry_port)
        import logging
        self.log.setLevel(logging.DEBUG)
        self.set_state(initial_state)

    @property
    def state(self):
        return self.telemetry.state

    @property
    def offline_substate(self):
        return self.telemetry.offline_substate

    @property
    def enabled_substate(self):
        return self.telemetry.enabled_substate

    async def close(self):
        """Kill command and telemetry tasks and close the connections.

        Always safe to call.
        """
        self.rotator.stop()
        self.tracking_timer_task.cancel()
        await super().close()

    async def run_command(self, command):
        self.log.debug(f"run_command: command={enums.CommandCode(command.cmd)!r}; "
                       f"param1={command.param1}; param2={command.param2}; param3={command.param3}")
        if self.state == enums.ControllerState.OFFLINE:
            if command.cmd == enums.CommandCode.SET_STATE and \
                    command.param1 == enums.SetStateParam.ENTER_CONTROL:
                self.set_state(enums.ControllerState.STANDBY)
            elif command.cmd == enums.CommandCode.SET_STATE and \
                    command.param1 == enums.SetStateParam.CLEAR_ERROR:
                # silently ignore extra CLEAR_ERROR commands
                pass
            else:
                self.log_rejected_command(command)
        elif self.state == enums.ControllerState.STANDBY:
            if command.cmd == enums.CommandCode.SET_STATE and command.param1 == enums.SetStateParam.START:
                self.set_state(enums.ControllerState.DISABLED)
            elif command.cmd == enums.CommandCode.SET_STATE and command.param1 == enums.SetStateParam.EXIT:
                self.set_state(enums.ControllerState.OFFLINE)
            else:
                self.log_rejected_command(command)
        elif self.state == enums.ControllerState.DISABLED:
            if command.cmd == enums.CommandCode.SET_STATE and command.param1 == enums.SetStateParam.ENABLE:
                self.set_state(enums.ControllerState.ENABLED)
            elif command.cmd == enums.CommandCode.SET_STATE and command.param1 == enums.SetStateParam.STANDBY:
                self.set_state(enums.ControllerState.STANDBY)
            else:
                self.log_rejected_command(command)
        elif self.state == enums.ControllerState.ENABLED:
            if command.cmd == enums.CommandCode.SET_STATE and command.param1 == enums.SetStateParam.DISABLE:
                self.set_state(enums.ControllerState.DISABLED)
            elif self.telemetry.enabled_substate == enums.EnabledSubstate.STATIONARY:
                if command.cmd == enums.CommandCode.POSITION_SET:
                    self.telemetry.set_pos = command.param1
                elif command.cmd == enums.CommandCode.CONFIG_VEL:
                    if 0 < command.param1 <= constants.MAX_VEL_LIMIT:
                        self.config.velocity_limit = command.param1
                    else:
                        self.log.error(f"Requested velocity limit {command.param1} "
                                       f"not in range (0, {constants.MAX_VEL_LIMIT}]")
                    await self.write_config()
                elif command.cmd == enums.CommandCode.CONFIG_ACCEL:
                    if 0 < command.param1 <= constants.MAX_ACCEL_LIMIT:
                        self.config.accel_limit = command.param1
                    else:
                        self.log.error(f"Requested accel limit {command.param1} "
                                       f"not in range (0, {constants.MAX_ACCEL_LIMIT}]")
                    await self.write_config()
                elif command.cmd == enums.CommandCode.SET_CONSTANT_VEL:
                    self.log.error("Mock controller does not support constant velocity motion.")
                elif command.cmd == enums.CommandCode.SET_SUBSTATE:
                    if command.param1 == enums.EnabledSetSubstateParam.TRACK:
                        self.telemetry.enabled_substate = enums.EnabledSubstate.SLEWING_OR_TRACKING
                        self.tracking_timer_task.cancel()
                        self.tracking_timer_task = asyncio.create_task(self.tracking_timer())
                    elif command.param1 == enums.EnabledSetSubstateParam.STOP:
                        self.rotator.stop()
                        self.tracking_timer_task.cancel()
                        self.telemetry.enabled_substate = enums.EnabledSubstate.STATIONARY
                    elif command.param1 == enums.EnabledSetSubstateParam.MOVE_POINT_TO_POINT:
                        self.rotator.set_pos(self.telemetry.set_pos)
                        self.telemetry.enabled_substate = enums.EnabledSubstate.MOVING_POINT_TO_POINT
                        self.telemetry.flags_pt2pt_move_complete = 0
                    else:
                        self.log_rejected_command(command)
                else:
                    self.log_rejected_command(command)
            elif self.telemetry.enabled_substate == enums.EnabledSubstate.MOVING_POINT_TO_POINT:
                if command.cmd == enums.CommandCode.SET_SUBSTATE and \
                        command.param1 == enums.EnabledSetSubstateParam.STOP:
                    self.rotator.stop()
                    self.tracking_timer_task.cancel()
                    self.telemetry.enabled_substate = enums.EnabledSubstate.STATIONARY
                else:
                    self.log_rejected_command(command)
            elif self.telemetry.enabled_substate == enums.EnabledSubstate.SLEWING_OR_TRACKING:
                if command.cmd == enums.CommandCode.TRACK_VEL_CMD:
                    tai, pos, vel = command.param1, command.param2, command.param3
                    dt = salobj.current_tai() - tai
                    curr_pos = pos + vel*dt
                    if self.config.lower_pos_limit <= curr_pos <= self.config.upper_pos_limit:
                        self.rotator.set_pos(curr_pos)
                        self.tracking_timer_task.cancel()
                        self.tracking_timer_task = asyncio.create_task(self.tracking_timer())
                        self.track_vel_cmd_seen = True
                    else:
                        self.log.error(f"fault: commanded position {curr_pos} not in range "
                                       f"[{self.config.lower_pos_limit}, {self.config.upper_pos_limit}]")
                        self.set_state(enums.ControllerState.FAULT)
                elif command.cmd == enums.CommandCode.SET_SUBSTATE and \
                        command.param1 == enums.EnabledSetSubstateParam.STOP:
                    self.rotator.stop()
                    self.tracking_timer_task.cancel()
                    self.telemetry.enabled_substate = enums.EnabledSubstate.STATIONARY
                else:
                    self.log_rejected_command(command)
            else:
                self.log_rejected_command(command)
        elif self.state == enums.ControllerState.FAULT:
            if command.cmd == enums.CommandCode.SET_STATE and \
                    command.param1 == enums.SetStateParam.CLEAR_ERROR:
                self.set_state(enums.ControllerState.OFFLINE)
                self.log_rejected_command(command)
        else:
            self.log.error(command, reason=f"Unsupported state {self.telemetry.state}")

    def set_state(self, state):
        """Set the current state and substates.

        Parameters
        ----------
        state : `enums.ControllerState` or `int`
            New state.

        Notes
        -----
        Sets the substates as follows:

        * `enums.OfflineSubstate.AVAILABLE`
          if state == `enums.ControllerState.OFFLINE`
        * `enums.EnabledSubstate.STATIONARY`
          if state == `enums.ControllerState.ENABLED`

        The real controller goes to substate
        `enums.OfflineSubstate.PUBLISH_ONLY` when going offline, but requires
        the engineering user interface (EUI) to get out of that state,
        and we don't have an EUI for the mock controller!
        """
        self.telemetry.state = enums.ControllerState(state)
        self.telemetry.offline_substate = enums.OfflineSubstate.AVAILABLE \
            if self.telemetry.state == enums.ControllerState.OFFLINE else 0
        self.telemetry.enabled_substate = enums.EnabledSubstate.STATIONARY \
            if self.telemetry.state == enums.ControllerState.ENABLED else 0
        self.log.debug("set_state: state={enums.ControllerState(self.telemetry.state)!r}; "
                       f"offline_substate={enums.OfflineSubstate(self.telemetry.offline_substate)}; "
                       f"enabled_substate={enums.EnabledSubstate(self.telemetry.enabled_substate)}")

    def log_rejected_command(self, command, reason=None):
        """Log an error message describing a rejected command."""
        if reason is None:
            reason = f"the current state is {self.telemetry.state}, " \
                     f"enabled_substate={self.telemetry.enabled_substate}"
        self.log.error(f"Ignoring command cmd={command.cmd}, "
                       f"param1={command.param1}, param2={command.param2}: {reason}")

    async def tracking_timer(self):
        """If this times out then go into a FAULT state.

        Used to make sure TRACK commands arrive often enough.
        """
        await asyncio.sleep(TRACK_TIMEOUT)
        self.log.error("Tracking timer expired; going to FAULT")
        self.set_state(enums.ControllerState.FAULT)

    async def update_telemetry(self):
        try:
            curr_pos = self.rotator.curr_pos
            curr_pos_counts = self.encoder_resolution * curr_pos
            cmd_pos = self.rotator.end_pos
            in_position = abs(curr_pos - cmd_pos) < self.config.track_success_pos_threshold
            self.telemetry.biss_motor_encoder_axis_a = int(curr_pos_counts)
            self.telemetry.biss_motor_encoder_axis_b = int(curr_pos_counts)
            self.telemetry.status_word_drive0 = 0
            self.telemetry.status_word_drive0_axis_b = 0
            self.telemetry.status_word_drive1 = 0
            self.telemetry.latching_fault_status_register = 0
            self.telemetry.latching_fault_status_register_axis_b = 0
            self.telemetry.input_pin_states = 0
            self.telemetry.actual_torque_axis_a = 0
            self.telemetry.actual_torque_axis_b = 0
            self.telemetry.copley_fault_status_register = (0, 0)
            self.telemetry.application_status = 1
            self.telemetry.commanded_pos = cmd_pos
            if self.telemetry.current_pos != self.rotator.curr_pos:
                self.log.debug(f"update_telemetry: curr_pos={curr_pos:0.2f}; cmd_pos={cmd_pos:0.2f}; "
                               f"in_position={in_position}")
            self.telemetry.current_pos = self.rotator.curr_pos
            self.telemetry.set_pos = self.rotator.end_pos
            if self.telemetry.state != enums.ControllerState.ENABLED:
                self.telemetry.enabled_substate = enums.EnabledSubstate.STATIONARY
            if self.telemetry.state != enums.ControllerState.OFFLINE:
                self.telemetry.offline_substate = enums.OfflineSubstate.AVAILABLE
            if self.telemetry.state == enums.ControllerState.ENABLED and \
                    self.telemetry.enabled_substate == enums.EnabledSubstate.SLEWING_OR_TRACKING:
                self.telemetry.flags_slew_complete = self.track_vel_cmd_seen and in_position
            else:
                self.telemetry.flags_slew_complete = 0
                self.track_vel_cmd_seen = False
            if self.telemetry.state == enums.ControllerState.ENABLED and \
                    self.telemetry.enabled_substate == enums.EnabledSubstate.MOVING_POINT_TO_POINT and \
                    in_position:
                self.telemetry.flags_pt2pt_move_complete = 1
                self.telemetry.enabled_substate = enums.EnabledSubstate.STATIONARY
            else:
                self.telemetry.flags_pt2pt_move_complete = 0
            self.telemetry.flags_stop_complete = 1
            self.telemetry.flags_following_error = 0.001
            self.telemetry.flags_move_success = 1
            self.telemetry.flags_tracking_success = in_position
            self.telemetry.flags_position_feedback_fault = 0
            self.telemetry.flags_tracking_lost = 0
            self.telemetry.state_estimation_ch_a_fb = 0
            self.telemetry.state_estimation_ch_b_fb = 0
            self.telemetry.state_estimation_ch_a_motor_encoder = 0
            self.telemetry.state_estimation_ch_b_motor_encoder = 0
            self.telemetry.rotator_pos_deg = curr_pos
        except Exception:
            self.log.exception("update_telemetry failed; output incomplete telemetry")
