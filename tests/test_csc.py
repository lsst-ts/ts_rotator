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

import asyncio
import logging
import unittest
import shutil
import time

import asynctest

from lsst.ts import salobj
from lsst.ts import rotator
from lsst.ts.idl.enums import Rotator

STD_TIMEOUT = 5  # timeout for command ack
LONG_TIMEOUT = 30  # timeout for CSCs to start
NODATA_TIMEOUT = 0.1  # timeout for when we expect no new data


class TestRotatorCsc(asynctest.TestCase):
    async def setUp(self):
        salobj.test_utils.set_random_lsst_dds_domain()
        self.csc = None  # set by make_csc
        self.remote = None

    async def tearDown(self):
        close_tasks = []
        if self.csc is not None:
            close_tasks.append(self.csc.close())
        if self.remote is not None:
            close_tasks.append(self.remote.close())
        if close_tasks:
            await asyncio.wait_for(asyncio.gather(*close_tasks), timeout=STD_TIMEOUT)

    async def make_csc(self, initial_state, simulation_mode=1, wait_connected=True, log_level=logging.INFO):
        """Create a RotatorCsc and remote and wait for them to start.

        Parameters
        ----------
        initial_state : `lsst.ts.salobj.State` or `int` (optional)
            The initial state of the CSC. Ignored except in simulation mode
            because in normal operation the initial state is the current state
            of the controller.
        simulation_mode : `int` (optional)
            Simulation mode.
        wait_connected : `bool`
            If True then wait for the controller to connect.
        """
        self.csc = rotator.RotatorCsc(initial_state=initial_state, simulation_mode=simulation_mode)
        if len(self.csc.log.handlers) < 2:
            # The first handler is the salobj handler;
            # add a stream handler as well.
            self.csc.log.addHandler(logging.StreamHandler())
        self.csc.log.setLevel(log_level)
        self.remote = salobj.Remote(domain=self.csc.domain, name="Rotator")

        await asyncio.wait_for(asyncio.gather(self.csc.start_task, self.remote.start_task),
                               timeout=LONG_TIMEOUT)
        self.csc.mock_ctrl.log.setLevel(log_level)
        if wait_connected:
            for i in range(3):
                data = await self.remote.evt_connected.next(flush=False, timeout=STD_TIMEOUT)
                if data.command and data.telemetry:
                    print("Connected")
                    break

    async def assert_next_summary_state(self, state, timeout=STD_TIMEOUT):
        data = await self.remote.evt_summaryState.next(flush=False, timeout=timeout)
        self.assertEqual(data.summaryState, state)

    async def assert_next_controller_state(self, controllerState=None,
                                           offlineSubstate=None, enabledSubstate=None):
        """Wait for and check the next controllerState event.

        Parameters
        ----------
        controllerState : `lsst.ts.idl.enums.Rotator.ControllerState`
            Desired controller state.
        offlineSubstate : `lsst.ts.idl.enums.Rotator.OfflineSubstate`
            Desired offline substate.
        enabledSubstate : `lsst.ts.idl.enums.Rotator.EnabledSubstate`
            Desired enabled substate.
        """
        data = await self.remote.evt_controllerState.next(flush=False, timeout=STD_TIMEOUT)
        if controllerState is not None:
            self.assertEqual(data.controllerState, controllerState)
        if offlineSubstate is not None:
            self.assertEqual(data.offlineSubstate, offlineSubstate)
        if enabledSubstate is not None:
            self.assertEqual(data.controllerState, controllerState)

    async def test_bin_script(self):
        """Test running from the command line script.
        """
        exe_name = "run_rotator.py"
        exe_path = shutil.which(exe_name)
        if exe_path is None:
            self.fail(f"Could not find bin script {exe_name}; did you setup or install this package?")

        process = await asyncio.create_subprocess_exec(exe_name, "--simulate")
        try:
            async with salobj.Domain() as domain:
                remote = salobj.Remote(domain=domain, name="Rotator")
                summaryState_data = await remote.evt_summaryState.next(flush=False, timeout=60)
                self.assertEqual(summaryState_data.summaryState, salobj.State.OFFLINE)

        finally:
            process.terminate()

    async def test_configure_acceleration(self):
        """Test the configureAcceleration command.
        """
        await self.make_csc(initial_state=salobj.State.ENABLED)
        data = await self.remote.evt_settingsApplied.next(flush=False, timeout=STD_TIMEOUT)
        initial_limit = data.accelerationLimit
        print("initial_limit=", initial_limit)
        new_limit = initial_limit - 0.1
        await self.remote.cmd_configureAcceleration.set_start(alimit=new_limit, timeout=STD_TIMEOUT)
        data = await self.remote.evt_settingsApplied.next(flush=False, timeout=STD_TIMEOUT)
        self.assertAlmostEqual(data.accelerationLimit, new_limit)

        for bad_alimit in (-1, 0, rotator.MAX_ACCEL_LIMIT + 0.001):
            with self.subTest(bad_alimit=bad_alimit):
                with salobj.assertRaisesAckError(ack=salobj.SalRetCode.CMD_FAILED):
                    await self.remote.cmd_configureAcceleration.set_start(alimit=bad_alimit,
                                                                          timeout=STD_TIMEOUT)

    async def test_configure_velocity(self):
        """Test the configureVelocity command.
        """
        await self.make_csc(initial_state=salobj.State.ENABLED)
        data = await self.remote.evt_settingsApplied.next(flush=False, timeout=STD_TIMEOUT)
        initial_limit = data.velocityLimit
        new_limit = initial_limit - 0.1
        await self.remote.cmd_configureVelocity.set_start(vlimit=new_limit, timeout=STD_TIMEOUT)
        data = await self.remote.evt_settingsApplied.next(flush=False, timeout=STD_TIMEOUT)
        self.assertAlmostEqual(data.velocityLimit, new_limit)

        for bad_vlimit in (0, -1, rotator.MAX_VEL_LIMIT + 0.001):
            with self.subTest(bad_vlimit=bad_vlimit):
                with salobj.assertRaisesAckError(ack=salobj.SalRetCode.CMD_FAILED):
                    await self.remote.cmd_configureVelocity.set_start(vlimit=bad_vlimit, timeout=STD_TIMEOUT)

    async def test_standard_state_transitions(self):
        """Test standard CSC state transitions.

        The initial state is STANDBY.
        The standard commands and associated state transitions are:

        * start: STANDBY to DISABLED
        * enable: DISABLED to ENABLED

        * disable: ENABLED to DISABLED
        * standby: DISABLED or FAULT to STANDBY
        * exitControl: STANDBY to OFFLINE (quit)
        """
        await self.make_csc(initial_state=salobj.State.OFFLINE)
        print("CSC running")

        await self.assert_next_summary_state(salobj.State.OFFLINE)
        await self.check_bad_commands(good_commands=("enterControl", "setLogLevel"))

        # send enterControl; new state is STANDBY
        await self.remote.cmd_enterControl.start(timeout=STD_TIMEOUT)
        # Check CSC summary state directly to make sure it has changed
        # before the command is acknowledged as done.
        self.assertEqual(self.csc.summary_state, salobj.State.STANDBY)
        await self.assert_next_summary_state(salobj.State.STANDBY)
        await self.check_bad_commands(good_commands=("start", "exitControl", "setLogLevel"))

        # send start; new state is DISABLED
        await self.remote.cmd_start.start(timeout=STD_TIMEOUT)
        self.assertEqual(self.csc.summary_state, salobj.State.DISABLED)
        await self.assert_next_summary_state(salobj.State.DISABLED)
        await self.check_bad_commands(good_commands=("enable", "standby", "setLogLevel"))

        # send enable; new state is ENABLED
        await self.remote.cmd_enable.start(timeout=STD_TIMEOUT)
        self.assertEqual(self.csc.summary_state, salobj.State.ENABLED)
        await self.assert_next_summary_state(salobj.State.ENABLED)
        await self.check_bad_commands(good_commands=("disable", "setLogLevel",
                                                     "configureVelocity", "configureAcceleration",
                                                     "move", "positionSet", "stop", "trackStart"))

        # send disable; new state is DISABLED
        await self.remote.cmd_disable.start(timeout=STD_TIMEOUT)
        self.assertEqual(self.csc.summary_state, salobj.State.DISABLED)
        await self.assert_next_summary_state(salobj.State.DISABLED)

        # send standby; new state is STANDBY
        await self.remote.cmd_standby.start(timeout=STD_TIMEOUT)
        self.assertEqual(self.csc.summary_state, salobj.State.STANDBY)
        await self.assert_next_summary_state(salobj.State.STANDBY)

        # send exitControl; new state is OFFLINE
        await self.remote.cmd_exitControl.start(timeout=STD_TIMEOUT)
        self.assertEqual(self.csc.summary_state, salobj.State.OFFLINE)
        await self.assert_next_summary_state(salobj.State.OFFLINE)

    async def check_bad_commands(self, bad_commands=None, good_commands=None):
        """Check that bad commands fail.

        Parameters
        ----------
        bad_commands : `List`[`str`] or `None` (optional)
            Names of bad commands to try, or None for all commands.
        good_commands : `List`[`str`] or `None` (optional)
            Names of good commands to skip, or None to skip none.

        Notes
        -----
        If a command appears in both lists it is considered a good command.
        """
        if bad_commands is None:
            bad_commands = self.remote.salinfo.command_names
        if good_commands is None:
            good_commands = ()
        commands = self.remote.salinfo.command_names
        for command in commands:
            print(f"Try bad_command={command}")
            if command in good_commands:
                continue
            with self.subTest(command=command):
                cmd_attr = getattr(self.remote, f"cmd_{command}")
                with salobj.assertRaisesAckError(ack=salobj.SalRetCode.CMD_FAILED):
                    await cmd_attr.start(timeout=STD_TIMEOUT)

    async def test_initial_state_offline(self):
        await self.check_initial_state(salobj.State.OFFLINE)

    async def test_initial_state_standby(self):
        await self.check_initial_state(salobj.State.STANDBY)

    async def test_initial_state_disabled(self):
        await self.check_initial_state(salobj.State.DISABLED)

    async def test_initial_state_enabled(self):
        await self.check_initial_state(salobj.State.ENABLED)

    async def check_initial_state(self, initial_state):
        await self.make_csc(initial_state)
        await self.assert_next_summary_state(initial_state)

    async def test_move(self):
        """Test the positionSet and move commands for point to point motion.
        """
        destination = 2  # a small move so the test runs quickly
        # Estimated time to move; a crude estimate used for timeouts
        est_move_duration = 1
        await self.make_csc(initial_state=salobj.State.ENABLED)
        await self.assert_next_controller_state(controllerState=Rotator.ControllerState.ENABLED,
                                                enabledSubstate=Rotator.EnabledSubstate.STATIONARY)
        data = await self.remote.tel_Application.next(flush=True, timeout=STD_TIMEOUT)
        self.assertAlmostEqual(data.Demand, 0)
        self.assertAlmostEqual(data.Position, 0)
        data = await self.remote.evt_inPosition.next(flush=False, timeout=STD_TIMEOUT)
        self.assertFalse(data.inPosition)
        await self.remote.cmd_positionSet.set_start(angle=destination, timeout=STD_TIMEOUT)
        t0 = time.time()
        await self.remote.cmd_move.start(timeout=STD_TIMEOUT)
        await self.assert_next_controller_state(
            controllerState=Rotator.ControllerState.ENABLED,
            enabledSubstate=Rotator.EnabledSubstate.MOVING_POINT_TO_POINT)
        data = await self.remote.evt_inPosition.next(flush=False, timeout=STD_TIMEOUT+est_move_duration)
        self.assertTrue(data.inPosition)
        print(f"Move duration: {time.time() - t0:0.2f} seconds")
        await self.assert_next_controller_state(controllerState=Rotator.ControllerState.ENABLED,
                                                enabledSubstate=Rotator.EnabledSubstate.STATIONARY)
        data = await self.remote.tel_Application.next(flush=True, timeout=STD_TIMEOUT)
        self.assertAlmostEqual(data.Position, destination)

    async def test_stop_move(self):
        """Test stopping a point to point move.
        """
        destination = 20  # a large move so we have plenty of time to stop
        # Estimated time to move; a crude estimate used for timeouts
        await self.make_csc(initial_state=salobj.State.ENABLED)
        await self.assert_next_controller_state(controllerState=Rotator.ControllerState.ENABLED,
                                                enabledSubstate=Rotator.EnabledSubstate.STATIONARY)
        data = await self.remote.tel_Application.next(flush=True, timeout=STD_TIMEOUT)
        self.assertAlmostEqual(data.Demand, 0)
        self.assertAlmostEqual(data.Position, 0)
        data = await self.remote.evt_inPosition.next(flush=False, timeout=STD_TIMEOUT)
        self.assertFalse(data.inPosition)
        await self.remote.cmd_positionSet.set_start(angle=destination, timeout=STD_TIMEOUT)
        await self.remote.cmd_move.start(timeout=STD_TIMEOUT)
        await self.assert_next_controller_state(
            controllerState=Rotator.ControllerState.ENABLED,
            enabledSubstate=Rotator.EnabledSubstate.MOVING_POINT_TO_POINT)

        # Let the move run for a short time, then stop it.
        await asyncio.sleep(0.2)
        await self.remote.cmd_stop.start(timeout=STD_TIMEOUT)
        await self.assert_next_controller_state(controllerState=Rotator.ControllerState.ENABLED,
                                                enabledSubstate=Rotator.EnabledSubstate.STATIONARY)
        data = await self.remote.tel_Application.next(flush=True, timeout=STD_TIMEOUT)
        self.assertGreater(data.Position, 0)
        self.assertLess(data.Position, destination)

    async def test_track(self):
        """Test the trackStart and track commands.
        """
        pos0 = 2  # a small move so the slew ends quickly
        vel = 0.01
        # Estimated time to slew; a crude estimate used for timeouts
        est_slew_duration = 1
        await self.make_csc(initial_state=salobj.State.ENABLED)
        await self.assert_next_controller_state(controllerState=Rotator.ControllerState.ENABLED,
                                                enabledSubstate=Rotator.EnabledSubstate.STATIONARY)
        data = await self.remote.tel_Application.next(flush=True, timeout=STD_TIMEOUT)
        self.assertAlmostEqual(data.Demand, 0)
        self.assertAlmostEqual(data.Position, 0)
        data = await self.remote.evt_inPosition.next(flush=False, timeout=STD_TIMEOUT)
        self.assertFalse(data.inPosition)
        await self.remote.cmd_trackStart.start(timeout=STD_TIMEOUT)
        await self.assert_next_controller_state(
            controllerState=Rotator.ControllerState.ENABLED,
            enabledSubstate=Rotator.EnabledSubstate.SLEWING_OR_TRACKING)
        st = time.time()

        async def track():
            while True:
                t = salobj.current_tai()
                dt = t - t0
                pos = pos0 + vel*dt
                await self.remote.cmd_track.set_start(angle=pos, velocity=vel, tai=t, timeout=STD_TIMEOUT)
                data = await self.remote.tel_Application.next(flush=True, timeout=STD_TIMEOUT)
                self.assertAlmostEqual(pos, data.Demand)
                await asyncio.sleep(0.1)

        t0 = salobj.current_tai()
        track_task = asyncio.create_task(track())
        try:
            data = await self.remote.evt_inPosition.next(flush=False, timeout=STD_TIMEOUT+est_slew_duration)
            self.assertTrue(data.inPosition)
        finally:
            track_task.cancel()

        elt = time.time() - st
        print(f"Slew duration: {elt:0.2} seconds")

        await self.remote.cmd_stop.start(timeout=STD_TIMEOUT)
        await self.assert_next_controller_state(
            controllerState=Rotator.ControllerState.ENABLED,
            enabledSubstate=Rotator.EnabledSubstate.STATIONARY)
        data = await self.remote.evt_inPosition.next(flush=False, timeout=STD_TIMEOUT+est_slew_duration)
        self.assertFalse(data.inPosition)

    def test_bad_simulation_modes(self):
        """Test simulation_mode argument of TestCsc constructor.

        The only allowed values are 0 and 1
        """
        for simulation_mode in (-1, 2, 3):
            with self.assertRaises(ValueError):
                rotator.RotatorCsc(simulation_mode=simulation_mode)

    async def test_track_bad_values(self):
        """Test the track command with bad values.

        This should go into FAULT.
        """
        await self.make_csc(initial_state=salobj.State.ENABLED)
        await self.assert_next_summary_state(salobj.State.ENABLED)
        await self.assert_next_controller_state(controllerState=Rotator.ControllerState.ENABLED,
                                                enabledSubstate=Rotator.EnabledSubstate.STATIONARY)
        settings = await self.remote.evt_settingsApplied.next(flush=False, timeout=STD_TIMEOUT)
        await self.remote.cmd_trackStart.start(timeout=STD_TIMEOUT)
        await self.assert_next_controller_state(
            controllerState=Rotator.ControllerState.ENABLED,
            enabledSubstate=Rotator.EnabledSubstate.SLEWING_OR_TRACKING)

        # Run these quickly enough and the controller will still be enabled
        curr_tai = salobj.current_tai()
        for pos, vel, tai in (
            # Position out of range.
            (settings.positionAngleLowerLimit - 0.001, 0, curr_tai),
            (settings.positionAngleUpperLimit + 0.001, 0, curr_tai),
            # Velocity out of range.
            (0, settings.velocityLimit + 0.001, curr_tai),
            # Current position and velocity OK but the position
            # at the specified tai is out of bounds.
            (settings.positionAngleUpperLimit - 0.001, settings.velocityLimit - 0.001, curr_tai + 1),
        ):
            with self.subTest(pos=pos, vel=vel, tai=tai):
                with salobj.assertRaisesAckError(ack=salobj.SalRetCode.CMD_FAILED):
                    await self.remote.cmd_track.set_start(angle=pos, velocity=vel, tai=tai,
                                                          timeout=STD_TIMEOUT)
                # Send a valid pvt to reset the tracking timer
                # and give the controller time to deal with it.
                await self.remote.cmd_track.set_start(angle=0, velocity=0, tai=curr_tai,
                                                      timeout=STD_TIMEOUT)
                await asyncio.sleep(0.01)

    async def test_track_start_no_track(self):
        """Test the trackStart command with no track command.

        This should go into FAULT.
        """
        await self.make_csc(initial_state=salobj.State.ENABLED)
        await self.assert_next_summary_state(salobj.State.ENABLED)
        await self.assert_next_controller_state(controllerState=Rotator.ControllerState.ENABLED,
                                                enabledSubstate=Rotator.EnabledSubstate.STATIONARY)
        data = await self.remote.tel_Application.next(flush=True, timeout=STD_TIMEOUT)
        self.assertAlmostEqual(data.Demand, 0)
        self.assertAlmostEqual(data.Position, 0)
        data = await self.remote.evt_inPosition.next(flush=False, timeout=STD_TIMEOUT)
        self.assertFalse(data.inPosition)
        await self.remote.cmd_trackStart.start(timeout=STD_TIMEOUT)
        await self.assert_next_controller_state(
            controllerState=Rotator.ControllerState.ENABLED,
            enabledSubstate=Rotator.EnabledSubstate.SLEWING_OR_TRACKING)
        # Wait a bit longer than usual to allow the tracking timer to expire.
        await self.assert_next_summary_state(salobj.State.FAULT, timeout=STD_TIMEOUT+1)
        await self.assert_next_controller_state(controllerState=Rotator.ControllerState.FAULT)

    async def test_track_start_late_track(self):
        """Test the trackStart command with a late track command.

        Also test that we can send the track command immediately after
        the trackStart command. before the controller has time to tell
        the CSC that it is in SLEWING_OR_TRACKING mode.
        This exercises special code in the CSC that checks if the
        track command is allowed.

        After the first track command send no more.
        This should send the CSC into FAULT.
        """
        await self.make_csc(initial_state=salobj.State.ENABLED)
        await self.assert_next_summary_state(salobj.State.ENABLED)
        await self.assert_next_controller_state(controllerState=Rotator.ControllerState.ENABLED,
                                                enabledSubstate=Rotator.EnabledSubstate.STATIONARY)
        data = await self.remote.tel_Application.next(flush=True, timeout=STD_TIMEOUT)
        self.assertAlmostEqual(data.Demand, 0)
        self.assertAlmostEqual(data.Position, 0)
        data = await self.remote.evt_inPosition.next(flush=False, timeout=STD_TIMEOUT)
        self.assertFalse(data.inPosition)
        await self.remote.cmd_trackStart.start(timeout=STD_TIMEOUT)
        # Immediately send a track commands (not giving the CSC time to see
        # the changed controller state), as explained in the doc string.
        curr_tai = salobj.current_tai()
        await self.remote.cmd_track.set_start(angle=0, velocity=0, tai=curr_tai, timeout=STD_TIMEOUT)
        # Now make sure the trackStart command did send the controller
        # into the state SLEWING_OR_TRACKING
        await self.assert_next_controller_state(
            controllerState=Rotator.ControllerState.ENABLED,
            enabledSubstate=Rotator.EnabledSubstate.SLEWING_OR_TRACKING)
        # Wait for the lack of track commands to send the CSC into FAULT;
        # wait a bit longer than usual to allow the tracking timer to expire.
        await self.assert_next_summary_state(salobj.State.FAULT, timeout=STD_TIMEOUT+1)
        await self.assert_next_controller_state(controllerState=Rotator.ControllerState.FAULT)

    async def test_non_simulation_mode(self):
        ignored_initial_state = salobj.State.FAULT
        async with rotator.RotatorCsc(simulation_mode=0, initial_state=ignored_initial_state) as csc:
            self.assertIsNone(csc.mock_ctrl)
            await asyncio.sleep(0.2)
            self.assertFalse(csc.server.command_connected)
            self.assertFalse(csc.server.telemetry_connected)
            self.assertFalse(csc.server.connected)


if __name__ == "__main__":
    unittest.main()
