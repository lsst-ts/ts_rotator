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
import unittest
import time

import asynctest

from lsst.ts import hexrotcomm
from lsst.ts import salobj
from lsst.ts import rotator
from lsst.ts.idl.enums import Rotator

STD_TIMEOUT = 30  # timeout for command ack


class TestRotatorCsc(hexrotcomm.BaseCscTestCase, asynctest.TestCase):
    def basic_make_csc(self, initial_state, simulation_mode=1, config_dir=None):
        return rotator.RotatorCsc(
            initial_state=initial_state,
            simulation_mode=simulation_mode,
            config_dir=config_dir,
        )

    async def test_bin_script(self):
        """Test running from the command line script.
        """
        await self.check_bin_script(
            name="MTRotator",
            index=None,
            exe_name="run_rotator.py",
            cmdline_args=["--simulate"],
        )

    async def test_standard_state_transitions(self):
        enabled_commands = (
            "configureVelocity",
            "configureAcceleration",
            "move",
            "stop",
            "trackStart",
        )
        async with self.make_csc(initial_state=salobj.State.STANDBY, simulation_mode=1):
            await self.check_standard_state_transitions(
                enabled_commands=enabled_commands
            )

    async def test_configure_acceleration(self):
        """Test the configureAcceleration command.
        """
        async with self.make_csc(initial_state=salobj.State.ENABLED, simulation_mode=1):
            data = await self.remote.evt_configuration.next(
                flush=False, timeout=STD_TIMEOUT
            )
            initial_limit = data.accelerationLimit
            print("initial_limit=", initial_limit)
            new_limit = initial_limit - 0.1
            await self.remote.cmd_configureAcceleration.set_start(
                alimit=new_limit, timeout=STD_TIMEOUT
            )
            data = await self.remote.evt_configuration.next(
                flush=False, timeout=STD_TIMEOUT
            )
            self.assertAlmostEqual(data.accelerationLimit, new_limit)

            for bad_alimit in (-1, 0, rotator.MAX_ACCEL_LIMIT + 0.001):
                with self.subTest(bad_alimit=bad_alimit):
                    with salobj.assertRaisesAckError(ack=salobj.SalRetCode.CMD_FAILED):
                        await self.remote.cmd_configureAcceleration.set_start(
                            alimit=bad_alimit, timeout=STD_TIMEOUT
                        )

    async def test_configure_velocity(self):
        """Test the configureVelocity command.
        """
        async with self.make_csc(initial_state=salobj.State.ENABLED, simulation_mode=1):
            data = await self.remote.evt_configuration.next(
                flush=False, timeout=STD_TIMEOUT
            )
            initial_limit = data.velocityLimit
            new_limit = initial_limit - 0.1
            await self.remote.cmd_configureVelocity.set_start(
                vlimit=new_limit, timeout=STD_TIMEOUT
            )
            data = await self.remote.evt_configuration.next(
                flush=False, timeout=STD_TIMEOUT
            )
            self.assertAlmostEqual(data.velocityLimit, new_limit)

            for bad_vlimit in (0, -1, rotator.MAX_VEL_LIMIT + 0.001):
                with self.subTest(bad_vlimit=bad_vlimit):
                    with salobj.assertRaisesAckError(ack=salobj.SalRetCode.CMD_FAILED):
                        await self.remote.cmd_configureVelocity.set_start(
                            vlimit=bad_vlimit, timeout=STD_TIMEOUT
                        )

    async def assert_rotation(
        self,
        demand_position=None,
        demand_velocity=None,
        demand_acceleration=None,
        actual_position=None,
        actual_velocity=None,
        tai_time=None,
    ):
        """Check the next rotation and Application telemetry messages.

        Parameters
        ----------
        demand_position : `float` or `None`
            Predicted value of demand position. Ignore if `None`.
        demand_velocity : `float` or `None`
            Predicted value of demand velocity. Ignore if `None`.
        demand_acceleration : `float` or `None`
            Predicted value of demand acceleration. Ignore if `None`.
        actual_position : `float` or `None`
            Predicted value of actual position. Ignore if `None`.
        actual_velocity : `float` or `None`
            Predicted value of actual velocity. Ignore if `None`.
        tai_time : `float` or `None`
            TAI time at which you computed demand and actual position.
            If not `None` then  adjust demand_position and actual_position for
            the reported demand velocity, demand acceleration and timestamp.
        """
        application_data = await self.remote.tel_application.next(
            flush=True, timeout=STD_TIMEOUT
        )
        rotation_data = await self.remote.tel_rotation.next(
            flush=True, timeout=STD_TIMEOUT
        )
        if tai_time is None:
            delta_tai = 0
        else:
            delta_tai = rotation_data.timestamp - tai_time

        if demand_velocity is not None:
            self.assertAlmostEqual(rotation_data.demandVelocity, demand_velocity)
        if demand_acceleration is not None:
            self.assertAlmostEqual(
                rotation_data.demandAcceleration, demand_acceleration
            )
        if demand_position is not None:
            adjusted_demand_position = (
                demand_position
                + rotation_data.demandVelocity * delta_tai
                + 0.5 * rotation_data.demandAcceleration * delta_tai ** 2
            )
            self.assertAlmostEqual(application_data.demand, adjusted_demand_position)
            self.assertAlmostEqual(
                rotation_data.demandPosition, adjusted_demand_position
            )
        if actual_velocity is not None:
            self.assertAlmostEqual(
                rotation_data.actualVelocity,
                actual_velocity,
                delta=self.csc.mock_ctrl.position_jitter,
            )
        if actual_position is not None:
            adjusted_actual_position = (
                actual_position
                + rotation_data.demandVelocity * delta_tai
                + 0.5 * rotation_data.demandAcceleration * delta_tai ** 2
            )
            self.assertAlmostEqual(
                application_data.position,
                adjusted_actual_position,
                delta=self.csc.mock_ctrl.position_jitter,
            )
            self.assertAlmostEqual(
                rotation_data.actualPosition,
                adjusted_actual_position,
                delta=self.csc.mock_ctrl.position_jitter,
            )

    async def test_move(self):
        """Test the move command for point to point motion.
        """
        destination = 2  # a small move so the test runs quickly
        # Estimated time to move; a crude estimate used for timeouts
        est_move_duration = 1

        async with self.make_csc(initial_state=salobj.State.ENABLED, simulation_mode=1):
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=Rotator.ControllerState.ENABLED,
                enabledSubstate=Rotator.EnabledSubstate.STATIONARY,
            )
            await self.assert_rotation(
                demand_position=0,
                demand_velocity=0,
                demand_acceleration=0,
                actual_position=0,
                actual_velocity=0,
            )
            data = await self.remote.evt_inPosition.next(
                flush=False, timeout=STD_TIMEOUT
            )
            self.assertFalse(data.inPosition)
            t0 = time.monotonic()
            await self.remote.cmd_move.set_start(
                position=destination, timeout=STD_TIMEOUT
            )
            data = await self.remote.evt_target.next(flush=False, timeout=STD_TIMEOUT)
            target_event_delay = time.monotonic() - t0
            self.assertAlmostEqual(data.position, destination)
            self.assertEqual(data.velocity, 0)
            target_time_difference = salobj.current_tai() - data.tai
            self.assertLessEqual(abs(target_time_difference), target_event_delay)
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=Rotator.ControllerState.ENABLED,
                enabledSubstate=Rotator.EnabledSubstate.MOVING_POINT_TO_POINT,
            )
            data = await self.remote.evt_inPosition.next(
                flush=False, timeout=STD_TIMEOUT + est_move_duration
            )
            self.assertTrue(data.inPosition)
            print(f"Move duration: {time.monotonic() - t0:0.2f} seconds")
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=Rotator.ControllerState.ENABLED,
                enabledSubstate=Rotator.EnabledSubstate.STATIONARY,
            )
            # Read a few telemetry samples to allow the
            # slew to truly finish.
            for i in range(3):
                data = await self.remote.tel_application.next(
                    flush=True, timeout=STD_TIMEOUT
                )
            await self.assert_rotation(
                demand_position=destination,
                demand_velocity=0,
                demand_acceleration=0,
                actual_position=destination,
                actual_velocity=0,
            )

    async def test_stop_move(self):
        """Test stopping a point to point move.
        """
        destination = 20  # a large move so we have plenty of time to stop
        # Estimated time to move; a crude estimate used for timeouts
        async with self.make_csc(initial_state=salobj.State.ENABLED, simulation_mode=1):
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=Rotator.ControllerState.ENABLED,
                enabledSubstate=Rotator.EnabledSubstate.STATIONARY,
            )
            data = await self.remote.tel_application.next(
                flush=True, timeout=STD_TIMEOUT
            )
            self.assertAlmostEqual(data.demand, 0)
            self.assertAlmostEqual(
                data.position, 0, delta=self.csc.mock_ctrl.position_jitter
            )
            data = await self.remote.evt_inPosition.next(
                flush=False, timeout=STD_TIMEOUT
            )
            self.assertFalse(data.inPosition)
            await self.remote.cmd_move.set_start(
                position=destination, timeout=STD_TIMEOUT
            )
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=Rotator.ControllerState.ENABLED,
                enabledSubstate=Rotator.EnabledSubstate.MOVING_POINT_TO_POINT,
            )

            # Let the move run for a short time, then stop it.
            await asyncio.sleep(0.2)
            await self.remote.cmd_stop.start(timeout=STD_TIMEOUT)
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=Rotator.ControllerState.ENABLED,
                enabledSubstate=Rotator.EnabledSubstate.STATIONARY,
            )
            data = await self.remote.tel_application.next(
                flush=True, timeout=STD_TIMEOUT
            )
            self.assertGreater(data.position, 0)
            self.assertLess(data.position, destination)

    async def test_track_good(self):
        """Test the trackStart and track commands.
        """
        pos0 = 2  # a small move so the slew ends quickly
        vel = 0.01
        # Estimated time to slew; a crude estimate used for timeouts
        est_slew_duration = 1
        async with self.make_csc(initial_state=salobj.State.ENABLED, simulation_mode=1):
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=Rotator.ControllerState.ENABLED,
                enabledSubstate=Rotator.EnabledSubstate.STATIONARY,
            )
            data = await self.remote.tel_application.next(
                flush=True, timeout=STD_TIMEOUT
            )
            self.assertAlmostEqual(data.demand, 0)
            self.assertAlmostEqual(
                data.position, 0, delta=self.csc.mock_ctrl.position_jitter
            )
            data = await self.remote.evt_inPosition.next(
                flush=False, timeout=STD_TIMEOUT
            )
            self.assertFalse(data.inPosition)
            await self.remote.cmd_trackStart.start(timeout=STD_TIMEOUT)
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=Rotator.ControllerState.ENABLED,
                enabledSubstate=Rotator.EnabledSubstate.SLEWING_OR_TRACKING,
            )
            st = time.monotonic()

            async def track():
                tai0 = salobj.current_tai()
                while True:
                    tai = salobj.current_tai()
                    dt = tai - tai0
                    pos = pos0 + vel * dt
                    await self.remote.cmd_track.set_start(
                        angle=pos, velocity=vel, tai=tai, timeout=STD_TIMEOUT
                    )
                    data = await self.remote.evt_target.next(
                        flush=False, timeout=STD_TIMEOUT
                    )
                    self.assertAlmostEqual(data.position, pos)
                    self.assertAlmostEqual(data.velocity, vel)
                    self.assertAlmostEqual(data.tai, tai)
                    data = await self.remote.tel_application.next(
                        flush=True, timeout=STD_TIMEOUT
                    )
                    tslop = salobj.current_tai() - tai
                    pos_slop = vel * tslop
                    self.assertAlmostEqual(data.demand, pos, delta=pos_slop)
                    await asyncio.sleep(0.1)

            track_task = asyncio.create_task(track())
            try:
                data = await self.remote.evt_inPosition.next(
                    flush=False, timeout=STD_TIMEOUT + est_slew_duration
                )
                self.assertTrue(data.inPosition)
                # Make sure the track task did not fail.
                if track_task.done():
                    self.fail(f"Track task failed: {track_task.exception()}")
            finally:
                track_task.cancel()

            elt = time.monotonic() - st
            print(f"Slew duration: {elt:0.2} seconds")

            await self.remote.cmd_stop.start(timeout=STD_TIMEOUT)
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=Rotator.ControllerState.ENABLED,
                enabledSubstate=Rotator.EnabledSubstate.STATIONARY,
            )
            data = await self.remote.evt_inPosition.next(
                flush=False, timeout=STD_TIMEOUT + est_slew_duration
            )
            self.assertFalse(data.inPosition)

    async def test_track_bad_values(self):
        """Test the track command with bad values.

        This should go into FAULT.
        """
        async with self.make_csc(initial_state=salobj.State.ENABLED, simulation_mode=1):
            await self.assert_next_summary_state(salobj.State.ENABLED)
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=Rotator.ControllerState.ENABLED,
                enabledSubstate=Rotator.EnabledSubstate.STATIONARY,
            )
            settings = await self.remote.evt_configuration.next(
                flush=False, timeout=STD_TIMEOUT
            )
            await self.remote.cmd_trackStart.start(timeout=STD_TIMEOUT)
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=Rotator.ControllerState.ENABLED,
                enabledSubstate=Rotator.EnabledSubstate.SLEWING_OR_TRACKING,
            )

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
                (
                    settings.positionAngleUpperLimit - 0.001,
                    settings.velocityLimit - 0.001,
                    curr_tai + 1,
                ),
            ):
                with self.subTest(pos=pos, vel=vel, tai=tai):
                    with salobj.assertRaisesAckError(ack=salobj.SalRetCode.CMD_FAILED):
                        await self.remote.cmd_track.set_start(
                            angle=pos, velocity=vel, tai=tai, timeout=STD_TIMEOUT
                        )
                    # Send a valid pvt to reset the tracking timer
                    # and give the controller time to deal with it.
                    await self.remote.cmd_track.set_start(
                        angle=0,
                        velocity=0,
                        tai=salobj.current_tai(),
                        timeout=STD_TIMEOUT,
                    )
                    await asyncio.sleep(0.01)

    async def test_track_start_no_track(self):
        """Test the trackStart command with no track command.

        This should go into FAULT.
        """
        async with self.make_csc(initial_state=salobj.State.ENABLED, simulation_mode=1):
            await self.assert_next_summary_state(salobj.State.ENABLED)
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=Rotator.ControllerState.ENABLED,
                enabledSubstate=Rotator.EnabledSubstate.STATIONARY,
            )
            data = await self.remote.tel_application.next(
                flush=True, timeout=STD_TIMEOUT
            )
            self.assertAlmostEqual(data.demand, 0)
            self.assertAlmostEqual(
                data.position, 0, delta=self.csc.mock_ctrl.position_jitter
            )
            data = await self.remote.evt_inPosition.next(
                flush=False, timeout=STD_TIMEOUT
            )
            self.assertFalse(data.inPosition)
            await self.remote.cmd_trackStart.start(timeout=STD_TIMEOUT)
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=Rotator.ControllerState.ENABLED,
                enabledSubstate=Rotator.EnabledSubstate.SLEWING_OR_TRACKING,
            )
            # Send a tracking position
            await self.remote.cmd_track.set_start(
                angle=0, velocity=0, tai=salobj.current_tai(), timeout=STD_TIMEOUT
            )
            # Wait a bit longer than usual to allow the tracking timer
            # to expire.
            await self.assert_next_summary_state(
                salobj.State.FAULT, timeout=STD_TIMEOUT + 1
            )
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=Rotator.ControllerState.FAULT,
            )

    async def test_track_start_late_track(self):
        """Test tracking with a late track command.

        Also test that we can send the track command immediately after
        the trackStart command. before the controller has time to tell
        the CSC that it is in SLEWING_OR_TRACKING mode.
        This exercises special code in the CSC that checks if the
        track command is allowed.

        After the first track command send no more.
        This should send the CSC into FAULT.
        """
        async with self.make_csc(initial_state=salobj.State.ENABLED, simulation_mode=1):
            await self.assert_next_summary_state(salobj.State.ENABLED)
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=Rotator.ControllerState.ENABLED,
                enabledSubstate=Rotator.EnabledSubstate.STATIONARY,
            )
            await self.assert_next_sample(
                topic=self.remote.evt_tracking, tracking=False, noNewCommand=False
            )
            data = await self.remote.tel_application.next(
                flush=True, timeout=STD_TIMEOUT
            )
            self.assertAlmostEqual(data.demand, 0)
            self.assertAlmostEqual(
                data.position, 0, delta=self.csc.mock_ctrl.position_jitter
            )
            data = await self.remote.evt_inPosition.next(
                flush=False, timeout=STD_TIMEOUT
            )
            self.assertFalse(data.inPosition)
            await self.remote.cmd_trackStart.start(timeout=STD_TIMEOUT)
            # Immediately send a track commands (not giving the CSC time to see
            # the changed controller state), as explained in the doc string.
            curr_tai = salobj.current_tai()
            await self.remote.cmd_track.set_start(
                angle=0, velocity=0, tai=curr_tai, timeout=STD_TIMEOUT
            )
            # Now make sure the trackStart command did send the controller
            # into the state SLEWING_OR_TRACKING
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=Rotator.ControllerState.ENABLED,
                enabledSubstate=Rotator.EnabledSubstate.SLEWING_OR_TRACKING,
            )
            await self.assert_next_sample(
                topic=self.remote.evt_tracking, tracking=True, noNewCommand=False
            )

            # Wait for the lack of track commands to send the CSC into FAULT;
            # wait a bit longer than usual to allow the tracking timer
            # to expire.
            await self.assert_next_summary_state(
                salobj.State.FAULT, timeout=STD_TIMEOUT + 1
            )
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=Rotator.ControllerState.FAULT,
            )
            await self.assert_next_sample(
                topic=self.remote.evt_tracking, tracking=False, noNewCommand=True
            )


if __name__ == "__main__":
    unittest.main()
