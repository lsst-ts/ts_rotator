# This file is part of ts_mtrotator.
#
# Developed for the Rubin Observatory Telescope and Site System.
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
from lsst.ts import mtrotator
from lsst.ts.idl.enums.MTRotator import ControllerState, EnabledSubstate

STD_TIMEOUT = 30  # timeout for command ack


class TestRotatorCsc(hexrotcomm.BaseCscTestCase, asynctest.TestCase):
    def basic_make_csc(self, initial_state, simulation_mode=1, config_dir=None):
        return mtrotator.RotatorCsc(
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
            exe_name="run_mtrotator.py",
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

            for bad_alimit in (-1, 0, mtrotator.MAX_ACCEL_LIMIT + 0.001):
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

            for bad_vlimit in (0, -1, mtrotator.MAX_VEL_LIMIT + 0.001):
                with self.subTest(bad_vlimit=bad_vlimit):
                    with salobj.assertRaisesAckError(ack=salobj.SalRetCode.CMD_FAILED):
                        await self.remote.cmd_configureVelocity.set_start(
                            vlimit=bad_vlimit, timeout=STD_TIMEOUT
                        )

    async def check_rotation(self):
        """Check the next rotation telemetry messages.
        """
        # We need some margin, slightly more than I naively expected,
        # possibly because there is some roundoff error in converting time
        # between TAI unix seconds and astropy times.
        slop = 1e-6

        rotation_data = await self.remote.tel_rotation.next(
            flush=True, timeout=STD_TIMEOUT
        )
        path_segment = self.csc.mock_ctrl.rotator.path.at(rotation_data.timestamp)
        # Print the values to get some idea of how much slop is needed
        # and so it is easier to determine which test failed
        # in the tracking task in test_track_good.
        print(
            f"delta_pos={abs(rotation_data.demandPosition - path_segment.position):0.8f}"
        )
        print(
            f"delta_vel={abs(rotation_data.demandVelocity - path_segment.velocity):0.8f}"
        )
        print(
            f"delta_acc={abs(rotation_data.demandAcceleration - path_segment.acceleration):0.8f}"
        )
        self.assertAlmostEqual(
            rotation_data.demandPosition, path_segment.position, delta=slop
        )
        self.assertAlmostEqual(
            rotation_data.demandVelocity, path_segment.velocity, delta=slop
        )
        self.assertAlmostEqual(
            rotation_data.demandAcceleration, path_segment.acceleration, delta=slop
        )
        # actualPosition has jitter but actualVelocity does not.
        self.assertAlmostEqual(
            rotation_data.actualPosition,
            path_segment.position,
            delta=slop + self.csc.mock_ctrl.position_jitter,
        )
        self.assertAlmostEqual(
            rotation_data.actualVelocity, path_segment.velocity, delta=slop
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
                controllerState=ControllerState.ENABLED,
                enabledSubstate=EnabledSubstate.STATIONARY,
            )
            await self.check_rotation()
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
                controllerState=ControllerState.ENABLED,
                enabledSubstate=EnabledSubstate.MOVING_POINT_TO_POINT,
            )
            data = await self.remote.evt_inPosition.next(
                flush=False, timeout=STD_TIMEOUT + est_move_duration
            )
            self.assertTrue(data.inPosition)
            print(f"Move duration: {time.monotonic() - t0:0.2f} seconds")
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=ControllerState.ENABLED,
                enabledSubstate=EnabledSubstate.STATIONARY,
            )
            await self.check_rotation()

    async def test_stop_move(self):
        """Test stopping a point to point move.
        """
        destination = 20  # a large move so we have plenty of time to stop
        # Estimated time to move; a crude estimate used for timeouts
        async with self.make_csc(initial_state=salobj.State.ENABLED, simulation_mode=1):
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=ControllerState.ENABLED,
                enabledSubstate=EnabledSubstate.STATIONARY,
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
                controllerState=ControllerState.ENABLED,
                enabledSubstate=EnabledSubstate.MOVING_POINT_TO_POINT,
            )

            # Let the move run for a short time, then stop it.
            await asyncio.sleep(0.2)
            await self.remote.cmd_stop.start(timeout=STD_TIMEOUT)
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=ControllerState.ENABLED,
                enabledSubstate=EnabledSubstate.STATIONARY,
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
                controllerState=ControllerState.ENABLED,
                enabledSubstate=EnabledSubstate.STATIONARY,
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
                controllerState=ControllerState.ENABLED,
                enabledSubstate=EnabledSubstate.SLEWING_OR_TRACKING,
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
                    await self.check_rotation()
                    await asyncio.sleep(0.1)

            track_task = asyncio.create_task(track())
            try:
                data = await self.remote.evt_inPosition.next(
                    flush=False, timeout=STD_TIMEOUT + est_slew_duration
                )
                self.assertTrue(data.inPosition)
            except asyncio.TimeoutError:
                # It is likely the track task failed.
                if track_task.done():
                    self.fail(f"Tracking task failed: {track_task.exception()}")
                else:
                    self.fail("Timed out while waiting for the inPosition event")
            finally:
                track_task.cancel()

            elt = time.monotonic() - st
            print(f"Slew duration: {elt:0.2} seconds")

            await self.remote.cmd_stop.start(timeout=STD_TIMEOUT)
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=ControllerState.ENABLED,
                enabledSubstate=EnabledSubstate.STATIONARY,
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
                controllerState=ControllerState.ENABLED,
                enabledSubstate=EnabledSubstate.STATIONARY,
            )
            settings = await self.remote.evt_configuration.next(
                flush=False, timeout=STD_TIMEOUT
            )
            await self.remote.cmd_trackStart.start(timeout=STD_TIMEOUT)
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=ControllerState.ENABLED,
                enabledSubstate=EnabledSubstate.SLEWING_OR_TRACKING,
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
                controllerState=ControllerState.ENABLED,
                enabledSubstate=EnabledSubstate.STATIONARY,
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
                controllerState=ControllerState.ENABLED,
                enabledSubstate=EnabledSubstate.SLEWING_OR_TRACKING,
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
                controllerState=ControllerState.FAULT,
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
                controllerState=ControllerState.ENABLED,
                enabledSubstate=EnabledSubstate.STATIONARY,
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
                controllerState=ControllerState.ENABLED,
                enabledSubstate=EnabledSubstate.SLEWING_OR_TRACKING,
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
                controllerState=ControllerState.FAULT,
            )
            await self.assert_next_sample(
                topic=self.remote.evt_tracking, tracking=False, noNewCommand=True
            )


if __name__ == "__main__":
    unittest.main()
