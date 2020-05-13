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

__all__ = ["RotatorCommander"]

import asyncio
import math

import numpy as np

from lsst.ts import salobj
from lsst.ts import hexrotcomm

STD_TIMEOUT = 5  # timeout for command ack

TRACK_INTERVAL = 0.1  # interval between tracking updates (seconds)


class RotatorCommander(hexrotcomm.CscCommander):
    def __init__(self):
        self.tracking_task = asyncio.Future()
        super().__init__(
            name="Rotator",
            index=0,
            help_text="""Special commands:
* exit  # Quit the interpreter, after stopping existing motion.
* help  # Print this help.
* ramp start_position end_position velocity
        # Track a constant velocity ramp from start_position
        # to end_position at the specified velocity.
        # This stops any existing ramp or sine.
* sine start_position amplitude period
        # Track one cycle of a sine wave centered at start_position,
        # with the specified amplitude (half the full range of motion)
        # and period (sec).
        # This stops any existing ramp or sine.

State transitions commands (none take arguments):
* enterControl
* start
* enable
* disable
* standby
* exitControl
* clearError

Other commands and arguments:
* configureAcceleration max_acceleration    # Set maximum acceleration
* configureVelocity max_velocity            # Set maximum velocity
* move position         # Move to the specified position
* stop                  # Send the stop command and stop any existing ramp or sine.
* trackStart            # Put the controller into the FAULT state
                        # (due to lack of position, velocity, time updates).

Units are degrees and degrees/second

For example:
  move 5
  stop  # In case you want to stop the move early
  exit""",
        )

    async def close(self):
        self.tracking_task.cancel()
        await super().close()

    async def do_configureAcceleration(self, args):
        kwargs = self.check_arguments(args, "alimit")
        await self.remote.cmd_configureAcceleration.set_start(
            **kwargs, timeout=STD_TIMEOUT
        )

    async def do_configureVelocity(self, args):
        kwargs = self.check_arguments(args, "vlimit")
        await self.remote.cmd_configureVelocity.set_start(**kwargs, timeout=STD_TIMEOUT)

    async def do_move(self, args):
        kwargs = self.check_arguments(args, "position")
        await self.remote.cmd_move.set_start(**kwargs, timeout=STD_TIMEOUT)

    async def do_stop(self, args):
        # For safety don't check arguments, just *stop*
        self.tracking_task.cancel()
        await self.remote.cmd_stop.start(timeout=STD_TIMEOUT)

    async def do_trackStart(self, args):
        self.check_arguments(args)
        await self.remote.cmd_trackStart.start(timeout=STD_TIMEOUT)

    async def do_ramp(self, args):
        """Track from start_position to end_position at the specified velocity.
        """
        self.tracking_task.cancel()
        kwargs = self.check_arguments(
            args, "start_position", "end_position", "velocity"
        )
        self.tracking_task = asyncio.ensure_future(self._ramp(**kwargs))

    async def do_sine(self, args):
        """Track along a sine wave (one full cycle).
        """
        self.tracking_task.cancel()
        kwargs = self.check_arguments(args, "start_position", "amplitude", "period")
        self.tracking_task = asyncio.ensure_future(self._sine(**kwargs))

    async def tel_motors_callback(self, data):
        rounded_value = np.around(data.calibrated, decimals=3)
        if np.array_equal(self.previous_tel_motors, rounded_value):
            return
        self.previous_tel_motors = rounded_value
        print(f"motors: {self.format_data(data)}")

    async def _ramp(self, start_position, end_position, velocity):
        try:
            if velocity == 0:
                raise ValueError(f"velocity {velocity} must be nonzero")
            dt = (end_position - start_position) / velocity
            if dt < 0:
                raise ValueError(f"velocity {velocity} has the wrong sign")
            print(
                f"starting tracking a ramp from {start_position} to {end_position} at velocity {velocity}; "
                f"this will take {dt:0.2f} seconds"
            )
            dpos = velocity * TRACK_INTERVAL
            nelts = int(dt / TRACK_INTERVAL)
            await self.remote.cmd_trackStart.start(timeout=STD_TIMEOUT)
            for i in range(nelts):
                pos = start_position + i * dpos
                await self.remote.cmd_track.set_start(
                    angle=pos,
                    velocity=velocity,
                    tai=salobj.current_tai(),
                    timeout=STD_TIMEOUT,
                )
                await asyncio.sleep(TRACK_INTERVAL)
        except asyncio.CancelledError:
            print("ramp cancelled")
        except Exception as e:
            print(f"ramp failed: {e}")
        finally:
            await self.remote.cmd_stop.start(timeout=STD_TIMEOUT)

    async def _sine(self, start_position, amplitude, period):
        try:
            if period <= 0:
                raise ValueError(f"period {period} must be positive")
            print(
                f"starting tracking one cycle of a sine wave centered at {start_position} "
                f"with amplitude {amplitude} and a period of {period}"
            )
            nelts = int(period / TRACK_INTERVAL)
            vmax = amplitude * 2 * math.pi / period
            settings = self.remote.evt_configuration.get()
            if settings is None:
                raise RuntimeError(
                    "Must wait until configuration seen so we can check max velocity"
                )
            if abs(vmax) > settings.velocityLimit:
                raise ValueError(
                    f"maximum velocity {vmax} > allowed {settings.velocityLimit}"
                )
            await self.remote.cmd_trackStart.start(timeout=STD_TIMEOUT)
            for i in range(nelts):
                angle_rad = 2 * math.pi * i / nelts
                pos = amplitude * math.sin(angle_rad) + start_position
                velocity = vmax * math.cos(angle_rad)
                await self.remote.cmd_track.set_start(
                    angle=pos,
                    velocity=velocity,
                    tai=salobj.current_tai(),
                    timeout=STD_TIMEOUT,
                )
                await asyncio.sleep(TRACK_INTERVAL)
        except asyncio.CancelledError:
            print("sine cancelled")
        except Exception as e:
            print(f"sine failed: {e}")
        finally:
            await self.remote.cmd_stop.start(timeout=STD_TIMEOUT)
