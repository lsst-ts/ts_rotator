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

__all__ = ["RotatorCommander"]

import asyncio

from lsst.ts import salobj
from lsst.ts import simactuators

STD_TIMEOUT = 5  # timeout for command ack

# How far in advance to set the time field of tracking commands (seconds)
TRACK_ADVANCE_TIME = 0.05

TRACK_INTERVAL = 0.1  # interval between tracking updates (seconds)


class RotatorCommander(salobj.CscCommander):
    def __init__(self, enable):
        self.tracking_task = asyncio.Future()
        super().__init__(
            name="MTRotator", index=0, enable=enable,
        )
        self.help_dict["ramp"] = "start_position end_position speed "
        "# track a path of constant",
        self.help_dict["cosine"] = "center_position, amplitude, max_speed "
        "# track one cycle of a cosine wave",
        for command_to_ignore in ("abort", "setValue"):
            del self.command_dict[command_to_ignore]

    async def close(self):
        self.tracking_task.cancel()
        await super().close()

    async def do_ramp(self, args):
        """Track from start_position to end_position at the specified speed.
        """
        self.tracking_task.cancel()
        kwargs = self.check_arguments(args, "start_position", "end_position", "speed")
        self.tracking_task = asyncio.ensure_future(self._ramp(**kwargs))

    async def do_cosine(self, args):
        """Track along a cosine wave (one full cycle).
        """
        self.tracking_task.cancel()
        kwargs = self.check_arguments(args, "center_position", "amplitude", "max_speed")
        self.tracking_task = asyncio.ensure_future(self._cosine(**kwargs))

    def _special_telemetry_callback(self, data, name, omit_field, digits=2):
        """Callback for telemetry omitting one specified field
        from the comparison, but printing it.

        Parameters
        ----------
        data : `object`
            Telemetry data.
        name : `str`
            Name of telemetry topic, without the `tel_` prefix.
        omit_field : `list` [`str`]
            Field to omit from the comparison.
        """
        prev_value_name = f"previous_tel_{name}"
        public_data = self.get_rounded_public_fields(data, digits=digits)
        trimmed_data = public_data.copy()
        trimmed_data.pop(omit_field)
        if trimmed_data == getattr(self, prev_value_name):
            return
        setattr(self, prev_value_name, trimmed_data)
        formatted_data = self.format_dict(public_data)
        self.output(f"{data.private_sndStamp:0.3f}: {name}: {formatted_data}")

    async def tel_motors_callback(self, data):
        """Don't print if only the raw field has changed.

        Parameters
        ----------
        data : `object`
            MTRotator motors telemetry data.
        """
        self._special_telemetry_callback(
            data=data, name="motors", omit_field="raw", digits=1
        )

    async def tel_rotation_callback(self, data):
        """Don't print if only the timestamp has changed.

        Parameters
        ----------
        data : `object`
            MTRotator rotation telemetry data.
        """
        self._special_telemetry_callback(
            data=data, name="rotation", omit_field="timestamp"
        )

    async def _ramp(self, start_position, end_position, speed):
        """Track a linear ramp.

        Parameters
        ----------
        start_position : `float`
            Starting position of ramp (deg).
        end_position : `float`
            Ending position of ramp (deg).
        speed : `float`
            Speed of motion along the ramp (deg/sec).
        """
        try:
            ramp_generator = simactuators.RampGenerator(
                start_positions=[start_position],
                end_positions=[end_position],
                speeds=[speed],
                advance_time=TRACK_ADVANCE_TIME,
            )
            print(
                f"Tracking a ramp from {start_position} to {end_position} at speed {speed}; "
                f"this will take {ramp_generator.duration:0.2f} seconds"
            )
            await self.remote.cmd_trackStart.start(timeout=STD_TIMEOUT)
            for positions, velocities, tai in ramp_generator():
                await self.remote.cmd_track.set_start(
                    angle=positions[0],
                    velocity=velocities[0],
                    tai=tai,
                    timeout=STD_TIMEOUT,
                )
                await asyncio.sleep(TRACK_INTERVAL)
        except asyncio.CancelledError:
            print("ramp cancelled")
        except Exception as e:
            print(f"ramp failed: {e}")
        finally:
            await self.remote.cmd_stop.start(timeout=STD_TIMEOUT)

    async def _cosine(self, center_position, amplitude, max_speed):
        """Track one sine wave of specified amplitude and period.

        The range of motion is period - amplitude to period + amplitude,
        plus whatever motion is required to slew to the path.

        Parameters
        ----------
        center_position : `float`
            Midpoint of cosine wave (deg).
        amplitude : `float`
            Amplitude of cosine wave (deg).
        max_speed : `float`
            Maximum speed of motion (deg/sec).
        """
        try:
            settings = self.remote.evt_configuration.get()
            if settings is None:
                raise RuntimeError(
                    "Must wait until configuration seen so we can check max velocity"
                )
            if abs(max_speed) > settings.velocityLimit:
                raise ValueError(
                    f"maximum speed {max_speed} > allowed {settings.velocityLimit}"
                )
            cosine_generator = simactuators.CosineGenerator(
                center_positions=[center_position],
                amplitudes=[amplitude],
                max_speeds=[max_speed],
                advance_time=TRACK_ADVANCE_TIME,
            )
            print(
                f"Tracking one cycle of a cosine wave centered at {center_position} "
                f"with amplitude {amplitude}; "
                f"this will take {cosine_generator.duration:0.2f} seconds"
            )
            await self.remote.cmd_trackStart.start(timeout=STD_TIMEOUT)
            for positions, velocities, tai in cosine_generator():
                await self.remote.cmd_track.set_start(
                    angle=positions[0],
                    velocity=velocities[0],
                    tai=tai,
                    timeout=STD_TIMEOUT,
                )
                await asyncio.sleep(TRACK_INTERVAL)
        except asyncio.CancelledError:
            print("sine cancelled")
        except Exception as e:
            print(f"sine failed: {e}")
        finally:
            await self.remote.cmd_stop.start(timeout=STD_TIMEOUT)
