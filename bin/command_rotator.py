#!/usr/bin/env python
"""Monitor and command the MT Rotator.

To use:

command_rotator.py

Then wait for it to connect. Once it has connected it will print
initial rotator status and help.

Commands are entered by typing the command and arguments (if any),
separated by spaces, then <return>. "help" is a command.
"""
import asyncio
import functools
import math
import sys

from lsst.ts import salobj
from lsst.ts.idl.enums import Rotator

STD_TIMEOUT = 5  # timeout for command ack

TRACK_INTERVAL = 0.1  # interval between tracking updates (seconds)


def round_any(value, digits=5):
    """Round any value to the specified number of digits.

    This is a no-op for int and str values.
    """
    if isinstance(value, float):
        return round(value, digits)
    return value


async def stdin_generator():
    """Thanks to http://blog.mathieu-leplatre.info
    """
    loop = asyncio.get_event_loop()
    reader = asyncio.StreamReader(loop=loop)
    reader_protocol = asyncio.StreamReaderProtocol(reader)
    await loop.connect_read_pipe(lambda: reader_protocol, sys.stdin)
    while True:
        line = await reader.readline()
        if not line:  # EOF.
            break
        yield line.decode("utf-8").strip()


class RemoteWatcher:
    """Monitor a Rotator remote.
    """
    def __init__(self, remote):
        self.remote = remote

        for name in remote.salinfo.event_names:
            if name == "heartbeat":
                continue
            topic = getattr(self.remote, f"evt_{name}")
            func = getattr(self, f"{name}_callback", None)
            if func is None:
                func = functools.partial(self.event_callback, name=name)
            setattr(topic, "callback", functools.partial(self.event_callback, name=name))

        for name in remote.salinfo.telemetry_names:
            setattr(self, f"prev_{name}", None)
            topic = getattr(self.remote, f"tel_{name}")
            setattr(topic, "callback", functools.partial(self.telemetry_callback, name=name))

    def format_item(self, key, value):
        if isinstance(value, float):
            return f"{key}={value:0.2f}"
        return f"{key}={value}"

    def format_data(self, data):
        return ", ".join(self.format_item(key, value) for key, value in self.get_public_fields(data).items())

    def get_public_fields(self, data):
        return dict((key, value) for key, value in data.get_vars().items()
                    if not key.startswith("private_") and
                    key not in ("priority", "timestamp"))

    def get_rounded_public_fields(self, data):
        return dict((key, round_any(value)) for key, value in data.get_vars().items()
                    if not key.startswith("private_") and
                    key not in ("priority", "timestamp"))

    def event_callback(self, data, name):
        """Generic callback for events."""
        print(f"{name}: {self.format_data(data)}")

    def telemetry_callback(self, data, name):
        """Generic callback for telemetry."""
        public_fields = self.get_rounded_public_fields(data)
        if public_fields != getattr(self, f"prev_{name}"):
            setattr(self, f"prev_{name}", public_fields)
            print(f"{name}: {self.format_data(data)}")

    def controllerState_callback(self, data):
        print(f"controllerState: state={Rotator.ControllerState(data.controllerState)!r}; "
              f"offline_substate={Rotator.OfflineSubstate(data.offlineSubstate)!r}; "
              f"enabled_substate={Rotator.EnabledSubstate(data.enabledSubstate)!r}; "
              f"applicationStatus={data.applicationStatus}")


help_text = """Special commands:
* ramp pos0 pos1 vel  # Track a constant velocity ramp from pos0 to pos1
                      # at the specified velocity.
                      # This stops any existing ramp or sine.
* sine pos0 amplitude period  # Track one cycle of a sine wave centered at pos0,
                              # with the specified amplitude (half the full range of motion)
                              # and period (sec).
                              # This stops any existing ramp or sine.
* stop  # Send the stop command and stop any existing ramp or sine.
* help  # Print this help.
* exit  # Quit the interpreter, after stopping existing motion.

State transitions commands (none take arguments):
* enterControl
* start
* enable
* disable
* standby
* exitControl
* clearError

Other commands and arguments:
* configureAcceleration maxaccel  # Set maximum acceleration
* configureVelocity maxvel        # Set maximum velocity
* positionSet pos   # Set a position for the move command
* move              # Move to the position set by positionSet
* trackStart        # Puts the controller into FAULT

Units are degrees and degrees/second

For example:
  positionSet 5
  move
  stop  # In case you want to stop the move early
  exit"""


async def command_loop():
    print("Creating domain and remote.")
    async with salobj.Domain() as domain:
        remote = salobj.Remote(domain=domain, name="Rotator")
        tracking_task = asyncio.Future()
        try:
            RemoteWatcher(remote)
            print("Waiting for the remote to connect.")
            await remote.start_task

            no_arguments_commands = ("enterControl", "start", "enable", "disable",
                                     "standby", "exitControl", "clearError",
                                     "move", "stop", "trackStart")

            async def ramp(pos0, pos1, vel):
                """Track from pos0 to pos1 in time dt.
                """
                try:
                    if vel == 0:
                        raise ValueError(f"velocity {vel} must be nonzero")
                    dt = (pos1 - pos0) / vel
                    if dt < 0:
                        raise ValueError(f"velocity {vel} has the wrong sign")
                    print(f"starting tracking a ramp from {pos0} to {pos1} at velocity {vel}; "
                          f"this will take {dt:0.2f} seconds")
                    dpos = vel * TRACK_INTERVAL
                    nelts = int(dt / TRACK_INTERVAL)
                    await remote.cmd_trackStart.start(timeout=STD_TIMEOUT)
                    for i in range(nelts):
                        pos = pos0 + i*dpos
                        await remote.cmd_track.set_start(angle=pos,
                                                         velocity=vel,
                                                         tai=salobj.current_tai(),
                                                         timeout=STD_TIMEOUT)
                        await asyncio.sleep(TRACK_INTERVAL)
                except asyncio.CancelledError:
                    print(f"ramp cancelled")
                except Exception as e:
                    print(f"ramp failed: {e}")
                finally:
                    await remote.cmd_stop.start(timeout=STD_TIMEOUT)

            async def sine(pos0, amplitude, period):
                """Track along a sine wave (one full cycle).
                """
                try:
                    if period <= 0:
                        raise ValueError(f"period {period} must be positive")
                    print(f"starting tracking one cycle of a sine wave centered at {pos0} "
                          f"with amplitude {amplitude} and a period of {period}")
                    nelts = int(period / TRACK_INTERVAL)
                    vmax = amplitude * 2 * math.pi / period
                    settings = remote.evt_settingsApplied.get()
                    if settings is None:
                        raise RuntimeError("Must wait until settingsApplied seen so we can check max vel")
                    if abs(vmax) > settings.velocityLimit:
                        raise ValueError(f"maximum velocity {vmax} > allowed {settings.velocityLimit}")
                    await remote.cmd_trackStart.start(timeout=STD_TIMEOUT)
                    for i in range(nelts):
                        angle_rad = 2*math.pi*i/nelts
                        pos = amplitude*math.sin(angle_rad) + pos0
                        vel = vmax*math.cos(angle_rad)
                        await remote.cmd_track.set_start(angle=pos,
                                                         velocity=vel,
                                                         tai=salobj.current_tai(),
                                                         timeout=STD_TIMEOUT)
                        await asyncio.sleep(TRACK_INTERVAL)
                except asyncio.CancelledError:
                    print(f"sine cancelled")
                except Exception as e:
                    print(f"sine failed: {e}")
                finally:
                    await remote.cmd_stop.start(timeout=STD_TIMEOUT)

            print(f"\n{help_text}")
            async for line in stdin_generator():
                # Strip trailing comment, if any.
                if "#" in line:
                    line = line.split("#", maxsplit=1)[0].strip()
                if not line:
                    continue
                tokens = line.split()
                command = tokens[0]
                args = [float(token) for token in tokens[1:]]
                try:
                    if command == "exit":
                        break
                    elif command == "help":
                        print(help_text)
                    elif command == "stop":
                        tracking_task.cancel()
                        await remote.cmd_stop.start(timeout=STD_TIMEOUT)
                    elif command in no_arguments_commands:
                        remote_command = getattr(remote, f"cmd_{command}")
                        await remote_command.start(timeout=STD_TIMEOUT)
                    elif command == "positionSet":
                        assert len(args) == 1
                        await remote.cmd_positionSet.set_start(angle=args[0], timeout=STD_TIMEOUT)
                    elif command == "configureAcceleration":
                        assert len(args) == 1
                        await remote.cmd_configureAcceleration.set_start(alimit=args[0], timeout=STD_TIMEOUT)
                    elif command == "configureVelocity":
                        assert len(args) == 1
                        await remote.cmd_configureVelocity.set_start(vlimit=args[0], timeout=STD_TIMEOUT)
                    elif command == "ramp":
                        tracking_task.cancel()
                        assert len(args) == 3
                        pos0, pos1, vel = args
                        tracking_task = asyncio.ensure_future(ramp(pos0=pos0, pos1=pos1, vel=vel))
                    elif command == "sine":
                        tracking_task.cancel()
                        assert len(args) == 3
                        pos0, amplitude, period = args
                        tracking_task = asyncio.ensure_future(sine(pos0=pos0, amplitude=amplitude,
                                                                   period=period))
                    else:
                        print(f"Unrecognized command {command}")
                        continue
                except Exception as e:
                    print(f"Command {command} failed: {e}")
                    continue
                print(f"Finished command {command}")
        finally:
            print("Exiting; please wait.")
            tracking_task.cancel()
            try:
                await remote.cmd_stop.start(timeout=STD_TIMEOUT)
            except Exception:
                # Best effort attempt to stop. The command may not
                # even be valid in the current state.
                pass
            await remote.close()

asyncio.run(command_loop())
