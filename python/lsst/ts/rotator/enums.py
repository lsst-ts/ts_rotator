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

__all__ = ["ControllerState", "EnabledSubstate", "OfflineSubstate",
           "CommandCode", "SetStateParam",
           "EnabledSetSubstateParam", "OfflineSetSubstateParam",
           "ApplicationState"]

import enum
from lsst.ts.idl.enums import Rotator


# Enumerations in lsst.ts.id.enums.Rotator have values 1 larger
# than those reported by the controller, because XML enumeration values
# cannot start at 0. So create a mirror set of enumerations here
# that have the correct values, to simplify the code.
def make_decremented_enum(original, doc):
    """Make a copy of an enum.IntEnum with values decremented by 1
    """
    new_items = dict((key, value-1) for key, value in original.__members__.items())
    new_enum = enum.IntEnum(original.__name__, new_items)
    new_enum.__doc__ = doc
    return new_enum


ControllerState = make_decremented_enum(
    original=Rotator.ControllerState,
    doc="""States reported as ``telemetry.state``.

Warning: these have *different numeric values* than the same-named
CSC summary states ``salobj.State``.

Called ``States`` in Moog code and the enum items have State appended,
e.g StandbyState.
""")


EnabledSubstate = make_decremented_enum(
    original=Rotator.EnabledSubstate,
    doc="""Substates for the ENABLED state.

Values for ``telemetry.enabled_substate``.

Called ``EnabledSubStates`` in Moog code.
""")


OfflineSubstate = make_decremented_enum(
    original=Rotator.OfflineSubstate,
    doc="""Substates for the OFFLINE state.

Values for ``telemetry.offline_substate``.

Called ``OfflineSubStates`` in Moog code.
""")


class CommandCode(enum.IntEnum):
    """Value of ``command.cmd``.
    """
    SET_STATE = 0x8000
    SET_SUBSTATE = 0x8002
    POSITION_SET = 0x8007
    SET_CONSTANT_VEL = 0x800B
    CONFIG_VEL = 0x9001
    CONFIG_ACCEL = 0x9002
    TRACK_VEL_CMD = 0x9031


class SetStateParam(enum.IntEnum):
    """Values for ``command.param1`` when
    ``command.cmd = CommandCode.SET_STATE``.

    Called ``TriggerCmds`` in Moog code.
    """
    INVALID = 0
    START = enum.auto()
    ENABLE = enum.auto()
    STANDBY = enum.auto()
    DISABLE = enum.auto()
    EXIT = enum.auto()
    CLEAR_ERROR = enum.auto()
    ENTER_CONTROL = enum.auto()


class EnabledSetSubstateParam(enum.IntEnum):
    """Enabled substate parameters.

    Values for ``command.param1`` when
    ``command.cmd = CommandCode.SET_SUBSTATE``
    and the current state is ``ENABLED``.

    Called ``EnabledSubStateTriggers`` in Moog code.
    """
    ENABLED_INVALID = 0
    MOVE_POINT_TO_POINT = enum.auto()
    TRACK = enum.auto()
    STOP = enum.auto()
    INITIALIZE = enum.auto()
    RELATIVE = enum.auto()
    CONSTANT_VELOCITY = enum.auto()
    SPARE2 = enum.auto()
    MOVE_LUT = enum.auto()


class OfflineSetSubstateParam(enum.IntEnum):
    """Offline substate command parameters.

    Values for ``command.param1`` when
    ``command.cmd = CommandCode.SET_SUBSTATE``
    and the current state is ``OFFLINE``.

    Called ``OfflineSubStateTriggers`` in Moog code.
    """
    INVALID = 0
    PUBLISH_ONLY = enum.auto()
    AVAILABLE = enum.auto()


class ApplicationState(enum.IntFlag):
    """Mask bits for `telemetry.application_status`.

    These names match constants in Moog code.
    """
    HEX_FOLLOWING_ERROR_MASK = 0x00000001
    HEX_MOVE_COMPLETE_MASK = 0x00000002
    COMMAND_REJECT_MASK = 0x00000020
    SAFTEY_INTERLOCK = 0x00000040
    EXTEND_LIMIT_SWITCH = 0x00000080
    RETRACT_LIMIT_SWITCH = 0x00000100
    ETHERCAT_PROBLEM = 0x00000200
    DDS_COMMAND_SOURCE = 0x00000400
    MOTION_TIMEOUT = 0x00000800
    DRIVE_FAULT = 0x00002000
    SIMULINK_FAULT = 0x00004000
    ENCODER_FAULT = 0x00008000
