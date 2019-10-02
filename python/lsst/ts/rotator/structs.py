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

__all__ = ["FrameId", "TelemetryHeader", "RotatorTelemetry", "RotatorConfig"]

import ctypes
import enum

ERROR_CODE = 6401


class CommandType(enum.IntEnum):
    # why trigger and cmd (same value)?
    STATE_TRIGGER = 0x8000
    STATE_CMD = 0x8000
    ENABLE_SUBSTATE_TRIGGER = 0x8002
    POSITION_SET = 0x8007
    SET_CONSTANT_VEL = 0x800B
    CONFIG_VEL = 0x9001
    CONFIG_ACCEL = 0x9002
    # what is this; is it for rotator or hexapod?
    TRACK_VEL_CMD = 0x9031


class FrameId(enum.IntEnum):
    TELEMETRY = 5
    CONFIG = 25


class Command(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("sync_pattern", ctypes.c_ushort),
        ("counter", ctypes.c_ushort),
        ("cmd", ctypes.c_uint),
        ("param1", ctypes.double),
        ("param2", ctypes.double),
        ("param3", ctypes.double),
        ("param4", ctypes.double),
        ("param5", ctypes.double),
        ("param6", ctypes.double),
    ]


class TelemetryHeader(ctypes.Structure):
    """Telemetry header from MT Rotator or Rotator.
    """
    _pack_ = 1
    _fields_ = [
        ("sync_pattern", ctypes.c_ushort),
        ("frame_id", ctypes.c_ushort),
        ("counter", ctypes.c_ushort),
        ("mjd", ctypes.c_int),
        ("mjd_frac", ctypes.c_double),
        # tv_sec is time_t in C
        ("tv_sec", ctypes.c_int64),
        ("tv_nsec", ctypes.c_long),
    ]


class RotatorConfig(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("velocity_limit", ctypes.c_double),
        ("accel_limit", ctypes.c_double),
        ("pos_error_threshold", ctypes.c_double),
        ("upper_pos_limit", ctypes.c_double),
        ("lower_pos_limit", ctypes.c_double),
        ("following_error_threshold", ctypes.c_double),
        ("track_success_pos_threshold", ctypes.c_double),
        ("tracking_lost_timeout", ctypes.c_double),
    ]


class RotatorMainTelemetry(ctypes.Structure):
    """Rotator telemetry.
    """
    _pack_ = 1
    _fields_ = [
        ("biss_motor_encoder_axis_a", ctypes.c_uint),
        ("biss_motor_encoder_axis_b", ctypes.c_uint),
        ("status_word_drive0", ctypes.c_ushort),
        ("status_word_drive0_axis_b", ctypes.c_ushort),
        ("status_word_drive1", ctypes.c_ushort),
        ("latching_fault_status_register", ctypes.c_ushort),
        ("latching_fault_status_register_axis_b", ctypes.c_ushort),
        ("input_pin_states", ctypes.c_int),
        ("actual_torque_axis_a", ctypes.c_short),
        ("actual_torque_axis_b", ctypes.c_short),
        ("copley_fault_status_register", ctypes.c_uint32 * 2),
        ("application_status", ctypes.c_uint),
    ]


class RotatorSimulinkTelemetry(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("cmd_pos", ctypes.c_double),
        ("set_pos", ctypes.c_double),
        ("state", ctypes.c_double),
        ("enabled_substate", ctypes.c_double),
        ("offline_substate", ctypes.c_double),
        ("flags_slew_complete", ctypes.c_double),
        ("flags_pt2pt_move_complete", ctypes.c_double),
        ("flags_stop_complete", ctypes.c_double),
        ("flags_following_error", ctypes.c_double),
        ("flags_move_success", ctypes.c_double),
        ("flags_tracking_success", ctypes.c_double),
        ("flags_position_feedback_fault", ctypes.c_double),
        ("flags_tracking_lost", ctypes.c_double),
        ("state_estimation_ch_a_fb", ctypes.c_double),
        ("state_estimation_ch_b_fb", ctypes.c_double),
        ("state_estimation_ch_a_motor_encoder", ctypes.c_double),
        ("state_estimation_ch_b_motor_encoder", ctypes.c_double),
        ("rotator_pos_deg", ctypes.c_double),
    ]


class RotatorTelemetry(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("main", RotatorMainTelemetry),
        ("simulink", RotatorSimulinkTelemetry),
    ]
