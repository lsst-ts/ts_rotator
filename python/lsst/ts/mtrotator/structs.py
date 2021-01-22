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

__all__ = ["Config", "Telemetry"]

import ctypes


class Config(ctypes.Structure):
    """Configuration: rotConfigTelemetryStreamStructure_t in Moog code.
    """

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
    FRAME_ID = 0x19


class Telemetry(ctypes.Structure):
    """Telemetry: rotTelemetryStreamStructure_t in Moog code.

    Note that application_status is a bit mask whose bits are
    defined starting with ROTTLMSYNCPATTERN in Moog code.
    """

    _pack_ = 1
    _fields_ = [
        ("biss_motor_encoder_axis_a", ctypes.c_uint),
        ("biss_motor_encoder_axis_b", ctypes.c_uint),
        ("status_word_drive0", ctypes.c_ushort),
        ("status_word_drive0_axis_b", ctypes.c_ushort),
        ("latching_fault_status_register", ctypes.c_ushort),
        ("latching_fault_status_register_axis_b", ctypes.c_ushort),
        ("input_pin_states", ctypes.c_int),
        ("copley_fault_status_register", ctypes.c_uint32 * 2),
        ("application_status", ctypes.c_uint),
        # simulink telemetry
        # Values computed by the path generator.
        # They are called "Cmd" in Moog's code.
        ("demand_pos", ctypes.c_double),
        ("demand_vel", ctypes.c_double),
        ("demand_accel", ctypes.c_double),
        # Position set by the POSITION_SET command.
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
        ("flags_no_new_track_cmd_error", ctypes.c_double),
        ("state_estimation_ch_a_fb", ctypes.c_double),
        ("state_estimation_ch_b_fb", ctypes.c_double),
        ("state_estimation_ch_a_motor_encoder", ctypes.c_double),
        ("state_estimation_ch_b_motor_encoder", ctypes.c_double),
        # Position and velocity measured by the position encoders.
        ("current_pos", ctypes.c_double),
        ("current_vel_ch_a_fb", ctypes.c_double),
        ("current_vel_ch_b_fb", ctypes.c_double),
    ]
    FRAME_ID = 0x5
