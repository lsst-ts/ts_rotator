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

__all__ = ["MAX_VEL_LIMIT", "MAX_ACCEL_LIMIT", "ROTATOR_SYNC_PATTERN"]

# What is this for? Do we need it?
ERROR_CODE = 6401

# Maximum velocity limit (deg/sec)
MAX_VEL_LIMIT = 3.5

# Maximum acceleration limit (deg/sec^2)
MAX_ACCEL_LIMIT = 1.0

# Required value of command.sync_pattern for rotator commands.
ROTATOR_SYNC_PATTERN = 0x5555
