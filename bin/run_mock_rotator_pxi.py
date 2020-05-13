#!/usr/bin/env python
# This file is part of ts_rotator.
#
# Developed for the LSST Telescope and Site Systems.
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

import argparse
import asyncio
import logging

from lsst.ts import rotator


async def main():
    parser = argparse.ArgumentParser("Run mock rotator PXI code")
    parser.add_argument("host", help="IP address of rotator CSC")
    args = parser.parse_args()

    log = logging.getLogger("MockRotator")
    log.addHandler(logging.StreamHandler())
    log.setLevel(logging.DEBUG)
    rotator.MockMTRotatorController(log=log, host=args.host)
    print(f"Mock rotator controller constructed with host={args.host}; waiting forever")
    await asyncio.Future()


asyncio.run(main())
