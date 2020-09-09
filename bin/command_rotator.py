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

from lsst.ts import rotator

asyncio.run(rotator.RotatorCommander.amain(index=None))
