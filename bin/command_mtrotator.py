#!/usr/bin/env python
"""Monitor and command the MTRotator.

To use::

    command_mtrotator.py

Then wait for it to connect. Once it has connected it will print
initial rotator status and help.

Commands are entered by typing the command and arguments (if any),
separated by spaces, then <return>. "help" is a command.
"""
import asyncio

from lsst.ts import mtrotator

asyncio.run(mtrotator.RotatorCommander.amain(index=None))
