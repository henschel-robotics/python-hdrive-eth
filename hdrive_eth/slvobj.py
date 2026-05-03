"""
CAN slave object dictionary helpers (``slvobj`` over HTTP).

Firmware completes ``getData.cgi?slvobj=…`` on the device; use
:class:`hdrive_eth.HDriveETH.read_slvobj` / :meth:`hdrive_eth.HDriveETH.write_slvobj`,
which build ``r_<slot>_<mainKey>_<subKey>`` and ``w_<slot>_<mainKey>_<subKey>_<value>`` on the wire.

``slot`` is **0..7** (first slave = slot ``0``). ``main_key`` / ``sub_key`` are the slave
node OD indices (``enMainKeys`` row + column).
"""

from __future__ import annotations


class SlaveOd:
    """Typical ``main_key`` / ``sub_key`` pairs for tests and tooling."""

    MAIN_ACTUAL_MOTOR_DATA = 0
    SUB_ACTUAL_VOLTAGE = 6

    MAIN_DEMANDED_VALUES = 1
    SUB_DEMANDED_TORQUE = 2
