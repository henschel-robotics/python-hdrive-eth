"""
HDriveETH Python SDK
=================

Control Henschel Robotics HDrive17-ETH servo drives from Python.

Quickstart::

    from hdrive_eth import HDriveETH

    with HDriveETH("192.168.122.102") as motor:
        motor.move_to(90)

Full documentation: https://henschel-robotics.ch
"""

from .motor import HDriveETH
from .telemetry import (
    BinaryCanFullTelemetryFrame,
    BinaryCanTelemetryFrame,
    BinaryDebugTelemetryFrame,
    TelemetryFrame,
    TelemetryPayload,
    TelemetryReceiver,
    parse_telemetry_udp_payload,
)
from .protocol import Mode, TXTicket, position_degrees_to_od_tenths
from .exceptions import (
    HDriveError,
    ConnectionError,
    CommandError,
    TimeoutError,
    NotConnectedError,
    FirmwareVersionError,
)

__version__ = "0.1.3"
__author__ = "Henschel Robotics GmbH"

__all__ = [
    "HDriveETH",
    "TelemetryFrame",
    "BinaryCanTelemetryFrame",
    "BinaryCanFullTelemetryFrame",
    "BinaryDebugTelemetryFrame",
    "TelemetryPayload",
    "TelemetryReceiver",
    "parse_telemetry_udp_payload",
    "Mode",
    "TXTicket",
    "position_degrees_to_od_tenths",
    "HDriveError",
    "ConnectionError",
    "CommandError",
    "TimeoutError",
    "NotConnectedError",
    "FirmwareVersionError",
]
