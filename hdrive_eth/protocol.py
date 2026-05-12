"""
HDrive XML command protocol.

The HDrive17-ETH uses an XML-based command format over TCP.
Commands are sent as ASCII-encoded strings.

Command format:
    "<control pos="..." speed="..." torque="..." mode="..." acc="..." decc="..." />"
"""

from __future__ import annotations

import re
from typing import List, Optional, Sequence, Tuple


# ---------------------------------------------------------------------------
# Control mode constants
# ---------------------------------------------------------------------------

class Mode:
    """HDrive control mode constants.

    Values match ``IOperationMode::opModes`` in firmware (``Common/OperationModes/IOperationMode.h``).
    The TCP ``<control ... mode="..."/>`` field is stored as ``motorMode`` and selects the active
    operation mode (see ``Statemachine.cpp`` / ``RXControlTicket.h``).
    """

    # Maps to IOperationMode::opModes — must match firmware integers exactly.
    TORQUE_CONTROL = 0x80  # AXIS_STATE_MOTORMODE_CURRENT (128)
    POSITION_CONTROL = 0x81  # AXIS_STATE_MOTORMODE_POSITION (129)
    POSITION_CONTROL_NPP = 133  # AXIS_STATE_MOTORMODE_POSITION_NPP (133)
    VELOCITY_CONTROL = 0x82  # AXIS_STATE_MOTORMODE_SPEED (130)
    DISABLE = 0x00  # AXIS_STATE_MOTORMODE_STOP


class TXTicket:
    """UDP/TCP TX telemetry composer ids — ``TicketManager::TX_Ticket`` (``Communication/TicketManager.h``).

    Stored in object dictionary ``communicationValues.TXTicket`` (``m4s22``).
    Binary UDP payloads use little-endian ``int32`` rows as in ``BinaryTicket.h`` / ``BinaryCanTicket*.h``.
    """

    HDRIVE_XML = 0
    CAN_TICKET_XML = 1  # ``BinaryCanTicket(..., binary=False)`` — ASCII
    DEBUG_BINARY = 2  # ``BinaryDebugTicket`` — 15 × int32 (60 B)
    BINARY = 3  # ``BinaryTicket`` — 33 × int32 (132 B)
    BINARY_CAN = 4  # ``BinaryCanTicket(..., binary=True)`` — 29 × int32 (116 B)
    EEPROM_CONFIG = 5
    OBJ_TABLE = 6
    DATA_LOGGER = 7
    UNKNOWN = 8
    READ_OBJECTS = 9
    BINARY_CAN_FULL = 10  # ``BinaryCanTicketFull(..., binary=True)`` — 49 × int32 (196 B)


# Payload sizes for binary UDP tickets (firmware composeTicket binary branches).
TX_TICKET_UDP_PAYLOAD_BYTES = {
    TXTicket.DEBUG_BINARY: 15 * 4,
    TXTicket.BINARY_CAN: 29 * 4,
    TXTicket.BINARY: 33 * 4,
    TXTicket.BINARY_CAN_FULL: 49 * 4,
}


# ---------------------------------------------------------------------------
# XML command builder
# ---------------------------------------------------------------------------

def build_control_command(
    position: int = 0,
    speed: int = 500,
    torque: int = 200,
    mode: int = Mode.POSITION_CONTROL,
    acc: int = 5000,
    decc: int = 5000,
) -> bytes:
    """Build an XML control command for the HDrive.

    Args:
        position: Target position in degrees.
        speed: Target speed value.
        torque: Torque (**mNm**); maps to object dictionary ``demandedTorque``.
        mode: Control mode (``Mode`` constants in this module).
        acc: Acceleration ramp value.
        decc: Deceleration ramp value.

    Returns:
        ASCII-encoded bytes ready to send over TCP.
    """
    xml = (
        f'"<control'
        f' pos="{round(position * 10)}"'
        f' speed="{round(speed)}"'
        f' torque="{round(torque)}"'
        f' mode="{round(mode)}"'
        f' acc="{round(acc)}"'
        f' decc="{round(decc)}"'
        f' />"'
    )
    return xml.encode("ascii")


def build_disable_command() -> bytes:
    """Build a command that disables the drive."""
    return build_control_command(
        position=0,
        speed=0,
        torque=0,
        mode=Mode.DISABLE,
        acc=0,
        decc=0,
    )


def parse_ticket_int32s(text: str, max_count: int = 256) -> List[int]:
    """Signed decimal ints in **attribute value** order — mirrors firmware ``parseTicketInt32s``.

    Only runs after each ``=`` (optional whitespace, optional ``"``, then an integer). Digits in
    attribute names (``d1``, ``s1s``, ``n00``, …) are not collected.
    """
    out: List[int] = []
    i = 0
    nmax = max(0, max_count)
    while i < len(text) and len(out) < nmax:
        if text[i] != "=":
            i += 1
            continue
        i += 1
        while i < len(text) and text[i].isspace():
            i += 1
        if i < len(text) and text[i] == '"':
            i += 1
        if i >= len(text):
            break
        c = text[i]
        if c not in "-0123456789":
            i += 1
            continue
        if c == "-" and (i + 1 >= len(text) or not text[i + 1].isdigit()):
            i += 1
            continue
        m = re.match(r"-?\d+", text[i:])
        if not m:
            i += 1
            continue
        out.append(int(m.group(0), 10))
        i += len(m.group(0))
        if i < len(text) and text[i] == '"':
            i += 1
    return out


# ``<canConf/>`` uses ``a``…``z`` for stable ordering and compact wire form (firmware
# ``RXConfigTicketCAN``). Names with digits are safe on current firmware; letters stay the norm.
_CANCONF_ATTR = "abcdefghijklmnopqrstuvwxyz"


def build_can_conf_command(values: Sequence[int]) -> bytes:
    """Build quoted ``<canConf …/>`` TCP bytes (firmware ``Communication/CommRX_Tickets/RXConfigTicketCAN.h``).

    The ticket hash matches the prefix ``"<canConf "``. ``interpretTicket`` uses
    ``parseTicketInt32s`` (``ParseTicketInt32s.h``): signed decimals from each attribute **value**
    in left-to-right order. Attribute names are conventionally ``a``…``z`` for the 26 values.

    - ``a`` ``demandedTorque`` (master, **mNm**) — ``numbers[0]``
    - ``b`` ``demandedMode`` (master; also copied to ``motorMode``) — ``numbers[1]``
    - ``c``…``j`` slaves 1–8 ``targetCurrent`` — ``numbers[2]``…``numbers[9]``
    - ``k``…``r`` slaves 1–8 ``targetMode`` — ``numbers[10]``…``numbers[17]``
    - ``s``…``z`` slaves 1–8 ``specialCommand`` — ``numbers[18]``…``numbers[25]``

    Then sets ``slaveData/newCANDataReceived`` and ``demandedValues/newDataReceived``.
    """
    vals = [int(x) for x in values]
    if len(vals) != 26:
        raise ValueError(f"canConf requires exactly 26 integers, got {len(vals)}")
    attrs = " ".join(f'{_CANCONF_ATTR[i]}="{vals[i]}"' for i in range(26))
    body = f"<canConf {attrs} />"
    return (f'"{body}"').encode("ascii")


def build_can_conf_reset_command() -> bytes:
    """``build_can_conf_command`` with 26 zeros (one-ticket CAN demand reset)."""
    return build_can_conf_command([0] * 26)


# Master + eight slaves: single-letter names ``a``…``i`` (compact; digit names work on current fw).
_CANPOS_ATTR = "abcdefghi"  # master tenths, then slaves 1–8 (tenths each)


def build_can_pos_command(master_deg: float, *slave_deg: float) -> bytes:
    """Build a ``<canPos .../>`` TCP command (multi-axis positions in degrees).

    Values are encoded as integer tenths of a degree on the wire. Up to eight
    CAN slaves are sent after the master; omitted slaves default to 0°.

    Args:
        master_deg: Master axis target (degrees).
        *slave_deg: Slave targets in chain order (degrees); pad/truncate to eight slaves.

    Returns:
        ASCII bytes matching the firmware ``canPos`` ticket (quoted payload). Attribute names
        are ``a``…``i`` (master + eight slaves) in wire order.
    """
    m = int(round(master_deg * 10.0))
    s_vals = [int(round(d * 10.0)) for d in slave_deg]
    while len(s_vals) < 8:
        s_vals.append(0)
    s_vals = s_vals[:8]
    vals = [m] + s_vals
    attrs = " ".join(f'{_CANPOS_ATTR[i]}="{vals[i]}"' for i in range(9))
    return f'"<canPos {attrs} />"'.encode("ascii")


def build_can_c2_command(
    master_speed: int,
    master_acc: int,
    master_decc: int,
    slave_speed_acc_decc: Optional[Sequence[Tuple[int, int, int]]] = None,
) -> bytes:
    """Build ``"<canC2 ... />"`` — firmware ``RXConfigTicketCANAdvanced`` (ticket 27).

    ``interpretTicket`` parses **27 integers in textual order** into master ``demandedValues``
    (speed, acc, decc) and ``slaveDataAdvanced`` fields ``slave1`` … ``slave8`` (speed, acc,
    decc each), then sets ``slaveData/newCANDataAdvancedReceived``. Attribute names are ignored;
    XML follows the product manual (``ms``, ``ma``, ``md``, ``sN{s,a,d}``).

    Args:
        master_speed: Master demanded speed (same units as ``<control speed="..."/>``).
        master_acc: Master demanded acceleration.
        master_decc: Master demanded deceleration.
        slave_speed_acc_decc: Up to eight ``(speed, acc, decc)`` tuples for slaves 1…8;
            shorter lists are padded with zeros.

    Returns:
        Quoted ASCII payload bytes for TCP send (same framing as ``build_can_pos_command``).
    """
    triplets: List[Tuple[int, int, int]] = list(slave_speed_acc_decc or [])
    while len(triplets) < 8:
        triplets.append((0, 0, 0))
    triplets = triplets[:8]

    xml = (
        f'"<canC2 ms="{int(master_speed)}" ma="{int(master_acc)}" md="{int(master_decc)}"'
    )
    for i, (sv, sa, sd) in enumerate(triplets, start=1):
        xml += f' s{i}s="{int(sv)}" s{i}a="{int(sa)}" s{i}d="{int(sd)}"'
    xml += ' />"'
    return xml.encode("ascii")


def position_degrees_to_od_tenths(degrees: float) -> int:
    """Convert degrees to integer tenths — same scaling as ``build_can_pos_command`` and m7 ``targetPosition``."""
    return int(round(degrees * 10.0))
