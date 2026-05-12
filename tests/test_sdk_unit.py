"""Unit tests for hdrive_eth (no hardware required)."""

from __future__ import annotations

import struct
from unittest.mock import MagicMock, patch

import pytest

import hdrive_eth
from hdrive_eth.exceptions import HDriveError
from hdrive_eth.protocol import Mode, build_control_command
from hdrive_eth.protocol import TXTicket
from hdrive_eth.protocol import position_degrees_to_od_tenths
from hdrive_eth.telemetry import (
    BinaryCanFullTelemetryFrame,
    BinaryCanTelemetryFrame,
    BinaryDebugTelemetryFrame,
    TelemetryFrame,
    parse_telemetry_udp_payload,
)


def test_parse_ticket_int32s_attribute_values_only():
    """Firmware ``parseTicketInt32s``: ints from values after ``=``, not from names like ``d1``."""
    from hdrive_eth.protocol import parse_ticket_int32s

    assert parse_ticket_int32s('<system d1="2" d2="1" d3="2" d4="3" />') == [2, 1, 2, 3]
    assert parse_ticket_int32s('<system mode="4" b="0" c="0" d="0" />') == [4, 0, 0, 0]
    assert parse_ticket_int32s('<canC2 ms="500" ma="200" md="2000" s1s="77" s1a="1000" />') == [
        500,
        200,
        2000,
        77,
        1000,
    ]


def test_mode_constants_match_firmware_op_modes():
    """Values align with IOperationMode::opModes in firmwarev1."""
    assert Mode.TORQUE_CONTROL == 0x80  # AXIS_STATE_MOTORMODE_CURRENT
    assert Mode.POSITION_CONTROL == 0x81  # AXIS_STATE_MOTORMODE_POSITION
    assert Mode.VELOCITY_CONTROL == 0x82  # AXIS_STATE_MOTORMODE_SPEED
    assert Mode.DISABLE == 0x00


def test_build_can_pos_command_wire_format():
    """Quoting and °×10 encoding; wire order is ``a``…``i`` (see ``protocol.build_can_pos_command``)."""
    from hdrive_eth.protocol import build_can_pos_command

    cmd = build_can_pos_command(12.3, -4.5, 6.0).decode("ascii")
    assert cmd.startswith('"<canPos a="123"')
    assert 'b="-45"' in cmd
    assert 'c="60"' in cmd
    assert 'i="0"' in cmd
    assert cmd.endswith(' />"')


def test_position_degrees_to_od_tenths_matches_can_pos_encoding():
    """m7 ``targetPosition`` units align with ``canPos`` tenths-of-degree rounding."""
    assert position_degrees_to_od_tenths(12.3) == 123
    assert position_degrees_to_od_tenths(-4.5) == -45
    assert position_degrees_to_od_tenths(6.0) == 60


def test_torque_control_command_zeros_position_and_speed():
    """Torque-only commands must not send build_control_command's default speed=500."""
    cmd = build_control_command(
        position=0,
        speed=0,
        torque=200,
        mode=Mode.TORQUE_CONTROL,
        acc=0,
        decc=0,
    ).decode("ascii")
    assert 'pos="0"' in cmd
    assert 'speed="0"' in cmd
    assert f'mode="{Mode.TORQUE_CONTROL}"' in cmd


def test_build_control_command_scales_position_and_mode():
    cmd = build_control_command(
        position=90,
        speed=500,
        torque=200,
        mode=Mode.POSITION_CONTROL,
        acc=5000,
        decc=5000,
    ).decode("ascii")
    assert 'pos="900"' in cmd
    assert 'speed="500"' in cmd
    assert 'torque="200"' in cmd
    assert f'mode="{Mode.POSITION_CONTROL}"' in cmd
    assert cmd.startswith('"<control')
    assert cmd.endswith('/>"')


def test_slave_od_shortcuts_stable():
    """Common slave OD row/column indices for examples and hardware tests."""
    from hdrive_eth.slvobj import SlaveOd

    assert SlaveOd.MAIN_ACTUAL_MOTOR_DATA == 0
    assert SlaveOd.SUB_ACTUAL_VOLTAGE == 6
    assert SlaveOd.MAIN_DEMANDED_VALUES == 1
    assert SlaveOd.SUB_DEMANDED_TORQUE == 2


def test_read_slvobj_get_url_and_decode():
    from unittest.mock import MagicMock, patch

    from hdrive_eth.motor import HDriveETH

    mock_resp = MagicMock()
    mock_resp.read.return_value = b" 42 \n"
    mock_cm = MagicMock()
    mock_cm.__enter__.return_value = mock_resp
    mock_cm.__exit__.return_value = None

    with patch("hdrive_eth.motor.urlopen", return_value=mock_cm) as uo:
        out = HDriveETH.read_slvobj("1.2.3.4", 0, 1, 2, 3.0)

    assert out == " 42 \n"
    uo.assert_called_once()
    call_url = uo.call_args[0][0]
    assert call_url == "http://1.2.3.4/getData.cgi?slvobj=r_0_1_2"


def test_read_slave_object_uses_objreadcan_and_parses_value():
    from hdrive_eth.motor import HDriveETH

    motor = HDriveETH("127.0.0.1", connect=False)
    sock = MagicMock()
    sock.gettimeout.return_value = None
    sock.recv.side_effect = [b'<r sl="0" a="3" b="15" v="-42" />']
    motor._socket = sock
    motor._connected = True

    out = motor.read_slave_object(0, 3, 15)

    assert out == -42
    sock.sendall.assert_called_once_with(b'<objReadCAN sl="0" m="3" s="15" />')


def test_read_slave_object_http_transport_uses_slvobj_gateway():
    from hdrive_eth.motor import HDriveETH

    mock_resp = MagicMock()
    mock_resp.read.return_value = b"OK 42\n"
    mock_cm = MagicMock()
    mock_cm.__enter__.return_value = mock_resp
    mock_cm.__exit__.return_value = None

    motor = HDriveETH("1.2.3.4", connect=False)
    with patch("hdrive_eth.motor.urlopen", return_value=mock_cm) as uo:
        out = motor.read_slave_object(0, 1, 2, transport="http", timeout=3.0)

    assert out == 42
    uo.assert_called_once()
    call_url = uo.call_args[0][0]
    assert call_url == "http://1.2.3.4/getData.cgi?slvobj=r_0_1_2"


def test_read_slave_object_http_transport_accepts_plain_integer_body():
    from hdrive_eth.motor import HDriveETH

    mock_resp = MagicMock()
    mock_resp.read.return_value = b" 42 \n"
    mock_cm = MagicMock()
    mock_cm.__enter__.return_value = mock_resp
    mock_cm.__exit__.return_value = None

    motor = HDriveETH("1.2.3.4", connect=False)
    with patch("hdrive_eth.motor.urlopen", return_value=mock_cm):
        out = motor.read_slave_object(0, 1, 2, transport="http")

    assert out == 42


def test_read_slave_object_raises_on_compact_error_response():
    from hdrive_eth.motor import HDriveETH

    motor = HDriveETH("127.0.0.1", connect=False)
    sock = MagicMock()
    sock.gettimeout.return_value = None
    sock.recv.side_effect = [b'<r sl="0" a="3" b="15" error="7" />']
    motor._socket = sock
    motor._connected = True

    with pytest.raises(hdrive_eth.CommandError, match=r"sl0m3s15"):
        motor.read_slave_object(0, 3, 15)


def test_read_slave_object_http_transport_raises_on_err_body():
    from hdrive_eth.motor import HDriveETH

    mock_resp = MagicMock()
    mock_resp.read.return_value = b"ERR 3"
    mock_cm = MagicMock()
    mock_cm.__enter__.return_value = mock_resp
    mock_cm.__exit__.return_value = None

    motor = HDriveETH("1.2.3.4", connect=False)
    with patch("hdrive_eth.motor.urlopen", return_value=mock_cm):
        with pytest.raises(hdrive_eth.CommandError, match=r"sl0m1s2"):
            motor.read_slave_object(0, 1, 2, transport="http")


def test_read_slave_object_rejects_unknown_transport():
    from hdrive_eth.motor import HDriveETH

    motor = HDriveETH("127.0.0.1", connect=False)

    with pytest.raises(ValueError, match="transport"):
        motor.read_slave_object(0, 1, 2, transport="serial")


def test_build_can_conf_reset_command_wire_format():
    """26 ``name="value"`` attrs for ``RXConfigTicketCAN`` / ``parseTicketInt32s``."""
    import re

    from hdrive_eth.protocol import build_can_conf_command, build_can_conf_reset_command

    assert build_can_conf_reset_command() == build_can_conf_command([0] * 26)
    s = build_can_conf_reset_command().decode("ascii")
    assert s.startswith('"<canConf ')
    assert s.endswith(' />"')
    inner = s[1:-1]
    assert inner.startswith("<canConf ")
    assert 'a="0"' in inner and 'z="0"' in inner
    nums = [int(m) for m in re.findall(r'="(-?\d+)"', inner)]
    assert len(nums) == 26
    assert nums == [0] * 26


def test_build_can_conf_command_requires_26_values():
    from hdrive_eth.protocol import build_can_conf_command

    with pytest.raises(ValueError, match="26"):
        build_can_conf_command([0] * 25)


def test_send_can_c2_matches_build_can_c2_command():
    """HDriveETH.send_can_c2 sends the same bytes as build_can_c2_command."""
    from hdrive_eth.motor import HDriveETH
    from hdrive_eth.protocol import build_can_c2_command

    motor = HDriveETH("127.0.0.1", connect=False)
    captured: list[bytes] = []

    def capture(data: bytes) -> None:
        captured.append(data)

    motor._connected = True
    motor._send = capture  # type: ignore[method-assign]

    expected = build_can_c2_command(1, 2, 3, [(4, 5, 6)])
    motor.send_can_c2(1, 2, 3, (4, 5, 6))
    assert captured == [expected]


def test_build_can_c2_command_wire_format():
    """27 integers in order per ``RXConfigTicketCANAdvanced::interpretTicket``."""
    from hdrive_eth.protocol import build_can_c2_command

    cmd = build_can_c2_command(
        500,
        200,
        2000,
        [(77, 1000, 1000), (77, 1000, 1000)],
    ).decode("ascii")
    assert cmd.startswith('"<canC2 ms="500" ma="200" md="2000"')
    assert ' s1s="77" s1a="1000" s1d="1000"' in cmd
    assert ' s2s="77" s2a="1000" s2d="1000"' in cmd
    assert ' s8s="0" s8a="0" s8d="0"' in cmd
    assert cmd.endswith(' />"')


def test_txticket_udp_sizes_align_with_firmware():
    """Sizes match TicketManager binary composers (firmwarev1)."""
    from hdrive_eth.protocol import TX_TICKET_UDP_PAYLOAD_BYTES

    assert TX_TICKET_UDP_PAYLOAD_BYTES[TXTicket.DEBUG_BINARY] == 60
    assert TX_TICKET_UDP_PAYLOAD_BYTES[TXTicket.BINARY_CAN] == 116
    assert TX_TICKET_UDP_PAYLOAD_BYTES[TXTicket.BINARY] == 132
    assert TX_TICKET_UDP_PAYLOAD_BYTES[TXTicket.BINARY_CAN_FULL] == 196


def test_parse_telemetry_can_binary_frames():
    can_vals = list(range(29))
    can_pay = struct.pack("<29i", *can_vals)
    cf = BinaryCanTelemetryFrame.from_bytes(can_pay)
    assert cf.master_position == 1
    assert cf.slave_positions == list(range(2, 10))
    assert parse_telemetry_udp_payload(can_pay) == cf


def test_parse_telemetry_can_full_frame():
    full_vals = list(range(49))
    full_pay = struct.pack("<49i", *full_vals)
    ff = BinaryCanFullTelemetryFrame.from_bytes(full_pay)
    assert ff.master_torque == 19
    assert ff.slave_torques == list(range(20, 28))
    assert ff.reserved_tail == (47, 48)
    assert parse_telemetry_udp_payload(full_pay) == ff


def test_parse_telemetry_debug_frame():
    dbg_vals = list(range(15))
    dbg_pay = struct.pack("<15i", *dbg_vals)
    df = BinaryDebugTelemetryFrame.from_bytes(dbg_pay)
    assert df.position_deg10 == 1
    assert parse_telemetry_udp_payload(dbg_pay) == df


def test_telemetry_frame_from_bytes_round_trip():
    raw_vals = list(range(33))
    payload = struct.pack("<33i", *raw_vals)
    frame = TelemetryFrame.from_bytes(payload)
    assert frame.time_us == 0
    assert frame.position == 1
    assert frame.velocity == 2
    assert frame.slave_positions == list(range(23, 31))
    assert frame.active_slaves == 31
    assert frame.can_status == 32
    assert frame.raw == raw_vals


def test_connect_failure_raises_sdk_connection_error():
    with patch("hdrive_eth.motor.socket.socket") as mock_sock_cls:
        mock_inst = MagicMock()
        mock_inst.connect.side_effect = OSError("connection refused")
        mock_sock_cls.return_value = mock_inst

        with pytest.raises(hdrive_eth.ConnectionError) as excinfo:
            hdrive_eth.HDriveETH("192.168.254.254", connect=True)

        assert isinstance(excinfo.value, HDriveError)
