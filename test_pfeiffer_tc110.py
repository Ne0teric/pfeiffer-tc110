"""
Unit tests for the in-tree TC110 Pfeiffer-protocol driver (vacuum.pfeiffer_turbo).
No hardware: a FakeSerial plays back telegrams. Frames + checksums are asserted
byte-exact against the worked examples in the TC110 manual §5.2.3/5.2.4.
"""
import pytest

import pfeiffer_tc110 as pt


class FakeSerial:
    """Minimal pyserial stand-in: records writes, replays a fixed reply byte-by-byte."""
    def __init__(self, reply: bytes = b""):
        self.written = bytearray()
        self._reply = reply
        self._pos = 0

    def write(self, b):
        self.written += b
        return len(b)

    def read(self, n=1):
        chunk = self._reply[self._pos:self._pos + n]
        self._pos += len(chunk)
        return bytes(chunk)


# ── checksum + frame building (manual §5.2.3 / §5.2.4) ───────────────────────
def test_checksum_matches_manual_examples():
    assert pt._checksum("1230030902=?") == "112"        # query P:309@123
    assert pt._checksum("0421001006111111") == "020"    # "pump on" P:010@042


def test_query_frame_is_byte_exact():
    assert pt._build(123, "00", 309, "=?") == b"1230030902=?112\r"


def test_control_frame_is_byte_exact():
    assert pt._build(42, "10", 10, "111111") == b"0421001006111111020\r"


# ── reads ────────────────────────────────────────────────────────────────────
def test_read_rotation_speed_parses_and_queries():
    ser = FakeSerial(b"1231030906000633037\r")   # manual response: 633 Hz
    assert pt.read_rotation_speed(ser, 123) == 633
    assert bytes(ser.written) == b"1230030902=?112\r"   # sent the right query


def test_read_bearing_temperature_returns_float_celsius():
    # P:342 @ addr 1, value 045 °C: body "0010034206000045" + checksum
    body = "0010034206000045"
    ser = FakeSerial((body + pt._checksum(body) + "\r").encode())
    assert pt.read_bearing_temperature(ser, 1) == pytest.approx(45.0)


# ── writes ───────────────────────────────────────────────────────────────────
def test_enable_pumping_station_on_sends_and_accepts_echo():
    ser = FakeSerial(b"0421001006111111020\r")   # drive echoes the command = ack
    pt.enable_pumping_station(ser, 42, True)
    assert bytes(ser.written) == b"0421001006111111020\r"


def test_enable_pumping_station_off_sends_zeros():
    body = "0421001006000000"
    ser = FakeSerial((body + pt._checksum(body) + "\r").encode())
    pt.enable_pumping_station(ser, 42, False)
    assert bytes(ser.written) == (body + pt._checksum(body) + "\r").encode()


# ── error handling ───────────────────────────────────────────────────────────
def test_no_def_response_raises():
    body = "1231030906NO_DEF"
    ser = FakeSerial((body + pt._checksum(body) + "\r").encode())
    with pytest.raises(pt.PfeifferError):
        pt.read_rotation_speed(ser, 123)


def test_bad_checksum_raises():
    ser = FakeSerial(b"1231030906000633999\r")   # wrong checksum
    with pytest.raises(pt.PfeifferError):
        pt.read_rotation_speed(ser, 123)


def test_timeout_no_cr_raises():
    ser = FakeSerial(b"")                          # serial returns nothing
    with pytest.raises(pt.PfeifferError):
        pt.read_rotation_speed(ser, 123)


# ── motor + spin sequence (manual §7.3 / §7.4) ───────────────────────────────
def _echo(addr, param, data):
    return pt._build(addr, "10", param, data)      # the drive echoes a control cmd


def test_set_motor_writes_p023():
    ser = FakeSerial(_echo(1, 23, "111111"))
    pt.set_motor(ser, 1, True)
    assert bytes(ser.written) == pt._build(1, "10", 23, "111111")


def test_spin_up_does_motor_then_pumping_station():
    ser = FakeSerial(_echo(1, 23, "111111") + _echo(1, 10, "111111"))
    pt.spin(ser, 1, True)
    # P:023 (motor) first, then P:010 (pumping station) — manual §7.3
    assert bytes(ser.written) == pt._build(1, "10", 23, "111111") + pt._build(1, "10", 10, "111111")


def test_spin_down_clears_pumping_station_only():
    ser = FakeSerial(_echo(1, 10, "000000"))
    pt.spin(ser, 1, False)
    assert bytes(ser.written) == pt._build(1, "10", 10, "000000")   # §7.4: P:010=0


# ── generic typed access + codecs ────────────────────────────────────────────
def test_read_parameter_by_name_queries_right_number():
    ser = FakeSerial(b"0011030906000633" + pt._checksum("0011030906000633").encode() + b"\r")
    assert pt.read_parameter(ser, 1, "ActualSpdHz") == 633
    assert bytes(ser.written) == pt._build(1, "00", 309, "=?")


def test_u_real_decodes_with_two_decimals():
    # P:310 DrvCurrent is u_real: "000150" → 1.50 A
    body = "0011031006000150"
    ser = FakeSerial((body + pt._checksum(body) + "\r").encode())
    assert pt.read_parameter(ser, 1, 310) == pytest.approx(1.50)


def test_u_expo_roundtrip_matches_manual():
    assert pt._encode(pt.U_EXPO, 1000.0) == "100023"      # manual example 1.0·10^3
    assert pt._decode(pt.U_EXPO, "100023") == pytest.approx(1000.0)


def test_write_readonly_parameter_rejected():
    with pytest.raises(pt.PfeifferError):
        pt.write_parameter(FakeSerial(), 1, 309, 5)        # P:309 is read-only


def test_read_writeonly_parameter_rejected():
    with pytest.raises(pt.PfeifferError):
        pt.read_parameter(FakeSerial(), 1, 9)              # P:009 ErrorAckn is write-only


def test_full_param_table_present():
    # the whole documented set is mapped (control + status + set-values + DCU)
    assert pt.PARAMS[23].name == "MotorPump"
    assert {10, 23, 60, 309, 342, 346, 316, 303, 797, 740}.issubset(pt.PARAMS)
