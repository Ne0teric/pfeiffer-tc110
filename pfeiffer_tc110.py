"""
pfeiffer_tc110 — Pfeiffer Vacuum Protocol driver for the TC110 turbopump
drive unit (RS-485), implemented in-tree.

Why this exists: hw.py drives the Pfeiffer TC110 turbo(s) over RS-485. The public
Pfeiffer libraries don't fit — PyPI `pfeiffer-vacuum-protocol` (electronsandstuff) is
gauge-only, and `tspspi/pfeifferpumps` is a low-level class encoder — so we implement
the protocol directly from the TC110 manual (§5.2 telegram frame, §6 parameter set).

This is the FULL parameter set: every documented parameter is in `PARAMS` (with a
plain-English description), reachable through the generic `read_parameter()` /
`write_parameter()` (by number or name, data type handled for you). Named convenience
wrappers exist for the handful the controller uses. Run `python -m pfeiffer_tc110`
(or call `print_parameters()`) to print the whole table — what every parameter does.

Quick glossary of the two you care about most:
  • P:023 MotorPump   — the TURBO MOTOR itself (the thing that spins). 1 = run.
  • P:010 PumpgStatn  — the PUMPING STATION master switch: 1 = start the whole station
                        (spins the pump up, also acknowledges errors), 0 = stop. The
                        documented spin-up is motor ON (P:023) THEN station ON (P:010).

Telegram frame (ASCII, CR-terminated), manual §5.2.1:

    aaa | AA | nnn | ll | d… | ccc | CR
    aaa  address "001".."255"  (group "9xx"/global "000" → NO reply)
    AA   action  "00" = query,  "10" = control / response
    nnn  parameter number          ll  data length    d…  data
    ccc  checksum = (Σ ASCII of aaa..d0) mod 256, 3 digits     CR  ASCII 13

A query carries data "=?" (len 02). Verified vs the manual's worked examples:
query P:309@123 → checksum "112"; "pump on" P:010@042 (data "111111") → checksum "020".

Data types (manual §5.2.5): 0 boolean_old(06), 1 u_integer(06), 2 u_real(06, ×100),
4 string(06), 6 boolean_new(01), 7 u_short_int(03), 10 u_expo_new(06), 11 string(16).

⚠ Spin commands need the drive in RS-485 control: call `set_control_via_rs485()` once
at commissioning (P:060=2).
"""
from __future__ import annotations

import math
from collections import namedtuple

# ── data type codes (manual §5.2.5) ───────────────────────────────────────────
BOOL_OLD, U_INT, U_REAL, STR6, BOOL_NEW, U_SHORT, U_EXPO, STR16 = 0, 1, 2, 4, 6, 7, 10, 11
_DTYPE_NAME = {BOOL_OLD: "bool", U_INT: "u_integer", U_REAL: "u_real", STR6: "string",
               BOOL_NEW: "bool_new", U_SHORT: "u_short", U_EXPO: "u_expo", STR16: "string16"}

Param = namedtuple("Param", "name dtype access unit desc")

# ── complete TC110 parameter set (manual §6). number → (name, type, access, unit,
#    plain-English description of what it does). ─────────────────────────────────
PARAMS = {
    # 6.2 control commands ----------------------------------------------------
    1:   Param("Heating",     BOOL_OLD, "RW", "",     "Casing heater on/off"),
    2:   Param("Standby",     BOOL_OLD, "RW", "",     "Run at reduced 'standby' speed on/off"),
    4:   Param("RUTimeCtrl",  BOOL_OLD, "RW", "",     "Run-up time monitoring on/off"),
    9:   Param("ErrorAckn",   BOOL_OLD, "W",  "",     "Acknowledge a cleared fault (write 1)"),
    10:  Param("PumpgStatn",  BOOL_OLD, "RW", "",     "PUMPING STATION master on/off: 1 = start the station (spins the pump up) + ack errors, 0 = stop"),
    12:  Param("EnableVent",  BOOL_OLD, "RW", "",     "Allow venting on/off"),
    17:  Param("CfgSpdSwPt",  U_SHORT,  "RW", "",     "Which speed switch-points are active (0=SwPt1, 1=SwPt1&2)"),
    19:  Param("CfgDO2",      U_SHORT,  "RW", "",     "Function assigned to digital output DO2"),
    23:  Param("MotorPump",   BOOL_OLD, "RW", "",     "TURBO MOTOR on/off (the spinning motor itself); 1 = run"),
    24:  Param("CfgDO1",      U_SHORT,  "RW", "",     "Function assigned to digital output DO1"),
    25:  Param("OpModeBKP",   U_SHORT,  "RW", "",     "Backing-pump operating mode (continuous/intermittent/delayed)"),
    26:  Param("SpdSetMode",  U_SHORT,  "RW", "",     "Speed-setting mode on/off (run at a chosen % instead of full speed)"),
    27:  Param("GasMode",     U_SHORT,  "RW", "",     "Gas-type compensation (0=heavy, 1=light, 2=helium)"),
    30:  Param("VentMode",    U_SHORT,  "RW", "",     "Venting mode (0=delayed, 1=none, 2=direct)"),
    35:  Param("CfgAccA1",    U_SHORT,  "RW", "",     "Function on accessory connector A1 (fan, venting valve, backing pump, …)"),
    36:  Param("CfgAccB1",    U_SHORT,  "RW", "",     "Function on accessory connector B1"),
    37:  Param("CfgAccA2",    U_SHORT,  "RW", "",     "Function on accessory connector A2"),
    38:  Param("CfgAccB2",    U_SHORT,  "RW", "",     "Function on accessory connector B2"),
    41:  Param("Press1HVen",  U_SHORT,  "RW", "",     "Enable integrated HV sensor (IKT gauges only)"),
    50:  Param("SealingGas",  BOOL_OLD, "RW", "",     "Sealing-gas valve on/off"),
    55:  Param("CfgAO1",      U_SHORT,  "RW", "",     "Function on analog output AO1 (speed/current/pressure/…)"),
    60:  Param("CtrlViaInt",  U_SHORT,  "RW", "",     "WHO may command the drive: 1=remote(pins), 2=RS-485, 4=PV.can, 8=Fieldbus, 16=E74, 255=unlock. Set 2 for bus control."),
    61:  Param("IntSelLckd",  BOOL_OLD, "RW", "",     "Lock the interface selection"),
    62:  Param("CfgDI1",      U_SHORT,  "RW", "",     "Function assigned to digital input DI1"),
    63:  Param("CfgDI2",      U_SHORT,  "RW", "",     "Function assigned to digital input DI2"),
    # 6.3 status requests (read-only) -----------------------------------------
    300: Param("RemotePrio",  BOOL_OLD, "R",  "",     "Remote priority active (hardware pins override the bus)"),
    302: Param("SpdSwPtAtt",  BOOL_OLD, "R",  "",     "Speed switch-point reached"),
    303: Param("ErrorCode",   STR6,     "R",  "",     "Current error code ('000000' = no error)"),
    304: Param("OvTempElec",  BOOL_OLD, "R",  "",     "Drive-electronics over-temperature"),
    305: Param("OvTempPump",  BOOL_OLD, "R",  "",     "Pump over-temperature"),
    306: Param("SetSpdAtt",   BOOL_OLD, "R",  "",     "Target speed reached (pump fully spun up)"),
    307: Param("PumpAccel",   BOOL_OLD, "R",  "",     "Pump is accelerating"),
    308: Param("SetRotSpdHz", U_INT,    "R",  "Hz",   "Set (target) rotation speed"),
    309: Param("ActualSpdHz", U_INT,    "R",  "Hz",   "ACTUAL rotation speed"),
    310: Param("DrvCurrent",  U_REAL,   "R",  "A",    "Motor drive current"),
    311: Param("OpHrsPump",   U_INT,    "R",  "h",    "Pump operating hours"),
    312: Param("FwVersion",   STR6,     "R",  "",     "Drive firmware version"),
    313: Param("DrvVoltage",  U_REAL,   "R",  "V",    "Motor drive voltage"),
    314: Param("OpHrsElec",   U_INT,    "R",  "h",    "Electronics operating hours"),
    315: Param("NominalSpdHz", U_INT,   "R",  "Hz",   "Nominal (rated) rotation speed"),
    316: Param("DrvPower",    U_INT,    "R",  "W",    "Motor drive power"),
    319: Param("PumpCycles",  U_INT,    "R",  "",     "Number of on/off cycles"),
    326: Param("TempElec",    U_INT,    "R",  "C",    "Electronics temperature"),
    330: Param("TempPmpBot",  U_INT,    "R",  "C",    "Pump bottom-part temperature"),
    336: Param("AccelDecel",  U_INT,    "R",  "rpm/s", "Current acceleration/deceleration"),
    337: Param("SealGasFlw",  U_INT,    "R",  "sccm", "Sealing-gas flow"),
    342: Param("TempBearng",  U_INT,    "R",  "C",    "BEARING temperature"),
    346: Param("TempMotor",   U_INT,    "R",  "C",    "MOTOR temperature"),
    349: Param("ElecName",    STR6,     "R",  "",     "Drive-unit model name"),
    354: Param("HWVersion",   STR6,     "R",  "",     "Drive hardware version"),
    360: Param("ErrHist1",    STR6,     "R",  "",     "Error history, most recent"),
    361: Param("ErrHist2",    STR6,     "R",  "",     "Error history, item 2"),
    362: Param("ErrHist3",    STR6,     "R",  "",     "Error history, item 3"),
    363: Param("ErrHist4",    STR6,     "R",  "",     "Error history, item 4"),
    364: Param("ErrHist5",    STR6,     "R",  "",     "Error history, item 5"),
    365: Param("ErrHist6",    STR6,     "R",  "",     "Error history, item 6"),
    366: Param("ErrHist7",    STR6,     "R",  "",     "Error history, item 7"),
    367: Param("ErrHist8",    STR6,     "R",  "",     "Error history, item 8"),
    368: Param("ErrHist9",    STR6,     "R",  "",     "Error history, item 9"),
    369: Param("ErrHist10",   STR6,     "R",  "",     "Error history, item 10 (oldest)"),
    397: Param("SetRotSpdRpm", U_INT,   "R",  "rpm",  "Set rotation speed (rpm)"),
    398: Param("ActualSpdRpm", U_INT,   "R",  "rpm",  "Actual rotation speed (rpm)"),
    399: Param("NominalSpdRpm", U_INT,  "R",  "rpm",  "Nominal rotation speed (rpm)"),
    # 6.4 set-value settings --------------------------------------------------
    700: Param("RUTimeSVal",  U_INT,    "RW", "min",  "Allowed run-up time"),
    701: Param("SpdSwPt1",    U_INT,    "RW", "%",    "Speed switch-point 1 (% of nominal)"),
    707: Param("SpdSVal",     U_REAL,   "RW", "%",    "Target speed in speed-setting mode (%)"),
    708: Param("PwrSVal",     U_SHORT,  "RW", "%",    "Power-consumption limit (%)"),
    710: Param("SwoffBKP",    U_INT,    "RW", "W",    "Backing-pump switch-OFF threshold (interval mode)"),
    711: Param("SwOnBKP",     U_INT,    "RW", "W",    "Backing-pump switch-ON threshold (interval mode)"),
    717: Param("StdbySVal",   U_REAL,   "RW", "%",    "Standby speed (% of nominal)"),
    719: Param("SpdSwPt2",    U_INT,    "RW", "%",    "Speed switch-point 2 (%)"),
    720: Param("VentSpd",     U_SHORT,  "RW", "%",    "Speed at which delayed venting starts (%)"),
    721: Param("VentTime",    U_INT,    "RW", "s",    "Venting time (delayed venting)"),
    730: Param("PrsSwPt1",    U_EXPO,   "RW", "hPa",  "Pressure switch-point 1"),
    732: Param("PrsSwPt2",    U_EXPO,   "RW", "hPa",  "Pressure switch-point 2"),
    739: Param("PrsSn1Name",  STR6,     "R",  "",     "Pressure-sensor 1 name"),
    740: Param("Pressure1",   U_EXPO,   "RW", "hPa",  "Pressure reading from gauge 1 (ActiveLine/DigiLine gauge on the drive)"),
    742: Param("PrsCorrPi1",  U_REAL,   "RW", "",     "Pressure correction factor, gauge 1"),
    749: Param("PrsSn2Name",  STR6,     "R",  "",     "Pressure-sensor 2 name"),
    750: Param("Pressure2",   U_EXPO,   "RW", "hPa",  "Pressure reading from gauge 2"),
    752: Param("PrsCorrPi2",  U_REAL,   "RW", "",     "Pressure correction factor, gauge 2"),
    777: Param("NomSpdConf",  U_INT,    "RW", "Hz",   "Nominal-speed confirmation"),
    791: Param("SlgWrnThrs",  U_INT,    "RW", "sccm", "Sealing-gas flow warning threshold"),
    797: Param("RS485Adr",    U_INT,    "RW", "",     "This drive's RS-485 node address (1..255)"),
    # 6.5 DCU / ActiveLine extras ---------------------------------------------
    340: Param("Pressure",    U_EXPO,   "R",  "hPa",  "Actual pressure value (ActiveLine)"),
    350: Param("CtrName",     STR6,     "R",  "",     "Connected display/control panel type"),
    351: Param("CtrSoftware", STR6,     "R",  "",     "Display/control panel software version"),
    738: Param("GaugeType",   STR6,     "RW", "",     "Type of attached pressure gauge"),
    794: Param("ParamSet",    U_SHORT,  "RW", "",     "Parameter set (0=basic, 1=extended)"),
}
_NAME2NUM = {p.name.lower(): n for n, p in PARAMS.items()}

CTRL_VIA_RS485 = 2                                  # P:060 value for RS-485 control
_ERROR_PAYLOADS = {"NO_DEF", "_RANGE", "_LOGIC"}    # manual §5.2.2


class PfeifferError(Exception):
    """Protocol/transport error (bad checksum, NO_DEF/_RANGE/_LOGIC, timeout, …)."""


# ── data type encode / decode (manual §5.2.5) ─────────────────────────────────
def _decode(dtype: int, data: str):
    if dtype == BOOL_OLD:  return data == "111111"
    if dtype == BOOL_NEW:  return data == "1"
    if dtype == U_INT:     return int(data)
    if dtype == U_SHORT:   return int(data)
    if dtype == U_REAL:    return int(data) / 100.0
    if dtype == U_EXPO:    return (int(data[:4]) / 1000.0) * 10.0 ** (int(data[4:6]) - 20)
    if dtype in (STR6, STR16): return data.strip()
    raise PfeifferError(f"unknown data type {dtype}")


def _encode(dtype: int, value) -> str:
    if dtype == BOOL_OLD:  return "111111" if value else "000000"
    if dtype == BOOL_NEW:  return "1" if value else "0"
    if dtype == U_INT:     return f"{int(value):06d}"
    if dtype == U_SHORT:   return f"{int(value):03d}"
    if dtype == U_REAL:    return f"{int(round(float(value) * 100)):06d}"
    if dtype == STR6:      return f"{str(value):<6}"[:6]
    if dtype == STR16:     return f"{str(value):<16}"[:16]
    if dtype == U_EXPO:
        v = float(value)
        if v <= 0:
            return "000020"
        e = max(0, min(99, math.floor(math.log10(v)) + 20))
        mant = max(0, min(9999, round(v / 10.0 ** (e - 20) * 1000)))
        return f"{mant:04d}{e:02d}"
    raise PfeifferError(f"unknown data type {dtype}")


# ── telegram framing ──────────────────────────────────────────────────────────
def _checksum(body: str) -> str:
    return f"{sum(ord(c) for c in body) % 256:03d}"


def _build(addr: int, action: str, param: int, data: str) -> bytes:
    if not 0 <= addr <= 255:
        raise ValueError(f"address {addr} out of range 0..255")
    body = f"{addr:03d}{action}{param:03d}{len(data):02d}{data}"
    return (body + _checksum(body) + "\r").encode("ascii")


def _read_telegram(ser, max_bytes: int = 96) -> str:
    buf = bytearray()
    while len(buf) < max_bytes:
        b = ser.read(1)
        if not b:                       # serial timeout
            break
        if b == b"\r":
            return buf.decode("ascii", "replace")
        buf += b
    raise PfeifferError(f"no CR-terminated reply (got {bytes(buf)!r})")


def _parse(resp: str):
    if len(resp) < 13:
        raise PfeifferError(f"reply too short: {resp!r}")
    try:
        length = int(resp[8:10])
    except ValueError as e:
        raise PfeifferError(f"bad length field in {resp!r}") from e
    body, data = resp[:10 + length], resp[10:10 + length]
    chk = resp[10 + length:10 + length + 3]
    if _checksum(body) != chk:
        raise PfeifferError(f"checksum mismatch in {resp!r} (want {_checksum(body)}, got {chk})")
    if data in _ERROR_PAYLOADS:
        raise PfeifferError(f"TC110 returned {data} for param {resp[5:8]}")
    return resp[0:3], resp[3:5], resp[5:8], data


def _query_raw(ser, addr: int, param: int) -> str:
    ser.write(_build(addr, "00", param, "=?"))
    _, _, rparam, data = _parse(_read_telegram(ser))
    if int(rparam) != param:
        raise PfeifferError(f"reply was for P:{rparam}, expected P:{param:03d}")
    return data


def _control_raw(ser, addr: int, param: int, data: str) -> None:
    ser.write(_build(addr, "10", param, data))
    _, _, rparam, rdata = _parse(_read_telegram(ser))
    if int(rparam) != param or rdata != data:
        raise PfeifferError(f"P:{param:03d} not acknowledged (reply P:{rparam} data {rdata!r})")


# ── generic typed access (the whole parameter set) ────────────────────────────
def _resolve(param) -> int:
    """Accept a parameter number or its display name (case-insensitive)."""
    if isinstance(param, int):
        return param
    num = _NAME2NUM.get(str(param).lower())
    if num is None:
        raise PfeifferError(f"unknown parameter {param!r}")
    return num


def read_parameter(ser, addr: int, param):
    """Read any TC110 parameter (number or name) → decoded Python value."""
    num = _resolve(param)
    spec = PARAMS.get(num)
    if spec and "R" not in spec.access:
        raise PfeifferError(f"P:{num:03d} ({spec.name}) is write-only")
    raw = _query_raw(ser, addr, num)
    return _decode(spec.dtype, raw) if spec else raw


def write_parameter(ser, addr: int, param, value) -> None:
    """Write any writable TC110 parameter (number or name), encoding by data type."""
    num = _resolve(param)
    spec = PARAMS.get(num)
    if spec is None:
        raise PfeifferError(f"P:{num:03d} not in the parameter table")
    if "W" not in spec.access:
        raise PfeifferError(f"P:{num:03d} ({spec.name}) is read-only")
    _control_raw(ser, addr, num, _encode(spec.dtype, value))


# ── named convenience (names match the common turbo operations) ────────────────────
def read_rotation_speed(ser, addr: int) -> int:
    """Actual rotation speed, Hz (P:309)."""
    return int(read_parameter(ser, addr, 309))


def read_bearing_temperature(ser, addr: int) -> float:
    """Bearing temperature, °C (P:342)."""
    return float(read_parameter(ser, addr, 342))


def read_motor_temperature(ser, addr: int) -> float:
    """Motor temperature, °C (P:346)."""
    return float(read_parameter(ser, addr, 346))


def read_drive_power(ser, addr: int) -> int:
    """Drive power, W (P:316)."""
    return int(read_parameter(ser, addr, 316))


def at_set_speed(ser, addr: int) -> bool:
    """True once the target rotation speed is reached (P:306)."""
    return bool(read_parameter(ser, addr, 306))


def read_error_code(ser, addr: int) -> str:
    """Drive/gauge error code (P:303); '000000' = no error."""
    return read_parameter(ser, addr, 303)


def read_pressure(ser, addr: int) -> float:
    """Pressure in mbar from a Pfeiffer DigiLine gauge (e.g. MPT 200) on the bus —
    reads P:740 (u_expo, hPa) and returns it as mbar (1 hPa = 1 mbar). The DigiLine
    gauges speak this same Pfeiffer Vacuum protocol; their address is the rotary
    switch (1..16). Verified vs the MPT 200 manual: P:740@001 query checksum '106',
    '100023' → 1000 mbar."""
    return float(read_parameter(ser, addr, 740))


def set_motor(ser, addr: int, on: bool) -> None:
    """Switch the turbo motor on/off (P:023)."""
    write_parameter(ser, addr, 23, on)


def enable_pumping_station(ser, addr: int, on: bool) -> None:
    """Switch the pumping station on/off (P:010). Use spin() for the full sequence."""
    write_parameter(ser, addr, 10, on)


def spin(ser, addr: int, on: bool) -> None:
    """Spin the turbo up/down using the manual's sequence. Up (§7.3): motor (P:023)
    then pumping station (P:010). Down (§7.4): clear the pumping station (controlled
    deceleration + configured venting). Requires set_control_via_rs485() first."""
    if on:
        set_motor(ser, addr, True)
        enable_pumping_station(ser, addr, True)
    else:
        enable_pumping_station(ser, addr, False)


def set_control_via_rs485(ser, addr: int) -> None:
    """Put the drive under RS-485 control so it accepts commands (P:060 = 2)."""
    write_parameter(ser, addr, 60, CTRL_VIA_RS485)


def set_rs485_address(ser, addr: int, new_addr: int) -> None:
    """Change the drive's RS-485 node address (P:797). Commissioning only."""
    if not 1 <= new_addr <= 255:
        raise ValueError("RS-485 address must be 1..255")
    write_parameter(ser, addr, 797, new_addr)


def acknowledge_error(ser, addr: int) -> None:
    """Acknowledge a cleared fault (P:009)."""
    write_parameter(ser, addr, 9, True)


# ── reference: print the whole parameter table ────────────────────────────────
def print_parameters(file=None) -> None:
    """Print every supported TC110 parameter and what it does (a quick reference)."""
    import sys
    f = file or sys.stdout
    print(f"{'P#':>4}  {'name':<13} {'type':<9} {'acc':<3} {'unit':<5} description", file=f)
    print("-" * 96, file=f)
    for num in sorted(PARAMS):
        p = PARAMS[num]
        print(f"{num:>4}  {p.name:<13} {_DTYPE_NAME[p.dtype]:<9} {p.access:<3} "
              f"{p.unit:<5} {p.desc}", file=f)


if __name__ == "__main__":
    print_parameters()
