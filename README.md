# pfeiffer-tc110

A tiny, dependency-free Python driver for the **Pfeiffer TC110** turbopump electronic
drive unit over **RS-485** (the *Pfeiffer Vacuum protocol*). Pure stdlib — you bring a
pyserial-like port.

It implements the protocol straight from the TC110 manual (§5.2 telegram frame, §6
parameter set): the **complete documented parameter set** is mapped, reachable via
generic `read_parameter()` / `write_parameter()`, with named convenience wrappers for
the common operations. Extracted from the `vacuum-storage-unified` project so it can be
reused and inspected on its own.

> **Why this exists:** the two public Pfeiffer libraries don't cover this — PyPI
> `pfeiffer-vacuum-protocol` (electronsandstuff) is *gauge-only*, and `tspspi/pfeifferpumps`
> is a low-level class encoder. Neither exposes `read_rotation_speed` / `enable_pumping_station`
> etc., so this fills the gap for the TC110 turbo drive.

## Install / use

No install needed — drop `pfeiffer_tc110.py` next to your code.

```python
import serial
import pfeiffer_tc110 as tc

s = serial.Serial("/dev/ttyUSB0", 9600, timeout=0.5)   # TC110: 9600 8N1
addr = 1                                                # the drive's RS-485 address (P:797)

print(tc.read_rotation_speed(s, addr), "Hz")           # actual speed
print(tc.read_bearing_temperature(s, addr), "°C")

tc.set_control_via_rs485(s, addr)                       # ONE-TIME: let the bus command it (P:060=2)
tc.spin(s, addr, True)                                  # spin up  (motor P:023, then station P:010)
tc.spin(s, addr, False)                                 # spin down (clears P:010)

# anything else in the table, generically (by number or name):
print(tc.read_parameter(s, addr, "DrvPower"), "W")
print(tc.read_parameter(s, addr, 313), "V")            # drive voltage
```

`python -m pfeiffer_tc110` (or `python pfeiffer_tc110.py`) prints the whole parameter
table — the same reference shown below.

## What "pumping station" and friends mean

The two that trip people up:

- **P:023 `MotorPump`** — the **turbo motor itself** (the thing that spins). `1` = run.
- **P:010 `PumpgStatn`** — the **pumping station master switch**: `1` starts the whole
  station (spins the pump up, *and* acknowledges errors), `0` stops it.

Per the manual the **spin-up order is motor (P:023) → station (P:010)**, and spin-down
just clears the station (P:010=0), which does a controlled deceleration + configured
venting. `spin()` does this for you. The drive only accepts these over the bus once it's
in **RS-485 control** (`P:060 = 2`, via `set_control_via_rs485()`).

## Protocol (manual §5.2)

Telegram: `aaa | AA | nnn | ll | data | ccc | CR` — 3-digit address, action (`00`=query,
`10`=write/response), 3-digit parameter, 2-digit length, data, checksum (Σ ASCII of all
preceding chars, mod 256, 3 digits), carriage return. Queries send data `"=?"`. Data
types: boolean, u_integer, u_real (×100), u_short_int, u_expo (pressure), string.

## Full TC110 parameter reference

Generated from the driver (`PARAMS`). `acc` = R read / W write / RW both.

```
  P#  name          type      acc unit  description
------------------------------------------------------------------------------------------------
   1  Heating       bool      RW        Casing heater on/off
   2  Standby       bool      RW        Run at reduced 'standby' speed on/off
   4  RUTimeCtrl    bool      RW        Run-up time monitoring on/off
   9  ErrorAckn     bool      W         Acknowledge a cleared fault (write 1)
  10  PumpgStatn    bool      RW        PUMPING STATION master on/off: 1 = start the station (spins the pump up) + ack errors, 0 = stop
  12  EnableVent    bool      RW        Allow venting on/off
  17  CfgSpdSwPt    u_short   RW        Which speed switch-points are active (0=SwPt1, 1=SwPt1&2)
  19  CfgDO2        u_short   RW        Function assigned to digital output DO2
  23  MotorPump     bool      RW        TURBO MOTOR on/off (the spinning motor itself); 1 = run
  24  CfgDO1        u_short   RW        Function assigned to digital output DO1
  25  OpModeBKP     u_short   RW        Backing-pump operating mode (continuous/intermittent/delayed)
  26  SpdSetMode    u_short   RW        Speed-setting mode on/off (run at a chosen % instead of full speed)
  27  GasMode       u_short   RW        Gas-type compensation (0=heavy, 1=light, 2=helium)
  30  VentMode      u_short   RW        Venting mode (0=delayed, 1=none, 2=direct)
  35  CfgAccA1      u_short   RW        Function on accessory connector A1 (fan, venting valve, backing pump, …)
  36  CfgAccB1      u_short   RW        Function on accessory connector B1
  37  CfgAccA2      u_short   RW        Function on accessory connector A2
  38  CfgAccB2      u_short   RW        Function on accessory connector B2
  41  Press1HVen    u_short   RW        Enable integrated HV sensor (IKT gauges only)
  50  SealingGas    bool      RW        Sealing-gas valve on/off
  55  CfgAO1        u_short   RW        Function on analog output AO1 (speed/current/pressure/…)
  60  CtrlViaInt    u_short   RW        WHO may command the drive: 1=remote(pins), 2=RS-485, 4=PV.can, 8=Fieldbus, 16=E74, 255=unlock. Set 2 for bus control.
  61  IntSelLckd    bool      RW        Lock the interface selection
  62  CfgDI1        u_short   RW        Function assigned to digital input DI1
  63  CfgDI2        u_short   RW        Function assigned to digital input DI2
 300  RemotePrio    bool      R         Remote priority active (hardware pins override the bus)
 302  SpdSwPtAtt    bool      R         Speed switch-point reached
 303  ErrorCode     string    R         Current error code ('000000' = no error)
 304  OvTempElec    bool      R         Drive-electronics over-temperature
 305  OvTempPump    bool      R         Pump over-temperature
 306  SetSpdAtt     bool      R         Target speed reached (pump fully spun up)
 307  PumpAccel     bool      R         Pump is accelerating
 308  SetRotSpdHz   u_integer R   Hz    Set (target) rotation speed
 309  ActualSpdHz   u_integer R   Hz    ACTUAL rotation speed
 310  DrvCurrent    u_real    R   A     Motor drive current
 311  OpHrsPump     u_integer R   h     Pump operating hours
 312  FwVersion     string    R         Drive firmware version
 313  DrvVoltage    u_real    R   V     Motor drive voltage
 314  OpHrsElec     u_integer R   h     Electronics operating hours
 315  NominalSpdHz  u_integer R   Hz    Nominal (rated) rotation speed
 316  DrvPower      u_integer R   W     Motor drive power
 319  PumpCycles    u_integer R         Number of on/off cycles
 326  TempElec      u_integer R   C     Electronics temperature
 330  TempPmpBot    u_integer R   C     Pump bottom-part temperature
 336  AccelDecel    u_integer R   rpm/s Current acceleration/deceleration
 337  SealGasFlw    u_integer R   sccm  Sealing-gas flow
 340  Pressure      u_expo    R   hPa   Actual pressure value (ActiveLine)
 342  TempBearng    u_integer R   C     BEARING temperature
 346  TempMotor     u_integer R   C     MOTOR temperature
 349  ElecName      string    R         Drive-unit model name
 350  CtrName       string    R         Connected display/control panel type
 351  CtrSoftware   string    R         Display/control panel software version
 354  HWVersion     string    R         Drive hardware version
 360  ErrHist1      string    R         Error history, most recent
 361  ErrHist2      string    R         Error history, item 2
 362  ErrHist3      string    R         Error history, item 3
 363  ErrHist4      string    R         Error history, item 4
 364  ErrHist5      string    R         Error history, item 5
 365  ErrHist6      string    R         Error history, item 6
 366  ErrHist7      string    R         Error history, item 7
 367  ErrHist8      string    R         Error history, item 8
 368  ErrHist9      string    R         Error history, item 9
 369  ErrHist10     string    R         Error history, item 10 (oldest)
 397  SetRotSpdRpm  u_integer R   rpm   Set rotation speed (rpm)
 398  ActualSpdRpm  u_integer R   rpm   Actual rotation speed (rpm)
 399  NominalSpdRpm u_integer R   rpm   Nominal rotation speed (rpm)
 700  RUTimeSVal    u_integer RW  min   Allowed run-up time
 701  SpdSwPt1      u_integer RW  %     Speed switch-point 1 (% of nominal)
 707  SpdSVal       u_real    RW  %     Target speed in speed-setting mode (%)
 708  PwrSVal       u_short   RW  %     Power-consumption limit (%)
 710  SwoffBKP      u_integer RW  W     Backing-pump switch-OFF threshold (interval mode)
 711  SwOnBKP       u_integer RW  W     Backing-pump switch-ON threshold (interval mode)
 717  StdbySVal     u_real    RW  %     Standby speed (% of nominal)
 719  SpdSwPt2      u_integer RW  %     Speed switch-point 2 (%)
 720  VentSpd       u_short   RW  %     Speed at which delayed venting starts (%)
 721  VentTime      u_integer RW  s     Venting time (delayed venting)
 730  PrsSwPt1      u_expo    RW  hPa   Pressure switch-point 1
 732  PrsSwPt2      u_expo    RW  hPa   Pressure switch-point 2
 738  GaugeType     string    RW        Type of attached pressure gauge
 739  PrsSn1Name    string    R         Pressure-sensor 1 name
 740  Pressure1     u_expo    RW  hPa   Pressure reading from gauge 1 (ActiveLine/DigiLine gauge on the drive)
 742  PrsCorrPi1    u_real    RW        Pressure correction factor, gauge 1
 749  PrsSn2Name    string    R         Pressure-sensor 2 name
 750  Pressure2     u_expo    RW  hPa   Pressure reading from gauge 2
 752  PrsCorrPi2    u_real    RW        Pressure correction factor, gauge 2
 777  NomSpdConf    u_integer RW  Hz    Nominal-speed confirmation
 791  SlgWrnThrs    u_integer RW  sccm  Sealing-gas flow warning threshold
 794  ParamSet      u_short   RW        Parameter set (0=basic, 1=extended)
 797  RS485Adr      u_integer RW        This drive's RS-485 node address (1..255)
```

## Tests

`python -m pytest` — frames + checksums are asserted byte-exact against the manual's
worked examples (query P:309 → `112`; "pump on" P:010 → `020`; the 633 Hz response),
plus the motor→station spin order, codecs, and error/timeout handling. No hardware.

## License / status

Unofficial, community driver — not affiliated with Pfeiffer Vacuum. Verify against your
own TC110 manual before relying on it for control. Parameter data transcribed from the
TC110 operating instructions (Pfeiffer Vacuum protocol, §5.2 / §6).
