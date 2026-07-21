===============================================================================
                         VN-100 / STM32 COMMAND REFERENCE
===============================================================================

> This document lists the HEX equivalents of commands sent through the STM32 bridge
> (`VN RAW ...`). All checksums have been cross-verified with `pyvn100.protocol.xor_checksum`
> and the examples in VN100_ICD_fw3.1.0.0.pdf. HEX sequences end each line with a **single
> `\n`** — `host_link.c` only processes a host command line once it sees `\n`, and silently
> discards any incoming `\r` (which is why Tera Term's default CR+LF line ending also works
> fine).

Quick Commands
-------------------------------------------------------------------------------

Stop the Stream
----------------
STM Command : VN RAW $VNASY,0*4F
HEX         : 56 4E 20 52 41 57 20 24 56 4E 41 53 59 2C 30 2A 34 46 0A
Note        : Stops continuous sensor output, reducing the UART interrupt load on the STM32.

Reset the Sensor
-----------------
STM Command : VN RAW $VNRST*4D
HEX         : 56 4E 20 52 41 57 20 24 56 4E 52 53 54 2A 34 44 0A
Note        : Triggers a software reboot of the sensor without disconnecting the physical link.


===============================================================================
ASYNC OUTPUT CONTROL
===============================================================================

Purpose               : Pause Async Output
Send to STM           : VN RAW $VNASY,0*4F
ASCII Sent to VN-100  : $VNASY,0*4F
HEX                   : 56 4E 20 52 41 57 20 24 56 4E 41 53 59 2C 30 2A 34 46 0A
Note                  : Used to silence asynchronous data noise while making configuration changes, so the command/response flow stays clean.


Purpose               : Resume Async Output
Send to STM           : VN RAW $VNASY,1*4E
ASCII Sent to VN-100  : $VNASY,1*4E
HEX                   : 56 4E 20 52 41 57 20 24 56 4E 41 53 59 2C 31 2A 34 45 0A
Note                  : Restarts the paused data stream so the sensor resumes sending live orientation data.


===============================================================================
GENERAL COMMANDS
===============================================================================

1) Connectivity Test
---------------------
STM Command : VN PING
HEX         : 56 4E 20 50 49 4E 47 0A
Note        : The simplest query to verify that UART communication between the STM32 and the VN-100 is working, both physically and logically. Response: `VNPONG` (produced only by the STM32 bridge — not returned over a direct connection).


2) ASCII Mode
--------------
STM Command : VN MODE ASCII
HEX         : 56 4E 20 4D 4F 44 45 20 41 53 43 49 49 0A
Note        : Switches the sensor output to human-readable text ($VNYMR), making it easy to monitor data in a terminal during debugging.


3) Binary Mode
---------------
STM Command : VN MODE BINARY
HEX         : 56 4E 20 4D 4F 44 45 20 42 49 4E 41 52 59 0A
Note        : Switches output to a compact binary format, reducing bandwidth and letting the STM32 process data much faster. Binary mode does NOT include magnetometer fields (they change slowly and are read via ASCII/registers instead).


4) Stop Async
---------------
STM Command : VN RAW $VNASY,0*4F
ASCII       : $VNASY,0*4F
HEX         : 56 4E 20 52 41 57 20 24 56 4E 41 53 59 2C 30 2A 34 46 0A
Note        : Stops asynchronous output via a raw command; does NOT change any registers — it simply shields configuration writes from the live telemetry stream.


5) Start Async
----------------
STM Command : VN RAW $VNASY,1*4E
ASCII       : $VNASY,1*4E
HEX         : 56 4E 20 52 41 57 20 24 56 4E 41 53 59 2C 31 2A 34 45 0A
Note        : Re-enables asynchronous output via a raw command, so the sensor resumes sending data packets at the configured rate.


6) Save Settings to Flash
---------------------------
STM Command : VN SAVE
or          : VN RAW $VNWNV*57
ASCII       : $VNWNV*57
HEX (SAVE)  : 56 4E 20 53 41 56 45 0A
HEX (RAW)   : 56 4E 20 52 41 57 20 24 56 4E 57 4E 56 2A 35 37 0A
Note        : Writes the sensor's active RAM settings to persistent flash memory, so they survive a power cycle.


7) Factory Settings
---------------------
STM Command : VN FACTORY
or          : VN RAW $VNRFS*5F
ASCII       : $VNRFS*5F
HEX (FACTORY) : 56 4E 20 46 41 43 54 4F 52 59 0A
HEX (RAW)     : 56 4E 20 52 41 57 20 24 56 4E 52 46 53 2A 35 46 0A
Note        : Restores all of the sensor's calibration and communication settings to the manufacturer's factory defaults.


8) Software Reset
-------------------
STM Command : VN RESET
or          : VN RAW $VNRST*4D
ASCII       : $VNRST*4D
HEX (RESET) : 56 4E 20 52 45 53 45 54 0A
HEX (RAW)   : 56 4E 20 52 41 57 20 24 56 4E 52 53 54 2A 34 44 0A
Note        : Reboots the sensor's processor — useful for simulating a power interruption or applying certain newly saved settings.


9) Capture Gyro Bias
-----------------------
STM Command : VN RAW $VNSGB*4E
ASCII       : $VNSGB*4E
HEX         : 56 4E 20 52 41 57 20 24 56 4E 53 47 42 2A 34 45 0A
Note        : With the sensor held ABSOLUTELY still, copies the Kalman filter's current gyro bias
              estimate (not zero) into the Filter Startup Gyro Bias register (Reg 43 on FW 3.1.0.0,
              Reg 74 on FW 2.1). Requires a follow-up `VN SAVE` ($VNWNV) to persist it; the sensor
              then uses this value as its startup bias on the next power-up, reducing startup drift.


===============================================================================
REGISTER READ COMMANDS
===============================================================================

Register 4
----------
Meaning     : Firmware Version
STM Command : VN RAW $VNRRG,4*47
ASCII       : $VNRRG,4*47
HEX         : 56 4E 20 52 41 57 20 24 56 4E 52 52 47 2C 34 2A 34 37 0A
Note        : Reads the sensor's embedded firmware version (the first step during bring-up: which ICD generation applies — see `docs/protocol.md` §5.3).


Register 6
----------
Meaning     : Async Data Output Type (ADOR)
STM Command : VN RAW $VNRRG,6*45
ASCII       : $VNRRG,6*45
HEX         : 56 4E 20 52 41 57 20 24 56 4E 52 52 47 2C 36 2A 34 35 0A
Note        : Queries which ASCII message type is being broadcast asynchronously (in this project, only `14` = $VNYMR); `0` = off.


Register 7
----------
Meaning     : Async Data Output Freq (ADOF)
STM Command : VN RAW $VNRRG,7*44
ASCII       : $VNRRG,7*44
HEX         : 56 4E 20 52 41 57 20 24 56 4E 52 52 47 2C 37 2A 34 34 0A
Note        : Reads the output frequency (Hz) of the ASCII message selected in ADOR.


Register 23
-----------
Meaning     : Magnetometer Calibration
STM Command : VN RAW $VNRRG,23*72
ASCII       : $VNRRG,23*72
HEX         : 56 4E 20 52 41 57 20 24 56 4E 52 52 47 2C 32 33 2A 37 32 0A
Note        : Reads the user hard/soft-iron calibration matrix (C 3x3 + B 3x1); always active.


Register 43
-----------
Meaning     : Filter Startup Gyro Bias (FW 3.1.0.0; was Reg 74 on FW 2.1)
STM Command : VN RAW $VNRRG,43*74
ASCII       : $VNRRG,43*74
HEX         : 56 4E 20 52 41 57 20 24 56 4E 52 52 47 2C 34 33 2A 37 34 0A
Note        : Reads the gyro bias value the AHRS filter uses at startup — not a continuously updated telemetry value, but a startup-state register populated by `$VNSGB`.


Register 44
-----------
Meaning     : Real-Time HSI Control
STM Command : VN RAW $VNRRG,44*73
ASCII       : $VNRRG,44*73
HEX         : 56 4E 20 52 41 57 20 24 56 4E 52 52 47 2C 34 34 2A 37 33 0A
Note        : Reads the current Mode/ApplyCompensation/ConvergeRate values of the onboard HSI (Hard/Soft Iron) algorithm. On this firmware the factory default is `0,1,5` (Off, Disable) — onboard HSI ships OFF.


Register 47
-----------
Meaning     : Real-Time HSI Results
STM Command : VN RAW $VNRRG,47*70
ASCII       : $VNRRG,47*70
HEX         : 56 4E 20 52 41 57 20 24 56 4E 52 52 47 2C 34 37 2A 37 30 0A
Note        : Reads the C/B solution computed by the onboard HSI algorithm; its input is the magnetometer reading after Reg 23 has been applied. On this firmware, convergence is measured from this register (there is no Reg 46).


Register 75
-----------
Meaning     : Binary Output Message Configuration #1
STM Command : VN RAW $VNRRG,75*71
ASCII       : $VNRRG,75*71
HEX         : 56 4E 20 52 41 57 20 24 56 4E 52 52 47 2C 37 35 2A 37 31 0A
Note        : Queries the binary output's AsyncMode/RateDivisor/OutputGroup/OutputField configuration.


===============================================================================
REGISTER WRITE COMMANDS
===============================================================================

ASCII Type = VNYMR
--------------------
STM Command : VN RAW $VNWRG,6,14*69
HEX         : 56 4E 20 52 41 57 20 24 56 4E 57 52 47 2C 36 2C 31 34 2A 36 39 0A
Note        : Sets the asynchronous output type to "YMR" ($VNYMR) — combined Yaw, Pitch, Roll, Magnetometer, Accelerometer, and Gyroscope data.


ASCII Frequency = 50 Hz
--------------------------
STM Command : VN RAW $VNWRG,7,50*68
HEX         : 56 4E 20 52 41 57 20 24 56 4E 57 52 47 2C 37 2C 35 30 2A 36 38 0A
Note        : Sets the output rate to 50 packets/second — a balanced frequency for standard control loops and logging.


ASCII Frequency = 10 Hz
--------------------------
STM Command : VN RAW $VNWRG,7,10*6C
HEX         : 56 4E 20 52 41 57 20 24 56 4E 57 52 47 2C 37 2C 31 30 2A 36 43 0A
Note        : Lowers the output rate to 10 packets/second, saving bandwidth for low-power or slow-tracking use cases.


Enable Binary Output (200 Hz, Port 2)
----------------------------------------
STM Command : VN RAW $VNWRG,75,2,4,01,0128*78
HEX         : 56 4E 20 52 41 57 20 24 56 4E 57 52 47 2C 37 35 2C 32 2C 34 2C 30 31 2C 30 31 32 38 2A 37 38 0A
Note        : On Register 75, enables binary output with AsyncMode=2 (TTL Serial Port 2), RateDivisor=4 (800/4 = 200 Hz), OutputGroup=Common, OutputField=YPR+AngularRate+Accel.


Disable Binary Output
------------------------
STM Command : VN RAW $VNWRG,75,0,4,01,0128*7A
HEX         : 56 4E 20 52 41 57 20 24 56 4E 57 52 47 2C 37 35 2C 30 2C 34 2C 30 31 2C 30 31 32 38 2A 37 41 0A
Note        : Sets AsyncMode=0 to disable binary output, removing the STM32's high-frequency binary frame-parsing load.


===============================================================================
HSI (MAGNETOMETER CALIBRATION) — Register 44
===============================================================================

> Register 44 has 3 fields: Mode {0 Off, 1 Run, 2 Reset}, ApplyCompensation {1 Disable,
> 3 Enable}, ConvergeRate {1..5}. All three must be supplied on every write (see ICD §3.5.1
> example: `$VNWRG,44,1,1,5`).

HSI RESET + RUN (start convergence, apply to output)
-------------------------------------------------------
STM Command : VN RAW $VNWRG,44,2,3,5*6E
HEX         : 56 4E 20 52 41 57 20 24 56 4E 57 52 47 2C 34 34 2C 32 2C 33 2C 35 2A 36 45 0A
Note        : Mode=2 (RESET — clear the solution and run), ApplyCompensation=3 (Enable), ConvergeRate=5 (fast, ~15-20 s). Since HSI ships off on this firmware, this step also starts it.


HSI RUN (continue, without resetting)
----------------------------------------
STM Command : VN RAW $VNWRG,44,1,3,5*6D
HEX         : 56 4E 20 52 41 57 20 24 56 4E 57 52 47 2C 34 34 2C 31 2C 33 2C 35 2A 36 44 0A
Note        : Mode=1 (RUN), ApplyCompensation=3 (Enable), ConvergeRate=5.


HSI OFF (freeze)
-------------------
STM Command : VN RAW $VNWRG,44,0,3,5*6C
HEX         : 56 4E 20 52 41 57 20 24 56 4E 57 52 47 2C 34 34 2C 30 2C 33 2C 35 2A 36 43 0A
Note        : Mode=0 (OFF) while ApplyCompensation stays 3 → the algorithm stops but keeps applying its last converged solution ("freeze"). Per UM001: "once a valid solution is found in a static environment, turn HSI OFF."


Read Status
-------------
STM Command : VN RAW $VNRRG,44*73
HEX         : 56 4E 20 52 41 57 20 24 56 4E 52 52 47 2C 34 34 2A 37 33 0A
Note        : Reads back the current HSI mode (Off/Run/Reset) along with the ApplyCompensation/ConvergeRate settings, to confirm the configuration.


Read Result
-------------
STM Command : VN RAW $VNRRG,47*70
HEX         : 56 4E 20 52 41 57 20 24 56 4E 52 52 47 2C 34 37 2A 37 30 0A
Note        : Reads the C/B solution computed so far by the running HSI algorithm; it is considered converged once the solution moves away from identity and settles across consecutive reads (Reg 46 does not exist on this firmware — see `docs/protocol.md` §5.3).

===============================================================================
