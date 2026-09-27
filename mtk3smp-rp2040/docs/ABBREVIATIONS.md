# PicoCar — Abbreviations and Acronyms

BARR-C:2018 Rule 1.5.b asks for a version-controlled table of the
project-specific abbreviations used in names and comments. Anything not listed
here is spelled out in full. Units are written as suffixes (`_mm`, `_ms`,
`_us`, `_deg`, `_dps`, `_pct`, `_mg`, `_mm_s`, `_mm_s2`).

## Units and physical quantities

| Abbreviation | Meaning |
|---|---|
| `deg` | degrees |
| `dps` | degrees per second |
| `hz` | hertz |
| `mg` | milli-g (1/1000 of standard gravity), accelerometer unit |
| `mm`, `mm_s`, `mm_s2` | millimetres, millimetres per second, millimetres per second squared |
| `ms`, `us` | milliseconds, microseconds |
| `pct` | percent (PWM duty) |
| `permille` | parts per thousand (normalised IR readings) |
| `rad` | radians |
| `dt`, `ds` | time step, distance step |

## Hardware and protocols

| Abbreviation | Meaning |
|---|---|
| `adc` | analogue-to-digital converter |
| `cdc` | USB Communications Device Class (the USB serial console) |
| `cyw43` | Infineon CYW43439 WiFi chip on the Pico W, and its driver |
| `dmb` | Data Memory Barrier (Arm instruction) |
| `gpio` | general-purpose input/output |
| `i2c` | Inter-Integrated Circuit bus |
| `imu` | inertial measurement unit (the LSM303DLHC on the GY-511 board) |
| `ir` | infrared (reflectance sensors) |
| `irq`, `isr` | interrupt request, interrupt service routine |
| `lwip` | lightweight IP, the TCP/IP stack used by the template |
| `lwt` | MQTT last will and testament |
| `mqtt` | MQ Telemetry Transport, version 3.1.1 |
| `nvic` | Nested Vectored Interrupt Controller (Cortex-M0+) |
| `pwm` | pulse-width modulation |
| `qos` | MQTT quality of service |
| `rx`, `tx` | receive, transmit |
| `scl`, `sda` | I2C clock and data lines |
| `sdk` | the Raspberry Pi Pico C SDK |
| `sio` | RP2040 single-cycle I/O block |
| `spsc` | single-producer / single-consumer (lock-free ring) |
| `tcp` | Transmission Control Protocol |
| `uart` | universal asynchronous receiver-transmitter |
| `usb` | Universal Serial Bus |

## Control and algorithms

| Abbreviation | Meaning |
|---|---|
| `corr` | correction |
| `ema` | exponential moving average |
| `ff`, `gain_ff` | feed-forward |
| `integ` | integral term of a PID controller |
| `kp`, `ki`, `kd`, `kf` | proportional, integral, derivative and feed-forward gains |
| `lpf` | low-pass filter |
| `pid` | proportional-integral-derivative controller |
| `q4` | fixed-point format with 4 fraction bits |
| `unsat` | unsaturated (before output clamping) |

## Project names

| Abbreviation | Meaning |
|---|---|
| `avoid`, `obst` | obstacle avoidance, obstacle |
| `bc` | barcode |
| `btn` | button (GP20 START, GP21 STOP) |
| `buf` | buffer |
| `c39` | Code 39 barcode symbology |
| `calib`, `cal` | calibration |
| `cb` | callback function |
| `cmd` | command |
| `ctrl` | control |
| `dir` | direction (+1 or −1) |
| `enc` | wheel encoder |
| `evt` | event |
| `hb` | heartbeat |
| `hdg` | heading |
| `hump` | speed hump on the course |
| `idx` | index |
| `len` | length |
| `lf` | line follower |
| `mag` | magnetometer |
| `mbf`, `mbuf` | µT-Kernel message buffer |
| `nav` | navigation command decoded from a barcode (left, right, straight, U-turn) |
| `norm` | normalised (0 = white, 1 = black) |
| `odo` | odometer (centre-line distance) |
| `pkt` | packet |
| `pos` | position |
| `prev` | previous |
| `prof` | obstacle profile |
| `reacq` | reacquire (the line) |
| `seq` | sequence number |
| `snap` | snapshot (a consistent copy of shared data) |
| `st` | status |
| `tel` | telemetry |
| `veh` | vehicle |
| `vs_`, `vcmd_` | vehicle state and vehicle command enumerations |

## Naming prefixes (BARR-C Rules 7.1.j–7.1.o)

| Prefix | Meaning |
|---|---|
| `g` | global (file-scope) variable, e.g. `g_status` |
| `p`, `pp` | pointer, pointer to pointer, e.g. `p_out`, `pp_topic` |
| `b` | Boolean, e.g. `b_done`, `gb_calibrating`, `pb_retain` |
| `h` | kernel object handle (µT-Kernel `ID`), e.g. `h_task`, `gh_mbuf` |

## µT-Kernel types and calls (fixed by the kernel)

| Name | Meaning |
|---|---|
| `ID`, `ER`, `INT`, `UINT`, `UB`, `UW`, `W`, `TMO`, `FP`, `SYSTIM` | kernel data types: object ID, error code, native int, native unsigned, unsigned byte, unsigned word, signed word, time-out, function pointer, system time |
| `DI`, `EI` | disable / enable interrupts (short critical sections) |
| `tk_cre_*`, `tk_sta_tsk`, `tk_dly_tsk`, `tk_wai_flg`, `tk_snd_mbf`, `tk_rcv_mbf` | kernel service calls (create, start task, delay task, wait flag, send / receive message buffer) |
| `TA_HLNG`, `TA_RNG3`, `TA_TFIFO` | kernel object attributes (high-level-language handler, protection ring 3, FIFO wait queue) |
