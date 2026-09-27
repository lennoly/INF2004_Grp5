# 3. Hardware and Wiring

![Wiring diagram](img/wiring.png){width=100%}

| Device | Device pin | Pico / Robo Pico | Notes |
|---|---|---|---|
| Left motor | + / − | M1 terminals (GP8 / GP9) | MxA=PWM, MxB=0 → forward; swap wires or set `MOTOR_L_INVERT` if reversed |
| Right motor | + / − | M2 terminals (GP10 / GP11) | `MOTOR_R_INVERT` likewise |
| Left encoder | VCC / GND / D0 | 3V3 / GND / **GP2** | Grove 2 |
| Right encoder | VCC / GND / D0 | 3V3 / GND / **GP3** | Grove 2 |
| IR left | VCC / GND / **A0** | 3V3 / GND / **GP26 (ADC0)** | use analog output, D0 unused |
| IR right | VCC / GND / **A0** | 3V3 / GND / **GP27 (ADC1)** | |
| IR barcode | VCC / GND / **A0** | 3V3 / GND / **GP28 (ADC2)** | |
| GY-511 IMU | VIN / GND / SDA / SCL | 3V3 / GND / **GP4 / GP5** | I2C0 at 400 kHz; chip X axis forward, Z up |
| HC-SR04 | VCC / GND / TRIG / ECHO | 3V3 / GND / **GP6 / GP7** | per course slide; a 5 V module needs a divider on ECHO |
| Servo | signal / V+ / GND | **Servo port GP12** | 90° = straight ahead |
| Buttons | — | GP20 START, GP21 STOP | on-board |
| Power | — | VIN terminal 3.6–6 V | common ground for everything |

**Mounting notes.**

- **Line sensors.** Place the two sensors so each IR spot sits on one edge of the 18 mm line (≈18 mm apart). Centred, each reads ≈0.5, which gives the steepest, most linear error signal.
- **Barcode sensor.** Offset it to the barcode side, ≈45–50 mm from the line centre. The barcode lies 20 mm beyond the line edge (see the course barcode specification).
- **Grove port 5 and 6 share GP26.** Wire only the listed pins.

