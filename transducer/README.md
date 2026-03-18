# HX711 Force Transducer Interface

Rugeduino (Arduino Uno) firmware for interfacing HX711 load cell amplifier with ClearCore controller.

## Hardware Required

- **Ruggeduino-SE ST** (Screw Terminal version - ruggedized Arduino Uno)
- **HX711 Load Cell Amplifier Module**
- **Load Cell** (configured for force measurement)
- 22-24 AWG wire for screw terminal connections

## Wiring

### Load Cell → HX711

```
Load Cell Color    HX711 Pin
---------------    ---------
Red (E+)       →   E+
Black (E-)     →   E-
White (A-)     →   A-
Green (A+)     →   A+
```

### HX711 → Ruggeduino-SE

**Use the screw terminals - no soldering required!**

```
HX711 Pin      Ruggeduino-SE Screw Terminal
---------      ----------------------------
VCC        →   5V
GND        →   GND
DOUT       →   D3
SCK        →   D2
RATE       →   5V (enables 80Hz mode; leave disconnected for 10Hz)
```

### Rugeduino → ClearCore COM-0

**ClearCore COM-0 must be configured in TTL mode!**

```
Ruggeduino-SE Pin    ClearCore COM-0 (8-pin connector)         Wire Color
-----------------    ----------------------------------         ----------
D1 (TX)          →   Pin 8 (Data In / COM0 RX)                 Brown
D0 (RX)          →   Pin 5 (Data Out / COM0 TX)                Green/White
GND              →   Pin 4 (GND)                               Blue
VIN or 5V        →   Pin 6 (5V Power) ⚡ 450mA available       Green
```

**Note:** 
- The ClearCore COM-0 port can power the Ruggeduino-SE directly (up to 450mA available from 5VOB). Connect to VIN or 5V screw terminal. Ruggeduino draws ~50mA typical.
- **During USB programming:** Disconnect D0/D1 from ClearCore to avoid conflicts with USB serial

## Software Setup

### Arduino

1. Install the **HX711 library** by Bogde:
   - Arduino IDE → Tools → Manage Libraries
   - Search "HX711"
   - Install "HX711 by Bogde"

2. Upload `hx711_arduino.ino` to the Ruggeduino-SE
   - **Important:** Disconnect D0/D1 from ClearCore during USB upload

3. **Calibrate the scale:**
   - Set `calibration_factor = 1.0` initially
   - Place a known weight (e.g., 10 kg) on the load cell
   - Read the output in Serial Monitor
   - Calculate: `calibration_factor = output_value / 10.0`
   - Update the code and re-upload

### ClearCore

The force sensor is integrated into the firmware:
- `force_sensor.h` / `force_sensor.cpp` - Force sensor class
- COM-0 configured as TTL UART at 115200 baud
- Force readings automatically appear in telemetry

## Serial Protocol

### Arduino → ClearCore (Continuous Output)

**Format:** Simple floating point value in kg  
**Example:** `12.34\n`  
**Rate:** 80Hz (synchronized with HX711 samples when RATE pin = 5V)

### ClearCore → Arduino (Commands)

| Command | Function |
|---------|----------|
| `T` or `t` | Tare/zero the scale |
| `R` or `r` | Get raw ADC value (for debugging) |
| `C` or `c` | Calibration mode (future) |

## Calibration Procedure

1. **Tare with no load:**
   - Remove all weight from load cell
   - Send 'T' command or power cycle
   - Reading should be ~0.00 kg

2. **Determine calibration factor:**
   - Set `calibration_factor = 1.0` in code
   - Upload to Arduino
   - Place known weight (e.g., 10.00 kg)
   - Read the output value (e.g., 234567)
   - Calculate: `calibration_factor = 234567 / 10.0 = 23456.7`
   - Update code with new factor and re-upload

3. **Verify accuracy:**
   - Test with multiple known weights
   - Adjust calibration factor if needed
   - Typical accuracy: ±0.1% with proper calibration

## Troubleshooting

**No readings:**
- Check wiring connections
- Verify HX711 power (3.3V-5V)
- Check Arduino serial output in Serial Monitor
- Ensure load cell is properly connected

**Noisy/unstable readings:**
- Improve grounding
- Shorten wires between load cell and HX711
- Use shielded cable for load cell wires
- Reduce mechanical vibration

**Readings drift:**
- Allow 10-15 minutes warm-up time
- Reduce temperature variations
- Re-tare periodically

**Readings don't change with force:**
- Check load cell wiring (especially A+/A-)
- Verify load cell is mechanically loaded correctly
- Check calibration factor

## References

- **HX711 Datasheet:** https://cdn.sparkfun.com/assets/b/f/5/a/e/hx711F_EN.pdf
- **HX711 Library:** https://github.com/bogde/HX711
- **SparkFun Hookup Guide:** https://learn.sparkfun.com/tutorials/load-cell-amplifier-hx711-breakout-hookup-guide

## License

MIT License - See main repository LICENSE file
