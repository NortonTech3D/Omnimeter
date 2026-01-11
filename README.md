# Omnimeter - ESP32-S2 Precision Digital Multimeter

A production-ready firmware implementation for a precision DC voltmeter using the ESP32-S2FN4R2 (Lolin S2 Mini). Features auto-ranging across five voltage ranges (0-60V), web-based monitoring interface, and USB serial output.

## Features

- **5-Range Auto-Ranging**: Automatically selects optimal range for best resolution
  - 0-60V (lowest resolution, ~14.6mV)
  - 0-20V (~4.9mV resolution)
  - 0-10V (~2.4mV resolution)
  - 0-5V (~1.2mV resolution)
  - 0-2.5V (highest resolution, ~0.6mV)

- **Precision ADC**: Uses ESP32-S2 factory-calibrated VREF with 13-bit resolution
- **Oversampling**: 64-sample averaging for noise reduction
- **Dual Network Mode**: Access Point (standalone) or Station (connect to existing network)
- **Web Interface**: Real-time voltage display with AJAX updates every 500ms
- **Non-Blocking Architecture**: No `delay()` calls in main loop
- **USB Serial Output**: Real-time voltage data stream

## Hardware Requirements

### Target Hardware
- **MCU**: ESP32-S2FN4R2 (Lolin S2 Mini)
- **Framework**: Arduino / PlatformIO

### Input Topology
Single probe input branching into 5 parallel voltage dividers:

```
                     ┌──────────────────────────────────────────────────┐
                     │                 ESP32-S2 (Lolin S2 Mini)         │
                     │                                                  │
    Vin ─────┬───────┤─── 100kΩ ──┬── GPIO1 (60V Range, R2=4.3kΩ)      │
             │       │            │                                     │
             ├───────┤─── 100kΩ ──┬── GPIO2 (20V Range, R2=14.3kΩ)     │
             │       │            │                                     │
             ├───────┤─── 100kΩ ──┬── GPIO3 (10V Range, R2=33kΩ)       │
             │       │            │                                     │
             ├───────┤─── 100kΩ ──┬── GPIO4 (5V Range, R2=100kΩ)       │
             │       │            │                                     │
             └───────┤───────────────GPIO5 (2.5V Range, Direct)         │
                     │                                                  │
    GND ─────────────┤── GND                                            │
                     └──────────────────────────────────────────────────┘
```

### Voltage Divider Specifications

| Range | R1 (High) | R2 (Low) | Divider Ratio | Max ADC Voltage | Multiplier |
|-------|-----------|----------|---------------|-----------------|------------|
| 60V   | 100kΩ     | 4.3kΩ    | 0.04123       | ~2.47V          | 24.256     |
| 20V   | 100kΩ     | 14.3kΩ   | 0.12511       | ~2.50V          | 7.993      |
| 10V   | 100kΩ     | 33kΩ     | 0.24812       | ~2.48V          | 4.030      |
| 5V    | 100kΩ     | 100kΩ    | 0.50000       | ~2.50V          | 2.000      |
| 2.5V  | None      | Direct   | 1.00000       | 2.50V           | 1.000      |

### Protection
Zener diode clamps or MOSFET arrays should be installed on all ADC input pins for overvoltage protection.

## Software Architecture

### File Structure
```
Omnimeter/
├── platformio.ini          # PlatformIO configuration
├── src/
│   ├── main.cpp            # Main entry point and loop
│   ├── Voltmeter.h         # Voltmeter class header
│   ├── Voltmeter.cpp       # ADC driver implementation
│   ├── WebInterface.h      # Web server class header
│   └── WebInterface.cpp    # WiFi and web server implementation
├── data/
│   └── index.html          # Web interface (optional, embedded by default)
└── README.md
```

### Auto-Ranging Algorithm

```
START
  │
  ▼
Read 60V Range Pin
  │
  ├── Saturated? ──YES──▶ Return OVERRANGE (>60V)
  │
  ▼
Voltage >= 18V? ──YES──▶ Return 60V Range Reading
  │
  ▼
Read 20V Range Pin
  │
  ├── Saturated? ──YES──▶ Return 60V Range Reading (fallback)
  │
  ▼
Voltage >= 9V? ──YES──▶ Return 20V Range Reading
  │
  ▼
Read 10V Range Pin
  │
  ├── Saturated? ──YES──▶ Return 20V Range Reading (fallback)
  │
  ▼
Voltage >= 4.5V? ──YES──▶ Return 10V Range Reading
  │
  ▼
Read 5V Range Pin
  │
  ├── Saturated? ──YES──▶ Return 10V Range Reading (fallback)
  │
  ▼
Voltage >= 2.3V? ──YES──▶ Return 5V Range Reading
  │
  ▼
Read 2.5V Range Pin (Direct)
  │
  ├── Saturated? ──YES──▶ Return 5V Range Reading (fallback)
  │
  ▼
Return 2.5V Range Reading (highest resolution)
```

## Building and Flashing

### Prerequisites
- [PlatformIO](https://platformio.org/) (CLI or VS Code extension)
- USB-C cable for Lolin S2 Mini

### Build
```bash
# Clone the repository
git clone https://github.com/your-repo/Omnimeter.git
cd Omnimeter

# Build the firmware
pio run

# Build and upload
pio run --target upload

# Monitor serial output
pio device monitor
```

### First Boot
1. Connect to "ESP_Multimeter" WiFi network (password: `12345678`)
2. Open browser to `http://192.168.4.1`
3. View live voltage readings

## Configuration

### WiFi Settings
Edit `src/WebInterface.h` to configure:

```cpp
// Access Point Mode
constexpr const char* AP_SSID = "ESP_Multimeter";
constexpr const char* AP_PASSWORD = "12345678";

// Station Mode (connect to existing network)
constexpr bool WIFI_STA_ENABLED = false;  // Set to true to enable
constexpr const char* STA_SSID = "YourNetworkSSID";
constexpr const char* STA_PASSWORD = "YourNetworkPassword";
```

### ADC Pin Mapping
Edit `src/Voltmeter.h` to change GPIO assignments:

```cpp
constexpr uint8_t PIN_RANGE_60V  = 1;   // GPIO1
constexpr uint8_t PIN_RANGE_20V  = 2;   // GPIO2
constexpr uint8_t PIN_RANGE_10V  = 3;   // GPIO3
constexpr uint8_t PIN_RANGE_5V   = 4;   // GPIO4
constexpr uint8_t PIN_RANGE_2V5  = 5;   // GPIO5
```

### Calibration
Fine-tune readings by adjusting calibration constants in `src/Voltmeter.h`:

```cpp
// Offset calibration (in volts)
constexpr float CAL_OFFSET_60V = 0.0f;
constexpr float CAL_OFFSET_20V = 0.0f;
// ... etc

// Gain calibration (multiplier)
constexpr float CAL_GAIN_60V = 1.0f;
constexpr float CAL_GAIN_20V = 1.0f;
// ... etc
```

## API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Main web interface |
| `/api/voltage` | GET | JSON voltage data |
| `/api/info` | GET | Device information |
| `/health` | GET | Health check |

### JSON Response Format
```json
{
  "voltage": 12.3456,
  "adc_mv": 1543.2,
  "range": "20V",
  "valid": true,
  "overrange": false,
  "timestamp": 123456789
}
```

## Serial Output Format
```
V:  12.3456 V | Range: 20V | ADC:  1543.2 mV | OK
V:   5.1234 V | Range: 10V | ADC:  1271.1 mV | OK
V: OVERRANGE! | Range: 60V | ADC:  2450.0 mV | OVR
```

## Safety Considerations

⚠️ **WARNING**: This device measures DC voltages up to 60V. Ensure proper electrical safety:

1. Never exceed 60V input voltage
2. Always use proper overvoltage protection (Zener/TVS diodes)
3. Ensure proper isolation when measuring high voltages
4. Use appropriate probe insulation
5. Never measure AC mains voltage with this device

## Troubleshooting

### No Serial Output
- Ensure USB CDC is enabled in `platformio.ini`
- Try pressing the reset button after connecting USB
- Check that `ARDUINO_USB_CDC_ON_BOOT=1` is set

### WiFi Connection Issues
- Verify the AP password is at least 8 characters
- For Station mode, check SSID/password are correct
- Reduce AP_MAX_CONNECTIONS if having stability issues

### Inaccurate Readings
- Verify resistor values match specifications (1% tolerance recommended)
- Run calibration procedure with precision voltage source
- Check for cold solder joints on voltage dividers

## License

MIT License - See [LICENSE](LICENSE) for details.

## Contributing

Pull requests welcome! Please follow the existing code style and include appropriate documentation.
