# MLX90615

Arduino library for the [Melexis MLX90615](https://www.melexis.com/en/product/MLX90615/Digital-Plug-Play-Infrared-Thermometer-Ultra-Low-Noise) non-contact IR temperature sensor.

## Features

- Read object (IR) and ambient (die) temperature
- Temperature output in Kelvin, Celsius, or Fahrenheit
- Configure emissivity (0.1–1.0)
- Configure IIR digital filter coefficient
- SMBus communication with CRC-8 (PEC) validation

## Hardware

| MLX90615 Pin | Arduino (Uno) |
|---|---|
| SDA | A4 |
| SCL | A5 |
| VDD | 3.3V |
| GND | GND |

> **Note:** The MLX90615 operates at 3.3V. Use a logic-level shifter if connecting to a 5V Arduino.

## Installation

1. Download or clone this repository.
2. Copy the folder into your Arduino `libraries/` directory.
3. Restart the Arduino IDE.

## Usage

### Basic temperature reading

```cpp
#include <Wire.h>
#include "MLX90615.h"

MLX90615 sensor; // uses default I2C address 0x5B

void setup() {
    Serial.begin(9600);
    Wire.begin();
    sensor.begin();
}

void loop() {
    double objTemp = sensor.readTemp(MLX90615::MLX90615_SRCO, MLX90615::MLX90615_TC);
    double ambTemp = sensor.readTemp(MLX90615::MLX90615_SRCA, MLX90615::MLX90615_TC);

    Serial.print("Object: "); Serial.print(objTemp); Serial.println(" C");
    Serial.print("Ambient: "); Serial.print(ambTemp); Serial.println(" C");
    delay(1000);
}
```

### Custom I2C address

```cpp
MLX90615 sensor(0x5B); // explicit address
```

### Temperature units

```cpp
sensor.readTemp(MLX90615::MLX90615_SRCO, MLX90615::MLX90615_TK); // Kelvin
sensor.readTemp(MLX90615::MLX90615_SRCO, MLX90615::MLX90615_TC); // Celsius (default)
sensor.readTemp(MLX90615::MLX90615_SRCO, MLX90615::MLX90615_TF); // Fahrenheit
```

### Emissivity

```cpp
sensor.setEmissivity(0.95); // set emissivity (0.1–1.0, default 1.0)
float e = sensor.getEmissivity(); // read back current emissivity
```

### IIR filter

```cpp
sensor.setIIR(1); // 1 = IIR bypassed (default), 7 = strongest filtering
uint8_t iir = sensor.getIIR();
```

## API Reference

| Method | Description |
|---|---|
| `begin()` | Initialize the sensor and I2C interface |
| `readTemp(src, unit)` | Read temperature from `MLX90615_SRCO` (object) or `MLX90615_SRCA` (ambient) |
| `setEmissivity(float)` | Set emissivity coefficient (0.1–1.0) |
| `getEmissivity()` | Get current emissivity coefficient |
| `setIIR(uint8_t)` | Set IIR filter coefficient (1–7) |
| `getIIR()` | Get current IIR filter coefficient |
| `readEEprom(addr)` | Read a raw 16-bit EEPROM register |
| `writeEEprom(addr, data)` | Write a raw 16-bit EEPROM register |
| `readRAM(addr)` | Read a raw 16-bit RAM register |

## Error Handling

After any operation, R/W errors are stored in an internal bitmask. Error flags are defined in `MLX90615.h`:

| Flag | Value | Meaning |
|---|---|---|
| `MLX90615_NORWERROR` | `0x00` | No errors |
| `MLX90615_RXCRC` | `0x10` | CRC mismatch on received data |
| `MLX90615_INVALIDATA` | `0x20` | Data out of valid range |
| `MLX90615_EECORRUPT` | `0x40` | EEPROM may be corrupted |

## License

MIT
