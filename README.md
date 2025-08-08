# MAX31856 Generic Library

A **platform-independent** C library for MAX31856 Precision Thermocouple to Digital Converter with linearization. Works with **any microcontroller** by implementing simple platform interface functions.

## 🌟 Features

- ✅ **Platform Independent** - Works with STM32, Arduino, ESP32, PIC, AVR, etc.
- ✅ **8 Thermocouple Types** - B, E, J, K, N, R, S, T support
- ✅ **Voltage Mode** - Direct voltage measurement with 8x/32x gain
- ✅ **Fault Detection** - Comprehensive fault detection (open circuit, overvoltage, etc.)
- ✅ **Linearization** - Automatic thermocouple linearization
- ✅ **Cold-Junction Compensation** - Automatic cold junction compensation
- ✅ **Noise Filtering** - 50Hz/60Hz selectable filters with averaging
- ✅ **High Resolution** - 19-bit resolution (0.0078125°C)
- ✅ **Easy Porting** - Only 4 simple functions to implement

## 📁 File Structure

```
MAX31856_Library/
├── max31856.h          # Header file
└── max31856.c          # Implementation file  
Core/
└── Scr/
    └── main.c    # Example
```

## 🚀 Quick Start

### 1. Hardware Setup

#### SPI Connections (Example with STM32F103C8T6):
```
MCU              MAX31856
---------        --------
PA4       <->    CS
PA5       <->    SCK  
PA6       <->    SDO (MISO)
PA7       <->    SDI (MOSI)
3.3V      <->    AVDD/DVDD
GND       <->    AGND/DGND
```

#### Thermocouple Connections:
```
MAX31856         Thermocouple
--------         ------------
T+        <->    Red wire (+)
T-        <->    Blue/Black wire (-)
BIAS      <->    T- (connect together)
```

#### SPI Configuration:
- **Mode**: SPI Mode 1 or 3 (CPOL=1, CPHA=1)
- **Speed**: 1-5 MHz (typically 2 MHz)
- **Bit Order**: MSB First
- **Data Size**: 8 bits

### 2. Add Files to Your Project

1. Copy `max31856.h` and `max31856.c` to your project
2. Include header in your main file:
```c
#include "max31856.h"
```

### 3. Implement Platform Functions

You only need to implement **4 simple functions** for your platform:

```c
// SPI communication function
max31856_status_t my_spi_write_read(MAX31856_Handle_t *hmax, 
                                    uint8_t *tx_data, uint8_t *rx_data, uint16_t size) {
    // Your platform SPI implementation here
    // Return MAX31856_OK if successful, MAX31856_TIMEOUT if failed
}

// CS pin control functions
void my_cs_low(void) {
    // Set CS pin LOW
}

void my_cs_high(void) {
    // Set CS pin HIGH  
}

// Delay function
void my_delay_ms(uint32_t ms) {
    // Your platform delay implementation
}
```

### 4. Basic Usage

```c
#include "max31856.h"

MAX31856_Handle_t tc_sensor;

int main(void) {
    // Initialize your MCU, SPI, etc.
    
    // Create platform interface
    max31856_platform_t platform = {
        .spi_write_read = my_spi_write_read,
        .cs_low = my_cs_low,
        .cs_high = my_cs_high,
        .delay_ms = my_delay_ms,
        .platform_data = NULL
    };
    
    // Initialize MAX31856 with Type K thermocouple
    if (MAX31856_Init(&tc_sensor, &platform, NULL, 0, MAX31856_TC_TYPE_K) == MAX31856_OK) {
        printf("MAX31856 initialized successfully!\n");
    } else {
        printf("MAX31856 initialization failed!\n");
        return -1;
    }
    
    // Main loop
    while (1) {
        float temperature;
        
        if (MAX31856_ReadTemperature(&tc_sensor, &temperature) == MAX31856_OK) {
            printf("Temperature: %.2f°C\n", temperature);
        } else {
            printf("Temperature read failed\n");
        }
        
        delay_ms(2000); // Wait 2 seconds
    }
}
```

## 🔧 Platform Examples

### STM32 with HAL

```c
max31856_status_t stm32_spi_write_read(MAX31856_Handle_t *hmax, 
                                       uint8_t *tx_data, uint8_t *rx_data, uint16_t size) {
    if (HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, size, 1000) == HAL_OK) {
        return MAX31856_OK;
    }
    return MAX31856_TIMEOUT;
}

void stm32_cs_low(void) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}

void stm32_cs_high(void) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

void stm32_delay_ms(uint32_t ms) {
    HAL_Delay(ms);
}
```

### Arduino

```cpp
max31856_status_t arduino_spi_write_read(MAX31856_Handle_t *hmax, 
                                         uint8_t *tx_data, uint8_t *rx_data, uint16_t size) {
    for (int i = 0; i < size; i++) {
        rx_data[i] = SPI.transfer(tx_data[i]);
    }
    return MAX31856_OK;
}

void arduino_cs_low(void) {
    digitalWrite(CS_PIN, LOW);
}

void arduino_cs_high(void) {
    digitalWrite(CS_PIN, HIGH);
}

void arduino_delay_ms(uint32_t ms) {
    delay(ms);
}
```

### ESP32

```c
max31856_status_t esp32_spi_write_read(MAX31856_Handle_t *hmax, 
                                       uint8_t *tx_data, uint8_t *rx_data, uint16_t size) {
    spi_transaction_t trans = {
        .length = size * 8,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data
    };
    
    if (spi_device_transmit(spi_handle, &trans) == ESP_OK) {
        return MAX31856_OK;
    }
    return MAX31856_TIMEOUT;
}

void esp32_cs_low(void) {
    gpio_set_level(CS_PIN, 0);
}

void esp32_cs_high(void) {
    gpio_set_level(CS_PIN, 1);
}

void esp32_delay_ms(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}
```

## 📖 API Reference

### Initialization and Configuration

#### `MAX31856_Init()`
```c
max31856_status_t MAX31856_Init(MAX31856_Handle_t *hmax,
                                const max31856_platform_t *platform,
                                max31856_tc_type_t tc_type);
```
- **Description**: Initialize MAX31856 device
- **Parameters**: 
  - `hmax`: Handle pointer
  - `platform`: Platform interface functions
  - `tc_type`: Thermocouple type
- **Returns**: `MAX31856_OK` if successful

#### `MAX31856_SetTCType()`
```c
max31856_status_t MAX31856_SetTCType(MAX31856_Handle_t *hmax, max31856_tc_type_t tc_type);
```
**Supported thermocouple types:**
- `MAX31856_TC_TYPE_K` - Type K (-200°C to +1372°C)
- `MAX31856_TC_TYPE_J` - Type J (-210°C to +1200°C)  
- `MAX31856_TC_TYPE_T` - Type T (-200°C to +400°C)
- `MAX31856_TC_TYPE_E` - Type E (-200°C to +1000°C)
- `MAX31856_TC_TYPE_N` - Type N (-200°C to +1300°C)
- `MAX31856_TC_TYPE_R` - Type R (-50°C to +1768°C)
- `MAX31856_TC_TYPE_S` - Type S (-50°C to +1768°C) 
- `MAX31856_TC_TYPE_B` - Type B (250°C to +1820°C)
- `MAX31856_VOLTAGE_MODE_GAIN8` - Voltage mode ±78.125mV
- `MAX31856_VOLTAGE_MODE_GAIN32` - Voltage mode ±19.531mV

### Temperature Reading

#### `MAX31856_ReadTemperature()`
```c
max31856_status_t MAX31856_ReadTemperature(MAX31856_Handle_t *hmax, float *temperature);
```
- **Description**: Read linearized thermocouple temperature
- **Parameters**: Handle and pointer to store temperature (°C)
- **Returns**: `MAX31856_OK` if successful

#### `MAX31856_ReadCJTemperature()`
```c
max31856_status_t MAX31856_ReadCJTemperature(MAX31856_Handle_t *hmax, float *temperature);
```
- **Description**: Read cold-junction temperature
- **Parameters**: Handle and pointer to store CJ temperature (°C)

#### `MAX31856_ReadTCVoltage()`
```c
max31856_status_t MAX31856_ReadTCVoltage(MAX31856_Handle_t *hmax, float *voltage);
```
- **Description**: Read thermocouple voltage (voltage mode only)
- **Parameters**: Handle and pointer to store voltage (V)

### Fault Detection

#### `MAX31856_ReadFault()`
```c
max31856_status_t MAX31856_ReadFault(MAX31856_Handle_t *hmax, uint8_t *fault_status);
```
- **Description**: Read fault status register
- **Fault bits**:
  - `MAX31856_FAULT_OPEN` - Open circuit
  - `MAX31856_FAULT_OVUV` - Over/under voltage
  - `MAX31856_FAULT_TC_HIGH` - TC high threshold
  - `MAX31856_FAULT_TC_LOW` - TC low threshold
  - `MAX31856_FAULT_CJ_HIGH` - CJ high threshold  
  - `MAX31856_FAULT_CJ_LOW` - CJ low threshold
  - `MAX31856_FAULT_TC_RANGE` - TC out of range
  - `MAX31856_FAULT_CJ_RANGE` - CJ out of range

#### `MAX31856_ClearFault()`
```c
max31856_status_t MAX31856_ClearFault(MAX31856_Handle_t *hmax);
```
- **Description**: Clear all fault status bits

#### `MAX31856_GetFaultString()`
```c
const char* MAX31856_GetFaultString(uint8_t fault_bits);
```
- **Description**: Get human-readable fault description

### Configuration Functions

#### `MAX31856_SetFilter()`
```c
max31856_status_t MAX31856_SetFilter(MAX31856_Handle_t *hmax, max31856_filter_t filter);
```
- `MAX31856_FILTER_60HZ` - 60Hz noise rejection (default)
- `MAX31856_FILTER_50HZ` - 50Hz noise rejection

#### `MAX31856_SetAvgMode()`
```c
max31856_status_t MAX31856_SetAvgMode(MAX31856_Handle_t *hmax, max31856_avg_mode_t avg_mode);
```
- `MAX31856_AVG_1_SAMPLE` - Fastest (default)
- `MAX31856_AVG_2_SAMPLES` - 2 samples averaged
- `MAX31856_AVG_4_SAMPLES` - 4 samples averaged  
- `MAX31856_AVG_8_SAMPLES` - 8 samples averaged
- `MAX31856_AVG_16_SAMPLES` - 16 samples averaged (best noise reduction)

#### `MAX31856_SetOCFault()`
```c
max31856_status_t MAX31856_SetOCFault(MAX31856_Handle_t *hmax, max31856_ocfault_t ocfault);
```
- `MAX31856_OCFAULT_DISABLED` - Disabled (default)
- `MAX31856_OCFAULT_10MS` - 10ms detection time
- `MAX31856_OCFAULT_32MS` - 32ms detection time
- `MAX31856_OCFAULT_100MS` - 100ms detection time

### Return Status Codes

- `MAX31856_OK` - Operation successful
- `MAX31856_ERROR` - General error
- `MAX31856_TIMEOUT` - Communication timeout
- `MAX31856_FAULT` - Sensor fault detected

## 🔍 Advanced Usage

### Multiple Sensors

```c
MAX31856_Handle_t sensors[4];

// Initialize multiple sensors with different CS pins
for (int i = 0; i < 4; i++) {
    MAX31856_Init(&sensors[i], &platform, MAX31856_TC_TYPE_K);
}

// Read all sensors
for (int i = 0; i < 4; i++) {
    float temp;
    if (MAX31856_ReadTemperature(&sensors[i], &temp) == MAX31856_OK) {
        printf("Sensor %d: %.3f°C\n", i+1, temp);
    }
}
```

### Error Handling

```c
uint8_t fault_status;
if (MAX31856_ReadFault(&tc_sensor, &fault_status) == MAX31856_OK) {
    if (fault_status != 0) {
        printf("Fault detected: %s\n", MAX31856_GetFaultString(fault_status));
        
        if (fault_status & MAX31856_FAULT_OPEN) {
            printf("ERROR: Thermocouple not connected!\n");
        }
        
        MAX31856_ClearFault(&tc_sensor);
    }
}
```

### Voltage Mode Usage

```c
// Switch to voltage mode
MAX31856_SetTCType(&tc_sensor, MAX31856_VOLTAGE_MODE_GAIN8);
delay_ms(100);

float voltage;
if (MAX31856_ReadTCVoltage(&tc_sensor, &voltage) == MAX31856_OK) {
    printf("Input voltage: %.6f V\n", voltage);
    
    // Calculate temperature manually if needed
    float approx_temp = (voltage * 1000.0f) / 0.041276f; // Type K approximation
    printf("Approximate temperature: %.1f°C\n", approx_temp);
}

// Switch back to thermocouple mode
MAX31856_SetTCType(&tc_sensor, MAX31856_TC_TYPE_K);
```

## ⚡ Performance

- **Conversion Time**: ~155ms (60Hz filter), ~185ms (50Hz filter)
- **Resolution**: 19-bit (0.0078125°C)
- **Accuracy**: ±0.15% full-scale (depending on thermocouple type)
- **SPI Speed**: Up to 5MHz
- **Supply Current**: 2mA (active), 10µA (standby)

## 🔧 Troubleshooting

### Problem: Temperature reads very high (~200°C) at room temperature
**Solution**: Thermocouple not connected or floating input
```c
// Enable open circuit detection
MAX31856_SetOCFault(&tc_sensor, MAX31856_OCFAULT_10MS);

// Check for open circuit fault
uint8_t fault;
MAX31856_ReadFault(&tc_sensor, &fault);
if (fault & MAX31856_FAULT_OPEN) {
    printf("Thermocouple not connected!\n");
}
```

### Problem: Communication errors
**Solution**: Check SPI configuration and connections
- Verify CPOL=1, CPHA=1
- Check CS, SCK, MISO, MOSI connections
- Reduce SPI speed to 1-2 MHz
- Add delays between transactions

### Problem: Noisy readings
**Solution**: Enable filtering and averaging
```c
MAX31856_SetFilter(&tc_sensor, MAX31856_FILTER_60HZ);
MAX31856_SetAvgMode(&tc_sensor, MAX31856_AVG_16_SAMPLES);
```

## 📋 Tested Platforms

- ✅ STM32F103C8T6 (Blue Pill)

## 📄 License

MIT License - Free for commercial and personal use.

## project

---

**Note**: This library is designed to be platform-independent. The STM32 examples are just demonstrations - you can use this library with any microcontroller that supports SPI communication by implementing the 4 platform interface functions.
