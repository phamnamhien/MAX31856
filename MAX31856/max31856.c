/**
 * @file    max31856.c
 * @brief   MAX31856 Precision Thermocouple to Digital Converter Generic Library Implementation
 * @author  Generated based on MAX31865 style
 * @date    2024
 * @version 1.0 - Generic version
 */

#include "max31856.h"
#include <string.h>
#include <stdlib.h>

/* Private Functions */
static max31856_status_t MAX31856_WaitForConversion(MAX31856_Handle_t *hmax);
static float MAX31856_ConvertCJTemperature(uint16_t raw_value);
static float MAX31856_ConvertTCTemperature(uint32_t raw_value);
static uint16_t MAX31856_TemperatureToCJRaw(float temperature);
static uint32_t MAX31856_TemperatureToTCRaw(float temperature);

/* Wait for conversion completion */
static max31856_status_t MAX31856_WaitForConversion(MAX31856_Handle_t *hmax) {
    if (hmax->platform.delay_ms) {
        // Conversion time depends on averaging mode and filter setting
        uint32_t conv_time = 155; // Base time for 60Hz filter

        if (hmax->filter == MAX31856_FILTER_50HZ) {
            conv_time = 185; // Base time for 50Hz filter
        }

        // Add time for averaging
        switch (hmax->avg_mode) {
            case MAX31856_AVG_2_SAMPLES:
                conv_time += 33;
                break;
            case MAX31856_AVG_4_SAMPLES:
                conv_time += 66;
                break;
            case MAX31856_AVG_8_SAMPLES:
                conv_time += 132;
                break;
            case MAX31856_AVG_16_SAMPLES:
                conv_time += 264;
                break;
            default:
                break;
        }

        hmax->platform.delay_ms(conv_time);
    }
    return MAX31856_OK;
}

/* Convert cold-junction temperature from raw value */
static float MAX31856_ConvertCJTemperature(uint16_t raw_value) {
    // CJ temperature format: sign bit + 7 bits integer + 8 bits fractional
    int16_t signed_value = (int16_t)raw_value;
    return (float)signed_value / 256.0f;
}

/* Convert thermocouple temperature from raw value */
static float MAX31856_ConvertTCTemperature(uint32_t raw_value) {
    // TC temperature format: 19-bit signed value, 0.0078125°C per LSB
    int32_t signed_value = (int32_t)(raw_value << 13) >> 13; // Sign extend 19-bit to 32-bit
    return (float)signed_value * 0.0078125f;
}

/* Convert temperature to cold-junction raw value */
static uint16_t MAX31856_TemperatureToCJRaw(float temperature) {
    int16_t raw = (int16_t)(temperature * 256.0f);
    return (uint16_t)raw;
}

/* Convert temperature to thermocouple raw value */
static uint32_t MAX31856_TemperatureToTCRaw(float temperature) {
    int32_t raw = (int32_t)(temperature / 0.0078125f);
    return (uint32_t)(raw & 0x7FFFF); // 19-bit mask
}

/**
 * @brief Initialize MAX31856 device
 */
max31856_status_t MAX31856_Init(MAX31856_Handle_t *hmax,
                                const max31856_platform_t *platform,
                                void *cs_port, uint32_t cs_pin,
                                max31856_tc_type_t tc_type) {

    if (hmax == NULL || platform == NULL) {
        return MAX31856_ERROR;
    }

    /* Copy Platform Interface functions */
    hmax->platform = *platform;

    /* Initialize structure */
    hmax->cs_port = cs_port;
    hmax->cs_pin = cs_pin;
    hmax->tc_type = tc_type;
    hmax->avg_mode = MAX31856_AVG_1_SAMPLE;
    hmax->filter = MAX31856_FILTER_60HZ;
    hmax->conv_mode = MAX31856_NORMALLY_OFF;
    hmax->fault_mode = MAX31856_FAULT_COMPARATOR;
    hmax->ocfault = MAX31856_OCFAULT_DISABLED;
    hmax->initialized = false;
    hmax->last_fault = 0;
    hmax->last_temperature = 0.0f;
    hmax->last_cj_temperature = 0.0f;
    hmax->last_tc_raw = 0;

    /* Set CS high initially */
    if (hmax->platform.cs_high) {
        hmax->platform.cs_high();
    }

    if (hmax->platform.delay_ms) {
        hmax->platform.delay_ms(10);
    }

    /* Test communication */
    uint8_t config_reg = 0;
    if (MAX31856_ReadRegister(hmax, MAX31856_CR0_REG, &config_reg) != MAX31856_OK) {
        MAX31856_DEBUG_PRINT("MAX31856: Communication test failed\r\n");
        return MAX31856_ERROR;
    }

    /* Clear any existing faults */
    MAX31856_ClearFault(hmax);

    /* Configure thermocouple type */
    if (MAX31856_SetTCType(hmax, tc_type) != MAX31856_OK) {
        return MAX31856_ERROR;
    }

    /* Set default filter */
    if (MAX31856_SetFilter(hmax, MAX31856_FILTER_60HZ) != MAX31856_OK) {
        return MAX31856_ERROR;
    }

    /* Set default averaging mode */
    if (MAX31856_SetAvgMode(hmax, MAX31856_AVG_1_SAMPLE) != MAX31856_OK) {
        return MAX31856_ERROR;
    }

    /* Set default conversion mode (normally off) */
    if (MAX31856_SetConvMode(hmax, MAX31856_NORMALLY_OFF) != MAX31856_OK) {
        return MAX31856_ERROR;
    }

    /* Enable cold-junction sensor by default */
    if (MAX31856_EnableCJSensor(hmax, true) != MAX31856_OK) {
        return MAX31856_ERROR;
    }

    /* Set default fault mask (all faults enabled) */
    if (MAX31856_SetFaultMask(hmax, 0x00) != MAX31856_OK) {
        return MAX31856_ERROR;
    }

    /* Set default temperature thresholds */
    if (MAX31856_SetCJHighThreshold(hmax, 125.0f) != MAX31856_OK) {
        return MAX31856_ERROR;
    }
    if (MAX31856_SetCJLowThreshold(hmax, -64.0f) != MAX31856_OK) {
        return MAX31856_ERROR;
    }
    if (MAX31856_SetTCHighThreshold(hmax, 1600.0f) != MAX31856_OK) {
        return MAX31856_ERROR;
    }
    if (MAX31856_SetTCLowThreshold(hmax, -250.0f) != MAX31856_OK) {
        return MAX31856_ERROR;
    }

    hmax->initialized = true;
    MAX31856_DEBUG_PRINT("MAX31856: Initialized successfully\r\n");

    return MAX31856_OK;
}

/**
 * @brief Deinitialize MAX31856 device
 */
max31856_status_t MAX31856_DeInit(MAX31856_Handle_t *hmax) {
    if (hmax == NULL) return MAX31856_ERROR;

    /* Set to normally off mode */
    MAX31856_SetConvMode(hmax, MAX31856_NORMALLY_OFF);

    /* Clear faults */
    MAX31856_ClearFault(hmax);

    hmax->initialized = false;

    return MAX31856_OK;
}

/**
 * @brief Set thermocouple type
 */
max31856_status_t MAX31856_SetTCType(MAX31856_Handle_t *hmax, max31856_tc_type_t tc_type) {
    if (hmax == NULL) return MAX31856_ERROR;

    uint8_t cr1 = 0;
    if (MAX31856_ReadRegister(hmax, MAX31856_CR1_REG, &cr1) != MAX31856_OK) {
        return MAX31856_ERROR;
    }

    cr1 &= ~MAX31856_CR1_TC_TYPE_MASK;
    cr1 |= (tc_type & MAX31856_CR1_TC_TYPE_MASK);

    if (MAX31856_WriteRegister(hmax, MAX31856_CR1_REG, cr1) != MAX31856_OK) {
        return MAX31856_ERROR;
    }

    hmax->tc_type = tc_type;
    return MAX31856_OK;
}

/**
 * @brief Set noise filter
 */
max31856_status_t MAX31856_SetFilter(MAX31856_Handle_t *hmax, max31856_filter_t filter) {
    if (hmax == NULL) return MAX31856_ERROR;

    uint8_t cr0 = 0;
    if (MAX31856_ReadRegister(hmax, MAX31856_CR0_REG, &cr0) != MAX31856_OK) {
        return MAX31856_ERROR;
    }

    if (filter == MAX31856_FILTER_50HZ) {
        cr0 |= MAX31856_CR0_50_60HZ;
    } else {
        cr0 &= ~MAX31856_CR0_50_60HZ;
    }

    if (MAX31856_WriteRegister(hmax, MAX31856_CR0_REG, cr0) != MAX31856_OK) {
        return MAX31856_ERROR;
    }

    hmax->filter = filter;
    return MAX31856_OK;
}

/**
 * @brief Set averaging mode
 */
max31856_status_t MAX31856_SetAvgMode(MAX31856_Handle_t *hmax, max31856_avg_mode_t avg_mode) {
    if (hmax == NULL) return MAX31856_ERROR;

    uint8_t cr1 = 0;
    if (MAX31856_ReadRegister(hmax, MAX31856_CR1_REG, &cr1) != MAX31856_OK) {
        return MAX31856_ERROR;
    }

    cr1 &= ~(MAX31856_CR1_AVGSEL2 | MAX31856_CR1_AVGSEL1 | MAX31856_CR1_AVGSEL0);
    cr1 |= ((avg_mode & 0x07) << 4);

    if (MAX31856_WriteRegister(hmax, MAX31856_CR1_REG, cr1) != MAX31856_OK) {
        return MAX31856_ERROR;
    }

    hmax->avg_mode = avg_mode;
    return MAX31856_OK;
}

/**
 * @brief Set conversion mode
 */
max31856_status_t MAX31856_SetConvMode(MAX31856_Handle_t *hmax, max31856_conv_mode_t conv_mode) {
    if (hmax == NULL) return MAX31856_ERROR;

    uint8_t cr0 = 0;
    if (MAX31856_ReadRegister(hmax, MAX31856_CR0_REG, &cr0) != MAX31856_OK) {
        return MAX31856_ERROR;
    }

    if (conv_mode == MAX31856_AUTO_CONVERT) {
        cr0 |= MAX31856_CR0_CMODE;
    } else {
        cr0 &= ~MAX31856_CR0_CMODE;
    }

    if (MAX31856_WriteRegister(hmax, MAX31856_CR0_REG, cr0) != MAX31856_OK) {
        return MAX31856_ERROR;
    }

    hmax->conv_mode = conv_mode;
    return MAX31856_OK;
}

/**
 * @brief Set fault mode
 */
max31856_status_t MAX31856_SetFaultMode(MAX31856_Handle_t *hmax, max31856_fault_mode_t fault_mode) {
    if (hmax == NULL) return MAX31856_ERROR;

    uint8_t cr0 = 0;
    if (MAX31856_ReadRegister(hmax, MAX31856_CR0_REG, &cr0) != MAX31856_OK) {
        return MAX31856_ERROR;
    }

    if (fault_mode == MAX31856_FAULT_INTERRUPT) {
        cr0 |= MAX31856_CR0_FAULT;
    } else {
        cr0 &= ~MAX31856_CR0_FAULT;
    }

    if (MAX31856_WriteRegister(hmax, MAX31856_CR0_REG, cr0) != MAX31856_OK) {
        return MAX31856_ERROR;
    }

    hmax->fault_mode = fault_mode;
    return MAX31856_OK;
}

/**
 * @brief Set open-circuit fault detection
 */
max31856_status_t MAX31856_SetOCFault(MAX31856_Handle_t *hmax, max31856_ocfault_t ocfault) {
    if (hmax == NULL) return MAX31856_ERROR;

    uint8_t cr0 = 0;
    if (MAX31856_ReadRegister(hmax, MAX31856_CR0_REG, &cr0) != MAX31856_OK) {
        return MAX31856_ERROR;
    }

    cr0 &= ~(MAX31856_CR0_OCFAULT1 | MAX31856_CR0_OCFAULT0);
    cr0 |= ((ocfault & 0x03) << 4);

    if (MAX31856_WriteRegister(hmax, MAX31856_CR0_REG, cr0) != MAX31856_OK) {
        return MAX31856_ERROR;
    }

    hmax->ocfault = ocfault;
    return MAX31856_OK;
}

/**
 * @brief Enable/disable cold-junction sensor
 */
max31856_status_t MAX31856_EnableCJSensor(MAX31856_Handle_t *hmax, bool enable) {
    if (hmax == NULL) return MAX31856_ERROR;

    uint8_t cr0 = 0;
    if (MAX31856_ReadRegister(hmax, MAX31856_CR0_REG, &cr0) != MAX31856_OK) {
        return MAX31856_ERROR;
    }

    if (enable) {
        cr0 &= ~MAX31856_CR0_CJ; // Clear bit to enable
    } else {
        cr0 |= MAX31856_CR0_CJ;  // Set bit to disable
    }

    return MAX31856_WriteRegister(hmax, MAX31856_CR0_REG, cr0);
}

/**
 * @brief Set cold-junction temperature offset
 */
max31856_status_t MAX31856_SetCJOffset(MAX31856_Handle_t *hmax, float offset) {
    if (hmax == NULL) return MAX31856_ERROR;

    // Offset range is -8°C to +7.9375°C with 0.0625°C resolution
    if (offset < -8.0f || offset > 7.9375f) {
        return MAX31856_ERROR;
    }

    int8_t offset_raw = (int8_t)(offset / 0.0625f);
    return MAX31856_WriteRegister(hmax, MAX31856_CJTO_REG, (uint8_t)offset_raw);
}

/**
 * @brief Set cold-junction high threshold
 */
max31856_status_t MAX31856_SetCJHighThreshold(MAX31856_Handle_t *hmax, float temperature) {
    if (hmax == NULL) return MAX31856_ERROR;

    uint16_t raw = MAX31856_TemperatureToCJRaw(temperature);
    return MAX31856_WriteRegister(hmax, MAX31856_CJHF_REG, (uint8_t)(raw >> 8));
}

/**
 * @brief Set cold-junction low threshold
 */
max31856_status_t MAX31856_SetCJLowThreshold(MAX31856_Handle_t *hmax, float temperature) {
    if (hmax == NULL) return MAX31856_ERROR;

    uint16_t raw = MAX31856_TemperatureToCJRaw(temperature);
    return MAX31856_WriteRegister(hmax, MAX31856_CJLF_REG, (uint8_t)(raw >> 8));
}

/**
 * @brief Set thermocouple high threshold
 */
max31856_status_t MAX31856_SetTCHighThreshold(MAX31856_Handle_t *hmax, float temperature) {
    if (hmax == NULL) return MAX31856_ERROR;

    uint32_t raw = MAX31856_TemperatureToTCRaw(temperature);
    max31856_status_t status;

    status = MAX31856_WriteRegister(hmax, MAX31856_LTHFTH_REG, (uint8_t)(raw >> 8));
    if (status != MAX31856_OK) return status;

    return MAX31856_WriteRegister(hmax, MAX31856_LTHFTL_REG, (uint8_t)raw);
}

/**
 * @brief Set thermocouple low threshold
 */
max31856_status_t MAX31856_SetTCLowThreshold(MAX31856_Handle_t *hmax, float temperature) {
    if (hmax == NULL) return MAX31856_ERROR;

    uint32_t raw = MAX31856_TemperatureToTCRaw(temperature);
    max31856_status_t status;

    status = MAX31856_WriteRegister(hmax, MAX31856_LTLFTH_REG, (uint8_t)(raw >> 8));
    if (status != MAX31856_OK) return status;

    return MAX31856_WriteRegister(hmax, MAX31856_LTLFTL_REG, (uint8_t)raw);
}

/**
 * @brief Set external cold-junction temperature
 */
max31856_status_t MAX31856_SetExternalCJTemp(MAX31856_Handle_t *hmax, float temperature) {
    if (hmax == NULL) return MAX31856_ERROR;

    uint16_t raw = MAX31856_TemperatureToCJRaw(temperature);
    max31856_status_t status;

    status = MAX31856_WriteRegister(hmax, MAX31856_CJTH_REG, (uint8_t)(raw >> 8));
    if (status != MAX31856_OK) return status;

    return MAX31856_WriteRegister(hmax, MAX31856_CJTL_REG, (uint8_t)raw);
}

/**
 * @brief Trigger one-shot conversion
 */
max31856_status_t MAX31856_TriggerOneShot(MAX31856_Handle_t *hmax) {
    if (hmax == NULL) return MAX31856_ERROR;

    uint8_t cr0 = 0;
    if (MAX31856_ReadRegister(hmax, MAX31856_CR0_REG, &cr0) != MAX31856_OK) {
        return MAX31856_ERROR;
    }

    cr0 |= MAX31856_CR0_1SHOT;

    if (MAX31856_WriteRegister(hmax, MAX31856_CR0_REG, cr0) != MAX31856_OK) {
        return MAX31856_ERROR;
    }

    return MAX31856_WaitForConversion(hmax);
}

/**
 * @brief Read thermocouple temperature
 */
max31856_status_t MAX31856_ReadTemperature(MAX31856_Handle_t *hmax, float *temperature) {
    if (hmax == NULL || temperature == NULL) return MAX31856_ERROR;

    max31856_status_t status;
    uint8_t data[3];

    /* If in normally off mode, trigger one-shot conversion */
    if (hmax->conv_mode == MAX31856_NORMALLY_OFF) {
        status = MAX31856_TriggerOneShot(hmax);
        if (status != MAX31856_OK) return status;
    }

    /* Read linearized temperature registers */
    status = MAX31856_ReadMultipleRegisters(hmax, MAX31856_LTCBH_REG, data, 3);
    if (status != MAX31856_OK) return status;

    /* Combine 3 bytes to form 24-bit value, then extract 19-bit temperature */
    uint32_t raw_temp = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
    raw_temp >>= 5; // Shift right to get 19-bit value

    /* Convert to temperature */
    *temperature = MAX31856_ConvertTCTemperature(raw_temp);
    hmax->last_temperature = *temperature;
    hmax->last_tc_raw = (int32_t)raw_temp;

    return MAX31856_OK;
}

/**
 * @brief Read cold-junction temperature
 */
max31856_status_t MAX31856_ReadCJTemperature(MAX31856_Handle_t *hmax, float *temperature) {
    if (hmax == NULL || temperature == NULL) return MAX31856_ERROR;

    max31856_status_t status;
    uint8_t data[2];

    /* Read cold-junction temperature registers */
    status = MAX31856_ReadMultipleRegisters(hmax, MAX31856_CJTH_REG, data, 2);
    if (status != MAX31856_OK) return status;

    /* Combine 2 bytes to form 16-bit value */
    uint16_t raw_temp = ((uint16_t)data[0] << 8) | data[1];

    /* Convert to temperature */
    *temperature = MAX31856_ConvertCJTemperature(raw_temp);
    hmax->last_cj_temperature = *temperature;

    return MAX31856_OK;
}

/**
 * @brief Read thermocouple voltage (for voltage mode)
 */
max31856_status_t MAX31856_ReadTCVoltage(MAX31856_Handle_t *hmax, float *voltage) {
    if (hmax == NULL || voltage == NULL) return MAX31856_ERROR;

    int32_t raw;
    max31856_status_t status = MAX31856_ReadTCRaw(hmax, &raw);
    if (status != MAX31856_OK) return status;

    /* Convert raw value to voltage based on gain setting */
    if (hmax->tc_type == MAX31856_VOLTAGE_MODE_GAIN8) {
        *voltage = (float)raw / (8.0f * 1.6f * 131072.0f); // 2^17
    } else if (hmax->tc_type == MAX31856_VOLTAGE_MODE_GAIN32) {
        *voltage = (float)raw / (32.0f * 1.6f * 131072.0f); // 2^17
    } else {
        return MAX31856_ERROR; // Not in voltage mode
    }

    return MAX31856_OK;
}

/**
 * @brief Read raw thermocouple value
 */
max31856_status_t MAX31856_ReadTCRaw(MAX31856_Handle_t *hmax, int32_t *raw) {
    if (hmax == NULL || raw == NULL) return MAX31856_ERROR;

    max31856_status_t status;
    uint8_t data[3];

    /* If in normally off mode, trigger one-shot conversion */
    if (hmax->conv_mode == MAX31856_NORMALLY_OFF) {
        status = MAX31856_TriggerOneShot(hmax);
        if (status != MAX31856_OK) return status;
    }

    /* Read linearized temperature registers */
    status = MAX31856_ReadMultipleRegisters(hmax, MAX31856_LTCBH_REG, data, 3);
    if (status != MAX31856_OK) return status;

    /* Combine 3 bytes to form 24-bit value, then extract 19-bit value */
    uint32_t raw_temp = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
    raw_temp >>= 5; // Shift right to get 19-bit value

    /* Sign extend 19-bit to 32-bit */
    *raw = (int32_t)(raw_temp << 13) >> 13;
    hmax->last_tc_raw = *raw;

    return MAX31856_OK;
}

/**
 * @brief Read fault status
 */
max31856_status_t MAX31856_ReadFault(MAX31856_Handle_t *hmax, uint8_t *fault_status) {
    if (hmax == NULL || fault_status == NULL) return MAX31856_ERROR;

    max31856_status_t status = MAX31856_ReadRegister(hmax, MAX31856_SR_REG, fault_status);
    if (status == MAX31856_OK) {
        hmax->last_fault = *fault_status;
    }
    return status;
}

/**
 * @brief Clear fault status
 */
max31856_status_t MAX31856_ClearFault(MAX31856_Handle_t *hmax) {
    if (hmax == NULL) return MAX31856_ERROR;

    uint8_t cr0 = 0;
    if (MAX31856_ReadRegister(hmax, MAX31856_CR0_REG, &cr0) != MAX31856_OK) {
        return MAX31856_ERROR;
    }

    cr0 |= MAX31856_CR0_FAULTCLR;
    return MAX31856_WriteRegister(hmax, MAX31856_CR0_REG, cr0);
}

/**
 * @brief Set fault mask
 */
max31856_status_t MAX31856_SetFaultMask(MAX31856_Handle_t *hmax, uint8_t mask) {
    if (hmax == NULL) return MAX31856_ERROR;

    return MAX31856_WriteRegister(hmax, MAX31856_MASK_REG, mask);
}

/**
 * @brief Get fault status string
 */
const char* MAX31856_GetFaultString(uint8_t fault_bits) {
    static char fault_str[256];
    fault_str[0] = '\0';

    if (fault_bits == 0) {
        return "No faults";
    }

    if (fault_bits & MAX31856_FAULT_CJ_RANGE) {
        strcat(fault_str, "CJ Out of Range; ");
    }
    if (fault_bits & MAX31856_FAULT_TC_RANGE) {
        strcat(fault_str, "TC Out of Range; ");
    }
    if (fault_bits & MAX31856_FAULT_CJ_HIGH) {
        strcat(fault_str, "CJ High Threshold; ");
    }
    if (fault_bits & MAX31856_FAULT_CJ_LOW) {
        strcat(fault_str, "CJ Low Threshold; ");
    }
    if (fault_bits & MAX31856_FAULT_TC_HIGH) {
        strcat(fault_str, "TC High Threshold; ");
    }
    if (fault_bits & MAX31856_FAULT_TC_LOW) {
        strcat(fault_str, "TC Low Threshold; ");
    }
    if (fault_bits & MAX31856_FAULT_OVUV) {
        strcat(fault_str, "Over/Under Voltage; ");
    }
    if (fault_bits & MAX31856_FAULT_OPEN) {
        strcat(fault_str, "Open Circuit; ");
    }

    return fault_str;
}

/**
 * @brief Get thermocouple type string
 */
const char* MAX31856_GetTCTypeString(max31856_tc_type_t tc_type) {
    switch (tc_type) {
        case MAX31856_TC_TYPE_B: return "Type B";
        case MAX31856_TC_TYPE_E: return "Type E";
        case MAX31856_TC_TYPE_J: return "Type J";
        case MAX31856_TC_TYPE_K: return "Type K";
        case MAX31856_TC_TYPE_N: return "Type N";
        case MAX31856_TC_TYPE_R: return "Type R";
        case MAX31856_TC_TYPE_S: return "Type S";
        case MAX31856_TC_TYPE_T: return "Type T";
        case MAX31856_VOLTAGE_MODE_GAIN8: return "Voltage Mode (Gain 8)";
        case MAX31856_VOLTAGE_MODE_GAIN32: return "Voltage Mode (Gain 32)";
        default: return "Unknown";
    }
}

/**
 * @brief Read single register
 */
max31856_status_t MAX31856_ReadRegister(MAX31856_Handle_t *hmax, uint8_t reg, uint8_t *data) {
    if (hmax == NULL || data == NULL || hmax->platform.spi_write_read == NULL) {
        return MAX31856_ERROR;
    }

    uint8_t tx_data = reg & 0x7F; // Clear MSB for read
    uint8_t rx_data[2] = {0, 0};
    max31856_status_t status;

    if (hmax->platform.cs_low) {
        hmax->platform.cs_low();
    }

    if (hmax->platform.delay_ms) {
        hmax->platform.delay_ms(1);
    }

    /* Send register address and read data */
    status = hmax->platform.spi_write_read(hmax, &tx_data, rx_data, 2);

    if (hmax->platform.delay_ms) {
        hmax->platform.delay_ms(1);
    }

    if (hmax->platform.cs_high) {
        hmax->platform.cs_high();
    }

    if (status != MAX31856_OK) {
        return status;
    }

    *data = rx_data[1]; // Data comes in second byte
    return MAX31856_OK;
}

/**
 * @brief Write single register
 */
max31856_status_t MAX31856_WriteRegister(MAX31856_Handle_t *hmax, uint8_t reg, uint8_t data) {
    if (hmax == NULL || hmax->platform.spi_write_read == NULL) {
        return MAX31856_ERROR;
    }

    uint8_t tx_data[2];
    uint8_t rx_data[2];
    max31856_status_t status;

    tx_data[0] = reg | 0x80; // Set MSB for write
    tx_data[1] = data;

    if (hmax->platform.cs_low) {
        hmax->platform.cs_low();
    }

    if (hmax->platform.delay_ms) {
        hmax->platform.delay_ms(1);
    }

    status = hmax->platform.spi_write_read(hmax, tx_data, rx_data, 2);

    if (hmax->platform.delay_ms) {
        hmax->platform.delay_ms(1);
    }

    if (hmax->platform.cs_high) {
        hmax->platform.cs_high();
    }

    return status;
}

/**
 * @brief Read multiple registers
 */
max31856_status_t MAX31856_ReadMultipleRegisters(MAX31856_Handle_t *hmax, uint8_t start_reg, uint8_t *data, uint8_t count) {
    if (hmax == NULL || data == NULL || hmax->platform.spi_write_read == NULL || count == 0) {
        return MAX31856_ERROR;
    }

    uint8_t tx_data = start_reg & 0x7F; // Clear MSB for read
    uint8_t *rx_data = (uint8_t*)malloc(count + 1);
    if (rx_data == NULL) {
        return MAX31856_ERROR;
    }

    max31856_status_t status;

    if (hmax->platform.cs_low) {
        hmax->platform.cs_low();
    }

    if (hmax->platform.delay_ms) {
        hmax->platform.delay_ms(1);
    }

    /* Send register address and read data */
    status = hmax->platform.spi_write_read(hmax, &tx_data, rx_data, count + 1);

    if (hmax->platform.delay_ms) {
        hmax->platform.delay_ms(1);
    }

    if (hmax->platform.cs_high) {
        hmax->platform.cs_high();
    }

    if (status == MAX31856_OK) {
        /* Copy data (skip first byte which is dummy) */
        memcpy(data, &rx_data[1], count);
    }

    free(rx_data);
    return status;
}

