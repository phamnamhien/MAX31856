/**
 * @file    max31856.h
 * @brief   MAX31856 Precision Thermocouple to Digital Converter Generic Library
 * @author  Generated based on MAX31865 style
 * @date    2024
 * @version 1.0 - Generic version
 */

#ifndef MAX31856_H
#define MAX31856_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* Debug Configuration */
#ifndef MAX31856_DEBUG
#define MAX31856_DEBUG 0
#endif

#if MAX31856_DEBUG
#define MAX31856_DEBUG_PRINT(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
#define MAX31856_DEBUG_PRINT(fmt, ...) ((void)0)
#endif

/* MAX31856 Register Map */
#define MAX31856_CR0_REG              0x00
#define MAX31856_CR1_REG              0x01
#define MAX31856_MASK_REG             0x02
#define MAX31856_CJHF_REG             0x03
#define MAX31856_CJLF_REG             0x04
#define MAX31856_LTHFTH_REG           0x05
#define MAX31856_LTHFTL_REG           0x06
#define MAX31856_LTLFTH_REG           0x07
#define MAX31856_LTLFTL_REG           0x08
#define MAX31856_CJTO_REG             0x09
#define MAX31856_CJTH_REG             0x0A
#define MAX31856_CJTL_REG             0x0B
#define MAX31856_LTCBH_REG            0x0C
#define MAX31856_LTCBM_REG            0x0D
#define MAX31856_LTCBL_REG            0x0E
#define MAX31856_SR_REG               0x0F

/* Configuration Register 0 Bits */
#define MAX31856_CR0_CMODE            0x80  // Conversion Mode
#define MAX31856_CR0_1SHOT            0x40  // One-shot conversion
#define MAX31856_CR0_OCFAULT1         0x20  // Open-circuit fault detection 1
#define MAX31856_CR0_OCFAULT0         0x10  // Open-circuit fault detection 0
#define MAX31856_CR0_CJ               0x08  // Cold-junction sensor disable
#define MAX31856_CR0_FAULT            0x04  // Fault mode
#define MAX31856_CR0_FAULTCLR         0x02  // Fault status clear
#define MAX31856_CR0_50_60HZ          0x01  // 50Hz/60Hz filter

/* Configuration Register 1 Bits */
#define MAX31856_CR1_AVGSEL2          0x40  // Averaging mode bit 2
#define MAX31856_CR1_AVGSEL1          0x20  // Averaging mode bit 1
#define MAX31856_CR1_AVGSEL0          0x10  // Averaging mode bit 0
#define MAX31856_CR1_TC_TYPE_MASK     0x0F  // Thermocouple type mask

/* Fault Status Register Bits */
#define MAX31856_FAULT_CJ_RANGE       0x80
#define MAX31856_FAULT_TC_RANGE       0x40
#define MAX31856_FAULT_CJ_HIGH        0x20
#define MAX31856_FAULT_CJ_LOW         0x10
#define MAX31856_FAULT_TC_HIGH        0x08
#define MAX31856_FAULT_TC_LOW         0x04
#define MAX31856_FAULT_OVUV           0x02
#define MAX31856_FAULT_OPEN           0x01

/* Default Values */
#define MAX31856_SPI_TIMEOUT          100       // SPI timeout in ms

/* Thermocouple Types */
typedef enum {
    MAX31856_TC_TYPE_B = 0,
    MAX31856_TC_TYPE_E = 1,
    MAX31856_TC_TYPE_J = 2,
    MAX31856_TC_TYPE_K = 3,
    MAX31856_TC_TYPE_N = 4,
    MAX31856_TC_TYPE_R = 5,
    MAX31856_TC_TYPE_S = 6,
    MAX31856_TC_TYPE_T = 7,
    MAX31856_VOLTAGE_MODE_GAIN8 = 8,
    MAX31856_VOLTAGE_MODE_GAIN32 = 12
} max31856_tc_type_t;

/* Averaging Mode */
typedef enum {
    MAX31856_AVG_1_SAMPLE = 0,
    MAX31856_AVG_2_SAMPLES = 1,
    MAX31856_AVG_4_SAMPLES = 2,
    MAX31856_AVG_8_SAMPLES = 3,
    MAX31856_AVG_16_SAMPLES = 4
} max31856_avg_mode_t;

/* Filter Configuration */
typedef enum {
    MAX31856_FILTER_60HZ = 0,
    MAX31856_FILTER_50HZ = 1
} max31856_filter_t;

/* Conversion Mode */
typedef enum {
    MAX31856_NORMALLY_OFF = 0,
    MAX31856_AUTO_CONVERT = 1
} max31856_conv_mode_t;

/* Fault Mode */
typedef enum {
    MAX31856_FAULT_COMPARATOR = 0,
    MAX31856_FAULT_INTERRUPT = 1
} max31856_fault_mode_t;

/* Open-Circuit Fault Detection */
typedef enum {
    MAX31856_OCFAULT_DISABLED = 0,
    MAX31856_OCFAULT_10MS = 1,
    MAX31856_OCFAULT_32MS = 2,
    MAX31856_OCFAULT_100MS = 3
} max31856_ocfault_t;

/* Return Status */
typedef enum {
    MAX31856_OK = 0,
    MAX31856_ERROR = 1,
    MAX31856_TIMEOUT = 2,
    MAX31856_FAULT = 3
} max31856_status_t;

/* Forward declaration */
struct MAX31856_Handle;

/* Platform Interface Function Pointers */
typedef struct {
    /* SPI Communication Functions */
    max31856_status_t (*spi_write_read)(struct MAX31856_Handle *hmax, uint8_t *tx_data, uint8_t *rx_data, uint16_t size);

    /* GPIO Control Functions */
    void (*cs_low)(struct MAX31856_Handle *hmax);
    void (*cs_high)(struct MAX31856_Handle *hmax);

    /* Delay Function */
    void (*delay_ms)(uint32_t ms);

    /* Optional: Custom data pointer for platform-specific data */
    void *platform_data;
} max31856_platform_t;

/* MAX31856 Device Structure */
typedef struct MAX31856_Handle {
    /* Platform Interface */
    max31856_platform_t platform;

    /* Hardware Configuration - Platform specific identifiers */
    void *cs_port;      // Can be GPIO port, pin number, or any platform-specific identifier
    uint32_t cs_pin;    // Pin identifier

    /* Configuration */
    max31856_tc_type_t tc_type;        // Thermocouple type
    max31856_avg_mode_t avg_mode;      // Averaging mode
    max31856_filter_t filter;          // Noise filter selection
    max31856_conv_mode_t conv_mode;    // Conversion mode
    max31856_fault_mode_t fault_mode;  // Fault mode
    max31856_ocfault_t ocfault;        // Open-circuit fault detection

    /* Status */
    bool initialized;
    uint8_t last_fault;
    float last_temperature;
    float last_cj_temperature;
    int32_t last_tc_raw;
} MAX31856_Handle_t;

/* Function Prototypes */

/* Initialization and Configuration */
max31856_status_t MAX31856_Init(MAX31856_Handle_t *hmax,
                                const max31856_platform_t *platform,
                                void *cs_port, uint32_t cs_pin,
                                max31856_tc_type_t tc_type);

max31856_status_t MAX31856_DeInit(MAX31856_Handle_t *hmax);

max31856_status_t MAX31856_SetTCType(MAX31856_Handle_t *hmax, max31856_tc_type_t tc_type);
max31856_status_t MAX31856_SetFilter(MAX31856_Handle_t *hmax, max31856_filter_t filter);
max31856_status_t MAX31856_SetAvgMode(MAX31856_Handle_t *hmax, max31856_avg_mode_t avg_mode);
max31856_status_t MAX31856_SetConvMode(MAX31856_Handle_t *hmax, max31856_conv_mode_t conv_mode);
max31856_status_t MAX31856_SetFaultMode(MAX31856_Handle_t *hmax, max31856_fault_mode_t fault_mode);
max31856_status_t MAX31856_SetOCFault(MAX31856_Handle_t *hmax, max31856_ocfault_t ocfault);

/* Temperature Threshold Configuration */
max31856_status_t MAX31856_SetCJHighThreshold(MAX31856_Handle_t *hmax, float temperature);
max31856_status_t MAX31856_SetCJLowThreshold(MAX31856_Handle_t *hmax, float temperature);
max31856_status_t MAX31856_SetTCHighThreshold(MAX31856_Handle_t *hmax, float temperature);
max31856_status_t MAX31856_SetTCLowThreshold(MAX31856_Handle_t *hmax, float temperature);

/* Temperature Offset */
max31856_status_t MAX31856_SetCJOffset(MAX31856_Handle_t *hmax, float offset);

/* Cold-Junction Control */
max31856_status_t MAX31856_EnableCJSensor(MAX31856_Handle_t *hmax, bool enable);
max31856_status_t MAX31856_SetExternalCJTemp(MAX31856_Handle_t *hmax, float temperature);

/* Reading Functions */
max31856_status_t MAX31856_ReadTemperature(MAX31856_Handle_t *hmax, float *temperature);
max31856_status_t MAX31856_ReadCJTemperature(MAX31856_Handle_t *hmax, float *temperature);
max31856_status_t MAX31856_ReadTCVoltage(MAX31856_Handle_t *hmax, float *voltage);
max31856_status_t MAX31856_ReadTCRaw(MAX31856_Handle_t *hmax, int32_t *raw);

/* One-shot Conversion */
max31856_status_t MAX31856_TriggerOneShot(MAX31856_Handle_t *hmax);

/* Fault Detection */
max31856_status_t MAX31856_ReadFault(MAX31856_Handle_t *hmax, uint8_t *fault_status);
max31856_status_t MAX31856_ClearFault(MAX31856_Handle_t *hmax);
max31856_status_t MAX31856_SetFaultMask(MAX31856_Handle_t *hmax, uint8_t mask);

/* Utility Functions */
const char* MAX31856_GetFaultString(uint8_t fault_bits);
const char* MAX31856_GetTCTypeString(max31856_tc_type_t tc_type);

/* Low-Level Register Access */
max31856_status_t MAX31856_ReadRegister(MAX31856_Handle_t *hmax, uint8_t reg, uint8_t *data);
max31856_status_t MAX31856_WriteRegister(MAX31856_Handle_t *hmax, uint8_t reg, uint8_t data);
max31856_status_t MAX31856_ReadMultipleRegisters(MAX31856_Handle_t *hmax, uint8_t start_reg, uint8_t *data, uint8_t count);

#ifdef __cplusplus
}
#endif

#endif /* MAX31856_H */
