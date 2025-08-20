#ifndef BQ25620_H
#define BQ25620_H

#include <driver/i2c.h>
#include <esp_err.h>
#include <stdbool.h> // For bool type

// BQ25620 specific definitions
#define BQ25620_I2C_ADDR 0x6B // 7-bit I2C address of BQ25620

// BQ25620 Register Addresses
#define BQ25620_REG_CHARGE_CURRENT_LIMIT_LSB 0x02 // Charge Current Limit LSB Register (ICHG[1:0] and Reserved)
#define BQ25620_REG_CHARGE_CURRENT_LIMIT_MSB 0x03 // Charge Current Limit MSB Register (ICHG[5:2] and Reserved)
#define BQ25620_REG_CHARGE_VOLTAGE_LIMIT_LSB 0x04 // Charge Voltage Limit LSB Register (VREG[4:0] and Reserved)
#define BQ25620_REG_CHARGE_VOLTAGE_LIMIT_MSB 0x05 // Charge Voltage Limit MSB Register (VREG[8:5] and Reserved)
#define BQ25620_REG_INPUT_CURRENT_LIMIT_LSB 0x06  // Input Current Limit LSB Register (IINDPM[3:0] and Reserved)
#define BQ25620_REG_INPUT_CURRENT_LIMIT_MSB 0x07  // Input Current Limit MSB Register (IINDPM[7:4] and Reserved)
#define BQ25620_REG_MINIMAL_SYSTEM_VOLTAGE 0x0E   // Minimal System Voltage Register (VINDPM_DCHG)
#define BQ25620_REG_CHARGER_CONTROL_1 0x16        // Charger Control 1 Register (for Watchdog)
#define BQ25620_REG_CHARGER_STATUS_1 0x1E         // Charger Status 1 Register
#define BQ25620_REG_FAULT_STATUS_0 0x1F           // FAULT Status 0 Register
#define BQ25620_REG_CHARGER_CONTROL_3 0x18        // Charger Control 3 Register (for OTG mode)
#define BQ25620_REG_CHARGER_CONTROL_2 0x17        // Charger Control 2 Register (for REG_RST)
#define BQ25620_REG_NTC_CONTROL_0 0x1A            // NTC Control 0 Register (for TS_IGNORE)

// OTG Voltage Regulation Register
#define BQ25620_REG_VOTG_REGULATION_LSB     0x0C // VOTG regulation LSB Register
#define BQ25620_REG_VOTG_REGULATION_MSB     0x0D // VOTG regulation MSB Register

// System Control 0 Register for Ship Mode
#define BQ25620_REG_SYSTEM_CONTROL_0        0x1B // System Control 0 Register (for BATFET_DIS)
#define BQ25620_BATFET_DIS_BIT              5    // Bit 5 in REG0x1B to disable BATFET (enter ship mode)

// BQ25620 ADC Control and Measurement Registers (CORRECTED based on user feedback)
#define BQ25620_REG_ADC_CONTROL 0x26            // ADC Control Register (for ADC_EN)
#define BQ25620_REG_ADC_FUNCTION_DISABLE_0 0x27 // ADC Function Disable 0 (user provided, not used in this example)
#define BQ25620_REG_IBUS_ADC_LSB 0x28           // IBUS ADC LSB Register
#define BQ25620_REG_IBUS_ADC_MSB 0x29           // IBUS ADC MSB Register
#define BQ25620_REG_IBAT_ADC_LSB 0x2A           // IBAT ADC LSB Register (likely combines charge/discharge)
#define BQ25620_REG_IBAT_ADC_MSB 0x2B           // IBAT ADC MSB Register
#define BQ25620_REG_VBUS_ADC_LSB 0x2C           // VBUS ADC LSB Register
#define BQ25620_REG_VBUS_ADC_MSB 0x2D           // VBUS ADC MSB Register

#define BQ25620_REG_VPMID_ADC_LSB 0x2E // VPMID ADC LSB Register
#define BQ25620_REG_VPMID_ADC_MSB 0x2F // VPMID ADC MSB Register

#define BQ25620_REG_VBAT_ADC_LSB 0x30 // VBAT ADC LSB Register
#define BQ25620_REG_VBAT_ADC_MSB 0x31 // VBAT ADC MSB Register

#define BQ25620_REG_VSYS_ADC_LSB 0x32 // VSYS ADC LSB Register
#define BQ25620_REG_VSYS_ADC_MSB 0x33 // VSYS ADC MSB Register

#define BQ25620_REG_TS_ADC_LSB 0x34 // TS ADC LSB Register (Temperature Sensor)
#define BQ25620_REG_TS_ADC_MSB 0x35 // TS ADC MSB Register

#define BQ25620_REG_TDIE_ADC_LSB 0x36 // TDIE ADC LSB Register (Die Temperature)
#define BQ25620_REG_TDIE_ADC_MSB 0x37 // TDIE ADC MSB Register

// Function Prototypes for BQ25620 operations
/**
 * @brief Configures the BQ25620 battery charger IC with default values.
 * @param i2c_num The I2C port number.
 * @return esp_err_t ESP_OK on success, or an error code otherwise.
 */
esp_err_t bq25620_configure(i2c_port_t i2c_num);

/**
 * @brief Sets the input current limit of the BQ25620.
 * @param i2c_num The I2C port number.
 * @param limit_ma The desired input current limit in mA.
 * @return esp_err_t ESP_OK on success, or an error code otherwise.
 */
esp_err_t bq25620_set_input_current_limit(i2c_port_t i2c_num, uint16_t limit_ma);

/**
 * @brief Sets the charge current limit of the BQ25620.
 * @param i2c_num The I2C port number.
 * @param limit_ma The desired charge current limit in mA.
 * @return esp_err_t ESP_OK on success, or an error code otherwise.
 */
esp_err_t bq25620_set_charge_current_limit(i2c_port_t i2c_num, uint16_t limit_ma);

/**
 * @brief Sets the charge voltage limit of the BQ25620.
 * @param i2c_num The I2C port number.
 * @param limit_mv The desired charge voltage limit in mV.
 * @return esp_err_t ESP_OK on success, or an error code otherwise.
 */
esp_err_t bq25620_set_charge_voltage_limit(i2c_port_t i2c_num, uint16_t limit_mv);


/**
 * @brief Sets the OTG (On-The-Go) output voltage limit of the BQ25620.
 * @param i2c_num The I2C port number.
 * @param voltage_mv The desired OTG output voltage in mV (range 3840mV to 9600mV).
 * @return esp_err_t ESP_OK on success, or an error code otherwise.
 */
esp_err_t bq25620_set_otg_voltage_limit(i2c_port_t i2c_num, uint16_t voltage_mv);

/**
 * @brief Resets the BQ25620 watchdog timer.
 * @param i2c_num The I2C port number.
 * @return esp_err_t ESP_OK on success, or an error code otherwise.
 */
esp_err_t bq25620_reset_watchdog(i2c_port_t i2c_num);

/**
 * @brief Reads and interprets the BQ25620 Charger Status 1 register (0x1E).
 * @param i2c_num The I2C port number.
 * @return esp_err_t ESP_OK on success, or an error code otherwise.
 */
esp_err_t bq25620_check_status(i2c_port_t i2c_num);

/**
 * @brief Reads and interprets the BQ25620 FAULT Status 0 register (0x1F).
 * @param i2c_num The I2C port number.
 * @return esp_err_t ESP_OK on success, or an error code otherwise.
 */
esp_err_t bq25620_check_fault_status(i2c_port_t i2c_num);

/**
 * @brief Enables or disables OTG (On-The-Go) mode on the BQ25620.
 * @param i2c_num The I2C port number.
 * @param enable True to enable OTG mode, false to disable.
 * @return esp_err_t ESP_OK on success, or an error code otherwise.
 */
esp_err_t bq25620_enable_otg_mode(i2c_port_t i2c_num, bool enable);

/**
 * @brief Performs a software reset of BQ25620 registers to their default POR values.
 * @param i2c_num The I2C port number.
 * @return esp_err_t ESP_OK on success, or an error code otherwise.
 */
esp_err_t bq25620_software_reset(i2c_port_t i2c_num);

/**
 * @brief Reads the raw value of the Charger Status 1 register (0x1E).
 * @param i2c_num The I2C port number.
 * @param reg_value Pointer to a uint8_t to store the read register value.
 * @return esp_err_t ESP_OK on success, or an error code otherwise.
 */
esp_err_t bq25620_read_charger_status1_reg(i2c_port_t i2c_num, uint8_t *reg_value);

/**
 * @brief Sets or clears the TS_IGNORE bit in REG0x1A_NTC_Control_0.
 * When TS_IGNORE is set, TS function and NTC monitoring are disabled.
 * @param i2c_num The I2C port number.
 * @param ignore True to set TS_IGNORE, false to clear it.
 * @return esp_err_t ESP_OK on success, or an error code otherwise.
 */
esp_err_t bq25620_set_ts_ignore_bit(i2c_port_t i2c_num, bool ignore);

/**
 * @brief Sets the Minimal System Voltage (VSYSMIN) on the BQ25620.
 * The programmable range is 2.5V to 3.8V, with a 10mV step.
 * Values outside this range will be clamped to the min/max.
 * @param i2c_num The I2C port number.
 * @param voltage_mv The desired minimal system voltage in mV (e.g., 2500 for 2.5V).
 * @return esp_err_t ESP_OK on success, or an error code otherwise.
 */
esp_err_t bq25620_set_minimal_system_voltage(i2c_port_t i2c_num, uint16_t voltage_mv);

/**
 * @brief Reads VBUS voltage from BQ25620.
 * @param i2c_num I2C port number.
 * @param voltage_mv Pointer to store the read voltage in mV.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bq25620_read_vbus_voltage(i2c_port_t i2c_num, uint16_t *voltage_mv);

/**
 * @brief Reads VSYS voltage from BQ25620.
 * @param i2c_num I2C port number.
 * @param voltage_mv Pointer to store the read voltage in mV.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bq25620_read_vsys_voltage(i2c_port_t i2c_num, uint16_t *voltage_mv);

/**
 * @brief Reads VBAT voltage from BQ25620.
 * @param i2c_num I2C port number.
 * @param voltage_mv Pointer to store the read voltage in mV.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bq25620_read_vbat_voltage(i2c_port_t i2c_num, uint16_t *voltage_mv);

/**
 * @brief Reads IBUS current from BQ25620.
 * @param i2c_num I2C port number.
 * @param current_ma Pointer to store the read current in mA.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bq25620_read_ibus_current(i2c_port_t i2c_num, int16_t *current_ma);

/**
 * @brief Reads IBAT current from BQ25620. This represents the battery charge/discharge current.
 * @param i2c_num I2C port number.
 * @param current_ma Pointer to store the read current in mA.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bq25620_read_ibat_current(i2c_port_t i2c_num, int16_t *current_ma);

/**
 * @brief Reads VPMID voltage from BQ25620.
 * @param i2c_num I2C port number.
 * @param voltage_mv Pointer to store the read voltage in mV.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bq25620_read_vpmid_voltage(i2c_port_t i2c_num, uint16_t *voltage_mv);

/**
 * @brief Reads TS (Temperature Sensor) ADC value from BQ25620.
 * This is a raw ADC reading and typically needs further conversion to degrees Celsius.
 * @param i2c_num I2C port number.
 * @param adc_value Pointer to store the raw ADC value.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bq25620_read_ts_adc(i2c_port_t i2c_num, uint16_t *adc_value);

/**
 * @brief Reads TDIE (Die Temperature) ADC value from BQ25620.
 * This is a raw ADC reading and typically needs further conversion to degrees Celsius.
 * @param i2c_num I2C port number.
 * @param adc_value Pointer to store the raw ADC value.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bq25620_read_tdie_adc(i2c_port_t i2c_num, uint16_t *adc_value);

/**
 * @brief Displays the current voltage, current, and temperature status from BQ25620.
 * @param i2c_num I2C port number.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bq25620_display_levels(i2c_port_t i2c_num);

/**
 * @brief Enables the internal ADC and sets its sample resolution.
 * @param i2c_num I2C port number.
 * @param enable_adc True to enable ADC, false to disable.
 * @param use_12bit_resolution True to set to 12-bit resolution, false for 8-bit.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bq25620_set_adc_settings(i2c_port_t i2c_num, bool enable_adc, bool use_12bit_resolution);

/**
 * @brief Enters BQ25620 into ship mode by disabling the BATFET.
 * @param i2c_num The I2C port number.
 * @return esp_err_t ESP_OK on success, or an error code otherwise.
 */
esp_err_t bq25620_enter_ship_mode(i2c_port_t i2c_num);
#endif // BQ25620_H