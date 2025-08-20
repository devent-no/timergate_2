#include "bq25620.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h> // For vTaskDelay, pdMS_TO_TICKS

static const char *TAG = "BQ25620_DRV"; // Tag for ESP_LOG messages in this driver file

// I2C ACK values for internal use
#define ACK_CHECK_EN 0x1  // I2C master will check ack from slave
#define ACK_CHECK_DIS 0x0 // I2C master will not check ack from slave
#define ACK_VAL 0x0       // I2C ack value
#define NACK_VAL 0x1      // I2C nack value

// VSYSMIN limits
#define VSYSMIN_MIN_MV 2500 // 2.5V
#define VSYSMIN_MAX_MV 3800 // 3.8V
#define VSYSMIN_STEP_MV 10

// Common ADC scaling factors (based on typical BQ series datasheets, to be verified with full datasheet)
#define VOLTAGE_ADC_VBUS_STEP_MV 3.97 // 10mV/LSB for voltage ADCs (VBUS, VSYS, VBAT, VPMID)
#define VOLTAGE_ADC_VSYS_STEP_MV 1.99
#define VOLTAGE_ADC_BITS 11   // Voltage ADC typically 11 bits (0 to 2047, 0-20.47V range)
#define CURRENT_ADC_STEP_MA 4 // 4mA/LSB for current ADCs (IBUS, IBAT) - CORRECTED
#define CURRENT_ADC_BITS 15   // Current ADC (IBUS, IBAT) is 15 bits, 2's complement - CORRECTED
#define TEMP_ADC_STEP_MV 1    // Assuming 1mV/LSB for temperature ADCs (TS, TDIE), requires lookup table for Celsius

// VOTG limits
#define VOTG_MIN_MV                 3840  // 3.84V
#define VOTG_MAX_MV                 9600  // 9.60V
#define VOTG_STEP_MV                80    // 80mV/LSB

// Helper function to read two consecutive 8-bit registers as a single 16-bit value
// This performs a single I2C transaction: write starting register address, then read two bytes.
static esp_err_t bq25620_read_adc_register_pair(i2c_port_t i2c_num, uint8_t start_reg_addr, uint16_t *value)
{
    uint8_t read_data[2]; // read_data[0] will be LSB, read_data[1] will be MSB
    esp_err_t ret = i2c_master_write_read_device(i2c_num, BQ25620_I2C_ADDR,
                                                 &start_reg_addr, 1, // Write 1 byte: the starting register address (LSB)
                                                 read_data, 2,       // Read 2 bytes: LSB then MSB
                                                 100 / portTICK_PERIOD_MS);
    if (ret == ESP_OK)
    {
        // Combine bytes assuming little-endian register order (LSB at lower address, MSB at higher address)
        *value = ((uint16_t)read_data[1] << 8) | read_data[0];
    }
    else
    {
        // Log error at a higher level (display_levels or specific read function) for clarity
        // ESP_LOGE(TAG, "Failed to read 16-bit from reg 0x%02X: %s", start_reg_addr, esp_err_to_name(ret));
    }
    return ret;
}



/**
 * @brief Generic function to write to a 16-bit register pair (LSB then MSB).
 *
 * This function handles writing two consecutive 8-bit bytes (LSB and MSB)
 * to the BQ25620, starting from the specified `lsb_reg_addr`.
 *
 * @param i2c_num The I2C port number.
 * @param lsb_reg_addr The address of the LSB register.
 * @param lsb_data The 8-bit data for the LSB register.
 * @param msb_data The 8-bit data for the MSB register.
 * @return esp_err_t ESP_OK on success, or an error code otherwise.
 */
static esp_err_t bq25620_write_register_pair(i2c_port_t i2c_num, uint8_t lsb_reg_addr, uint8_t lsb_data, uint8_t msb_data) {
    uint8_t write_buffer[3];
    write_buffer[0] = lsb_reg_addr;
    write_buffer[1] = lsb_data;
    write_buffer[2] = msb_data;
    esp_err_t ret = i2c_master_write_to_device(i2c_num, BQ25620_I2C_ADDR, write_buffer, 3, 100 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write to registers 0x%02X/0x%02X: %s", lsb_reg_addr, lsb_reg_addr + 1, esp_err_to_name(ret));
    }
    return ret;
}

/**
 * @brief Sets the input current limit of the BQ25620.
 *
 * This function calculates the `IINDPM` register values based on the
 * desired input current limit and writes them to REG0x06 and REG0x07.
 * Programmable range: 0mA to 3500mA with a 20mA step.
 *
 * @param i2c_num The I2C port number.
 * @param limit_ma The desired input current limit in mA (e.g., 2000 for 2A).
 * @return esp_err_t ESP_OK on success, or an error code otherwise.
 */
esp_err_t bq25620_set_input_current_limit(i2c_port_t i2c_num, uint16_t limit_ma) {
    // Clamp limit_ma to the device's typical range if needed, though not explicitly requested
    // Max IINDPM value is (3500mA / 20mA) = 175. So max 8-bit value is 0xAF.
    if (limit_ma > 3500) {
        limit_ma = 3500;
        ESP_LOGW(TAG, "Input current limit clamped to 3500mA.");
    }

    uint16_t value = limit_ma / 20; // 20mA step
    // IINDPM[3:0] in bits 7:4 of LSB register (0x06)
    uint8_t lsb_data = (value & 0x0F) << 4;
    // IINDPM[7:4] in bits 3:0 of MSB register (0x07)
    uint8_t msb_data = (value >> 4) & 0x0F;

    ESP_LOGI(TAG, "Setting Input Current Limit to %u mA (Value: %u, LSB_data: 0x%02X, MSB_data: 0x%02X)", limit_ma, value, lsb_data, msb_data);
    return bq25620_write_register_pair(i2c_num, BQ25620_REG_INPUT_CURRENT_LIMIT_LSB, lsb_data, msb_data);
}

/**
 * @brief Sets the charge current limit of the BQ25620.
 *
 * This function calculates the `ICHG` register values based on the
 * desired charge current limit and writes them to REG0x02 and REG0x03.
 * Programmable range: 80mA to 3500mA with an 80mA step.
 *
 * @param i2c_num The I2C port number.
 * @param limit_ma The desired charge current limit in mA (e.g., 2000 for 2A).
 * @return esp_err_t ESP_OK on success, or an error code otherwise.
 */
esp_err_t bq25620_set_charge_current_limit(i2c_port_t i2c_num, uint16_t limit_ma) {
    // Clamp limit_ma to the device's typical range if needed
    // Max ICHG value is (3500mA / 80mA) = 43.75, so max 7-bit value is 0x2B.
    if (limit_ma > 3500) {
        limit_ma = 3500;
        ESP_LOGW(TAG, "Charge current limit clamped to 3500mA.");
    }
    if (limit_ma < 80) { // Minimum charge current
        limit_ma = 80;
        ESP_LOGW(TAG, "Charge current limit clamped to 80mA (minimum).");
    }

    uint16_t value = limit_ma / 80; // 80mA step
    // ICHG[1:0] in bits 7:6 of LSB register (0x02)
    uint8_t lsb_data = (value & 0x03) << 6;
    // ICHG[5:2] in bits 3:0 of MSB register (0x03)
    uint8_t msb_data = (value >> 2) & 0x0F;

    ESP_LOGI(TAG, "Setting Charge Current Limit to %u mA (Value: %u, LSB_data: 0x%02X, MSB_data: 0x%02X)", limit_ma, value, lsb_data, msb_data);
    return bq25620_write_register_pair(i2c_num, BQ25620_REG_CHARGE_CURRENT_LIMIT_LSB, lsb_data, msb_data);
}

/**
 * @brief Sets the charge voltage limit of the BQ25620.
 *
 * This function calculates the `VREG` register values based on the
 * desired charge voltage limit and writes them to REG0x04 and REG0x05.
 * Programmable range: 3.5V to 4.5V with a 10mV step.
 *
 * @param i2c_num The I2C port number.
 * @param limit_mv The desired charge voltage limit in mV (e.g., 3650 for 3.65V).
 * @return esp_err_t ESP_OK on success, or an error code otherwise.
 */
esp_err_t bq25620_set_charge_voltage_limit(i2c_port_t i2c_num, uint16_t limit_mv) {
    // Clamp limit_mv to the device's typical range if needed
    // Min VREG value is 3.5V (3500mV), max 4.5V (4500mV).
    // (3500 - 3500) / 10 = 0.
    // (4500 - 3500) / 10 = 100.
    // So 9-bit value range is 0 to 100.
    if (limit_mv < 3500) {
        limit_mv = 3500;
        ESP_LOGW(TAG, "Charge voltage limit clamped to 3500mV (minimum).");
    } else if (limit_mv > 4500) {
        limit_mv = 4500;
        ESP_LOGW(TAG, "Charge voltage limit clamped to 4500mV (maximum).");
    }

    // Offset is 3.5V (3500mV)
    uint16_t value = (limit_mv - 3500) / 10; // 10mV step, starting from 3.5V
    // VREG[4:0] in bits 7:3 of LSB register (0x04)
    uint8_t lsb_data = (value & 0x1F) << 3;
    // VREG[8:5] in bits 3:0 of MSB register (0x05)
    uint8_t msb_data = (value >> 5) & 0x0F;

    ESP_LOGI(TAG, "Setting Charge Voltage Limit to %u mV (Value: %u, LSB_data: 0x%02X, MSB_data: 0x%02X)", limit_mv, value, lsb_data, msb_data);
    return bq25620_write_register_pair(i2c_num, BQ25620_REG_CHARGE_VOLTAGE_LIMIT_LSB, lsb_data, msb_data);
}


/**
 * @brief Sets the OTG (On-The-Go) output voltage limit of the BQ25620.
 *
 * This function calculates the `VOTG` register values based on the
 * desired OTG output voltage and writes them to REG0x0C and REG0x0D.
 * Programmable range: 3840mV to 9600mV with an 80mV step.
 *
 * @param i2c_num The I2C port number.
 * @param voltage_mv The desired OTG output voltage in mV (e.g., 5040 for 5.04V).
 * @return esp_err_t ESP_OK on success, or an error code otherwise.
 */
esp_err_t bq25620_set_otg_voltage_limit(i2c_port_t i2c_num, uint16_t voltage_mv) {
    // Clamp voltage_mv to the specified range
    if (voltage_mv < VOTG_MIN_MV) {
        voltage_mv = VOTG_MIN_MV;
        ESP_LOGW(TAG, "OTG voltage limit clamped to %u mV (minimum).", VOTG_MIN_MV);
    } else if (voltage_mv > VOTG_MAX_MV) {
        voltage_mv = VOTG_MAX_MV;
        ESP_LOGW(TAG, "OTG voltage limit clamped to %u mV (maximum).", VOTG_MAX_MV);
    }

    // Calculate the 7-bit VOTG value
    uint8_t value = (voltage_mv) / VOTG_STEP_MV;

    // VOTG[1:0] falls in REG0x0C[7:6] (LSB register)
    uint8_t lsb_data = (value & 0x03) << 6;
    // VOTG[6:2] falls in REG0x0D[4:0] (MSB register)
    uint8_t msb_data = (value >> 2) & 0x1F;

    ESP_LOGI(TAG, "Setting OTG Voltage Limit to %u mV (Value: %u, LSB_data: 0x%02X, MSB_data: 0x%02X)", voltage_mv, value, lsb_data, msb_data);
    return bq25620_write_register_pair(i2c_num, BQ25620_REG_VOTG_REGULATION_LSB, lsb_data, msb_data);
}


/**
 * @brief Configures the BQ25620 battery charger IC with default values.
 *
 * This function now calls the individual setting functions to
 * set common parameters for the BQ25620.
 *
 * @param i2c_num The I2C port number to use (e.g., I2C_NUM_0).
 * @return esp_err_t ESP_OK on success, or an error code otherwise.
 */
esp_err_t bq25620_configure(i2c_port_t i2c_num) {
    esp_err_t ret;

    ESP_LOGI(TAG, "Configuring BQ25620...");

    // 1. Set Input Current Limit to 3.5A
    ret = bq25620_set_input_current_limit(i2c_num, 3500);
    if (ret != ESP_OK) return ret;

    // 2. Set Charge Current Limit to 3A
    ret = bq25620_set_charge_current_limit(i2c_num, 3000);
    if (ret != ESP_OK) return ret;

    // 3. Set Charge Voltage Limit to 3.65V (3650mV)
    ret = bq25620_set_charge_voltage_limit(i2c_num, 3650);
    if (ret != ESP_OK) return ret;

    // 3. Set OTG Voltage Limit to 4.5V.
    ret = bq25620_set_otg_voltage_limit(i2c_num, 4300);
    if (ret != ESP_OK) return ret;

    // Set the Minimal System Voltage (VSYSMIN) to 2.5V (lowest possible for LiFePO4 cutoff)
    // Note: The BQ25620's VSYSMIN cannot be set to 2.0V as its minimum is 2.5V.
    ret = bq25620_set_minimal_system_voltage(i2c_num, 2500); // Set to 2500mV (2.5V)
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "BQ25620 configuration complete.");
    return ESP_OK;
}


/**
 * @brief Resets the BQ25620 watchdog timer.
 *
 * This function reads the current value of the Charger Control 1 register (0x16),
 * sets the WD_RST bit (bit 2) to trigger a watchdog reset, and writes the value back.
 * The WD_RST bit is self-clearing.
 *
 * @param i2c_num The I2C port number to use (e.g., I2C_NUM_0).
 * @return esp_err_t ESP_OK on success, or an error code otherwise.
 */
esp_err_t bq25620_reset_watchdog(i2c_port_t i2c_num)
{
    esp_err_t ret;
    uint8_t reg_value;
    uint8_t write_buffer[2]; // Buffer for register address and data

    //ESP_LOGI(TAG, "Attempting to reset BQ25620 watchdog timer...");

    uint8_t register_address_to_read = BQ25620_REG_CHARGER_CONTROL_1;

    // 1. Read the current value of REG0x16_Charger_Control_1
    ret = i2c_master_write_read_device(i2c_num, BQ25620_I2C_ADDR,
                                       &register_address_to_read, 1, // Write register address (now a variable's address)
                                       &reg_value, 1,                // Read 1 byte of data
                                       100 / portTICK_PERIOD_MS);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read Charger Control 1 register (0x%02X): %s", BQ25620_REG_CHARGER_CONTROL_1, esp_err_to_name(ret));
        return ret;
    }
    //ESP_LOGI(TAG, "BQ25620: Current Charger Control 1 (0x%02X) value: 0x%02X", BQ25620_REG_CHARGER_CONTROL_1, reg_value);

    // 2. Set the WD_RST bit (bit 2) to 1
    reg_value |= (1 << 2); // Set bit 2

    // 3. Write the modified value back to REG0x16_Charger_Control_1
    write_buffer[0] = BQ25620_REG_CHARGER_CONTROL_1;
    write_buffer[1] = reg_value;
    ret = i2c_master_write_to_device(i2c_num, BQ25620_I2C_ADDR, write_buffer, 2, 100 / portTICK_PERIOD_MS);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to write to Charger Control 1 register (0x%02X) for watchdog reset: %s", BQ25620_REG_CHARGER_CONTROL_1, esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "BQ25620: Watchdog reset command sent. New Charger Control 1 (0x%02X) value: 0x%02X", BQ25620_REG_CHARGER_CONTROL_1, reg_value);

    return ESP_OK;
}

/**
 * @brief Reads and interprets the BQ25620 Charger Status 1 register (0x1E).
 *
 * This function reads the REG0x1E_Charger_Status_1 register, extracts
 * the CHG_STAT and VBUS_STAT fields, and logs their interpreted values.
 *
 * @param i2c_num The I2C port number to use (e.g., I2C_NUM_0).
 * @return esp_err_t ESP_OK on success, or an error code otherwise.
 */
esp_err_t bq25620_check_status(i2c_port_t i2c_num)
{
    esp_err_t ret;
    uint8_t reg_value;
    uint8_t register_address_to_read = BQ25620_REG_CHARGER_STATUS_1;

    ESP_LOGI(TAG, "Checking BQ25620 status register (0x%02X)...", BQ25620_REG_CHARGER_STATUS_1);

    // Read the Charger Status 1 register
    ret = i2c_master_write_read_device(i2c_num, BQ25620_I2C_ADDR,
                                       &register_address_to_read, 1, // Write register address
                                       &reg_value, 1,                // Read 1 byte of data
                                       100 / portTICK_PERIOD_MS);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read Charger Status 1 register (0x%02X): %s", BQ25620_REG_CHARGER_STATUS_1, esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "BQ25620: Charger Status 1 (0x%02X) raw value: 0x%02X", BQ25620_REG_CHARGER_STATUS_1, reg_value);

    // Extract CHG_STAT (bits 4:3)
    uint8_t chg_stat = (reg_value >> 3) & 0x03; // Mask bits 4 and 3

    // Interpret CHG_STAT
    switch (chg_stat)
    {
    case 0x0:
        ESP_LOGI(TAG, "  CHG_STAT: Not Charging or Charge Terminated");
        break;
    case 0x1:
        ESP_LOGI(TAG, "  CHG_STAT: Trickle Charge, Pre-charge or Fast charge (CC mode)");
        break;
    case 0x2:
        ESP_LOGI(TAG, "  CHG_STAT: Taper Charge (CV mode)");
        break;
    case 0x3:
        ESP_LOGI(TAG, "  CHG_STAT: Top-off Timer Active Charging");
        break;
    default:
        ESP_LOGI(TAG, "  CHG_STAT: Unknown (%X)", chg_stat);
        break;
    }

    // Extract VBUS_STAT (bits 2:0)
    uint8_t vbus_stat = reg_value & 0x07; // Mask bits 2, 1, and 0

    // Interpret VBUS_STAT
    switch (vbus_stat)
    {
    case 0x0:
        ESP_LOGI(TAG, "  VBUS_STAT: No qualified adapter, or EN_AUTO_INDET = 0.");
        break;
    case 0x1:
        ESP_LOGI(TAG, "  VBUS_STAT: USB SDP Adapter (500mA)");
        break;
    case 0x2:
        ESP_LOGI(TAG, "  VBUS_STAT: USB CDP Adapter (1.5A)");
        break;
    case 0x3:
        ESP_LOGI(TAG, "  VBUS_STAT: USB DCP Adapter (1.5A)");
        break;
    case 0x4:
        ESP_LOGI(TAG, "  VBUS_STAT: Unknown Adapter (500mA)");
        break;
    case 0x5:
        ESP_LOGI(TAG, "  VBUS_STAT: Non-Standard Adapter (1A/2.1A/2.4A)");
        break;
    case 0x6:
        ESP_LOGI(TAG, "  VBUS_STAT: HVDCP adapter (1.5A)");
        break;
    case 0x7:
        ESP_LOGI(TAG, "  VBUS_STAT: In boost OTG mode");
        break;
    default:
        ESP_LOGI(TAG, "  VBUS_STAT: Unknown (%X)", vbus_stat);
        break;
    }

    return ESP_OK;
}

/**
 * @brief Reads and interprets the BQ25620 FAULT Status 0 register (0x1F).
 *
 * This function reads the REG0x1F_FAULT_Status_0 register and logs
 * the status of various fault conditions.
 *
 * @param i2c_num The I2C port number to use (e.g., I2C_NUM_0).
 * @return esp_err_t ESP_OK on success, or an error code otherwise.
 */
esp_err_t bq25620_check_fault_status(i2c_port_t i2c_num)
{
    esp_err_t ret;
    uint8_t reg_value;
    uint8_t register_address_to_read = BQ25620_REG_FAULT_STATUS_0;

    ESP_LOGI(TAG, "Checking BQ25620 fault status register (0x%02X)...", BQ25620_REG_FAULT_STATUS_0);

    // Read the FAULT Status 0 register
    ret = i2c_master_write_read_device(i2c_num, BQ25620_I2C_ADDR,
                                       &register_address_to_read, 1, // Write register address
                                       &reg_value, 1,                // Read 1 byte of data
                                       100 / portTICK_PERIOD_MS);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read FAULT Status 0 register (0x%02X): %s", BQ25620_REG_FAULT_STATUS_0, esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "BQ25620: FAULT Status 0 (0x%02X) raw value: 0x%02X", BQ25620_REG_FAULT_STATUS_0, reg_value);

    // Interpret VBUS_FAULT_STAT (bit 7)
    if ((reg_value >> 7) & 0x01)
    {
        ESP_LOGI(TAG, "  VBUS_FAULT_STAT: Device not switching due to overvoltage protection or sleep comparator");
    }
    else
    {
        ESP_LOGI(TAG, "  VBUS_FAULT_STAT: Normal");
    }

    // Interpret BAT_FAULT_STAT (bit 6)
    if ((reg_value >> 6) & 0x01)
    {
        ESP_LOGI(TAG, "  BAT_FAULT_STAT: Device in battery overcurrent protection or battery overvoltage protection");
    }
    else
    {
        ESP_LOGI(TAG, "  BAT_FAULT_STAT: Normal");
    }

    // Interpret SYS_FAULT_STAT (bit 5)
    if ((reg_value >> 5) & 0x01)
    {
        ESP_LOGI(TAG, "  SYS_FAULT_STAT: SYS in SYS short circuit or overvoltage");
    }
    else
    {
        ESP_LOGI(TAG, "  SYS_FAULT_STAT: Normal");
    }

    // Interpret OTG_FAULT_STAT (bit 4)
    if ((reg_value >> 4) & 0x01)
    {
        ESP_LOGI(TAG, "  OTG_FAULT_STAT: Reverse-current fault or PMID or VBUS in overvoltage or undervoltage during OTG");
    }
    else
    {
        ESP_LOGI(TAG, "  OTG_FAULT_STAT: Normal");
    }

    // Interpret TSHUT_STAT (bit 3)
    if ((reg_value >> 3) & 0x01)
    {
        ESP_LOGI(TAG, "  TSHUT_STAT: IC temperature shutdown status: Device in thermal shutdown protection");
    }
    else
    {
        ESP_LOGI(TAG, "  TSHUT_STAT: IC temperature shutdown status: Normal");
    }

    // Extract TS_STAT (bits 2:0)
    uint8_t ts_stat = reg_value & 0x07; // Mask bits 2, 1, and 0

    // Interpret TS_STAT
    switch (ts_stat)
    {
    case 0x0:
        ESP_LOGI(TAG, "  TS_STAT: TS_NORMAL");
        break;
    case 0x1:
        ESP_LOGI(TAG, "  TS_STAT: TS_COLD or TS_OTG_COLD or TS resistor string power rail is not available.");
        break;
    case 0x2:
        ESP_LOGI(TAG, "  TS_STAT: TS_HOT or TS_OTG_HOT");
        break;
    case 0x3:
        ESP_LOGI(TAG, "  TS_STAT: TS_COOL");
        break;
    case 0x4:
        ESP_LOGI(TAG, "  TS_STAT: TS_WARM");
        break;
    case 0x5:
        ESP_LOGI(TAG, "  TS_STAT: TS_PRECOOL");
        break;
    case 0x6:
        ESP_LOGI(TAG, "  TS_STAT: TS_PREWARM");
        break;
    case 0x7:
        ESP_LOGI(TAG, "  TS_STAT: TS pin bias reference fault");
        break;
    default:
        ESP_LOGI(TAG, "  TS_STAT: Unknown (%X)", ts_stat);
        break;
    }

    return ESP_OK;
}

/**
 * @brief Enables or disables OTG (On-The-Go) mode on the BQ25620.
 *
 * This function reads the current value of the Charger Control 3 register (0x18),
 * sets or clears the EN_OTG bit (bit 6), and writes the value back.
 *
 * @param i2c_num The I2C port number to use (e.g., I2C_NUM_0).
 * @param enable True to enable OTG mode, false to disable.
 * @return esp_err_t ESP_OK on success, or an error code otherwise.
 */
esp_err_t bq25620_enable_otg_mode(i2c_port_t i2c_num, bool enable)
{
    esp_err_t ret;
    uint8_t reg_value;
    uint8_t write_buffer[2]; // Buffer for register address and data
    uint8_t register_address_to_read = BQ25620_REG_CHARGER_CONTROL_3;

    ESP_LOGI(TAG, "Attempting to %s OTG mode...", enable ? "enable" : "disable");

    // 1. Read the current value of REG0x18_Charger_Control_3
    ret = i2c_master_write_read_device(i2c_num, BQ25620_I2C_ADDR,
                                       &register_address_to_read, 1, // Write register address
                                       &reg_value, 1,                // Read 1 byte of data
                                       100 / portTICK_PERIOD_MS);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read Charger Control 3 register (0x%02X): %s", BQ25620_REG_CHARGER_CONTROL_3, esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "BQ25620: Current Charger Control 3 (0x%02X) value: 0x%02X", BQ25620_REG_CHARGER_CONTROL_3, reg_value);

    // 2. Modify the EN_OTG bit (bit 6)
    if (enable)
    {
        reg_value |= (1 << 6); // Set bit 6 to enable OTG
    }
    else
    {
        reg_value &= ~(1 << 6); // Clear bit 6 to disable OTG
    }

    // 3. Write the modified value back to REG0x18_Charger_Control_3
    write_buffer[0] = BQ25620_REG_CHARGER_CONTROL_3;
    write_buffer[1] = reg_value;
    ret = i2c_master_write_to_device(i2c_num, BQ25620_I2C_ADDR, write_buffer, 2, 100 / portTICK_PERIOD_MS);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to write to Charger Control 3 register (0x%02X) for OTG mode: %s", BQ25620_REG_CHARGER_CONTROL_3, esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "BQ25620: OTG mode %s. New Charger Control 3 (0x%02X) value: 0x%02X", enable ? "enabled" : "disabled", BQ25620_REG_CHARGER_CONTROL_3, reg_value);

    return ESP_OK;
}

/**
 * @brief Performs a software reset of BQ25620 registers to their default POR values.
 *
 * This function writes to the REG_RST bit (bit 7) of REG0x17_Charger_Control_2.
 * This bit is self-clearing.
 *
 * @param i2c_num The I2C port number to use (e.g., I2C_NUM_0).
 * @return esp_err_t ESP_OK on success, or an error code otherwise.
 */
esp_err_t bq25620_software_reset(i2c_port_t i2c_num)
{
    esp_err_t ret;
    uint8_t write_buffer[2]; // Buffer for register address and data

    ESP_LOGI(TAG, "Performing BQ25620 software reset...");

    // Set REG_RST bit (bit 7) of REG0x17_Charger_Control_2 to 1
    write_buffer[0] = BQ25620_REG_CHARGER_CONTROL_2;
    write_buffer[1] = (1 << 7); // Only set bit 7. According to datasheet, this bit is self-clearing.

    ret = i2c_master_write_to_device(i2c_num, BQ25620_I2C_ADDR, write_buffer, 2, 100 / portTICK_PERIOD_MS);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to perform BQ25620 software reset (0x%02X): %s", BQ25620_REG_CHARGER_CONTROL_2, esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "BQ25620 software reset command sent. Registers should now be at POR default values.");

    // A small delay to allow the chip to process the reset
    vTaskDelay(pdMS_TO_TICKS(50));

    return ESP_OK;
}

/**
 * @brief Reads the raw value of the Charger Status 1 register (0x1E).
 * @param i2c_num The I2C port number.
 * @param reg_value Pointer to a uint8_t to store the read register value.
 * @return esp_err_t ESP_OK on success, or an error code otherwise.
 */
esp_err_t bq25620_read_charger_status1_reg(i2c_port_t i2c_num, uint8_t *reg_value)
{
    esp_err_t ret;
    uint8_t register_address_to_read = BQ25620_REG_CHARGER_STATUS_1;

    ret = i2c_master_write_read_device(i2c_num, BQ25620_I2C_ADDR,
                                       &register_address_to_read, 1, // Write register address
                                       reg_value, 1,                 // Read 1 byte of data
                                       100 / portTICK_PERIOD_MS);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read Charger Status 1 register (0x%02X) for periodic check: %s", BQ25620_REG_CHARGER_STATUS_1, esp_err_to_name(ret));
    }
    return ret;
}

/**
 * @brief Sets or clears the TS_IGNORE bit in REG0x1A_NTC_Control_0.
 * When TS_IGNORE is set, TS function and NTC monitoring are disabled.
 * @param i2c_num The I2C port number.
 * @param ignore True to set TS_IGNORE, false to clear it.
 * @return esp_err_t ESP_OK on success, or an error code otherwise.
 */
esp_err_t bq25620_set_ts_ignore_bit(i2c_port_t i2c_num, bool ignore)
{
    esp_err_t ret;
    uint8_t reg_value;
    uint8_t write_buffer[2];
    uint8_t register_address_to_read = BQ25620_REG_NTC_CONTROL_0;

    ESP_LOGI(TAG, "Attempting to %s TS_IGNORE bit...", ignore ? "set" : "clear");

    // 1. Read the current value of REG0x1A_NTC_Control_0
    ret = i2c_master_write_read_device(i2c_num, BQ25620_I2C_ADDR,
                                       &register_address_to_read, 1, // Write register address
                                       &reg_value, 1,                // Read 1 byte of data
                                       100 / portTICK_PERIOD_MS);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read NTC Control 0 register (0x%02X): %s", BQ25620_REG_NTC_CONTROL_0, esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "BQ25620: Current NTC Control 0 (0x%02X) value: 0x%02X", BQ25620_REG_NTC_CONTROL_0, reg_value);

    // 2. Modify the TS_IGNORE bit (bit 7)
    if (ignore)
    {
        reg_value |= (1 << 7); // Set bit 7
    }
    else
    {
        reg_value &= ~(1 << 7); // Clear bit 7
    }

    // 3. Write the modified value back to REG0x1A_NTC_Control_0
    write_buffer[0] = BQ25620_REG_NTC_CONTROL_0;
    write_buffer[1] = reg_value;
    ret = i2c_master_write_to_device(i2c_num, BQ25620_I2C_ADDR, write_buffer, 2, 100 / portTICK_PERIOD_MS);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to write to NTC Control 0 register (0x%02X) for TS_IGNORE: %s", BQ25620_REG_NTC_CONTROL_0, esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "BQ25620: TS_IGNORE bit %s. New NTC Control 0 (0x%02X) value: 0x%02X", ignore ? "set" : "cleared", BQ25620_REG_NTC_CONTROL_0, reg_value);

    return ESP_OK;
}

/**
 * @brief Sets the Minimal System Voltage (VSYSMIN) on the BQ25620.
 * The programmable range is 2.5V to 3.8V, with a 10mV step.
 * Values outside this range will be clamped to the min/max.
 * @param i2c_num The I2C port number.
 * @param voltage_mv The desired minimal system voltage in mV (e.g., 2500 for 2.5V).
 * @return esp_err_t ESP_OK on success, or an error code otherwise.
 */
esp_err_t bq25620_set_minimal_system_voltage(i2c_port_t i2c_num, uint16_t voltage_mv)
{
    esp_err_t ret;
    uint8_t write_buffer[2]; // Register address and 1 byte of data for an 8-bit register
    uint8_t reg_value;

    // Clamp the input voltage to the valid range
    if (voltage_mv < VSYSMIN_MIN_MV)
    {
        voltage_mv = VSYSMIN_MIN_MV;
        ESP_LOGW(TAG, "Requested voltage below minimum. Clamping to %u mV.", VSYSMIN_MIN_MV);
    }
    else if (voltage_mv > VSYSMIN_MAX_MV)
    {
        voltage_mv = VSYSMIN_MAX_MV;
        ESP_LOGW(TAG, "Requested voltage above maximum. Clamping to %u mV.", VSYSMIN_MAX_MV);
    }

    // Calculate the register value
    reg_value = (voltage_mv - VSYSMIN_MIN_MV) / VSYSMIN_STEP_MV;

    write_buffer[0] = BQ25620_REG_MINIMAL_SYSTEM_VOLTAGE; // Address 0x0E
    write_buffer[1] = reg_value;

    ret = i2c_master_write_to_device(i2c_num, BQ25620_I2C_ADDR, write_buffer, 2, 100 / portTICK_PERIOD_MS);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set Minimal System Voltage (0x%02X): %s", BQ25620_REG_MINIMAL_SYSTEM_VOLTAGE, esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Minimal System Voltage set to %u mV (Reg value: 0x%02X).",
             (unsigned int)(reg_value * VSYSMIN_STEP_MV + VSYSMIN_MIN_MV), reg_value);

    return ESP_OK;
}

/**
 * @brief Reads VBUS voltage from BQ25620.
 * @param i2c_num I2C port number.
 * @param voltage_mv Pointer to store the read voltage in mV.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bq25620_read_vbus_voltage(i2c_port_t i2c_num, uint16_t *voltage_mv)
{
    uint16_t raw_16bit_value;
    esp_err_t ret = bq25620_read_adc_register_pair(i2c_num, BQ25620_REG_VBUS_ADC_LSB, &raw_16bit_value);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read VBUS ADC raw data.");
        return ret;
    }

    // VBUS_ADC is 12 bits
    *voltage_mv = ((raw_16bit_value >> 2) & 0xFFF) * VOLTAGE_ADC_VBUS_STEP_MV;
    return ESP_OK;
}

/**
 * @brief Reads VSYS voltage from BQ25620.
 * @param i2c_num I2C port number.
 * @param voltage_mv Pointer to store the read voltage in mV.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bq25620_read_vsys_voltage(i2c_port_t i2c_num, uint16_t *voltage_mv)
{
    uint16_t raw_16bit_value;
    esp_err_t ret = bq25620_read_adc_register_pair(i2c_num, BQ25620_REG_VSYS_ADC_LSB, &raw_16bit_value);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read VSYS ADC raw data.");
        return ret;
    }

    // VSYS_ADC is 11 bits
    *voltage_mv = ((raw_16bit_value >> 1) & 0x7FF) * VOLTAGE_ADC_VSYS_STEP_MV;
    return ESP_OK;
}

/**
 * @brief Reads VBAT voltage from BQ25620.
 * @param i2c_num I2C port number.
 * @param voltage_mv Pointer to store the read voltage in mV.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bq25620_read_vbat_voltage(i2c_port_t i2c_num, uint16_t *voltage_mv)
{
    uint16_t raw_16bit_value;
    esp_err_t ret = bq25620_read_adc_register_pair(i2c_num, BQ25620_REG_VBAT_ADC_LSB, &raw_16bit_value);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read VBAT ADC raw data.");
        return ret;
    }

    // VBAT_ADC is 11 bits
    *voltage_mv = ((raw_16bit_value >> 1) & 0x7FF) * VOLTAGE_ADC_VSYS_STEP_MV;
    return ESP_OK;
}

/**
 * @brief Reads IBUS current from BQ25620.
 * @param i2c_num I2C port number.
 * @param current_ma Pointer to store the read current in mA. This will be signed.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bq25620_read_ibus_current(i2c_port_t i2c_num, int16_t *current_ma)
{
    uint16_t raw_16bit_value;
    esp_err_t ret = bq25620_read_adc_register_pair(i2c_num, BQ25620_REG_IBUS_ADC_LSB, &raw_16bit_value);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read IBUS ADC raw data.");
        return ret;
    }

    // IBUS_ADC is a 15-bit 2's complement value located in bits 15:1 of the 16-bit register.
    // Bit 0 is reserved.
    // 1. Shift right by 1 to align the 15-bit value to the LSB (bit 0).
    uint16_t adc_value_15bit_unsigned = raw_16bit_value >> 1;

    // 2. Sign-extend the 15-bit value into a 16-bit signed integer.
    // This is done by shifting left by (16 - 15) and then right by (16 - 15) using a signed type.
    int16_t signed_ibus_adc_value = (int16_t)(adc_value_15bit_unsigned << (16 - CURRENT_ADC_BITS));
    signed_ibus_adc_value = signed_ibus_adc_value >> (16 - CURRENT_ADC_BITS);

    *current_ma = signed_ibus_adc_value * CURRENT_ADC_STEP_MA;
    return ESP_OK;
}

/**
 * @brief Reads IBAT current from BQ25620. This represents the battery charge/discharge current.
 * @param i2c_num I2C port number.
 * @param current_ma Pointer to store the read current in mA. This will be signed.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bq25620_read_ibat_current(i2c_port_t i2c_num, int16_t *current_ma)
{
    uint16_t raw_16bit_value;
    esp_err_t ret = bq25620_read_adc_register_pair(i2c_num, BQ25620_REG_IBAT_ADC_LSB, &raw_16bit_value);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read IBAT ADC raw data.");
        return ret;
    }

    // IBAT_ADC is a 14-bit 2's complement value located in bits 15:2 of the 16-bit register.
    // Bit 0 is reserved.
    // 1. Shift right by 2 to align the 15-bit value to the LSB (bit 0).
    uint16_t adc_value_14bit_unsigned = raw_16bit_value >> 2;

    // 2. Sign-extend the 15-bit value into a 16-bit signed integer.
    int16_t signed_ibat_adc_value = (int16_t)(adc_value_14bit_unsigned << (16 - CURRENT_ADC_BITS));
    signed_ibat_adc_value = signed_ibat_adc_value >> (16 - CURRENT_ADC_BITS);

    *current_ma = signed_ibat_adc_value * CURRENT_ADC_STEP_MA;
    return ESP_OK;
}

/**
 * @brief Reads VPMID voltage from BQ25620.
 * @param i2c_num I2C port number.
 * @param voltage_mv Pointer to store the read voltage in mV.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bq25620_read_vpmid_voltage(i2c_port_t i2c_num, uint16_t *voltage_mv)
{
    uint16_t raw_16bit_value;
    esp_err_t ret = bq25620_read_adc_register_pair(i2c_num, BQ25620_REG_VPMID_ADC_LSB, &raw_16bit_value);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read VPMID ADC raw data.");
        return ret;
    }

    // VPMID_ADC is 12 bits
    *voltage_mv = ((raw_16bit_value >> 2) & ((1 << 12) - 1)) * VOLTAGE_ADC_VBUS_STEP_MV;
    return ESP_OK;
}

/**
 * @brief Reads TS (Temperature Sensor) ADC value from BQ25620.
 * This is a raw ADC reading and typically needs further conversion to degrees Celsius.
 * @param i2c_num I2C port number.
 * @param adc_value Pointer to store the raw ADC value.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bq25620_read_ts_adc(i2c_port_t i2c_num, uint16_t *adc_value)
{
    esp_err_t ret = bq25620_read_adc_register_pair(i2c_num, BQ25620_REG_TS_ADC_LSB, adc_value);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read TS ADC raw data.");
    }
    return ret;
}

/**
 * @brief Reads TDIE (Die Temperature) ADC value from BQ25620.
 * This is a raw ADC reading and typically needs further conversion to degrees Celsius.
 * @param i2c_num I2C port number.
 * @param adc_value Pointer to store the raw ADC value.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bq25620_read_tdie_adc(i2c_port_t i2c_num, uint16_t *adc_value)
{
    esp_err_t ret = bq25620_read_adc_register_pair(i2c_num, BQ25620_REG_TDIE_ADC_LSB, adc_value);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read TDIE ADC raw data.");
    }
    return ret;
}

/**
 * @brief Displays the current voltage, current, and temperature status from BQ25620.
 * @param i2c_num I2C port number.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bq25620_display_levels(i2c_port_t i2c_num)
{
    uint16_t voltage_mv;
    int16_t current_ma; // Changed to signed
    uint16_t raw_adc_val;
    uint8_t fault_status_0_reg;
    esp_err_t ret;

    ESP_LOGI(TAG, "--- BQ25620 Current Levels ---");

    if (bq25620_read_vbus_voltage(i2c_num, &voltage_mv) == ESP_OK)
    {
        ESP_LOGI(TAG, "  VBUS Voltage: %.3f V", (float)voltage_mv / 1000.0f);
    }
    else
    {
        ESP_LOGE(TAG, "  Failed to read VBUS Voltage.");
    }

    if (bq25620_read_vsys_voltage(i2c_num, &voltage_mv) == ESP_OK)
    {
        ESP_LOGI(TAG, "  VSYS Voltage: %.3f V", (float)voltage_mv / 1000.0f);
    }
    else
    {
        ESP_LOGE(TAG, "  Failed to read VSYS Voltage.");
    }

    if (bq25620_read_vbat_voltage(i2c_num, &voltage_mv) == ESP_OK)
    {
        ESP_LOGI(TAG, "  VBAT Voltage: %.3f V", (float)voltage_mv / 1000.0f);
    }
    else
    {
        ESP_LOGE(TAG, "  Failed to read VBAT Voltage.");
    }

    if (bq25620_read_vpmid_voltage(i2c_num, &voltage_mv) == ESP_OK)
    {
        ESP_LOGI(TAG, "  VPMID Voltage: %.3f V", (float)voltage_mv / 1000.0f);
    }
    else
    {
        ESP_LOGE(TAG, "  Failed to read VPMID Voltage.");
    }

    if (bq25620_read_ibus_current(i2c_num, &current_ma) == ESP_OK)
    {
        ESP_LOGI(TAG, "  IBUS Current: %d mA", current_ma);
    }
    else
    {
        ESP_LOGE(TAG, "  Failed to read IBUS Current.");
    }

    if (bq25620_read_ibat_current(i2c_num, &current_ma) == ESP_OK)
    {
        ESP_LOGI(TAG, "  IBAT Current: %d mA (Positive for charge, Negative for discharge)", current_ma);
    }
    else
    {
        ESP_LOGE(TAG, "  Failed to read IBAT Current.");
    }

    if (bq25620_read_ts_adc(i2c_num, &raw_adc_val) == ESP_OK)
    {
        ESP_LOGI(TAG, "  TS ADC (Temp Sensor): %u (Raw ADC value, conversion to Celsius/Fahrenheit needed)", raw_adc_val);
    }
    else
    {
        ESP_LOGE(TAG, "  Failed to read TS ADC.");
    }

    if (bq25620_read_tdie_adc(i2c_num, &raw_adc_val) == ESP_OK)
    {
        ESP_LOGI(TAG, "  TDIE ADC (Die Temp): %u (Raw ADC value, conversion to Celsius/Fahrenheit needed)", raw_adc_val);
    }
    else
    {
        ESP_LOGE(TAG, "  Failed to read TDIE ADC.");
    }

    // Read TS_STAT from fault status register (remains the same)
    uint8_t register_address_to_read = BQ25620_REG_FAULT_STATUS_0;
    ret = i2c_master_write_read_device(i2c_num, BQ25620_I2C_ADDR,
                                       &register_address_to_read, 1,
                                       &fault_status_0_reg, 1,
                                       100 / portTICK_PERIOD_MS);
    if (ret == ESP_OK)
    {
        uint8_t ts_stat = fault_status_0_reg & 0x07; // Mask bits 2, 1, and 0
        switch (ts_stat)
        {
        case 0x0:
            ESP_LOGI(TAG, "  TS_STAT: TS_NORMAL");
            break;
        case 0x1:
            ESP_LOGI(TAG, "  TS_STAT: TS_COLD or TS_OTG_COLD or TS resistor string power rail is not available.");
            break;
        case 0x2:
            ESP_LOGI(TAG, "  TS_STAT: TS_HOT or TS_OTG_HOT");
            break;
        case 0x3:
            ESP_LOGI(TAG, "  TS_STAT: TS_COOL");
            break;
        case 0x4:
            ESP_LOGI(TAG, "  TS_STAT: TS_WARM");
            break;
        case 0x5:
            ESP_LOGI(TAG, "  TS_STAT: TS_PRECOOL");
            break;
        case 0x6:
            ESP_LOGI(TAG, "  TS_STAT: TS_PREWARM");
            break;
        case 0x7:
            ESP_LOGI(TAG, "  TS_STAT: TS pin bias reference fault");
            break;
        default:
            ESP_LOGI(TAG, "  TS_STAT: Unknown (%X)", ts_stat);
            break;
        }
    }
    else
    {
        ESP_LOGE(TAG, "  Failed to read TS_STAT.");
    }

    ESP_LOGI(TAG, "--------------------------");
    return ESP_OK;
}

/**
 * @brief Enables the internal ADC and sets its sample resolution.
 * @param i2c_num I2C port number.
 * @param enable_adc True to enable ADC, false to disable.
 * @param use_12bit_resolution True to set to 12-bit resolution, false for 8-bit.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bq25620_set_adc_settings(i2c_port_t i2c_num, bool enable_adc, bool use_12bit_resolution)
{
    esp_err_t ret;
    uint8_t adc_control_reg_value;       // For ADC Control (0x26)
    uint8_t charger_control_2_reg_value; // For Charger Control 2 (0x17)
    uint8_t write_buffer[2];

    ESP_LOGI(TAG, "Configuring BQ25620 ADC: Enable=%d, 12-bit Resolution=%d", enable_adc, use_12bit_resolution);

    // --- Configure ADC_EN bit (Bit 7 of REG0x26_ADC_Control) ---
    uint8_t read_reg_addr_26 = BQ25620_REG_ADC_CONTROL;
    ret = i2c_master_write_read_device(i2c_num, BQ25620_I2C_ADDR,
                                       &read_reg_addr_26, 1,
                                       &adc_control_reg_value, 1,
                                       100 / portTICK_PERIOD_MS);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read REG0x26 (ADC Control) for ADC_EN: %s", esp_err_to_name(ret));
        return ret;
    }

    if (enable_adc)
    {
        adc_control_reg_value |= (1 << 7); // Set ADC_EN bit
    }
    else
    {
        adc_control_reg_value &= ~(1 << 7); // Clear ADC_EN bit
    }

    write_buffer[0] = BQ25620_REG_ADC_CONTROL;
    write_buffer[1] = adc_control_reg_value;
    ret = i2c_master_write_to_device(i2c_num, BQ25620_I2C_ADDR, write_buffer, 2, 100 / portTICK_PERIOD_MS);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to write REG0x26 (ADC Control) for ADC_EN: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "BQ25620: ADC_EN %s.", enable_adc ? "enabled" : "disabled");

    // --- Configure ADC_SAMPLE bit (Bit 5 of REG0x17_Charger_Control_2) ---
    uint8_t read_reg_addr_17 = BQ25620_REG_CHARGER_CONTROL_2;
    ret = i2c_master_write_read_device(i2c_num, BQ25620_I2C_ADDR,
                                       &read_reg_addr_17, 1,
                                       &charger_control_2_reg_value, 1,
                                       100 / portTICK_PERIOD_MS);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read REG0x17 (Charger Control 2) for ADC_SAMPLE: %s", esp_err_to_name(ret));
        return ret;
    }

    if (use_12bit_resolution)
    {
        charger_control_2_reg_value &= ~(1 << 4 | 1 << 5); // Clear bit 4 and 5 in ADC_SAMPLE bit for 12-bit
    }
    else
    {
        charger_control_2_reg_value |= (1 << 4 | 1 << 5); // Set bit 4 and 5 in ADC_SAMPLE bits for 9-bit
    }

    write_buffer[0] = BQ25620_REG_CHARGER_CONTROL_2;
    write_buffer[1] = charger_control_2_reg_value;
    ret = i2c_master_write_to_device(i2c_num, BQ25620_I2C_ADDR, write_buffer, 2, 100 / portTICK_PERIOD_MS);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to write REG0x17 (Charger Control 2) for ADC_SAMPLE: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "BQ25620: ADC resolution set to %s.", use_12bit_resolution ? "12-bit" : "8-bit");

    return ESP_OK;
}


/**
 * @brief Enters BQ25620 into ship mode by disabling the BATFET.
 *
 * This function reads the current value of the System Control 0 register (0x1B),
 * sets the BATFET_DIS bit (bit 5) to 1, and writes the value back.
 * This should put the device into ship mode, provided no adapter is connected.
 *
 * @param i2c_num The I2C port number.
 * @return esp_err_t ESP_OK on success, or an error code otherwise.
 */
esp_err_t bq25620_enter_ship_mode(i2c_port_t i2c_num) {
    esp_err_t ret;
    uint8_t reg_value;
    uint8_t write_buffer[2]; // Buffer for register address and data
    uint8_t register_address_to_read = BQ25620_REG_CHARGER_CONTROL_3;

    ESP_LOGI(TAG, "Attempting to enter BQ25620 into Ship Mode...");

    // 1. Read the current value of REG0x1B_System_Control_0
    ret = i2c_master_write_read_device(i2c_num, BQ25620_I2C_ADDR,
                                       &register_address_to_read, 1, // Write register address
                                       &reg_value, 1, // Read 1 byte of data
                                       100 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read Charger Control 3 register (0x%02X) for ship mode: %s", BQ25620_REG_CHARGER_CONTROL_3, esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "BQ25620: Current Charger Control 3 (0x%02X) value: 0x%02X", BQ25620_REG_CHARGER_CONTROL_3, reg_value);

    // 2. Set the BATFET_DIS 
    reg_value &= ~(0x3); // Clear bit 0:1
    reg_value |= 0x02; // Set bit 0:1

    // 3. Write the modified value back to REG0x1B_System_Control_3
    write_buffer[0] = BQ25620_REG_CHARGER_CONTROL_3;
    write_buffer[1] = reg_value;
    ret = i2c_master_write_to_device(i2c_num, BQ25620_I2C_ADDR, write_buffer, 2, 100 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write to Charger Control 3 register (0x%02X) for ship mode: %s", BQ25620_REG_CHARGER_CONTROL_3, esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "BQ25620: Ship Mode command sent. New Charger Control 3 (0x%02X) value: 0x%02X. Device should now be in ship mode.", BQ25620_REG_CHARGER_CONTROL_3, reg_value);

    return ESP_OK;
}
