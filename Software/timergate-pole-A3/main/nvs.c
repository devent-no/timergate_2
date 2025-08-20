#include "nvs.h"
#include "nvs_flash.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h> // For vTaskDelay, pdMS_TO_TICKS
#include <stdio.h> // For sprintf
#include <inttypes.h> // For PRIu32
#include <stdbool.h> // For bool type

// Define NUM_SENSORS here or ensure it's available from another header
#define NUM_SENSORS 7 // Example value, adjust according to your actual number of sensors

static const char *TAG = "NVS_DRV"; // Tag for ESP_LOG messages in this driver file

void nvs_setup()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_LOGI(TAG, "NVS flash setup complete.");
}

/**
 * @brief Reads NVS stored data into provided variables.
 *
 * @param restart_counter Pointer to store the read restart counter.
 * @param offsets Array to store the read offset values.
 * @param break_limit Array to store the read break limit values.
 * @param enabled Array to store the read enabled states (bool values).
 */
void nvs_read(uint32_t *restart_counter, uint16_t offsets[], uint16_t break_limit[], bool enabled[])
{
    esp_err_t err;
    // Open
    ESP_LOGI(TAG, "Opening Non-Volatile Storage (NVS) handle... ");
    nvs_handle_t my_handle;
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
    }
    else
    {
        ESP_LOGI(TAG, "Done opening NVS handle.");

        // Read restart counter
        ESP_LOGI(TAG, "Reading restart counter from NVS ... ");
        err = nvs_get_i32(my_handle, "restart_counter", (int32_t *)restart_counter); // NVS get_i32 expects int32_t*
        switch (err)
        {
        case ESP_OK:
            ESP_LOGI(TAG, "Done reading restart counter.");
            ESP_LOGI(TAG, "Restart counter = %" PRIu32, *restart_counter);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGW(TAG, "The restart counter value is not initialized yet!");
            // Optionally initialize *restart_counter to 0 if not found
            *restart_counter = 0;
            break;
        default:
            ESP_LOGE(TAG, "Error (%s) reading restart counter!", esp_err_to_name(err));
        }

        char var_s[30];
        ESP_LOGI(TAG, "Reading offsets, break_limits, and enabled states from NVS ... ");
        esp_err_t overall_err = ESP_OK;
        for (int i = 0; i < NUM_SENSORS; i++)
        {
            sprintf(var_s, "offset_%d", i);
            overall_err |= nvs_get_u16(my_handle, var_s, &offsets[i]);

            sprintf(var_s, "break_limit_%d", i);
            overall_err |= nvs_get_u16(my_handle, var_s, &break_limit[i]);
            
            // For boolean values, read as uint16_t and then cast
            uint16_t temp_enabled_val;
            sprintf(var_s, "enabled_%d", i);
            overall_err |= nvs_get_u16(my_handle, var_s, &temp_enabled_val);
            enabled[i] = (bool)temp_enabled_val;
        }
        if (overall_err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read all sensor settings from NVS! (Error: %s)", esp_err_to_name(overall_err));
        } else {
            ESP_LOGI(TAG, "Done reading sensor settings from NVS.");
        }


        // Close
        nvs_close(my_handle);
    }
}

/**
 * @brief Updates the restart counter in NVS.
 *
 * @param restart_counter Pointer to the restart counter variable to update and store.
 */
void nvs_update_restart(uint32_t *restart_counter)
{
    esp_err_t err;
    ESP_LOGI(TAG, "Opening Non-Volatile Storage (NVS) handle... ");
    nvs_handle_t my_handle;
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
    }
    else
    {
        ESP_LOGI(TAG, "Done opening NVS handle.");
        // Write
        ESP_LOGI(TAG, "Updating restart counter in NVS ... ");
        (*restart_counter)++; // Increment the value pointed to
        err = nvs_set_i32(my_handle, "restart_counter", *restart_counter);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set restart counter in NVS! (Error: %s)", esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "Done setting restart counter.");
        }

        ESP_LOGI(TAG, "Committing updates in NVS ... ");
        err = nvs_commit(my_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to commit NVS updates! (Error: %s)", esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "Done committing NVS updates.");
        }
        // Close
        nvs_close(my_handle);
    }
}

/**
 * @brief Updates offset, break limit, and enabled states in NVS.
 *
 * @param offsets Array containing the offset values to store.
 * @param break_limit Array containing the break limit values to store.
 * @param enabled Array containing the enabled states to store (const bool array).
 */
void nvs_update_offsets(const uint16_t offsets[], const uint16_t break_limit[], const bool enabled[])
{
    esp_err_t err;
    ESP_LOGI(TAG, "Opening Non-Volatile Storage (NVS) handle... ");
    nvs_handle_t my_handle;
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
    }
    else
    {
        ESP_LOGI(TAG, "Done opening NVS handle.");
        char var_s[30];
        ESP_LOGI(TAG, "Updating offsets, break_limits, and enabled states in NVS ... ");
        esp_err_t overall_err = ESP_OK;
        for (int i = 0; i < NUM_SENSORS; i++)
        {
            sprintf(var_s, "offset_%d", i);
            overall_err |= nvs_set_u16(my_handle, var_s, offsets[i]);

            sprintf(var_s, "break_limit_%d", i);
            overall_err |= nvs_set_u16(my_handle, var_s, break_limit[i]);
            
            // For boolean values, convert to uint16_t before storing
            uint16_t temp_enabled_val = enabled[i];
            sprintf(var_s, "enabled_%d", i);
            overall_err |= nvs_set_u16(my_handle, var_s, temp_enabled_val);
        }
        if (overall_err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set all sensor settings in NVS! (Error: %s)", esp_err_to_name(overall_err));
        } else {
            ESP_LOGI(TAG, "Done setting sensor settings.");
        }


        ESP_LOGI(TAG, "Committing updates in NVS ... ");
        err = nvs_commit(my_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to commit NVS updates! (Error: %s)", esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "Done committing NVS updates.");
        }
        // Close
        nvs_close(my_handle);
    }
}
