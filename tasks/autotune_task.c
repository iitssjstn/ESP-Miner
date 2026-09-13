#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "global_state.h"
#include "nvs_config.h"
#include "autotune_task.h"

// Deliberately independent from power_management_task's hardware throttle
// constants: this is a soft, conservative comfort limit that should trip
// well before the hardware safety cutoff ever gets involved.
#define POLL_RATE_MS 10000
#define AUTOTUNE_TEMP_LIMIT_C 68.0f
#define HASHRATE_SHORTFALL_LIMIT 0.05f  // >5% below expected counts as unstable
#define ERROR_RATE_LIMIT_PCT 2.0f       // >2% ASIC error rate counts as unstable

#define STABLE_CHECKS_BEFORE_SHAVE 6    // ~60s of stability before trying to shave voltage
#define BACKOFF_CHECKS_AFTER_RESCUE 6   // ~60s cooldown after a rescue before shaving resumes
#define MAX_CONSECUTIVE_RESCUES 3

#define VENDOR_STEP_MV 25
#define OVERCLOCK_STEP_MV 10
#define OVERCLOCK_HEADROOM_MV 150       // soft ceiling above vendor max when custom settings are unlocked

static const char * TAG = "autotune";

static uint16_t voltage_table_min(const AsicConfig * asic)
{
    return asic->voltage_options[0];
}

static uint16_t voltage_table_max(const AsicConfig * asic)
{
    uint16_t max = asic->voltage_options[0];
    for (int i = 0; asic->voltage_options[i] != 0; i++) {
        max = asic->voltage_options[i];
    }
    return max;
}

static bool read_is_unstable(GlobalState * GLOBAL_STATE, float * out_temp)
{
    PowerManagementModule * pm = &GLOBAL_STATE->POWER_MANAGEMENT_MODULE;
    SystemModule * sys = &GLOBAL_STATE->SYSTEM_MODULE;

    float temp = pm->chip_temp_avg > pm->chip_temp2_avg ? pm->chip_temp_avg : pm->chip_temp2_avg;
    *out_temp = temp;

    if (temp > AUTOTUNE_TEMP_LIMIT_C) {
        return true;
    }

    if (pm->expected_hashrate > 0.0f) {
        float shortfall = (pm->expected_hashrate - sys->current_hashrate) / pm->expected_hashrate;
        if (shortfall > HASHRATE_SHORTFALL_LIMIT) {
            return true;
        }
    }

    if (sys->error_percentage > ERROR_RATE_LIMIT_PCT) {
        return true;
    }

    return false;
}

static void mark_action_time(AutotuneModule * at)
{
    at->last_action_time_s = (uint32_t)(esp_timer_get_time() / 1000000);
}

void autotune_task(void *pvParameters)
{
    GlobalState * GLOBAL_STATE = (GlobalState *) pvParameters;
    AutotuneModule * at = &GLOBAL_STATE->AUTOTUNE_MODULE;
    const AsicConfig * asic = &GLOBAL_STATE->DEVICE_CONFIG.family.asic;

    uint16_t vendor_min = voltage_table_min(asic);
    uint16_t vendor_max = voltage_table_max(asic);

    at->state = AUTOTUNE_STATE_IDLE;
    at->stable_checks = 0;
    at->backoff_remaining = 0;
    at->rescue_attempts = 0;
    at->last_step_mv = 0;
    at->last_action_time_s = 0;
    bool rescue_limit_warned = false;

    ESP_LOGI(TAG, "Starting (vendor range %u-%umV)", vendor_min, vendor_max);

    TickType_t taskWakeTime = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&taskWakeTime, POLL_RATE_MS / portTICK_PERIOD_MS);

        if (!nvs_config_get_bool(NVS_CONFIG_AUTOTUNE_ENABLED)) {
            at->state = AUTOTUNE_STATE_IDLE;
            at->stable_checks = 0;
            at->backoff_remaining = 0;
            at->rescue_attempts = 0;
            rescue_limit_warned = false;
            continue;
        }

        if (!GLOBAL_STATE->ASIC_initalized || GLOBAL_STATE->SELF_TEST_MODULE.is_active) {
            continue;
        }

        bool overclock_enabled = nvs_config_get_bool(NVS_CONFIG_OVERCLOCK_ENABLED);
        uint16_t core_voltage = nvs_config_get_u16(NVS_CONFIG_ASIC_VOLTAGE);

        float temp = 0.0f;
        bool unstable = read_is_unstable(GLOBAL_STATE, &temp);

        if (unstable) {
            at->stable_checks = 0;

            if (at->rescue_attempts >= MAX_CONSECUTIVE_RESCUES) {
                at->state = AUTOTUNE_STATE_HELD;
                if (!rescue_limit_warned) {
                    ESP_LOGW(TAG, "Hit max consecutive rescue attempts (%d) at %umV, %.1fC - holding, needs manual attention",
                             MAX_CONSECUTIVE_RESCUES, core_voltage, temp);
                    rescue_limit_warned = true;
                }
                continue;
            }

            uint16_t step = (overclock_enabled && core_voltage >= vendor_max) ? OVERCLOCK_STEP_MV : VENDOR_STEP_MV;
            uint16_t ceiling = overclock_enabled ? (vendor_max + OVERCLOCK_HEADROOM_MV) : vendor_max;
            uint16_t new_voltage = core_voltage + step;
            if (new_voltage > ceiling) {
                new_voltage = ceiling;
            }

            if (new_voltage > core_voltage) {
                ESP_LOGI(TAG, "Unstable (%.1fC, hashrate/error out of tolerance) - rescuing voltage %umV -> %umV",
                         temp, core_voltage, new_voltage);
                nvs_config_set_u16(NVS_CONFIG_ASIC_VOLTAGE, new_voltage);
                at->rescue_attempts++;
                at->backoff_remaining = BACKOFF_CHECKS_AFTER_RESCUE;
                at->state = AUTOTUNE_STATE_RESCUING;
                at->last_step_mv = (int16_t)(new_voltage - core_voltage);
                mark_action_time(at);
            } else {
                at->state = AUTOTUNE_STATE_HELD;
                ESP_LOGW(TAG, "Unstable at %umV but already at ceiling (%umV) - holding", core_voltage, ceiling);
            }
        } else {
            at->rescue_attempts = 0;
            rescue_limit_warned = false;
            at->stable_checks++;
            at->state = AUTOTUNE_STATE_STABLE;

            if (at->backoff_remaining > 0) {
                at->backoff_remaining--;
            } else if (at->stable_checks >= STABLE_CHECKS_BEFORE_SHAVE) {
                uint16_t step = (overclock_enabled && core_voltage > vendor_max) ? OVERCLOCK_STEP_MV : VENDOR_STEP_MV;
                uint16_t new_voltage = (core_voltage > vendor_min + step) ? core_voltage - step : vendor_min;

                if (new_voltage < core_voltage) {
                    ESP_LOGI(TAG, "Stable for %ds - shaving voltage %umV -> %umV",
                             STABLE_CHECKS_BEFORE_SHAVE * (POLL_RATE_MS / 1000), core_voltage, new_voltage);
                    nvs_config_set_u16(NVS_CONFIG_ASIC_VOLTAGE, new_voltage);
                    at->state = AUTOTUNE_STATE_SHAVING;
                    at->last_step_mv = (int16_t)(new_voltage - core_voltage);
                    mark_action_time(at);
                }
                at->stable_checks = 0;
            }
        }
    }
}
