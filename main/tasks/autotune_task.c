#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "global_state.h"
#include "nvs_config.h"
#include "asic.h"
#include "autotune_task.h"

// Deliberately independent from power_management_task's hardware throttle
// constants: this is a soft, conservative comfort limit that should trip
// well before the hardware safety cutoff ever gets involved.
#define POLL_RATE_MS 10000
#define AUTOTUNE_TEMP_LIMIT_C 68.0f
#define HASHRATE_SHORTFALL_LIMIT 0.05f     // >5% below expected total counts as unstable
#define DOMAIN_SHORTFALL_LIMIT 0.50f       // a single hash domain running below 50% of its expected share counts as unstable
#define ERROR_RATE_LIMIT_PCT 2.0f          // >2% ASIC error rate counts as unstable

#define STABLE_CHECKS_BEFORE_ACTION 6       // ~60s of stability before climbing freq or shaving voltage
#define BACKOFF_CHECKS_AFTER_ACTION 6        // ~60s cooldown after a climb, shave, or rescue before the next one
#define RETREAT_COOLDOWN_CHECKS 3            // ~30s cooldown between frequency retreats - lets each step actually prove itself instead of cascading down every poll
#define MAX_CONSECUTIVE_RESCUES 3

#define VENDOR_VOLTAGE_STEP_MV 25
#define OVERCLOCK_VOLTAGE_STEP_MV 5
#define OVERCLOCK_VOLTAGE_HEADROOM_MV 150 // soft ceiling above vendor max when custom settings are unlocked

#define VENDOR_FREQUENCY_STEP_MHZ 10.0f
#define OVERCLOCK_FREQUENCY_STEP_MHZ 5.0f
#define ECO_EFFICIENCY_TOLERANCE 0.98f // allow 2% noise before treating a climb as an efficiency regression

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

static uint16_t frequency_table_max(const AsicConfig * asic)
{
    uint16_t max = asic->frequency_options[0];
    for (int i = 0; asic->frequency_options[i] != 0; i++) {
        max = asic->frequency_options[i];
    }
    return max;
}

// Finer 5MHz steps once climbing beyond the vendor table (same idea as the
// voltage step split below): precise adjustments matter more when you're
// already outside the tested/validated range.
static float frequency_step(const AsicConfig * asic, float current_frequency, bool overclock_enabled)
{
    return (overclock_enabled && current_frequency >= (float) frequency_table_max(asic))
           ? OVERCLOCK_FREQUENCY_STEP_MHZ : VENDOR_FREQUENCY_STEP_MHZ;
}

// Effective ceilings: a user-set value (>0) wins, otherwise fall back to the
// vendor table max. A user value beyond the vendor table only applies when
// Custom Settings (overclock) is unlocked - same "understand the risks" gate
// used everywhere else in the settings UI.
static uint16_t effective_max_voltage(const AsicConfig * asic, bool overclock_enabled)
{
    uint16_t vendor_max = voltage_table_max(asic);
    uint16_t base_ceiling = overclock_enabled ? (vendor_max + OVERCLOCK_VOLTAGE_HEADROOM_MV) : vendor_max;
    uint16_t user_max = nvs_config_get_u16(NVS_CONFIG_AUTOTUNE_MAX_VOLTAGE);

    if (user_max == 0) {
        return base_ceiling;
    }
    return overclock_enabled ? user_max : (user_max < vendor_max ? user_max : vendor_max);
}

static float effective_max_frequency(const AsicConfig * asic, bool overclock_enabled)
{
    float vendor_max = (float) frequency_table_max(asic);
    float user_max = nvs_config_get_float(NVS_CONFIG_AUTOTUNE_MAX_FREQUENCY);

    if (user_max <= 0.0f) {
        return vendor_max;
    }
    return overclock_enabled ? user_max : (user_max < vendor_max ? user_max : vendor_max);
}

// Checks every individual hash domain against its expected share of the total
// hashrate. Catches a single degraded/failing domain that an aggregate-only
// check could mask (e.g. 3 healthy domains hiding one dead one on a 4-domain ASIC).
static bool any_domain_underperforming(GlobalState * GLOBAL_STATE)
{
    PowerManagementModule * pm = &GLOBAL_STATE->POWER_MANAGEMENT_MODULE;
    int asic_count = GLOBAL_STATE->DEVICE_CONFIG.family.asic_count;
    int hash_domains = GLOBAL_STATE->DEVICE_CONFIG.family.asic.hash_domains;

    if (hash_domains <= 1 || pm->expected_hashrate <= 0.0f) {
        return false; // nothing to compare per-domain on single-domain ASICs
    }

    float expected_per_domain = pm->expected_hashrate / (float)(asic_count * hash_domains);

    for (int asic_nr = 0; asic_nr < asic_count; asic_nr++) {
        for (int domain_nr = 0; domain_nr < hash_domains; domain_nr++) {
            asic_domain_measurement_t measurement = {0};
            if (ASIC_get_domain_measurement(GLOBAL_STATE, asic_nr, domain_nr, &measurement) != ESP_OK) {
                continue;
            }
            if (measurement.hashrate < expected_per_domain * DOMAIN_SHORTFALL_LIMIT) {
                ESP_LOGW(TAG, "ASIC %d domain %d underperforming: %.2f Gh/s (expected ~%.2f)",
                         asic_nr, domain_nr, measurement.hashrate, expected_per_domain);
                return true;
            }
        }
    }
    return false;
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

    if (any_domain_underperforming(GLOBAL_STATE)) {
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

    uint16_t vendor_min_mv = voltage_table_min(asic);
    float floor_freq_mhz = asic->default_frequency_mhz;

    at->state = AUTOTUNE_STATE_IDLE;
    at->stable_checks = 0;
    at->backoff_remaining = 0;
    at->rescue_attempts = 0;
    at->last_step_mv = 0;
    at->last_step_mhz = 0;
    at->last_action_time_s = 0;
    at->last_efficiency_ghs_w = 0.0f;
    at->eco_peak_found = false;
    bool rescue_limit_warned = false;

    ESP_LOGI(TAG, "Starting (voltage floor %umV, frequency floor %g MHz)", vendor_min_mv, floor_freq_mhz);

    TickType_t taskWakeTime = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&taskWakeTime, POLL_RATE_MS / portTICK_PERIOD_MS);

        if (!nvs_config_get_bool(NVS_CONFIG_AUTOTUNE_ENABLED)) {
            at->state = AUTOTUNE_STATE_IDLE;
            at->stable_checks = 0;
            at->backoff_remaining = 0;
            at->rescue_attempts = 0;
            at->eco_peak_found = false;
            at->last_efficiency_ghs_w = 0.0f;
            rescue_limit_warned = false;
            continue;
        }

        if (!GLOBAL_STATE->ASIC_initalized || GLOBAL_STATE->SELF_TEST_MODULE.is_active) {
            continue;
        }

        bool overclock_enabled = nvs_config_get_bool(NVS_CONFIG_OVERCLOCK_ENABLED);
        bool performance_mode = nvs_config_get_bool(NVS_CONFIG_AUTOTUNE_PROFILE);
        uint16_t core_voltage = nvs_config_get_u16(NVS_CONFIG_ASIC_VOLTAGE);
        float core_frequency = nvs_config_get_float(NVS_CONFIG_ASIC_FREQUENCY);

        uint16_t max_voltage = effective_max_voltage(asic, overclock_enabled);
        float max_frequency = effective_max_frequency(asic, overclock_enabled);

        float temp = 0.0f;
        bool unstable = read_is_unstable(GLOBAL_STATE, &temp);

        if (unstable) {
            at->stable_checks = 0;

            if (at->rescue_attempts < MAX_CONSECUTIVE_RESCUES && core_voltage < max_voltage) {
                uint16_t step = (overclock_enabled && core_voltage >= voltage_table_max(asic)) ? OVERCLOCK_VOLTAGE_STEP_MV : VENDOR_VOLTAGE_STEP_MV;
                uint16_t new_voltage = core_voltage + step;
                if (new_voltage > max_voltage) {
                    new_voltage = max_voltage;
                }

                ESP_LOGI(TAG, "Unstable (%.1fC) - rescuing voltage %umV -> %umV", temp, core_voltage, new_voltage);
                nvs_config_set_u16(NVS_CONFIG_ASIC_VOLTAGE, new_voltage);
                at->rescue_attempts++;
                at->backoff_remaining = BACKOFF_CHECKS_AFTER_ACTION;
                at->state = AUTOTUNE_STATE_RESCUING;
                at->last_step_mv = (int16_t)(new_voltage - core_voltage);
                mark_action_time(at);
            } else if (core_frequency > floor_freq_mhz) {
                // Voltage is maxed out or we've exhausted rescue attempts at this
                // frequency. Retreat is gated by its own short cooldown so a single
                // rough patch can't cascade multiple steps down before the previous
                // step even gets a chance to prove whether it's actually stable.
                if (at->backoff_remaining > 0) {
                    at->backoff_remaining--;
                    at->state = AUTOTUNE_STATE_HELD;
                } else {
                    float new_frequency = core_frequency - frequency_step(asic, core_frequency, overclock_enabled);
                    if (new_frequency < floor_freq_mhz) {
                        new_frequency = floor_freq_mhz;
                    }

                    ESP_LOGI(TAG, "Unstable at voltage ceiling (%umV) - retreating frequency %g -> %g MHz",
                             core_voltage, core_frequency, new_frequency);
                    nvs_config_set_float(NVS_CONFIG_ASIC_FREQUENCY, new_frequency);
                    at->rescue_attempts = 0;
                    rescue_limit_warned = false;
                    at->backoff_remaining = RETREAT_COOLDOWN_CHECKS;
                    at->state = AUTOTUNE_STATE_RETREATING;
                    at->last_step_mhz = (int16_t)(new_frequency - core_frequency);
                    mark_action_time(at);
                }
            } else {
                at->state = AUTOTUNE_STATE_HELD;
                if (!rescue_limit_warned) {
                    ESP_LOGW(TAG, "Unstable at voltage ceiling (%umV) and frequency floor (%g MHz) - holding, needs manual attention",
                             core_voltage, core_frequency);
                    rescue_limit_warned = true;
                }
            }
        } else {
            at->rescue_attempts = 0;
            rescue_limit_warned = false;
            at->stable_checks++;
            at->state = AUTOTUNE_STATE_STABLE;

            if (at->backoff_remaining > 0) {
                at->backoff_remaining--;
            } else if (at->stable_checks >= STABLE_CHECKS_BEFORE_ACTION) {
                PowerManagementModule * pm = &GLOBAL_STATE->POWER_MANAGEMENT_MODULE;
                SystemModule * sys = &GLOBAL_STATE->SYSTEM_MODULE;
                float efficiency = (pm->power > 0.0f) ? sys->current_hashrate / pm->power : 0.0f;

                bool eco_regressed = !performance_mode && at->eco_peak_found == false
                                      && at->last_efficiency_ghs_w > 0.0f
                                      && efficiency < at->last_efficiency_ghs_w * ECO_EFFICIENCY_TOLERANCE
                                      && core_frequency > floor_freq_mhz;

                float max_temp_c = nvs_config_get_float(NVS_CONFIG_AUTOTUNE_MAX_TEMP);
                bool temp_ceiling_reached = performance_mode && max_temp_c > 0.0f && temp >= max_temp_c;

                bool climbing_allowed = performance_mode ? !temp_ceiling_reached : !at->eco_peak_found;

                if (eco_regressed) {
                    // Eco mode: the last climb step made hash/watt worse - that step
                    // wasn't worth it. Undo it and lock in the previous point as the
                    // efficiency peak; from here on only shave voltage.
                    float new_frequency = core_frequency - frequency_step(asic, core_frequency, overclock_enabled);
                    if (new_frequency < floor_freq_mhz) {
                        new_frequency = floor_freq_mhz;
                    }
                    ESP_LOGI(TAG, "Eco: climb regressed efficiency (%.3f -> %.3f Gh/s/W) - reverting to %g MHz and locking peak",
                             at->last_efficiency_ghs_w, efficiency, new_frequency);
                    nvs_config_set_float(NVS_CONFIG_ASIC_FREQUENCY, new_frequency);
                    at->eco_peak_found = true;
                    at->state = AUTOTUNE_STATE_RETREATING;
                    at->last_step_mhz = (int16_t)(new_frequency - core_frequency);
                    mark_action_time(at);
                } else if (climbing_allowed && core_frequency < max_frequency) {
                    float new_frequency = core_frequency + frequency_step(asic, core_frequency, overclock_enabled);
                    if (new_frequency > max_frequency) {
                        new_frequency = max_frequency;
                    }

                    ESP_LOGI(TAG, "Stable - climbing frequency %g -> %g MHz", core_frequency, new_frequency);
                    nvs_config_set_float(NVS_CONFIG_ASIC_FREQUENCY, new_frequency);
                    at->last_efficiency_ghs_w = efficiency; // baseline to judge this climb against next time
                    at->state = AUTOTUNE_STATE_CLIMBING;
                    at->last_step_mhz = (int16_t)(new_frequency - core_frequency);
                    mark_action_time(at);
                } else {
                    // Climbing has stopped: at the frequency ceiling, at the Eco
                    // efficiency peak, or (Performance mode) at the user's temp
                    // ceiling - safe to shave voltage down for efficiency.
                    uint16_t step = (overclock_enabled && core_voltage > voltage_table_max(asic)) ? OVERCLOCK_VOLTAGE_STEP_MV : VENDOR_VOLTAGE_STEP_MV;
                    uint16_t new_voltage = (core_voltage > vendor_min_mv + step) ? core_voltage - step : vendor_min_mv;

                    if (new_voltage < core_voltage) {
                        if (temp_ceiling_reached) {
                            ESP_LOGI(TAG, "Stable but at temp ceiling (%.1fC >= %.1fC) - shaving voltage %umV -> %umV instead of climbing",
                                     temp, max_temp_c, core_voltage, new_voltage);
                        } else {
                            ESP_LOGI(TAG, "Stable, no more climbing - shaving voltage %umV -> %umV", core_voltage, new_voltage);
                        }
                        nvs_config_set_u16(NVS_CONFIG_ASIC_VOLTAGE, new_voltage);
                        at->state = AUTOTUNE_STATE_SHAVING;
                        at->last_step_mv = (int16_t)(new_voltage - core_voltage);
                        mark_action_time(at);
                    }
                }
                at->stable_checks = 0;
            }
        }
    }
}
