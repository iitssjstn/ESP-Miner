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
#define POLL_RATE_MS 2500
#define AUTOTUNE_TEMP_LIMIT_C 68.0f
#define DOMAIN_SHORTFALL_LIMIT 0.50f       // a single hash domain running below 50% of its expected share counts as unstable
#define ERROR_RATE_LIMIT_PCT 2.0f          // >2% ASIC error rate counts as unstable

// Check-counts below are scaled to POLL_RATE_MS so the real-world durations
// stay the same as before (poll rate went 10s -> 2.5s for faster settings
// pickup, so counts went up 4x to compensate) - only the responsiveness to
// settings changes improved, none of the actual safety timing changed.
#define STABLE_CHECKS_BEFORE_ACTION 24       // ~60s of stability in Eco mode before an adjustment
#define PERFORMANCE_STABLE_CHECKS_BEFORE_ACTION 12 // ~30s in Performance mode; temperature and instability checks remain unchanged
#define PERFORMANCE_HOLD_CHECKS 1440              // ~1 hour at the 2.5s poll rate
#define RESCUE_COOLDOWN_CHECKS 72             // ~180s hold after a rescue before the next climb/shave - a rescue means the lower voltage genuinely failed, not noise, so prove real stability before retesting that same edge
#define RETREAT_COOLDOWN_CHECKS 12            // ~30s cooldown between frequency retreats - lets each step actually prove itself instead of cascading down every poll
#define OVERTEMP_COOLDOWN_CHECKS 48           // ~120s cooldown after an overtemp-triggered reduction - thermal mass takes longer to actually settle than a voltage/power reading does
#define UNSTABLE_CONFIRM_CHECKS 8             // require 8 consecutive unstable readings (~20s) before reacting - filters a single noisy blip
#define MAX_CONSECUTIVE_RESCUES 3

#define VENDOR_VOLTAGE_STEP_MV 25
#define OVERCLOCK_VOLTAGE_STEP_MV 10
#define OVERCLOCK_VOLTAGE_HEADROOM_MV 150 // soft ceiling above vendor max when custom settings are unlocked

#define VENDOR_FREQUENCY_STEP_MHZ 10.0f
#define OVERCLOCK_FREQUENCY_STEP_MHZ 5.0f
#define ECO_EFFICIENCY_TOLERANCE 0.98f // allow 2% noise before treating a climb as an efficiency regression
#define PROACTIVE_POWER_MARGIN 0.95f // stop climbing at 95% of the global power limit, before the cap is actually hit
#define POWER_AVERAGE_ALPHA ((float) POLL_RATE_MS / 60000.0f)

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
// already outside the tested/validated range. In Performance mode, a user-set
// custom step (>0) overrides both of these entirely - full manual control.
static float frequency_step(const AsicConfig * asic, float current_frequency, bool overclock_enabled, bool performance_mode)
{
    if (performance_mode) {
        float custom_step = nvs_config_get_float(NVS_CONFIG_AUTOTUNE_FREQUENCY_STEP);
        if (custom_step > 0.0f) {
            return custom_step;
        }
    }
    return (overclock_enabled && current_frequency >= (float) frequency_table_max(asic))
           ? OVERCLOCK_FREQUENCY_STEP_MHZ : VENDOR_FREQUENCY_STEP_MHZ;
}

// Same idea, for voltage: a user-set custom step (>0) in Performance mode
// overrides the vendor/overclock split below entirely.
static uint16_t voltage_step(const AsicConfig * asic, uint16_t current_voltage, bool overclock_enabled, bool performance_mode)
{
    if (performance_mode) {
        uint16_t custom_step = nvs_config_get_u16(NVS_CONFIG_AUTOTUNE_VOLTAGE_STEP);
        if (custom_step > 0) {
            return custom_step;
        }
    }
    return (overclock_enabled && current_voltage >= voltage_table_max(asic))
           ? OVERCLOCK_VOLTAGE_STEP_MV : VENDOR_VOLTAGE_STEP_MV;
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

static bool read_is_unstable(GlobalState * GLOBAL_STATE, float * out_temp, bool * out_overtemp)
{
    PowerManagementModule * pm = &GLOBAL_STATE->POWER_MANAGEMENT_MODULE;
    SystemModule * sys = &GLOBAL_STATE->SYSTEM_MODULE;

    float temp = pm->chip_temp_avg > pm->chip_temp2_avg ? pm->chip_temp_avg : pm->chip_temp2_avg;
    *out_temp = temp;
    *out_overtemp = false;

    // In Performance mode, the user's own temp ceiling (if set) IS the real
    // limit - not just a soft "stop climbing" marker. Otherwise a custom
    // ceiling above the 68C default would never actually be reachable: this
    // check would keep calling it unstable and retreating well below it.
    float temp_limit = AUTOTUNE_TEMP_LIMIT_C;
    if (nvs_config_get_bool(NVS_CONFIG_AUTOTUNE_PROFILE)) {
        float user_max_temp = nvs_config_get_float(NVS_CONFIG_AUTOTUNE_MAX_TEMP);
        if (user_max_temp > 0.0f) {
            temp_limit = user_max_temp;
        }
    }

    if (temp > temp_limit) {
        *out_overtemp = true;
        return true;
    }

    // Deliberately not using instantaneous current_hashrate vs expected here:
    // it's noisy enough (job-timing variance) that it can flag "unstable" on
    // pure noise even at 0% hash error, which stalls climbing indefinitely.
    // The ASIC-reported error rate below is a much cleaner, direct signal of
    // real instability, and the per-domain check below still catches a fully
    // dead domain that error rate alone wouldn't (no errors from a domain
    // producing nothing).
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
    at->reason = "disabled";
    at->stable_checks = 0;
    at->unstable_checks = 0;
    at->backoff_remaining = 0;
    at->performance_hold_remaining = 0;
    at->rescue_attempts = 0;
    at->last_step_mv = 0;
    at->last_step_mhz = 0;
    at->last_action_time_s = 0;
    at->last_efficiency_ghs_w = 0.0f;
    at->temperature_c = 0.0f;
    at->power_w = 0.0f;
    at->power_1m_w = 0.0f;
    at->error_rate_pct = 0.0f;
    at->efficiency_ghs_w = 0.0f;
    at->eco_peak_found = false;
    bool rescue_limit_warned = false;

    ESP_LOGI(TAG, "Starting (voltage floor %umV, frequency floor %g MHz)", vendor_min_mv, floor_freq_mhz);

    TickType_t taskWakeTime = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&taskWakeTime, POLL_RATE_MS / portTICK_PERIOD_MS);

        if (!nvs_config_get_bool(NVS_CONFIG_AUTOTUNE_ENABLED)) {
            at->state = AUTOTUNE_STATE_IDLE;
            at->reason = "disabled";
            at->stable_checks = 0;
            at->unstable_checks = 0;
            at->backoff_remaining = 0;
            at->performance_hold_remaining = 0;
            at->rescue_attempts = 0;
            at->eco_peak_found = false;
            at->last_efficiency_ghs_w = 0.0f;
            at->power_1m_w = 0.0f;
            rescue_limit_warned = false;
            continue;
        }

        if (!GLOBAL_STATE->ASIC_initalized || GLOBAL_STATE->SELF_TEST_MODULE.is_active) {
            at->power_1m_w = 0.0f;
            at->performance_hold_remaining = 0;
            continue;
        }

        bool overclock_enabled = nvs_config_get_bool(NVS_CONFIG_OVERCLOCK_ENABLED);
        bool performance_mode = nvs_config_get_bool(NVS_CONFIG_AUTOTUNE_PROFILE);
        if (!performance_mode) {
            at->performance_hold_remaining = 0;
        }
        uint16_t core_voltage = nvs_config_get_u16(NVS_CONFIG_ASIC_VOLTAGE);
        float core_frequency = nvs_config_get_float(NVS_CONFIG_ASIC_FREQUENCY);

        uint16_t max_voltage = effective_max_voltage(asic, overclock_enabled);
        float max_frequency = effective_max_frequency(asic, overclock_enabled);

        float current_power = GLOBAL_STATE->POWER_MANAGEMENT_MODULE.power;
        float max_power_limit = nvs_config_get_float(NVS_CONFIG_MAX_POWER_LIMIT);
        bool over_power_limit = max_power_limit > 0.0f && current_power > max_power_limit;

        float temp = 0.0f;
        bool overtemp = false;
        bool unstable = read_is_unstable(GLOBAL_STATE, &temp, &overtemp);
        if (over_power_limit || overtemp || unstable) {
            // A safety event invalidates the previous Performance target. After
            // recovery, require a fresh stable climb before starting another hold.
            at->performance_hold_remaining = 0;
        }
        if (at->power_1m_w <= 0.0f) {
            at->power_1m_w = current_power;
        } else {
            at->power_1m_w += (current_power - at->power_1m_w) * POWER_AVERAGE_ALPHA;
        }
        // Use the existing one-minute hashrate average for efficiency decisions.
        // Pair it with averaged power so transient readings do not reject a good step.
        float efficiency = (at->power_1m_w > 0.0f) ? GLOBAL_STATE->SYSTEM_MODULE.hashrate_1m / at->power_1m_w : 0.0f;
        at->temperature_c = temp;
        at->power_w = current_power;
        at->error_rate_pct = GLOBAL_STATE->SYSTEM_MODULE.error_percentage;
        at->efficiency_ghs_w = efficiency;

        if (overtemp) {
            at->reason = "overtemp";
        } else if (over_power_limit) {
            at->reason = "power_limit";
        } else if (unstable) {
            at->reason = at->unstable_checks > 0 ? "confirming_instability" : "unstable";
        } else if (performance_mode && at->performance_hold_remaining > 0) {
            at->reason = "performance_hold";
        } else if (at->backoff_remaining > 0) {
            at->reason = "cooldown";
        } else {
            at->reason = "seeking";
        }
        if (over_power_limit || overtemp) {
            // Distinct from the general instability path on purpose: instability
            // there means "needs more voltage", which would only make an
            // over-power or overtemp situation worse. This always reduces -
            // voltage first, then frequency - and never rescues. Overtemp gets
            // a longer cooldown than a plain power/retreat step: thermal mass
            // takes real time to actually settle, so climbing right back up
            // after 30s just retriggers the same cycle.
            at->stable_checks = 0;
            at->unstable_checks = 0;
            int cooldown = overtemp ? OVERTEMP_COOLDOWN_CHECKS : RETREAT_COOLDOWN_CHECKS;

            if (at->backoff_remaining > 0) {
                at->backoff_remaining--;
                at->state = AUTOTUNE_STATE_HELD;
            } else if (core_voltage > vendor_min_mv) {
                uint16_t step = voltage_step(asic, core_voltage, overclock_enabled, performance_mode);
                uint16_t new_voltage = (core_voltage > vendor_min_mv + step) ? core_voltage - step : vendor_min_mv;

                if (overtemp) {
                    ESP_LOGI(TAG, "Overtemp (%.1fC) - reducing voltage %umV -> %umV, cooling down for %ds",
                             temp, core_voltage, new_voltage, cooldown * POLL_RATE_MS / 1000);
                } else {
                    ESP_LOGI(TAG, "Over power limit (%.1fW > %.1fW) - reducing voltage %umV -> %umV",
                             current_power, max_power_limit, core_voltage, new_voltage);
                }
                nvs_config_set_u16(NVS_CONFIG_ASIC_VOLTAGE, new_voltage);
                at->backoff_remaining = cooldown;
                at->state = AUTOTUNE_STATE_SHAVING;
                at->last_step_mv = (int16_t)(new_voltage - core_voltage);
                mark_action_time(at);
            } else if (core_frequency > floor_freq_mhz) {
                float new_frequency = core_frequency - frequency_step(asic, core_frequency, overclock_enabled, performance_mode);
                if (new_frequency < floor_freq_mhz) {
                    new_frequency = floor_freq_mhz;
                }

                if (overtemp) {
                    ESP_LOGI(TAG, "Overtemp (%.1fC) at voltage floor - reducing frequency %g -> %g MHz, cooling down for %ds",
                             temp, core_frequency, new_frequency, cooldown * POLL_RATE_MS / 1000);
                } else {
                    ESP_LOGI(TAG, "Over power limit (%.1fW > %.1fW) at voltage floor - reducing frequency %g -> %g MHz",
                             current_power, max_power_limit, core_frequency, new_frequency);
                }
                nvs_config_set_float(NVS_CONFIG_ASIC_FREQUENCY, new_frequency);
                at->backoff_remaining = cooldown;
                at->state = AUTOTUNE_STATE_RETREATING;
                at->last_step_mhz = (int16_t)(new_frequency - core_frequency);
                mark_action_time(at);
            } else {
                at->state = AUTOTUNE_STATE_HELD;
                if (overtemp) {
                    ESP_LOGW(TAG, "Overtemp (%.1fC) but already at voltage and frequency floor - cannot reduce further", temp);
                } else {
                    ESP_LOGW(TAG, "Over power limit (%.1fW > %.1fW) but already at voltage and frequency floor - cannot reduce further",
                             current_power, max_power_limit);
                }
            }
        } else if (unstable) {
            at->stable_checks = 0;
            at->unstable_checks++;

            if (at->unstable_checks < UNSTABLE_CONFIRM_CHECKS) {
                // Single noisy reading - wait for it to repeat before reacting.
                // Leave state as-is so the UI doesn't flicker on a one-off blip.
                ESP_LOGI(TAG, "Unstable reading (%d/%d) - waiting for confirmation before reacting",
                         at->unstable_checks, UNSTABLE_CONFIRM_CHECKS);
            } else if (at->rescue_attempts < MAX_CONSECUTIVE_RESCUES && core_voltage < max_voltage
                       && !(max_power_limit > 0.0f && current_power >= max_power_limit * PROACTIVE_POWER_MARGIN)) {
                // The power-limit check above matters: rescuing (raising voltage)
                // while already near the power cap would just push power back up
                // and undo the reason a prior shave brought voltage down in the
                // first place - a real oscillation risk. Skip straight to
                // frequency retreat in that case instead.
                uint16_t step = voltage_step(asic, core_voltage, overclock_enabled, performance_mode);
                uint16_t new_voltage = core_voltage + step;
                if (new_voltage > max_voltage) {
                    new_voltage = max_voltage;
                }

                ESP_LOGI(TAG, "Unstable (%.1fC) - rescuing voltage %umV -> %umV", temp, core_voltage, new_voltage);
                nvs_config_set_u16(NVS_CONFIG_ASIC_VOLTAGE, new_voltage);
                at->rescue_attempts++;
                at->backoff_remaining = RESCUE_COOLDOWN_CHECKS;
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
                    float new_frequency = core_frequency - frequency_step(asic, core_frequency, overclock_enabled, performance_mode);
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
            if (performance_mode && at->performance_hold_remaining > 0) {
                at->stable_checks = 0;
                at->state = AUTOTUNE_STATE_HOLDING;
                at->performance_hold_remaining--;
                continue;
            }

            at->rescue_attempts = 0;
            at->unstable_checks = 0;
            rescue_limit_warned = false;
            at->stable_checks++;
            at->state = AUTOTUNE_STATE_STABLE;

            if (at->backoff_remaining > 0) {
                at->backoff_remaining--;
            } else if (at->stable_checks >= (performance_mode
                                             ? PERFORMANCE_STABLE_CHECKS_BEFORE_ACTION
                                             : STABLE_CHECKS_BEFORE_ACTION)) {
                SystemModule * sys = &GLOBAL_STATE->SYSTEM_MODULE;
                efficiency = (at->power_1m_w > 0.0f) ? sys->hashrate_1m / at->power_1m_w : 0.0f;

                bool eco_regressed = !performance_mode && at->eco_peak_found == false
                                      && at->last_efficiency_ghs_w > 0.0f
                                      && efficiency < at->last_efficiency_ghs_w * ECO_EFFICIENCY_TOLERANCE
                                      && core_frequency > floor_freq_mhz;

                float max_temp_c = nvs_config_get_float(NVS_CONFIG_AUTOTUNE_MAX_TEMP);
                bool temp_ceiling_reached = performance_mode && max_temp_c > 0.0f && temp >= max_temp_c;
                bool power_ceiling_near = max_power_limit > 0.0f && current_power >= max_power_limit * PROACTIVE_POWER_MARGIN;

                bool climbing_allowed = (performance_mode ? !temp_ceiling_reached : !at->eco_peak_found) && !power_ceiling_near;

                if (eco_regressed) {
                    // Eco mode: the last climb step made hash/watt worse - that step
                    // wasn't worth it. Undo it and lock in the previous point as the
                    // efficiency peak; from here on only shave voltage.
                    float new_frequency = core_frequency - frequency_step(asic, core_frequency, overclock_enabled, performance_mode);
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
                    float new_frequency = core_frequency + frequency_step(asic, core_frequency, overclock_enabled, performance_mode);
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
                    uint16_t step = voltage_step(asic, core_voltage, overclock_enabled, performance_mode);
                    uint16_t new_voltage = (core_voltage > vendor_min_mv + step) ? core_voltage - step : vendor_min_mv;

                    if (new_voltage < core_voltage) {
                        if (power_ceiling_near) {
                            ESP_LOGI(TAG, "Stable but near power limit (%.1fW >= %.1fW) - shaving voltage %umV -> %umV instead of climbing",
                                     current_power, max_power_limit * PROACTIVE_POWER_MARGIN, core_voltage, new_voltage);
                        } else if (temp_ceiling_reached) {
                            ESP_LOGI(TAG, "Stable but at temp ceiling (%.1fC >= %.1fC) - shaving voltage %umV -> %umV instead of climbing",
                                     temp, max_temp_c, core_voltage, new_voltage);
                        } else {
                            ESP_LOGI(TAG, "Stable, no more climbing - shaving voltage %umV -> %umV", core_voltage, new_voltage);
                        }
                        nvs_config_set_u16(NVS_CONFIG_ASIC_VOLTAGE, new_voltage);
                        at->state = AUTOTUNE_STATE_SHAVING;
                        at->last_step_mv = (int16_t)(new_voltage - core_voltage);
                        mark_action_time(at);
                    } else if (performance_mode
                               && (core_frequency >= max_frequency || temp_ceiling_reached || power_ceiling_near)) {
                        at->performance_hold_remaining = PERFORMANCE_HOLD_CHECKS;
                        at->state = AUTOTUNE_STATE_HOLDING;
                        at->reason = "performance_hold";
                        ESP_LOGI(TAG, "Performance target reached at %g MHz / %umV - holding for 1 hour",
                                 core_frequency, core_voltage);
                    }
                }
                at->stable_checks = 0;
            }
        }
    }
}
