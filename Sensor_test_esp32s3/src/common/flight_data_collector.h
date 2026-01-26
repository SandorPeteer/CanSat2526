#pragma once

#include "flight_recorder.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize flight data collector
 * Call after all sensors are initialized
 */
esp_err_t flight_data_collector_init(void);

/**
 * @brief Collect current data from all sensors into a flight record
 * Call this at your desired sampling rate (e.g., 1Hz)
 * @param record Output record
 */
void flight_data_collector_sample(flight_record_t *record);

/**
 * @brief Sample and store in PSRAM buffer + optionally write to file
 * Convenience function that does sample + add + write
 */
void flight_data_collector_tick(void);

#ifdef __cplusplus
}
#endif
