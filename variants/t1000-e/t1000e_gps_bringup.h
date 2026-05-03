#pragma once

#include <Arduino.h>

/**
 * Airoha AG3335 bring-up for Seeed T1000-E (sequence).
 * @param log_progress If true, print [GPS] step labels and once: "first NMEA received".
 *                     Never prints raw NMEA.
 */
void t1000e_reset_and_assure_gps_functionality(bool log_progress);
