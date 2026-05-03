/**
 * T1000-E / Airoha AG3335 GPS bring-up — sequence (command order/delays unchanged).
 */

#include <Arduino.h>
#include <string.h>
#include "variant.h"
#include "t1000e_gps_bringup.h"

#ifndef GPS_EN
#error "T1000-E variant must define GPS_EN"
#endif

static void t1000e_note_first_nmea(bool log_progress, bool* announced) {
  if (!log_progress || announced == NULL || *announced) {
    return;
  }
  Serial.println(F("first NMEA received"));
  *announced = true;
}

static bool t1000e_is_gps_alive(unsigned long timeout_ms, bool log_progress, bool* first_nmea_announced) {
  unsigned long start_wait = millis();
  char nmea_buffer[200] = {0};
  int nmea_idx = 0;
  bool got_dollar = false;

  while (millis() - start_wait < timeout_ms) {
    if (Serial1.available() > 0) {
      char c = Serial1.read();

      if (c == '$') {
        nmea_idx = 0;
        got_dollar = true;
        nmea_buffer[nmea_idx++] = c;
      } else if (got_dollar && nmea_idx < (int)(sizeof(nmea_buffer) - 1)) {
        nmea_buffer[nmea_idx++] = c;
      }

      if (got_dollar && nmea_idx > 10 && (c == '\n' || c == '\r')) {
        nmea_buffer[nmea_idx] = '\0';

        if (strstr(nmea_buffer, "GPGGA") != NULL ||
            strstr(nmea_buffer, "GPRMC") != NULL ||
            strstr(nmea_buffer, "GNGGA") != NULL ||
            strstr(nmea_buffer, "GNRMC") != NULL ||
            strstr(nmea_buffer, "PAIR") != NULL ||
            strstr(nmea_buffer, "PMTK") != NULL) {
          t1000e_note_first_nmea(log_progress, first_nmea_announced);
          return true;
        }
        nmea_idx = 0;
        got_dollar = false;
        memset(nmea_buffer, 0, sizeof(nmea_buffer));
      }
    }
    delay(10);
  }

  return false;
}

void t1000e_reset_and_assure_gps_functionality(bool log_progress) {
  const int MAX_RETRIES = 10;
  bool first_nmea_announced = false;

  if (log_progress) {
    Serial.println(F("[GNSS] AG3335 bring-up (terminaattori)"));
    Serial.println(F("[GPS] ========================================"));
    Serial.println(F("[GPS] Full reset and initialization (sequence)"));
    Serial.println(F("[GPS] ========================================"));
  }

  Serial1.begin(115200);
  pinMode(PIN_SERIAL1_RX, INPUT_PULLUP);

  if (t1000e_is_gps_alive(10000, log_progress, &first_nmea_announced)) {
    if (log_progress) {
      Serial.println(F("[GPS] Already alive; saving PAIR513"));
    }
    Serial1.write("$PAIR513*3D\r\n");
    delay(100);
    return;
  }

  for (int retry = 0; retry < MAX_RETRIES; retry++) {
    if (log_progress) {
      Serial.print(F("[GPS] Reset attempt "));
      Serial.print(retry + 1);
      Serial.print(F("/"));
      Serial.println(MAX_RETRIES);
    }

    pinMode(GPS_EN, OUTPUT);
    digitalWrite(GPS_EN, LOW);
    delay(50);
    digitalWrite(GPS_EN, HIGH);
    delay(100);

    pinMode(GPS_VRTC_EN, OUTPUT);
    digitalWrite(GPS_VRTC_EN, HIGH);
    delay(100);

    pinMode(GPS_RESET, OUTPUT);
    digitalWrite(GPS_RESET, HIGH);
    delay(100);
    digitalWrite(GPS_RESET, LOW);
    delay(1000);

    pinMode(GPS_SLEEP_INT, OUTPUT);
    digitalWrite(GPS_SLEEP_INT, LOW);
    delay(50);
    digitalWrite(GPS_SLEEP_INT, HIGH);
    delay(100);

    pinMode(GPS_RTC_INT, OUTPUT);
    digitalWrite(GPS_RTC_INT, LOW);
    pinMode(GPS_RESETB, INPUT_PULLUP);

    delay(2000);

    int cleared = 0;
    while (Serial1.available()) {
      Serial1.read();
      cleared++;
    }
    if (log_progress && cleared > 0) {
      Serial.print(F("[GPS] Cleared "));
      Serial.print(cleared);
      Serial.println(F(" stale bytes"));
    }

    if (log_progress) {
      Serial.println(F("[GPS] PAIR101 (wake query)"));
    }
    Serial1.write("$PAIR101*2E\r\n");
    delay(300);

    bool got_response = false;
    unsigned long query_start = millis();
    while (millis() - query_start < 1000) {
      if (Serial1.available() > 0) {
        char c = Serial1.read();
        if (c == '$') {
          got_response = true;
          t1000e_note_first_nmea(log_progress, &first_nmea_announced);
          break;
        }
      }
      delay(10);
    }
    if (log_progress && !got_response) {
      Serial.println(F("[GPS] No PAIR101 response (continuing)"));
    }

    if (log_progress) {
      Serial.println(F("[GPS] PAIR100 / baud / NMEA enable burst"));
    }
    Serial1.write("$PAIR100,1,0*3A\r\n");
    delay(500);
    Serial1.write("$PAIR100,1,0*3A\r\n");
    delay(500);
    Serial1.write("$PAIR100,1,0*3B\r\n");
    delay(500);
    Serial1.write("$PAIR100,1,0*3B\r\n");
    delay(500);
    Serial1.write("$PAIR100,1,0*3A\r\n");
    delay(500);
    Serial1.write("$PAIR100,1,0*3A\r\n");
    delay(500);
    Serial1.write("$PAIR100,1,0*3B\r\n");
    delay(500);
    Serial1.write("$PAIR100,1,0*3B\r\n");
    delay(500);
    Serial1.write("$PAIR864,0,0,115200*1B\r\n");
    delay(500);
    Serial1.write("$PAIR081*33\r\n");
    delay(500);
    Serial1.println("$PAIR081*33");
    delay(500);
    Serial1.write("$PAIR100,1,0*7F\r\n");
    delay(500);
    Serial1.write("$PAIR100,1,0*7F\r\n");
    delay(500);

    if (log_progress) {
      Serial.println(F("[GPS] PAIR062 sentence mask"));
    }
    Serial1.write("$PAIR062,0,1*3E\r\n");
    delay(200);
    Serial1.write("$PAIR062,4,1*38\r\n");
    delay(200);
    Serial1.write("$PAIR062,2,0*3C\r\n");
    delay(150);
    Serial1.write("$PAIR062,3,0*3D\r\n");
    delay(150);

    if (log_progress) {
      Serial.println(F("[GPS] PAIR514 (NVRAM)"));
    }
    Serial1.write("$PAIR514*29\r\n");
    delay(300);

    if (log_progress) {
      Serial.println(F("[GPS] PMTK fallback"));
    }
    Serial1.write("$PMTK605*31\r\n");
    delay(300);
    Serial1.write("$PMTK514,1*28\r\n");
    delay(300);
    Serial1.write("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n");
    delay(300);

    if (log_progress) {
      Serial.println(F("[GPS] Waiting for NMEA (10s)"));
    }

    if (t1000e_is_gps_alive(10000, log_progress, &first_nmea_announced)) {
      Serial1.write("$PAIR513*3D\r\n");
      delay(100);
      return;
    }

    if (log_progress) {
      Serial.println(F("[GPS] No NMEA yet, retrying..."));
    }
    delay(1000);
  }

  if (log_progress) {
    Serial.println(F("[GPS] Failed after all retries (module may still start later)"));
  }
}
