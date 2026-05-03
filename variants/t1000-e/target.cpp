#include <Arduino.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const unsigned long kGnssMeshIntervalMs = 300000UL;
static const double kGnssMeshMoveMeters = 500.0;
#if defined(T1000E_REPEATER_BUILD)
static const unsigned long kGnssSessionAutoOffMs = 7200000UL;
#endif

static double t1000e_haversine_m(long lat1_u, long lon1_u, long lat2_u, long lon2_u) {
  if (lat1_u == LONG_MIN) return 0.0;
  double lat1 = lat1_u * 1e-6;
  double lon1 = lon1_u * 1e-6;
  double lat2 = lat2_u * 1e-6;
  double lon2 = lon2_u * 1e-6;
  const double R = 6371000.0;
  double p1 = lat1 * (M_PI / 180.0);
  double p2 = lat2 * (M_PI / 180.0);
  double dphi = (lat2 - lat1) * (M_PI / 180.0);
  double dl = (lon2 - lon1) * (M_PI / 180.0);
  double sda = sin(dphi * 0.5);
  double sdl = sin(dl * 0.5);
  double a = sda * sda + cos(p1) * cos(p2) * sdl * sdl;
  if (a > 1.0) a = 1.0;
  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  return R * c;
}
#include "t1000e_sensors.h"
#include "t1000e_gps_bringup.h"
#include "target.h"
#include <helpers/sensors/MicroNMEALocationProvider.h>
#if defined(T1000E_REPEATER_BUILD)
#include "t1000e_secret_gnss_channel.h"
#endif

T1000eBoard board;

RADIO_CLASS radio = new Module(P_LORA_NSS, P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY, SPI);

WRAPPER_CLASS radio_driver(radio, board);

VolatileRTCClock rtc_clock;
MicroNMEALocationProvider nmea = MicroNMEALocationProvider(Serial1, &rtc_clock);
T1000SensorManager sensors = T1000SensorManager(nmea);

#ifdef DISPLAY_CLASS
  NullDisplayDriver display;
#endif

#ifndef LORA_CR
  #define LORA_CR      5
#endif

#ifdef RF_SWITCH_TABLE
static const uint32_t rfswitch_dios[Module::RFSWITCH_MAX_PINS] = {
  RADIOLIB_LR11X0_DIO5,
  RADIOLIB_LR11X0_DIO6,
  RADIOLIB_LR11X0_DIO7,
  RADIOLIB_LR11X0_DIO8, 
  RADIOLIB_NC
};

static const Module::RfSwitchMode_t rfswitch_table[] = {
  // mode                 DIO5  DIO6  DIO7  DIO8
  { LR11x0::MODE_STBY,   {LOW,  LOW,  LOW,  LOW  }},  
  { LR11x0::MODE_RX,     {HIGH, LOW,  LOW,  HIGH }},
  { LR11x0::MODE_TX,     {HIGH, HIGH, LOW,  HIGH }},
  { LR11x0::MODE_TX_HP,  {LOW,  HIGH, LOW,  HIGH }},
  { LR11x0::MODE_TX_HF,  {LOW,  LOW,  LOW,  LOW  }}, 
  { LR11x0::MODE_GNSS,   {LOW,  LOW,  HIGH, LOW  }},
  { LR11x0::MODE_WIFI,   {LOW,  LOW,  LOW,  LOW  }},  
  END_OF_MODE_TABLE,
};
#endif

bool radio_init() {
  //rtc_clock.begin(Wire);
  
#ifdef LR11X0_DIO3_TCXO_VOLTAGE
  float tcxo = LR11X0_DIO3_TCXO_VOLTAGE;
#else
  float tcxo = 1.6f;
#endif

  SPI.setPins(P_LORA_MISO, P_LORA_SCLK, P_LORA_MOSI);
  SPI.begin();
  int status = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, RADIOLIB_LR11X0_LORA_SYNC_WORD_PRIVATE, LORA_TX_POWER, 16, tcxo);
  if (status != RADIOLIB_ERR_NONE) {
    Serial.print("ERROR: radio init failed: ");
    Serial.println(status);
    return false;  // fail
  }
  
  radio.setCRC(2);
  radio.explicitHeader();

#ifdef RF_SWITCH_TABLE
  radio.setRfSwitchTable(rfswitch_dios, rfswitch_table);
#endif
#ifdef RX_BOOSTED_GAIN
  radio.setRxBoostedGainMode(RX_BOOSTED_GAIN);
#endif

  return true;  // success
}

uint32_t radio_get_rng_seed() {
  return radio.random(0x7FFFFFFF);
}

void radio_set_params(float freq, float bw, uint8_t sf, uint8_t cr) {
  radio.setFrequency(freq);
  radio.setSpreadingFactor(sf);
  radio.setBandwidth(bw);
  radio.setCodingRate(cr);
}

void radio_set_tx_power(int8_t dbm) {
  radio.setOutputPower(dbm);
}

mesh::LocalIdentity radio_new_identity() {
  RadioNoiseListener rng(radio);
  return mesh::LocalIdentity(&rng);  // create new random identity
}

#ifdef PIN_BUZZER
static void t1000e_play_first_gnss_fix_chirp() {
#ifdef PIN_BUZZER_EN
  digitalWrite(PIN_BUZZER_EN, HIGH);
#endif
  const int f_lo = 350;
  const int f_hi = 2200;
  const unsigned t_short = 70;
  const unsigned gap = 55;
  const unsigned t_long = 550;
  for (int i = 0; i < 3; i++) {
    tone(PIN_BUZZER, f_lo, t_short);
    delay(t_short + gap);
  }
  noTone(PIN_BUZZER);
  tone(PIN_BUZZER, f_hi, t_long);
  delay(t_long + 30);
  noTone(PIN_BUZZER);
}
#endif

void T1000SensorManager::start_gps() {
  gps_active = true;
  _gnss_first_fix_chirp_done = false;
#if defined(T1000E_REPEATER_BUILD)
  _gnss_mesh_msg_pending = false;
  _last_gnss_mesh_send_ms = 0;
  _last_gnss_mesh_lat_u = LONG_MIN;
  _last_gnss_mesh_lon_u = LONG_MIN;
  _gnss_mesh_retry_after_ms = 0;
  _gnss_session_start_ms = millis();
  _gnss_two_hour_auto_off_pending = false;
#endif
  t1000e_reset_and_assure_gps_functionality(false);
}

void T1000SensorManager::sleep_gps() {
  gps_active = false;
  _gnss_user_init_active = false;
  _gnss_user_init_reported = false;
  _gnss_first_fix_chirp_done = false;
  _gnss_mesh_msg_pending = false;
  _last_gnss_mesh_send_ms = 0;
  _last_gnss_mesh_lat_u = LONG_MIN;
  _last_gnss_mesh_lon_u = LONG_MIN;
  _gnss_mesh_retry_after_ms = 0;
#if defined(T1000E_REPEATER_BUILD)
  _gnss_session_start_ms = 0;
#endif
  _nmea->setSerialNmeaEcho(false);
  digitalWrite(GPS_VRTC_EN, HIGH);
  digitalWrite(GPS_EN, LOW);
  digitalWrite(GPS_RESET, HIGH);
  digitalWrite(GPS_SLEEP_INT, HIGH);
  digitalWrite(GPS_RTC_INT, LOW);
  pinMode(GPS_RESETB, OUTPUT);
  digitalWrite(GPS_RESETB, LOW);
  //_nmea->stop();
}

void T1000SensorManager::stop_gps() {
  gps_active = false;
  _gnss_user_init_active = false;
  _gnss_user_init_reported = false;
  _gnss_first_fix_chirp_done = false;
  _gnss_mesh_msg_pending = false;
  _last_gnss_mesh_send_ms = 0;
  _last_gnss_mesh_lat_u = LONG_MIN;
  _last_gnss_mesh_lon_u = LONG_MIN;
  _gnss_mesh_retry_after_ms = 0;
#if defined(T1000E_REPEATER_BUILD)
  _gnss_session_start_ms = 0;
#endif
  _nmea->setSerialNmeaEcho(false);
  digitalWrite(GPS_VRTC_EN, LOW);
  digitalWrite(GPS_EN, LOW);
  digitalWrite(GPS_RESET, HIGH);
  digitalWrite(GPS_SLEEP_INT, HIGH);
  digitalWrite(GPS_RTC_INT, LOW);
  pinMode(GPS_RESETB, OUTPUT);
  digitalWrite(GPS_RESETB, LOW);
  //_nmea->stop();
}


bool T1000SensorManager::begin() {
  // init GPS
  Serial1.begin(115200);
  return true;
}

bool T1000SensorManager::querySensors(uint8_t requester_permissions, CayenneLPP& telemetry) {
  if (requester_permissions & TELEM_PERM_LOCATION) {   // does requester have permission?
    telemetry.addGPS(TELEM_CHANNEL_SELF, node_lat, node_lon, node_altitude);
  }
  if (requester_permissions & TELEM_PERM_ENVIRONMENT) {
    // Firmware reports light as a 0-100 % scale, but expose it via Luminosity so app labels it "Luminosity".
    telemetry.addLuminosity(TELEM_CHANNEL_SELF, t1000e_get_light());
    telemetry.addTemperature(TELEM_CHANNEL_SELF, t1000e_get_temperature());
  }
  return true;
}

void T1000SensorManager::userGnssOnWithNmeaEcho() {
  _nmea->syncTime();
  _gnss_user_init_reported = false;
  _gnss_first_fix_chirp_done = false;
  _gnss_mesh_msg_pending = false;
  _last_gnss_mesh_send_ms = 0;
  _last_gnss_mesh_lat_u = LONG_MIN;
  _last_gnss_mesh_lon_u = LONG_MIN;
  _gnss_mesh_retry_after_ms = 0;
  _gnss_session_start_ms = millis();
  _gnss_two_hour_auto_off_pending = false;
  _nmea->setSerialNmeaEcho(false);
  gps_active = true;
  t1000e_reset_and_assure_gps_functionality(true);
  _gnss_user_init_active = true;
  _gnss_user_init_deadline = millis() + 120000;
}

void T1000SensorManager::userGnssOff() {
  sleep_gps();
}

void T1000SensorManager::loop() {
  static long next_gps_update = 0;

  _nmea->loop();

#if defined(T1000E_REPEATER_BUILD)
  if (gps_active && _gnss_session_start_ms != 0 &&
      (unsigned long)(millis() - _gnss_session_start_ms) >= kGnssSessionAutoOffMs) {
    Serial.println(F("[GNSS] Auto power-off after 2h session"));
    _gnss_two_hour_auto_off_pending = true;
    sleep_gps();
  }
#endif

  if (gps_active && _nmea->isValid() && !_gnss_first_fix_chirp_done) {
    _gnss_first_fix_chirp_done = true;
#if defined(T1000E_REPEATER_BUILD)
    _gnss_mesh_msg_pending = true;
#endif
#ifdef PIN_BUZZER
    t1000e_play_first_gnss_fix_chirp();
#endif
  }

#if defined(T1000E_REPEATER_BUILD)
  if (gps_active && _nmea->isValid() && _gnss_first_fix_chirp_done && !_gnss_mesh_msg_pending) {
    unsigned long now = millis();
    if (_last_gnss_mesh_send_ms != 0) {
      if ((unsigned long)(now - _last_gnss_mesh_send_ms) >= kGnssMeshIntervalMs) {
        _gnss_mesh_msg_pending = true;
      } else if (_last_gnss_mesh_lat_u != LONG_MIN) {
        long lat_u = _nmea->getLatitude();
        long lon_u = _nmea->getLongitude();
        if (t1000e_haversine_m(_last_gnss_mesh_lat_u, _last_gnss_mesh_lon_u, lat_u, lon_u) > kGnssMeshMoveMeters) {
          _gnss_mesh_msg_pending = true;
        }
      }
    } else if (_gnss_mesh_retry_after_ms != 0 && (long)(now - _gnss_mesh_retry_after_ms) >= 0) {
      _gnss_mesh_msg_pending = true;
    }
  }
#endif

  if (_gnss_user_init_active) {
    if (_nmea->isValid()) {
      if (!_gnss_user_init_reported) {
        _gnss_user_init_reported = true;
        _gnss_user_init_active = false;
      }
    } else if ((long)(millis() - _gnss_user_init_deadline) >= 0) {
      Serial.println(F("[GNSS] Post-bring-up window ended (no valid fix yet); GNSS stays powered"));
      _gnss_user_init_active = false;
    }
  }

  if (millis() > next_gps_update) {
    if (gps_active && _nmea->isValid()) {
      node_lat = ((double)_nmea->getLatitude())/1000000.;
      node_lon = ((double)_nmea->getLongitude())/1000000.;
      node_altitude = ((double)_nmea->getAltitude()) / 1000.0;
    }
    next_gps_update = millis() + 1000;
  }
}

bool T1000SensorManager::formatGnssFixMessageForMesh(char* msg, size_t cap) {
  if (!msg || cap < 16 || !_nmea->isValid()) return false;
  auto* p = static_cast<MicroNMEALocationProvider*>(_nmea);
  long lat_u = _nmea->getLatitude();
  long lon_u = _nmea->getLongitude();
  long alt_mm = _nmea->getAltitude();
  int sats = (int)_nmea->satellitesCount();
  long sp = p->getGnssSpeedMilliknots();
  long co = p->getGnssCourseMilliDeg();
  unsigned hd = (unsigned)p->getGnssHdopTenths();

  double lat = lat_u / 1000000.0;
  double lon = lon_u / 1000000.0;
  double alt_m = alt_mm / 1000.0;
  double hdop = hd / 10.0;

  float batt_v = (float)board.getBattMilliVolts() / 1000.0f;
  float temp_c = t1000e_get_temperature();
  const bool has_temp = (temp_c == temp_c) && temp_c >= -80.0f && temp_c <= 150.0f;
  char telem_tail[48];
  int tt = has_temp
               ? snprintf(telem_tail, sizeof(telem_tail), " T=%.1f°C V=%.2fV", temp_c, batt_v)
               : snprintf(telem_tail, sizeof(telem_tail), " V=%.2fV", batt_v);
  if (tt <= 0 || (size_t)tt >= sizeof(telem_tail)) return false;

  int n;
  if (sp != LONG_MIN && co != LONG_MIN) {
    double kn = sp / 1000.0;
    double crs = co / 1000.0;
    n = snprintf(msg, cap,
                 "GNSS lat=%.6f lon=%.6f alt=%.1fm sats=%d spd=%.2fkn crs=%.1f hdop=%.1f%s",
                 lat, lon, alt_m, sats, kn, crs, hdop, telem_tail);
  } else if (sp != LONG_MIN) {
    double kn = sp / 1000.0;
    n = snprintf(msg, cap,
                 "GNSS lat=%.6f lon=%.6f alt=%.1fm sats=%d spd=%.2fkn hdop=%.1f%s",
                 lat, lon, alt_m, sats, kn, hdop, telem_tail);
  } else if (co != LONG_MIN) {
    double crs = co / 1000.0;
    n = snprintf(msg, cap,
                 "GNSS lat=%.6f lon=%.6f alt=%.1fm sats=%d crs=%.1f hdop=%.1f%s",
                 lat, lon, alt_m, sats, crs, hdop, telem_tail);
  } else {
    n = snprintf(msg, cap,
                 "GNSS lat=%.6f lon=%.6f alt=%.1fm sats=%d hdop=%.1f%s",
                 lat, lon, alt_m, sats, hdop, telem_tail);
  }
  return n > 0 && (size_t)n < cap;
}

void T1000SensorManager::recordGnssMeshMessageSent() {
  _gnss_mesh_msg_pending = false;
  _last_gnss_mesh_send_ms = millis();
  _gnss_mesh_retry_after_ms = 0;
  if (_nmea->isValid()) {
    _last_gnss_mesh_lat_u = _nmea->getLatitude();
    _last_gnss_mesh_lon_u = _nmea->getLongitude();
  }
}

void T1000SensorManager::onSecretGnssMeshSendFailed() {
  _gnss_mesh_msg_pending = false;
  _gnss_mesh_retry_after_ms = millis() + 60000UL;
}

#if defined(T1000E_REPEATER_BUILD)
bool T1000SensorManager::consumeGnssTwoHourSessionAutoOff() {
  if (!_gnss_two_hour_auto_off_pending) return false;
  _gnss_two_hour_auto_off_pending = false;
  return true;
}
#endif

int T1000SensorManager::getNumSettings() const { return 1; }  // just one supported: "gps" (power switch)

const char* T1000SensorManager::getSettingName(int i) const {
  return i == 0 ? "gps" : NULL;
}
const char* T1000SensorManager::getSettingValue(int i) const {
  if (i == 0) {
    return gps_active ? "1" : "0";
  }
  return NULL;
}
bool T1000SensorManager::setSettingValue(const char* name, const char* value) {
  if (strcmp(name, "gps") == 0) {
    if (strcmp(value, "0") == 0) {
      sleep_gps(); // sleep for faster fix !
    } else {
      start_gps();
    }
    return true;
  }
  return false;  // not supported
}
