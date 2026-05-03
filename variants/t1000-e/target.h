#pragma once

#include <limits.h>
#define RADIOLIB_STATIC_ONLY 1
#include <RadioLib.h>
#include <helpers/radiolib/RadioLibWrappers.h>
#include "T1000eBoard.h"
#include <helpers/radiolib/CustomLR1110Wrapper.h>
#include <helpers/ArduinoHelpers.h>
#include <helpers/SensorManager.h>
#include <helpers/sensors/LocationProvider.h>
#ifdef DISPLAY_CLASS
  #include "NullDisplayDriver.h"
#endif

class T1000SensorManager: public SensorManager {
  bool gps_active = false;
  LocationProvider * _nmea;
  bool _gnss_user_init_active = false;
  unsigned long _gnss_user_init_deadline = 0;
  bool _gnss_user_init_reported = false;
  bool _gnss_first_fix_chirp_done = false;
  bool _gnss_mesh_msg_pending = false;
  unsigned long _last_gnss_mesh_send_ms = 0;
  long _last_gnss_mesh_lat_u = LONG_MIN;
  long _last_gnss_mesh_lon_u = LONG_MIN;
  unsigned long _gnss_mesh_retry_after_ms = 0;
#if defined(T1000E_REPEATER_BUILD)
  unsigned long _gnss_session_start_ms = 0;
  bool _gnss_two_hour_auto_off_pending = false;
#endif

  void start_gps();
  void sleep_gps();
  void stop_gps();
public:
  T1000SensorManager(LocationProvider &nmea): _nmea(&nmea) { }
  bool begin() override;
  bool querySensors(uint8_t requester_permissions, CayenneLPP& telemetry) override;
  void loop() override;
  int getNumSettings() const override;
  const char* getSettingName(int i) const override;
  const char* getSettingValue(int i) const override;
  bool setSettingValue(const char* name, const char* value) override;
  LocationProvider* getLocationProvider() { return _nmea; }

  bool isGnssPowered() const { return gps_active; }
  bool isGnssUserInitPending() const { return _gnss_user_init_active; }
  void userGnssOnWithNmeaEcho();
  void userGnssOff();

  bool isGnssMeshMessagePending() const { return _gnss_mesh_msg_pending; }
  void recordGnssMeshMessageSent();
  void onSecretGnssMeshSendFailed();
  bool formatGnssFixMessageForMesh(char* msg, size_t cap);
#if defined(T1000E_REPEATER_BUILD)
  bool consumeGnssTwoHourSessionAutoOff();
#endif
};

#ifdef DISPLAY_CLASS
  extern NullDisplayDriver display;
#endif

extern T1000eBoard board;
extern WRAPPER_CLASS radio_driver;
extern VolatileRTCClock rtc_clock;
extern T1000SensorManager sensors;

bool radio_init();
uint32_t radio_get_rng_seed();
void radio_set_params(float freq, float bw, uint8_t sf, uint8_t cr);
void radio_set_tx_power(int8_t dbm);
mesh::LocalIdentity radio_new_identity();
