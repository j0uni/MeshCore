#include "SensorMesh.h"

#ifdef DISPLAY_CLASS
  #include "UITask.h"
  static UITask ui_task(display);
#endif

#ifndef MAX_TEXT_LEN
#define MAX_TEXT_LEN 120
#endif

class MyMesh : public SensorMesh {
public:
  MyMesh(mesh::MainBoard& board, mesh::Radio& radio, mesh::MillisecondClock& ms, mesh::RNG& rng, mesh::RTCClock& rtc, mesh::MeshTables& tables)
     : SensorMesh(board, radio, ms, rng, rtc, tables), 
       battery_data(12*24, 5*60)    // 24 hours worth of battery data, every 5 minutes
  {
  }

#if defined(NRF52_PLATFORM)
  void sendTelemetryToChannelNow() {
    // Build a fresh telemetry snapshot immediately.
    refreshTelemetryNow();

    if (!telemetry_channel_initialized) {
      const char* channel_key_hex = "170d4fb04b507fb4693c37f183a47719";
      uint8_t channel_key[16];
      if (mesh::Utils::fromHex(channel_key, 16, channel_key_hex)) {
        memset(telemetry_channel.secret, 0, sizeof(telemetry_channel.secret));
        memcpy(telemetry_channel.secret, channel_key, 16);
        mesh::Utils::sha256(telemetry_channel.hash, sizeof(telemetry_channel.hash), telemetry_channel.secret, 16);
        telemetry_channel_initialized = true;
      }
    }

    if (!telemetry_channel_initialized) return;

    const float t = getTemperature(TELEM_CHANNEL_SELF);
    const float v = getVoltage(TELEM_CHANNEL_SELF);
    const float h = getRelativeHumidity(TELEM_CHANNEL_SELF);
    const float p = getBarometricPressure(TELEM_CHANNEL_SELF);
    const float lu = getTelemValue(TELEM_CHANNEL_SELF, LPP_LUMINOSITY);

    char msg[160];
    int len = snprintf(msg, sizeof(msg), "%s:", getNodePrefs()->node_name);
    if (!isnan(t) && t > -100.0f && t < 100.0f) {
      int n = snprintf(msg + len, sizeof(msg) - (size_t)len, " T=%.1fC", t);
      if (n > 0) len += n;
    }
    {
      int n = snprintf(msg + len, sizeof(msg) - (size_t)len, " V=%.2fV", v);
      if (n > 0) len += n;
    }
    if (!isnan(h) && h >= 0.0f && h <= 100.0f) {
      int n = snprintf(msg + len, sizeof(msg) - (size_t)len, " H=%.1f%%", h);
      if (n > 0) len += n;
    }
    if (!isnan(p) && p > 200.0f && p < 1300.0f) {
      int n = snprintf(msg + len, sizeof(msg) - (size_t)len, " P=%.1fhPa", p);
      if (n > 0) len += n;
    }
    if (!isnan(lu) && lu >= 0.0f) {
      int n = snprintf(msg + len, sizeof(msg) - (size_t)len, " LU=%.0f%%", lu);
      if (n > 0) len += n;
    }

    uint32_t timestamp = getRTCClock()->getCurrentTime();
    uint8_t payload[5 + MAX_TEXT_LEN + 1];
    memcpy(payload, &timestamp, 4);
    payload[4] = 0;

    int text_len = strlen(msg);
    if (text_len > MAX_TEXT_LEN) text_len = MAX_TEXT_LEN;
    memcpy(&payload[5], msg, text_len);
    payload[5 + text_len] = 0;

    auto pkt = createGroupDatagram(PAYLOAD_TYPE_GRP_TXT, telemetry_channel, payload, 5 + text_len);
    if (pkt) {
      sendFlood(pkt);
      MESH_DEBUG_PRINTLN("Sent #telemetry: %s", msg);
    }
  }

  bool hasPendingTxWork() const {
    return _mgr->getOutboundTotal() > 0;
  }
#endif

protected:
  /* ========================== custom logic here ========================== */
  Trigger low_batt, critical_batt;
  TimeSeriesData  battery_data;
#if defined(NRF52_PLATFORM)
  mesh::GroupChannel telemetry_channel = {};
  bool telemetry_channel_initialized = false;
#endif

  void onSensorDataRead() override {
    float batt_voltage = getVoltage(TELEM_CHANNEL_SELF);

    battery_data.recordData(getRTCClock(), batt_voltage);   // record battery
    alertIf(batt_voltage < 3.4f, critical_batt, HIGH_PRI_ALERT, "Battery is critical!");
    alertIf(batt_voltage < 3.6f, low_batt, LOW_PRI_ALERT, "Battery is low");
  }

  int querySeriesData(uint32_t start_secs_ago, uint32_t end_secs_ago, MinMaxAvg dest[], int max_num) override {
    battery_data.calcMinMaxAvg(getRTCClock(), start_secs_ago, end_secs_ago, &dest[0], TELEM_CHANNEL_SELF, LPP_VOLTAGE);
    return 1;
  }

  bool handleCustomCommand(uint32_t sender_timestamp, char* command, char* reply) override {
    if (strcmp(command, "magic") == 0) {    // example 'custom' command handling
      strcpy(reply, "**Magic now done**");
      return true;   // handled
    }
    return false;  // not handled
  }
  /* ======================================================================= */
};

StdRNG fast_rng;
SimpleMeshTables tables;

MyMesh the_mesh(board, radio_driver, *new ArduinoMillis(), fast_rng, rtc_clock, tables);

void halt() {
  while (1) ;
}

static char command[160];

#if defined(NRF52_PLATFORM)
static const uint32_t NRF52_SENSOR_SLEEP_SECS = 2UL * 60UL * 60UL;
static unsigned long sleep_until_ms = 0;
static unsigned long tx_wait_started_ms = 0;
enum Nrf52SensorCycleState : uint8_t { CYCLE_SEND, CYCLE_WAIT_TX, CYCLE_SLEEP };
static Nrf52SensorCycleState nrf52_cycle_state = CYCLE_SEND;
#endif

void setup() {
  Serial.begin(115200);
  delay(1000);

  board.begin();

#ifdef DISPLAY_CLASS
  if (display.begin()) {
    display.startFrame();
    display.print("Please wait...");
    display.endFrame();
  }
#endif

  if (!radio_init()) { halt(); }

  fast_rng.begin(radio_get_rng_seed());

  FILESYSTEM* fs;
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  InternalFS.begin();
  fs = &InternalFS;
  IdentityStore store(InternalFS, "");
#elif defined(ESP32)
  SPIFFS.begin(true);
  fs = &SPIFFS;
  IdentityStore store(SPIFFS, "/identity");
#elif defined(RP2040_PLATFORM)
  LittleFS.begin();
  fs = &LittleFS;
  IdentityStore store(LittleFS, "/identity");
  store.begin();
#else
  #error "need to define filesystem"
#endif
  if (!store.load("_main", the_mesh.self_id)) {
    MESH_DEBUG_PRINTLN("Generating new keypair");
    the_mesh.self_id = radio_new_identity();   // create new random identity
    int count = 0;
    while (count < 10 && (the_mesh.self_id.pub_key[0] == 0x00 || the_mesh.self_id.pub_key[0] == 0xFF)) {  // reserved id hashes
      the_mesh.self_id = radio_new_identity(); count++;
    }
    store.save("_main", the_mesh.self_id);
  }

  Serial.print("Sensor ID: ");
  mesh::Utils::printHex(Serial, the_mesh.self_id.pub_key, PUB_KEY_SIZE); Serial.println();

  command[0] = 0;

  sensors.begin();

  the_mesh.begin(fs);

#ifdef DISPLAY_CLASS
  ui_task.begin(the_mesh.getNodePrefs(), FIRMWARE_BUILD_DATE, FIRMWARE_VERSION);
#endif

  // send out initial zero hop Advertisement to the mesh
#if ENABLE_ADVERT_ON_BOOT == 1
  the_mesh.sendSelfAdvertisement(16000, false);
#endif

#if defined(NRF52_PLATFORM)
  // nRF52 sensor-only autonomous cycle:
  // wake/boot -> read sensors -> send #telemetry -> radio off -> sleep 2h -> reboot.
  the_mesh.sendTelemetryToChannelNow();
  tx_wait_started_ms = millis();
  nrf52_cycle_state = CYCLE_WAIT_TX;
#endif
}

void loop() {
  int len = strlen(command);
  while (Serial.available() && len < sizeof(command)-1) {
    char c = Serial.read();
    if (c != '\n') {
      command[len++] = c;
      command[len] = 0;
    }
    Serial.print(c);
  }
  if (len == sizeof(command)-1) {  // command buffer full
    command[sizeof(command)-1] = '\r';
  }

  if (len > 0 && command[len - 1] == '\r') {  // received complete line
    command[len - 1] = 0;  // replace newline with C string null terminator
    char reply[160];
    the_mesh.handleCommand(0, command, reply);  // NOTE: there is no sender_timestamp via serial!
    if (reply[0]) {
      Serial.print("  -> "); Serial.println(reply);
    }

    command[0] = 0;  // reset command buffer
  }

#if defined(NRF52_PLATFORM)
  the_mesh.loop();
  sensors.loop();
  rtc_clock.tick();

  if (nrf52_cycle_state == CYCLE_WAIT_TX) {
    // Let queued packets flush before sleeping.
    if (!the_mesh.hasPendingTxWork() && (unsigned long)(millis() - tx_wait_started_ms) > 4000UL) {
      radio_driver.powerOff();
      sleep_until_ms = millis() + (NRF52_SENSOR_SLEEP_SECS * 1000UL);
      nrf52_cycle_state = CYCLE_SLEEP;
      MESH_DEBUG_PRINTLN("Telemetry sent, entering low-power sleep for 2h");
    }
  } else if (nrf52_cycle_state == CYCLE_SLEEP) {
    if ((long)(millis() - sleep_until_ms) >= 0) {
      NVIC_SystemReset();
    }
    board.sleep(1);
  }
  return;
#endif

  the_mesh.loop();
  sensors.loop();
#ifdef DISPLAY_CLASS
  ui_task.loop();
#endif
  rtc_clock.tick();
}
