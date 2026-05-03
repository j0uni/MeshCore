#include <Arduino.h>   // needed for PlatformIO
#include <Mesh.h>

#include "MyMesh.h"

#if defined(T1000_E)
#ifndef T1000E_REPEATER_BUILD
#error "T1000-E simple_repeater must use env:t1000e_repeater. Set SECRET_GNSS_CHANNEL_KEY_HEX or .secret_gnss_channel_key (see variants/t1000-e/t1000e_repeater_gnss_channel_key.py)."
#endif
#endif

#ifdef DISPLAY_CLASS
  #include "UITask.h"
  static UITask ui_task(display);
#endif

StdRNG fast_rng;
SimpleMeshTables tables;

MyMesh the_mesh(board, radio_driver, *new ArduinoMillis(), fast_rng, rtc_clock, tables);

void halt() {
  while (1) ;
}

static char command[160];

// For power saving
unsigned long lastActive = 0; // mark last active time
unsigned long nextSleepinSecs = 120; // next sleep in seconds. The first sleep (if enabled) is after 2 minutes from boot

#if defined(PIN_USER_BTN) && defined(_SEEED_SENSECAP_SOLAR_H_)
static unsigned long userBtnDownAt = 0;
#define USER_BTN_HOLD_OFF_MILLIS 1500
#endif

#if defined(T1000_E) && defined(PIN_USER_BTN) && defined(PIN_BUZZER)
#ifndef USER_BTN_PRESSED
#define USER_BTN_PRESSED LOW
#endif

#ifndef T1000E_USER_BTN_HOLD_POWEROFF_MS
#define T1000E_USER_BTN_HOLD_POWEROFF_MS 1500ul
#endif
#ifndef T1000E_USER_BTN_SHORT_PRESS_MAX_MS
#define T1000E_USER_BTN_SHORT_PRESS_MAX_MS 900ul
#endif

static void t1000ePlayGnssRising(int pin) {
#ifdef PIN_BUZZER_EN
  digitalWrite(PIN_BUZZER_EN, HIGH);
#endif
  const int notes[] = { 523, 659, 784, 1047 };
  for (size_t i = 0; i < sizeof(notes) / sizeof(notes[0]); i++) {
    tone(pin, notes[i], 100);
    delay(110);
  }
  noTone(pin);
}

static void t1000ePlayGnssFalling(int pin) {
#ifdef PIN_BUZZER_EN
  digitalWrite(PIN_BUZZER_EN, HIGH);
#endif
  const int notes[] = { 1047, 784, 659, 523 };
  for (size_t i = 0; i < sizeof(notes) / sizeof(notes[0]); i++) {
    tone(pin, notes[i], 100);
    delay(110);
  }
  noTone(pin);
}

static int t1000e_btn_prev = -1;
static unsigned long t1000e_btn_down_at = 0;

static void t1000ePollUserBtnGnssToggle() {
  unsigned long now = millis();
  int st = digitalRead(PIN_USER_BTN);
  if (t1000e_btn_prev < 0) {
    t1000e_btn_prev = st;
    return;
  }
  if (st != t1000e_btn_prev) {
    if (st == USER_BTN_PRESSED) {
      t1000e_btn_down_at = now;
    } else {
      if (t1000e_btn_down_at != 0) {
        unsigned long dur = now - t1000e_btn_down_at;
        if (dur >= 40 && dur < T1000E_USER_BTN_SHORT_PRESS_MAX_MS) {
          bool on_or_pending = sensors.isGnssPowered() || sensors.isGnssUserInitPending();
          if (on_or_pending) {
            t1000ePlayGnssFalling(PIN_BUZZER);
            sensors.userGnssOff();
          } else {
            t1000ePlayGnssRising(PIN_BUZZER);
            sensors.userGnssOnWithNmeaEcho();
          }
        }
        t1000e_btn_down_at = 0;
      }
    }
    t1000e_btn_prev = st;
  }
  /* Hold past GNSS short-press window: same handler as SenseCAP long-press (board.powerOff). */
  if (st == USER_BTN_PRESSED && t1000e_btn_down_at != 0 &&
      (unsigned long)(now - t1000e_btn_down_at) >= T1000E_USER_BTN_HOLD_POWEROFF_MS) {
    Serial.println(F("Powering off..."));
    board.powerOff();
  }
}
#endif

void setup() {
  Serial.begin(115200);
  delay(1000);

  board.begin();

#if defined(MESH_DEBUG) && defined(NRF52_PLATFORM)
  // give some extra time for serial to settle so
  // boot debug messages can be seen on terminal
  delay(5000);
#endif

  // For power saving
  lastActive = millis(); // mark last active time since boot

#ifdef DISPLAY_CLASS
  if (display.begin()) {
    display.startFrame();
    display.setCursor(0, 0);
    display.print("Please wait...");
    display.endFrame();
  }
#endif

  if (!radio_init()) {
    MESH_DEBUG_PRINTLN("Radio init failed!");
    halt();
  }

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

  Serial.print("Repeater ID: ");
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
}

void loop() {
  int len = strlen(command);
  while (Serial.available() && len < sizeof(command)-1) {
    char c = Serial.read();
    if (c != '\n') {
      command[len++] = c;
      command[len] = 0;
      Serial.print(c);
    }
    if (c == '\r') break;
  }
  if (len == sizeof(command)-1) {  // command buffer full
    command[sizeof(command)-1] = '\r';
  }

  if (len > 0 && command[len - 1] == '\r') {  // received complete line
    Serial.print('\n');
    command[len - 1] = 0;  // replace newline with C string null terminator
    char reply[160];
    the_mesh.handleCommand(0, command, reply);  // NOTE: there is no sender_timestamp via serial!
    if (reply[0]) {
      Serial.print("  -> "); Serial.println(reply);
    }

    command[0] = 0;  // reset command buffer
  }

#if defined(PIN_USER_BTN) && defined(_SEEED_SENSECAP_SOLAR_H_)
  // Hold the user button to power off the SenseCAP Solar repeater.
  int btnState = digitalRead(PIN_USER_BTN);
  if (btnState == LOW) {
    if (userBtnDownAt == 0) {
      userBtnDownAt = millis();
    } else if ((unsigned long)(millis() - userBtnDownAt) >= USER_BTN_HOLD_OFF_MILLIS) {
      Serial.println("Powering off...");
      board.powerOff();  // does not return
    }
  } else {
    userBtnDownAt = 0;
  }
#endif

  the_mesh.loop();
  sensors.loop();
#if defined(T1000E_REPEATER_BUILD)
  if (sensors.consumeGnssTwoHourSessionAutoOff()) {
    the_mesh.onGnssTwoHourSessionAutoOff();
  }
  the_mesh.sendSecretGnssMeshMessageIfPending();
#endif
#if defined(T1000_E) && defined(PIN_USER_BTN) && defined(PIN_BUZZER)
  t1000ePollUserBtnGnssToggle();
#endif
#ifdef DISPLAY_CLASS
  ui_task.loop();
#endif
  rtc_clock.tick();

  if (the_mesh.getNodePrefs()->powersaving_enabled && !the_mesh.hasPendingWork()) {
    #if defined(NRF52_PLATFORM)
    board.sleep(1800); // nrf ignores seconds param, sleeps whenever possible
    #else
    if (the_mesh.millisHasNowPassed(lastActive + nextSleepinSecs * 1000)) { // To check if it is time to sleep
      board.sleep(1800);             // To sleep. Wake up after 30 minutes or when receiving a LoRa packet
      lastActive = millis();
      nextSleepinSecs = 5;  // Default: To work for 5s and sleep again
    } else {
      nextSleepinSecs += 5; // When there is pending work, to work another 5s
    }
    #endif
  }
}
