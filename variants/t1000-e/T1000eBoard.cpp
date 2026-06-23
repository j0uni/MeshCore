#include <Arduino.h>
#include <Wire.h>

#include "T1000eBoard.h"
#include "variant.h"
#include <helpers/radiolib/CustomLR1110Wrapper.h>
#include <nrf.h>
#include <nrf_soc.h>

extern WRAPPER_CLASS radio_driver;

#ifndef USER_BTN_PRESSED
#define USER_BTN_PRESSED HIGH
#endif

void T1000eBoard::powerOff() {
  /* Same order as companion_radio UITask::shutdown(): radio sleep, then rails/GPIO. */
  radio_driver.powerOff();

#ifdef PIN_BUZZER
  noTone(PIN_BUZZER);
#endif

#ifdef HAS_GPS
  digitalWrite(GPS_VRTC_EN, LOW);
  digitalWrite(GPS_RESET, LOW);
  digitalWrite(GPS_SLEEP_INT, LOW);
  digitalWrite(GPS_RTC_INT, LOW);
  digitalWrite(GPS_EN, LOW);
#endif

#ifdef BUZZER_EN
  digitalWrite(BUZZER_EN, LOW);
#endif

#ifdef PIN_3V3_EN
  digitalWrite(PIN_3V3_EN, LOW);
#endif

#ifdef PIN_3V3_ACC_EN
  digitalWrite(PIN_3V3_ACC_EN, LOW);
#endif
#ifdef SENSOR_EN
  digitalWrite(SENSOR_EN, LOW);
#endif

#ifdef LED_PIN
  digitalWrite(LED_PIN, HIGH);
#endif
#ifdef BUTTON_PIN
  while (digitalRead(BUTTON_PIN) == USER_BTN_PRESSED) {}
#endif
#ifdef LED_PIN
  digitalWrite(LED_PIN, LOW);
#endif

#ifdef BUTTON_PIN
  /* Align with terminaattori / Seeed T1000-E: stable idle before SENSE so DETECT is not latched high. */
  if (USER_BTN_PRESSED == HIGH) {
    pinMode(BUTTON_PIN, INPUT_PULLDOWN);
  } else {
    pinMode(BUTTON_PIN, INPUT_PULLUP);
  }
  delay(100);
  if (digitalRead(BUTTON_PIN) == USER_BTN_PRESSED) {
    delay(500);
  }

  const uint32_t btn_nrf = g_ADigitalPinMap[BUTTON_PIN];
#if defined(NRF_GPIO_LATCH_PRESENT)
  nrf_gpio_pin_latch_clear(btn_nrf);
#endif
  if (USER_BTN_PRESSED == HIGH) {
    nrf_gpio_cfg_sense_input(btn_nrf, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_HIGH);
  } else {
    nrf_gpio_cfg_sense_input(btn_nrf, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
  }
  delay(10);

  /* Clear stale reset reason bits (terminaattori); wake from SYSTEMOFF sets OFF in RESETREAS. */
  NRF_POWER->RESETREAS = POWER_RESETREAS_OFF_Msk;
#endif

  sd_power_system_off();
  /* SoftDevice may return with debugger/USB; direct SYSTEMOFF matches terminaattori + Nordic fallback. */
  NRF_POWER->SYSTEMOFF = 1;
  __WFE();
}

void T1000eBoard::begin() {
  NRF52BoardDCDC::begin();
  btn_prev_state = HIGH;

#ifdef BUTTON_PIN
  pinMode(BATTERY_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
#endif

#if defined(PIN_BOARD_SDA) && defined(PIN_BOARD_SCL)
  Wire.setPins(PIN_BOARD_SDA, PIN_BOARD_SCL);
#endif

  Wire.begin();

  delay(10);   // give sx1262 some time to power up
}