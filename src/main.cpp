#include <Arduino.h>
#include <Servo.h>
#include <FastLED.h>
#include <CRSFforArduino.hpp>
#include <math.h>

float x = INFINITY;
float y = -INFINITY;

extern "C" {
#include "hardware/watchdog.h"
}

// -------------------- Pins --------------------
constexpr uint8_t MOTOR_LEFT_PIN = 3;
constexpr uint8_t MOTOR_RIGHT_PIN = 4;
constexpr uint8_t MAGNET_ENABLE_PIN = 2;
constexpr uint8_t LED_PIN = 27;

// -------------------- LEDs --------------------
constexpr uint8_t LOGICAL_NUM_LEDS = 36;  // note this is double the real number so the blur works
constexpr uint8_t LED_BRIGHTNESS = 255;

CRGB leds[LOGICAL_NUM_LEDS];

// -------------------- Receiver --------------------
CRSFforArduino crsf = CRSFforArduino(&Serial1);

constexpr uint8_t YAW_CH = 1;
constexpr uint8_t FWD_CH = 2;
constexpr uint8_t MAGNET_CH = 3;
constexpr uint8_t TRACTC_CH = 8;
constexpr uint8_t FLIP_CH = 5;

bool link_active = false;

// -------------------- ESCs --------------------
Servo motorLeft;
Servo motorRight;

constexpr int ESC_MIN_US = 1000;
constexpr int ESC_NEUTRAL_US = 1500;
constexpr int ESC_MAX_US = 2000;

constexpr int LEFT_MOTOR_DIRECTION = -1;
constexpr int RIGHT_MOTOR_DIRECTION = 1;

// -------------------- Safety --------------------
unsigned long stopflag_time = 0;

constexpr int E_STOP_TIME_MS = 100;
constexpr int WATCHDOG_TIME_MS = 1000;

bool watchdog_enabled = false;

// -------------------- Control values --------------------
float yaw = 0.0f;
float forward = 0.0f;
float magnet = 0.0f;
constexpr float max_turn = 0.3;

// accel time sets max accel from 0 speed to max speed forwards
constexpr float accel_time_slow = 0.6;
constexpr float accel_time_med = 0.3;
constexpr float accel_rate_slow = 1.0f / accel_time_slow;  // stores portions of 0-1 per second
constexpr float accel_rate_med = 1.0f / accel_time_med;
constexpr float accel_rate_fast = INFINITY;
float accel_rate = accel_rate_fast;
bool flip = false;

// -------------------- LED animation --------------------
float ledPos = 0.0f;

// -------------------- Helpers --------------------
float servoToFloat(float servoSignal, bool centred) {
  constexpr int deadzone = 30;

  if (centred) {
    if (servoSignal >= 1500 - deadzone &&
        servoSignal <= 1500 + deadzone) {
      servoSignal = 1500;
    }

    return constrain((servoSignal - 1500.0f) / 500.0f, -1.0f, 1.0f);
  }

  servoSignal = (servoSignal <= 1000 + deadzone) ? 1000 : servoSignal;

  return constrain((servoSignal - 1000.0f) / 1000.0f, 0.0f, 1.0f);
}

float powerCurve(float value, float power) {
  float sign = (value >= 0.0f) ? 1.0f : -1.0f;

  return sign * powf(fabs(value), power);
}

int floatToServoUs(float value) {
  value = constrain(value, -1.0f, 1.0f);
  return ESC_NEUTRAL_US + int(value * 500.0f);
}

bool linkHealthy() {
  if (!link_active) {
    return false;
  }

  return millis() - stopflag_time <= E_STOP_TIME_MS;
}

// -------------------- CRSF Callback --------------------
void onLinkStatisticsUpdate(
    serialReceiverLayer::link_statistics_t linkStatistics) {
  if (linkStatistics.lqi > 10) {
    stopflag_time = millis();
    link_active = true;
  }
}

// -------------------- Receiver Update --------------------
void updateCRSF() {
  crsf.update();

  yaw = powerCurve(servoToFloat(crsf.rcToUs(crsf.getChannel(YAW_CH)), true), 3) * max_turn;

  forward = powerCurve(servoToFloat(crsf.rcToUs(crsf.getChannel(FWD_CH)), true), 3);

  magnet = servoToFloat(crsf.rcToUs(crsf.getChannel(MAGNET_CH)), false);

  flip = crsf.rcToUs(crsf.getChannel(FLIP_CH)) < 1500;  // flip direction of motors if enable switch is flipped

  int tractc_switch = crsf.rcToUs(crsf.getChannel(TRACTC_CH));

  if (tractc_switch < 1300) {
    accel_rate = accel_rate_slow;
  } else if (tractc_switch < 1700) {
    accel_rate = accel_rate_med;
  } else {
    accel_rate = accel_rate_fast;
  }
}

// -------------------- Watchdog --------------------
void updateWatchdog() {
  unsigned long now = millis();

  if (linkHealthy()) {
    if (watchdog_enabled) {
      watchdog_update();

    } else if (now > E_STOP_TIME_MS) {
      watchdog_enable(WATCHDOG_TIME_MS, 0);
      watchdog_enabled = true;
      watchdog_update();
    }
  }
}

// -------------------- Motor + Magnet Control --------------------
void commandOutputs() {
  static float forward_now = 0;

  // traction control
  static uint32_t then = 0;
  uint32_t now = micros();
  uint32_t time_step = now - then;
  then = now;
  float max_throt_step = time_step * accel_rate / 1000000;

  if (forward_now < 0) {
    forward_now = forward;
  } else {
    forward_now = min(forward, forward_now + max_throt_step);  // intentionally only limits positive forward acceleration
  }

  // flip direction if commanded
  float commanded_forward = flip ? -forward_now : forward_now;

  float left = commanded_forward + yaw;
  float right = commanded_forward - yaw;

  // prevent saturation
  float maxMag = max(fabs(left), fabs(right));

  // preserves ratio between left and right while clipping
  if (maxMag > 1.0f) {
    left /= maxMag;
    right /= maxMag;
  }

  // failsafe
  if (!linkHealthy()) {
    left = 0.0f;
    right = 0.0f;
    magnet = 0.0f;
  }

  motorLeft.writeMicroseconds(
      floatToServoUs(left * LEFT_MOTOR_DIRECTION));

  motorRight.writeMicroseconds(
      floatToServoUs(right * RIGHT_MOTOR_DIRECTION));

  //   Serial.println(floatToServoUs(left * LEFT_MOTOR_DIRECTION));
  analogWrite(
      MAGNET_ENABLE_PIN,
      int(constrain(magnet, 0.0f, 1.0f) * 255.0f));
}

// -------------------- LED Animation --------------------
void updateLinkLostLEDs() {
  unsigned long t = millis() % 1500;  // loop time

  bool pulseOn = (t < 200) || (t >= 500 && t < 700);  // flash flash

  if (pulseOn) {
    fill_solid(leds, LOGICAL_NUM_LEDS, CRGB::Green);
  } else {
    fadeToBlackBy(leds, LOGICAL_NUM_LEDS, 2);
  }

  FastLED.show();
}

void updateLEDs() {
  if (!linkHealthy()) {
    updateLinkLostLEDs();
    return;
  }

  static unsigned long lastUpdate = 0;

  unsigned long now = millis();

  float dt = (now - lastUpdate) / 1000.0f;

  lastUpdate = now;

  // speed influences animation speed
  float speedAmount = fabs(forward);

  // idle = about 1 sweep/sec
  // moving = faster
  float ledsPerSecond = LOGICAL_NUM_LEDS * (0.4f + (speedAmount * 4.0f));  // 0.5 gives one per second as code thinks strip is 18 long

  float direction = (forward >= 0.0f) ? 1.0f : -1.0f;

  ledPos += direction * ledsPerSecond * dt;

  while (ledPos >= LOGICAL_NUM_LEDS) ledPos -= LOGICAL_NUM_LEDS;
  while (ledPos < 0) ledPos += LOGICAL_NUM_LEDS;

  // white -> red with magnet strength
  CRGB colour = blend(
      CRGB::White,
      CRGB::Red,
      uint8_t(magnet * 255.0f));

  // two dots, 18 LEDs apart in virtual space
  int posA = int(ledPos) % LOGICAL_NUM_LEDS;
  int posB = int(ledPos + LOGICAL_NUM_LEDS / 2) % LOGICAL_NUM_LEDS;

  leds[posA] = colour;
  leds[posB] = colour;

  // create blur trail
  blur1d(leds, LOGICAL_NUM_LEDS, 100);

  // fade old pixels
  fadeToBlackBy(leds, LOGICAL_NUM_LEDS, 20);

  FastLED.show();
}

// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200);

  delay(1000);

  Serial.println("Sandfly starting...");

  // receiver
  if (!crsf.begin()) {
    Serial.println("CRSF initialisation failed");

    while (true) {
      delay(10);
    }
  }

  crsf.setLinkStatisticsCallback(
      onLinkStatisticsUpdate);

  // motors
  motorLeft.attach(MOTOR_LEFT_PIN, ESC_MIN_US, ESC_MAX_US);

  motorRight.attach(MOTOR_RIGHT_PIN, ESC_MIN_US, ESC_MAX_US);

  motorLeft.writeMicroseconds(ESC_NEUTRAL_US);
  motorRight.writeMicroseconds(ESC_NEUTRAL_US);

  // magnet
  pinMode(MAGNET_ENABLE_PIN, OUTPUT);
  analogWrite(MAGNET_ENABLE_PIN, 0);

  // LEDs
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, LOGICAL_NUM_LEDS);

  FastLED.setBrightness(LED_BRIGHTNESS);

  FastLED.clear();
  FastLED.show();

  // ESC arm delay
  delay(2000);

  Serial.println("Sandfly ready");
}

// -------------------- Main Loop --------------------
void loop() {
  updateCRSF();

  updateWatchdog();

  commandOutputs();
}

void loop1() {
  updateLEDs();
}