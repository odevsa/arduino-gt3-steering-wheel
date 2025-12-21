#include <Arduino.h>
#include <BleGamepad.h>
#include "driver/rtc_io.h"
#include "esp32-hal-cpu.h"

// #define DEBUG

// CPU Frequency and Sleep Timeout
#define CPU_FREQ_MHZ 80
#define SLEEP_TIMEOUT_MS 900000

unsigned long lastActivityTime = 0;

// Buttons Matrix
#define PIN_BUTTON_ARRAY_ROW (int[4]){32, 33, 25, 26}
#define PIN_BUTTON_ARRAY_COL (int[3]){14, 12, 13}

int buttonRowSize =
    sizeof(PIN_BUTTON_ARRAY_ROW) / sizeof(PIN_BUTTON_ARRAY_ROW[0]);
int buttonColSize =
    sizeof(PIN_BUTTON_ARRAY_COL) / sizeof(PIN_BUTTON_ARRAY_COL[0]);

// Encoders
#define PIN_LEFT_ENCODER_DATA 19
#define PIN_LEFT_ENCODER_CLOCK 18
#define PIN_LEFT_ENCODER_SWITCH 21
#define PIN_RIGHT_ENCODER_DATA 16
#define PIN_RIGHT_ENCODER_CLOCK 17
#define PIN_RIGHT_ENCODER_SWITCH 4

int previousLeftEncoderClock;
int previousRightEncoderClock;

// Battery
#define PIN_BATTERY_LEVEL 34
#define BATTERY_VOLTAGE_MAX 4.2f
#define BATTERY_VOLTAGE_MIN 3.0f
#define BATTERY_ADC_REFERENCE 3.575f
#define BATTERY_ADC_MAX 4095.0f
#define BATTERY_DIVIDER_RATIO ((100.0f + 100.0f) / 100.0f) // 2 resistors of 100k ohms
#define BATTERY_AVG_AMOUNT 10
#define BATTERY_READ_INTERVAL 1000
#define BATTERY_READ_PERCENT_DEVIATION_MAX 5

unsigned long lastBatteryRead = 0;
float lastBatteryPercent = 0.0f;
float batteryPercents[BATTERY_AVG_AMOUNT];
bool batteryFirstReport = true;

// Gamepad
#define NUMBER_OF_BUTTONS 18

byte physicalButtons[NUMBER_OF_BUTTONS] = {1, 2, 3, 4, 5, 6, 7, 8, 9,
                                           10, 11, 12, 13, 14, 15, 16, 17, 18};
bool statusChanged = false;

BleGamepad bleGamepad("GT3 Steering Wheel", "odevsa", 100);

void resetActivityTimer() { lastActivityTime = millis(); }

void setButton(int buttonIndex, bool pressed)
{
  if (!pressed)
  {
    bleGamepad.release(physicalButtons[buttonIndex]);
    statusChanged = true;
    return;
  }

  resetActivityTimer();
  bleGamepad.press(physicalButtons[buttonIndex]);
  statusChanged = true;
}

int loadEncoder(int pinClock, int pinData, int &previousEncoderClock)
{
  int encoderClock = digitalRead(pinClock);
  int encoderData = digitalRead(pinData);

  if (encoderClock == previousEncoderClock)
    return 0;

  previousEncoderClock = encoderClock;
  resetActivityTimer();

  return encoderClock ^ encoderData ? -1 : 1;
}

void loadButtons()
{
  int buttonCurrentIndex = 0;

  for (int row = 0; row < buttonRowSize; row++)
  {
    digitalWrite(PIN_BUTTON_ARRAY_ROW[row], LOW);

    for (int col = 0; col < buttonColSize; col++)
      setButton(buttonCurrentIndex++,
                digitalRead(PIN_BUTTON_ARRAY_COL[col]) == LOW);

    digitalWrite(PIN_BUTTON_ARRAY_ROW[row], HIGH);
  }

  int leftEncoder = loadEncoder(PIN_LEFT_ENCODER_CLOCK, PIN_LEFT_ENCODER_DATA,
                                previousLeftEncoderClock);
  setButton(buttonCurrentIndex++, leftEncoder == -1);
  setButton(buttonCurrentIndex++, leftEncoder == 1);
  setButton(buttonCurrentIndex++, digitalRead(PIN_LEFT_ENCODER_SWITCH) == LOW);

  int rightEncoder =
      loadEncoder(PIN_RIGHT_ENCODER_CLOCK, PIN_RIGHT_ENCODER_DATA,
                  previousRightEncoderClock);
  setButton(buttonCurrentIndex++, rightEncoder == -1);
  setButton(buttonCurrentIndex++, rightEncoder == 1);
  setButton(buttonCurrentIndex++, digitalRead(PIN_RIGHT_ENCODER_SWITCH) == LOW);
}

float loadAverageBatteryPercent()
{
  if (batteryFirstReport)
  {
    for (int i = 0; i < BATTERY_AVG_AMOUNT; i++)
    {
      float percent = loadBatteryPercent();
      delayMicroseconds(100);
      batteryPercents[i] = percent;
    }

    lastBatteryPercent = batteryPercents[BATTERY_AVG_AMOUNT - 1];
    batteryFirstReport = false;
  }

  float percent = loadBatteryPercent();
  float sum = 0.0f;

  if (abs(percent - lastBatteryPercent) < BATTERY_READ_PERCENT_DEVIATION_MAX)
  {
    for (int i = 1; i < BATTERY_AVG_AMOUNT; i++)
    {
      batteryPercents[i - 1] = batteryPercents[i];
      sum += batteryPercents[i - 1];
    }

    batteryPercents[BATTERY_AVG_AMOUNT - 1] = percent;
    sum += batteryPercents[BATTERY_AVG_AMOUNT - 1];
  }

  lastBatteryPercent = percent;
  return sum / BATTERY_AVG_AMOUNT;
}

float loadBatteryPercent()
{
  int adcValue = analogRead(PIN_BATTERY_LEVEL);

  float batteryVoltage = (adcValue / BATTERY_ADC_MAX) * BATTERY_ADC_REFERENCE * BATTERY_DIVIDER_RATIO;
  float percent = (batteryVoltage - BATTERY_VOLTAGE_MIN) / (BATTERY_VOLTAGE_MAX - BATTERY_VOLTAGE_MIN) * 100.0f;

#ifdef DEBUG
  Serial.println("Calculated Voltage: " + String(batteryVoltage, 3));
#endif

  if (percent > 100.0f)
    percent = 100.0f;
  if (percent < 0.0f)
    percent = 0.0f;

  lastBatteryRead = millis();

  return percent;
}

void loadBatteryLevel()
{
  if (millis() - lastActivityTime > SLEEP_TIMEOUT_MS)
    enterDeepSleep();
  if (millis() - lastBatteryRead < BATTERY_READ_INTERVAL)
    return;

  float percent = loadAverageBatteryPercent();

  bleGamepad.setBatteryLevel((int)percent);
  statusChanged = true;

  if (percent <= 0.0f)
  {
    bleGamepad.sendReport();
    delay(500);
    enterDeepSleep();
  }
}

void sendReport()
{
  if (!statusChanged || !bleGamepad.isConnected())
    return;

  bleGamepad.sendReport();
  statusChanged = false;
}

void enterDeepSleep()
{
  for (int i = 0; i < buttonRowSize; i++)
  {
    pinMode(PIN_BUTTON_ARRAY_ROW[i], OUTPUT);
    digitalWrite(PIN_BUTTON_ARRAY_ROW[i], HIGH);
    gpio_hold_en((gpio_num_t)PIN_BUTTON_ARRAY_ROW[i]);
  }

  uint64_t gpio_mask = 0;
  for (int i = 0; i < buttonColSize; i++)
  {
    int pin = PIN_BUTTON_ARRAY_COL[i];

    gpio_mask |= (1ULL << pin);
    rtc_gpio_pulldown_en((gpio_num_t)pin);
    rtc_gpio_pullup_dis((gpio_num_t)pin);
  }

  esp_sleep_enable_ext1_wakeup(gpio_mask, ESP_EXT1_WAKEUP_ANY_HIGH);
  esp_deep_sleep_start();
}

void setup()
{
#ifdef DEBUG
  Serial.begin(115200);
#endif

  // Set CPU Frequency
  setCpuFrequencyMhz(CPU_FREQ_MHZ);

  // Disable GPIO hold if woke up from deep sleep
  if (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_UNDEFINED)
    for (int i = 0; i < buttonRowSize; i++)
      gpio_hold_dis((gpio_num_t)PIN_BUTTON_ARRAY_ROW[i]);

  // Setup Rows of Buttons Matrix
  for (int i = 0; i < buttonRowSize; i++)
    pinMode(PIN_BUTTON_ARRAY_ROW[i], OUTPUT),
        digitalWrite(PIN_BUTTON_ARRAY_ROW[i], HIGH);

  // Setup Columns of Buttons Matrix
  for (int i = 0; i < buttonColSize; i++)
    pinMode(PIN_BUTTON_ARRAY_COL[i], INPUT_PULLUP);

  // Setup Encoders
  pinMode(PIN_LEFT_ENCODER_DATA, INPUT_PULLUP);
  pinMode(PIN_LEFT_ENCODER_CLOCK, INPUT_PULLUP);
  pinMode(PIN_LEFT_ENCODER_SWITCH, INPUT_PULLUP);
  previousLeftEncoderClock = digitalRead(PIN_LEFT_ENCODER_CLOCK);

  pinMode(PIN_RIGHT_ENCODER_DATA, INPUT_PULLUP);
  pinMode(PIN_RIGHT_ENCODER_CLOCK, INPUT_PULLUP);
  pinMode(PIN_RIGHT_ENCODER_SWITCH, INPUT_PULLUP);
  previousRightEncoderClock = digitalRead(PIN_RIGHT_ENCODER_CLOCK);

  // Setup Battery Level Pin
  pinMode(PIN_BATTERY_LEVEL, INPUT);

  // Setup BLE Gamepad
  BleGamepadConfiguration bleGamepadConfig;
  bleGamepadConfig.setAutoReport(false);
  bleGamepadConfig.setButtonCount(NUMBER_OF_BUTTONS);
  bleGamepad.begin(&bleGamepadConfig);

  resetActivityTimer();
}

void loop()
{
  loadButtons();
  loadBatteryLevel();
  sendReport();

  delay(10);
}