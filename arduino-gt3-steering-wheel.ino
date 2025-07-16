#include <Arduino.h>
#include <BleGamepad.h>

// Buttons
#define PIN_BUTTON_ARRAY_ROW \
  (int[4]) {32, 33, 25, 26}
#define PIN_BUTTON_ARRAY_COL \
  (int[3]) {14, 12, 13}

int buttonRowSize = sizeof(PIN_BUTTON_ARRAY_ROW) / sizeof(PIN_BUTTON_ARRAY_ROW[0]);
int buttonColSize = sizeof(PIN_BUTTON_ARRAY_COL) / sizeof(PIN_BUTTON_ARRAY_COL[0]);

// Encoders
#define ENCODER_DELAY 50
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
#define BATTERY_ADC_MAX 4095.0f
#define BATTERY_DIVIDER_RATIO ((100.0f + 100.0f) / 100.0f) // 2 resistores de 100k em série, então a tensão é dividida por 2
#define BATTERY_AVG_AMOUNT 10
#define BATTERY_READ_INTERVAL 1000

unsigned long lastBatteryRead = 0;
unsigned long lastBatteryReport = 0;
float batterySum = 0.0f;
int batterySamples = 0;
bool batteryFirstReport = true;

// Gamepad
#define NUMBER_OF_BUTTONS 18

byte physicalButtons[NUMBER_OF_BUTTONS] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18};
bool statusChanged = false;

BleGamepad bleGamepad("GT3 Steering Wheel", "odevsa", 100);

void setButton(int buttonIndex, bool pressed)
{
  if(!pressed)
  {
    bleGamepad.release(physicalButtons[buttonIndex]);
    statusChanged = true;
    return;
  }

  bleGamepad.press(physicalButtons[buttonIndex]);
  statusChanged = true;
}

int loadEncoder(int pinClock, int pinData, int previousEncoderClock)
{
  int encoderClock = digitalRead(pinClock);
  if (encoderClock == previousEncoderClock) return 0;

  return encoderClock ^ digitalRead(pinData) ? -1 : 1;
}

void loadButtons()
{
  int buttonCurrentIndex = 0;
  
  for (int row = 0; row < buttonRowSize; row++)
  {
    digitalWrite(PIN_BUTTON_ARRAY_ROW[row], LOW);

    for (int col = 0; col < buttonColSize; col++)
      setButton(buttonCurrentIndex++, digitalRead(PIN_BUTTON_ARRAY_COL[col]) == LOW);

    digitalWrite(PIN_BUTTON_ARRAY_ROW[row], HIGH);
  }

  int leftEncoder = loadEncoder(PIN_LEFT_ENCODER_CLOCK, PIN_LEFT_ENCODER_DATA, previousLeftEncoderClock);
  setButton(buttonCurrentIndex++, leftEncoder == -1);
  setButton(buttonCurrentIndex++, leftEncoder == 1);
  setButton(buttonCurrentIndex++, digitalRead(PIN_LEFT_ENCODER_SWITCH) == LOW);
  previousLeftEncoderClock = digitalRead(PIN_LEFT_ENCODER_CLOCK);

  int rightEncoder = loadEncoder(PIN_RIGHT_ENCODER_CLOCK, PIN_RIGHT_ENCODER_DATA, previousRightEncoderClock);
  setButton(buttonCurrentIndex++, rightEncoder == -1);
  setButton(buttonCurrentIndex++, rightEncoder == 1);
  setButton(buttonCurrentIndex++, digitalRead(PIN_RIGHT_ENCODER_SWITCH) == LOW);
  previousRightEncoderClock = digitalRead(PIN_RIGHT_ENCODER_CLOCK);

  if(leftEncoder + rightEncoder != 0) delay(ENCODER_DELAY);
}

void loadBatteryLevel() {
  if (millis() - lastBatteryRead < BATTERY_READ_INTERVAL) return;
  lastBatteryRead = millis();

  int adcValue = analogRead(PIN_BATTERY_LEVEL);

  float vOut = (adcValue / BATTERY_ADC_MAX) * 3.3f; // ESP32 ADC referencia 3.3V
  float vBat = vOut * BATTERY_DIVIDER_RATIO;
  float percent = (vBat - BATTERY_VOLTAGE_MIN) / (BATTERY_VOLTAGE_MAX - BATTERY_VOLTAGE_MIN) * 100.0f;
  
  if (percent > 100.0f) percent = 100.0f;
  if (percent < 0.0f) percent = 0.0f;
  
  if (batteryFirstReport) {
    batteryFirstReport = false;
    bleGamepad.setBatteryLevel(percent);
    return;
  }

  batterySum += percent;
  batterySamples++;

  if (batterySamples >= BATTERY_AVG_AMOUNT) {
    float avgPercent = batterySum / batterySamples;
    bleGamepad.setBatteryLevel(avgPercent);
    batterySum = 0.0f;
    batterySamples = 0;
  }
}

void sendReport()
{
  if (!statusChanged) return;

  bleGamepad.sendReport();
  statusChanged = false;
}

void setup()
{
  Serial.begin(9600);

  for (int i = 0; i < buttonRowSize; i++)
    pinMode(PIN_BUTTON_ARRAY_ROW[i], OUTPUT),
    digitalWrite(PIN_BUTTON_ARRAY_ROW[i], HIGH);

  for (int i = 0; i < buttonColSize; i++)
    pinMode(PIN_BUTTON_ARRAY_COL[i], INPUT_PULLUP);

  pinMode(PIN_LEFT_ENCODER_DATA, INPUT_PULLUP);
  pinMode(PIN_LEFT_ENCODER_CLOCK, INPUT_PULLUP);
  pinMode(PIN_LEFT_ENCODER_SWITCH, INPUT_PULLUP);
  previousLeftEncoderClock = digitalRead(PIN_LEFT_ENCODER_CLOCK);

  pinMode(PIN_RIGHT_ENCODER_DATA, INPUT_PULLUP);
  pinMode(PIN_RIGHT_ENCODER_CLOCK, INPUT_PULLUP);
  pinMode(PIN_RIGHT_ENCODER_SWITCH, INPUT_PULLUP);
  previousRightEncoderClock = digitalRead(PIN_RIGHT_ENCODER_CLOCK);

  pinMode(PIN_BATTERY_LEVEL, INPUT);

  BleGamepadConfiguration bleGamepadConfig;
  bleGamepadConfig.setAutoReport(false);
  bleGamepadConfig.setButtonCount(NUMBER_OF_BUTTONS);
  bleGamepadConfig.setWhichAxes(false, false, false, false, false, false, false, false);
  bleGamepadConfig.setWhichSimulationControls(false, false, false, false, false);
  bleGamepad.begin(&bleGamepadConfig);
}

void loop()
{
  loadButtons();
  loadBatteryLevel();
  sendReport();
}