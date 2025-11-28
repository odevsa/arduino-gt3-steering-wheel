# GT3 Steering Wheel (bluetooth)

![Cover](assets/cover.jpg "Cover")

This code will allow to make a bluetooth GT3 steering wheel.

# Features

- [x] Bluetooth
- [x] Rechargeble Battery
- [x] 10 Buttons
- [x] 2 Peddle shifters
- [x] 2 Encoders with buttons

# Requirements

- 1x ESP32 With Bluetooth

  ![ESP32 with BLuetooth](assets/esp32.jpg "ESP32 with BLuetooth")

- 1x Charger Module TP4056 USB-C

  ![TP4056 USB-C](assets/tp4056.jpg "TP4056 USB-C")

- 1x DC Step Up Module MT3608

  ![MT3608](assets/mt3608.jpg "MT3608")

- 1x 3.7v Rechargeable Lithium Battery

  ![Rechargeable Lithium Battery](assets/battery.jpg "Rechargeable Lithium Battery")

- 1x Switch On/Off

  ![Switch On/Off](assets/switch-onoff.jpg "Switch On/Off")

- 1x LED 3.3v

  ![LED](assets/led.jpg "LED")

- 10X 12mm Buttons

  ![12mm Buttons](assets/buttons.jpg "12mm Buttons")

- 2x Encoder with Button EC11

  ![Encoder with Button EC11](assets/ec11.jpg "Encoder with Button EC11")

- 2x Micro Switchs

  ![Micro Switch](assets/micro-switch.jpg "Micro Switch")

- 1x Resistors 220Ω

  ![Resistor](assets/resistor-220ohm.jpg "Resistor")

- 2x Resistors 100kΩ

  ![Resistor](assets/resistor-100kohm.jpg "Resistor")

- 2x 8x3mm Magnets

  ![Magnets](assets/magnets.jpg "Magnets")


# Dependencies

You will need install in your Arduino the library below:

- ESP32-BLE-Gamepad
- NimBLE-Arduino

# Wiring

This is just a wiring suggestion, you can customize and change in pin definitions.

If you are going to use this wiring suggestion it will just be `plug-n-play` with the current code.

![Wiring](assets/wiring.svg "Wiring")

Example
![Face PLate](assets/face-plate.jpg "Face PLate")
![Shell](assets/shell.jpg "Shell")

# Battery Reading Calibration (ADC Vref)

The ESP32's internal voltage reference (Vref) is often inaccurate, causing a calculated voltage (V_CALC) mismatch with the actual battery voltage (V_REAL).

Uncomment for debuging

```
// Serial.println("Calculated Voltage: " + String(batteryVoltage, 3));
```

To calibrate:

1. Measure the real battery voltage (V_REAL) with a multimeter (e.g., 4.08V).
2. Note the calculated voltage (V_CALC) from the Serial Monitor (e.g., 3.79V).
3. **Update the Vref in the formula below**: V_REF_NEW = (V_REAL / V_CALC) \* 3.3f.

Example:

```
(4.08 / 3.79) * 3.3 = 3.55
```

Finally update this constant with your calculated V_REF_NEW value.

```
#define BATTERY_ADC_REFERENCE 3.55f
```

# Firmware

Just change pin values if you have used a different wiring, select the board `ESP32 Dev Module` and upload the firmware.

# 3D Print Parts

I did a remix from another remix from Formula B1.

I needed to remix it to add:

- Hole for switch on/off
- Bed for charger module
- Holes resized for encoders
- Holes for LEDs

**My Remix:**
https://www.printables.com/model/1306908-formula-b1-remix-esp32-bluetooth

**Remixed from:**
https://www.printables.com/model/74032-formula-b1-remix-arduino-pro-micro

**Original Model:**
https://www.thingiverse.com/thing:4058270
