#pragma once

// HX711 load-cell amplifier
constexpr int PIN_HX711_DOUT = 18;
constexpr int PIN_HX711_SCK = 19;

// Steppers (A4988/DRV8825).
constexpr int PIN_LID_STEP = 25;
constexpr int PIN_LID_DIR = 26;
constexpr int PIN_ARM_STEP = 32;
constexpr int PIN_ARM_DIR = 33;
// Spool (reels the arm back in). DIR must not share GPIO13 with PIN_DRV_EN:
// setting the direction would switch every driver on or off.
// GPIO14 outputs a PWM burst during boot, which is harmless on DIR (it would
// jog the spool on STEP). GPIO12 is a boot-strapping pin: if the driver ever
// holds STEP high during reset the ESP won't boot, so move STEP if that happens.
constexpr int PIN_SPOOL_STEP = 12;
constexpr int PIN_SPOOL_DIR = 14;
constexpr int PIN_DRV_EN = 13; // shared by all three drivers, active LOW: HIGH = drivers disabled

// Inputs / outputs
constexpr int PIN_STATUS_LED = 21;
constexpr int PIN_BUZZER = 22;
