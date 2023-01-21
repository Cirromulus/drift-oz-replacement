#pragma once

#include <cmath>
#include <type_traits>

template <class T, std::size_t N>
constexpr std::size_t size(const T (&array)[N]) noexcept
{
    return N;
}

typedef uint8_t PinNumber;
typedef uint8_t JoystickPinNumber;

struct PinRange {
  PinNumber start;
  PinNumber end;
};

static constexpr PinNumber countActivePins(const PinRange buttons[], const size_t len) {
  PinNumber ret = 0;
  for(size_t i = 0; i < len; i++) {
    ret += (buttons[i].end - buttons[i].start) + 1;
  }
  return ret;
}


static constexpr PinNumber BASE_LIGHT_PIN = 24;
static constexpr uint8_t LIGHTING_PIN_COUNT = 4;
static constexpr PinNumber STEERING = A0;
static constexpr PinNumber THROTTLE = A1;
static constexpr PinNumber BRAKE = A2;

static constexpr uint8_t DEADZONE = 15;
static constexpr uint8_t BUTTON_BOUNCE_CYCLES = 5;
static constexpr PinRange buttonList[] = {
  PinRange{22, 23},     // Schaltwippe
  // Hole where LEDs are actuated (sorry)
  PinRange{28, 36},     // Right side of wheel (X,O,cube, R1-R3)
  PinRange{37, 38}      // Hand shifting arm
};
static constexpr uint8_t NUMBER_OF_BUTTONS = countActivePins(buttonList, size(buttonList));

static uint8_t bounce_counter[NUMBER_OF_BUTTONS];

void decrementButtonDebounce() {
  for(auto& bounce : bounce_counter) {
    if(bounce > 0) {
      bounce--;
    }
  }
}

void initButtons() {
  for(auto& count : bounce_counter) {
    count = 0;
  }
  for(const auto& range : buttonList){
    for(auto pin = range.start; pin <= range.end; pin++) {
      pinMode(pin, INPUT_PULLUP);
    }
  }
}

