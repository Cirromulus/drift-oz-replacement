#include <Arduino.h>
#include <Joystick.h>
#include <cmath>
#include <type_traits>

#include "buttons.hpp"


static constexpr uint8_t ANALOGRESOLUTION = 12;
static constexpr decltype(analogRead(1)) ANALOG_MAX_VAL = pow(2, ANALOGRESOLUTION)-1;


static constexpr unsigned JoystickUpdateInterval_ms = 5;
static constexpr unsigned lightingInterval_ms = 250;
static decltype(millis()) previousLightingUpdate = 0;
static decltype(millis()) previousJoystickUpdate = 0;


Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,
  JOYSTICK_TYPE_MULTI_AXIS, NUMBER_OF_BUTTONS,
  4, false, false, false, false, false, false,
  false, true, false, true, true);  // Throttle, Brake and steering

bool checkandsetButton(bool isPressed, JoystickPinNumber pos) {
  // TODO: State memory for supressing redundant updates
  if (bounce_counter[pos] == 0) {
    Joystick.setButton(pos, isPressed);
    bounce_counter[pos] = BUTTON_BOUNCE_CYCLES;
    return true;
  }
  return false;
}

bool updateButtons() {
  bool hasToUpdate = false;
  JoystickPinNumber i = 0;
  for(const auto& range : buttonList){
    for(auto pin = range.start; pin <= range.end; pin++) {
      hasToUpdate |= checkandsetButton(!digitalRead(pin), i);
      i++;
    }
  }
  return hasToUpdate;
}

void careForJoystick() {
  bool hasToUpdate = false;

  if(Serial) {
    const auto tmp = analogRead(STEERING);
    Serial.print("Steering: ");
    Serial.print(tmp);
    Serial.println();
  }

  Joystick.setSteering(analogRead(STEERING));
  Joystick.setThrottle(analogRead(THROTTLE));
  Joystick.setBrake(analogRead(BRAKE));

  hasToUpdate |= updateButtons();

  // Call Joystick.move
  if (hasToUpdate)
    Joystick.sendState();
}

static uint8_t lighting_state = 0;
void careForLighting() {
  for (int i = 0; i < LIGHTING_PIN_COUNT; i++) {
    digitalWrite(BASE_LIGHT_PIN + i, i != lighting_state);
  }
  if(true) { //Joystick.state.xRot.axis < 10){  // TODO: Read State
    //normal
    lighting_state = (lighting_state + 1) % LIGHTING_PIN_COUNT;
  }else{
    //braking
    lighting_state = lighting_state == 0 ? LIGHTING_PIN_COUNT : lighting_state - 1;
  }
}

void setup()
{
  Serial.begin(112500);

  analogReadResolution(ANALOGRESOLUTION);
  pinMode(13, OUTPUT);
  pinMode(STEERING, INPUT);
  pinMode(BRAKE, INPUT);
  pinMode(THROTTLE, INPUT);

  initButtons();

  for (int i = 0; i < LIGHTING_PIN_COUNT; i++) {
    pinMode(BASE_LIGHT_PIN + i, OUTPUT);
  }
  lighting_state = 0;

  Joystick.setSteeringRange(0, ANALOG_MAX_VAL);
  Joystick.setThrottleRange(ANALOG_MAX_VAL, 0); // Inverted
  Joystick.setBrakeRange(ANALOG_MAX_VAL, 0);    // Inverted
  Joystick.begin(false);  //initAutoSendState false

  Serial.println("Init successful");
}


void loop()
{
  const auto currentMillis = millis();
  if (currentMillis - previousJoystickUpdate >= JoystickUpdateInterval_ms) {
    previousJoystickUpdate = currentMillis;
    careForJoystick();
    decrementButtonDebounce();
  }

  if (currentMillis - previousLightingUpdate >= lightingInterval_ms) {
    previousLightingUpdate = currentMillis;
    careForLighting();
  }

  if(Serial) {
    Serial.println("loop done");
  }
}

