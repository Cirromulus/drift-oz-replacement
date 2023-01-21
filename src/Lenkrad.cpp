#include <Arduino.h>
#include <Joystick.h>
#include <cmath>

static constexpr uint8_t ANALOGRESOLUTION = 12;
static constexpr uint16_t ANALOG_MAX_VAL = pow(2, ANALOGRESOLUTION);

static constexpr uint8_t BASE_LIGHT_PIN = 24;
static constexpr uint8_t LIGHTING_PIN_COUNT = 4;
static constexpr uint8_t STEERING = A0;
static constexpr uint8_t THROTTLE = A1;
static constexpr uint8_t BRAKE = A2;
static constexpr uint8_t SCHALTWIPPEL = 22;
static constexpr uint8_t SCHALTWIPPEL_BUTTON = 30;  // Joystick button Number
static constexpr uint8_t SCHALTWIPPER = 23;
static constexpr uint8_t SCHALTWIPPER_BUTTON = 31;  // Joystick button Number
static constexpr uint8_t RIGHT_BUTTONS_START = 30;
static constexpr uint8_t RIGHT_BUTTONS_COUNT = 9;
static constexpr uint8_t RIGHT_BUTTONS_OFFSE = 0;

static constexpr uint8_t DEADZONE = 15;
static constexpr uint8_t BUTTON_BOUNCE = 20;
static constexpr uint8_t NUMBER_OF_BUTTONS = 32;

static uint8_t bounce_counter[NUMBER_OF_BUTTONS];
static uint8_t lighting_state = 0;

static constexpr unsigned lightingInterval_ms = 250;
static uint16_t previousMillis = 0;

void decrementButtonDebounce() {
  for(auto& bounce : bounce_counter) {
    if(bounce > 0) {
      bounce--;
    }
  }
}

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,
  JOYSTICK_TYPE_MULTI_AXIS,
  RIGHT_BUTTONS_COUNT+2,  // Buttons on Wheel + 2 shift button
  4, false, false, false, false, false, false,
  false, true, false, true, true);  // Throttle, Brake and steering


bool checkandsetButton(bool isPressed, uint8_t pos) {
  if (bounce_counter[pos] == 0) {
    Joystick.setButton(pos, isPressed);
    bounce_counter[pos] = BUTTON_BOUNCE;
    return true;
  }
  return false;
}

void careForJoystick() {
  boolean hasToUpdate = false;

  if(Serial) {
    const auto tmp = analogRead(STEERING);
    Serial.print("Steering: ");
    Serial.print(tmp);
    Serial.println();
  }

  Joystick.setSteering(analogRead(STEERING));
  Joystick.setThrottle(analogRead(THROTTLE));
  Joystick.setBrake(analogRead(BRAKE));

  hasToUpdate |= checkandsetButton(!digitalRead(SCHALTWIPPEL), SCHALTWIPPEL_BUTTON);
  hasToUpdate |= checkandsetButton(!digitalRead(SCHALTWIPPER), SCHALTWIPPER_BUTTON);

  for(int i = 0; i < RIGHT_BUTTONS_COUNT; i++){
    hasToUpdate |= checkandsetButton(!digitalRead(RIGHT_BUTTONS_START + i), RIGHT_BUTTONS_OFFSE + i);
  }
  //Serial.println();

  // Call Joystick.move
  if (hasToUpdate)
    Joystick.sendState();
}

void careForLighting() {
  for (int i = 0; i < LIGHTING_PIN_COUNT; i++) {
    digitalWrite(BASE_LIGHT_PIN + i, i != lighting_state);
  }

  if(true) { //Joystick.state.xRot.axis < 10){
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
  pinMode(A0, INPUT);
  pinMode(SCHALTWIPPEL, INPUT_PULLUP);
  pinMode(SCHALTWIPPER, INPUT_PULLUP);
  for(int i = 0; i < RIGHT_BUTTONS_COUNT; i++){
    pinMode(RIGHT_BUTTONS_START + i, INPUT_PULLUP);
  }

  for (int i = 0; i < LIGHTING_PIN_COUNT; i++) {
    pinMode(BASE_LIGHT_PIN + i, OUTPUT);
  }
  lighting_state = 0;

  Joystick.setSteeringRange(0, ANALOG_MAX_VAL);
  Joystick.setThrottleRange(ANALOG_MAX_VAL, 0); // Inverted
  Joystick.setBrakeRange(ANALOG_MAX_VAL, 0);    // Inverted
  Joystick.begin(false);  //initAutoSendState false
  memset(bounce_counter, 0, NUMBER_OF_BUTTONS);

  Serial.println("Init successful");
}


void loop()
{
  careForJoystick();
  decrementButtonDebounce();
  delay(4);
  const auto currentMillis = millis();
  if (currentMillis - previousMillis >= (lightingInterval_ms)) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;
    careForLighting();
  }
  if(Serial) {
    Serial.println("loop done");
  }
}

