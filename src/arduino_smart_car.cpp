/* Copyright (C) 2019  Samuel Trassare (https://github.com/tiogaplanet)

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/
#include "arduino_smart_car.h"

// Setup the pins to connect the Arduino to the devices on the robot.
#define AR_LEFT_MOTOR_INTERRUPT        2
#define AR_RIGHT_MOTOR_INTERRUPT       3
#define AR_ULTRASONIC_TRIGGER_PIN      4
#define AR_LEFT_MOTOR_ENABLE_PIN       5  // enA
#define AR_LEFT_MOTOR_INPUT_ONE_PIN    6  // in1
#define AR_LEFT_MOTOR_INPUT_TWO_PIN    7  // in2
#define AR_RIGHT_MOTOR_INPUT_THREE_PIN 8  // in3
#define AR_RIGHT_MOTOR_INPUT_FOUR_PIN  9  // in4
#define AR_ULTRASONIC_ECHO_PIN         10
#define AR_RIGHT_MOTOR_ENABLE_PIN      11 // enB
#define AR_SERVO_PIN                   12

// Set the maximum distance for the ultrasonic sensor.
#define ULTRASONIC_MAX_DISTANCE 250

// Set the serial baud rate.
#define BAUD_RATE 9600

// Setup some metrics for accurate distance tracking.
#define WHEEL_DIAMETER 6610 // Actual wheel diameter is 6.61 centimeters.
#define HALF_AXLE_TRACK 750 // Actual axle track is 15 centimeters
#define SPEED_ENCODER_SLOTS 20
#define DISTANCE_DIVISOR 1000


// Define an assert mechanism that can be used to log and halt when the user is found to be calling the API incorrectly.
#define ROBOT_ASSERT(EXPRESSION) if (!(EXPRESSION)) robotAssert(__LINE__);

static void robotAssert(uint32_t lineNumber)
{
    Serial.print(F("Robot Assert: arduino_smart_car.cpp: "));
        Serial.println(lineNumber);

    while (1)
    {
    }
}

#ifdef SPEED_SENSORS_INSTALLED
volatile int counter_A = 0;
volatile int counter_B = 0;

// Motor A pulse count ISR
void ISR_countA()
{
  counter_A++;  // increment Motor A counter value
}

// Motor B pulse count ISR
void ISR_countB()
{
  counter_B++;  // increment Motor B counter value
}
#endif

SmartCar::SmartCar()
{
    clear();
}

SmartCar::~SmartCar()
{

}

void SmartCar::begin()
{
    // For debugging output.
    Serial.begin(BAUD_RATE);

    // Enable the servo.
    m_Servo.attach(AR_SERVO_PIN);

    // Set all the motor control pins to outputs.
    pinMode(AR_LEFT_MOTOR_ENABLE_PIN, OUTPUT);
    pinMode(AR_RIGHT_MOTOR_ENABLE_PIN, OUTPUT);
    pinMode(AR_LEFT_MOTOR_INPUT_ONE_PIN, OUTPUT);
    pinMode(AR_LEFT_MOTOR_INPUT_TWO_PIN, OUTPUT);
    pinMode(AR_RIGHT_MOTOR_INPUT_THREE_PIN, OUTPUT);
    pinMode(AR_RIGHT_MOTOR_INPUT_FOUR_PIN, OUTPUT);

    pinMode(AR_ULTRASONIC_ECHO_PIN, INPUT);    // Define the ultrasonic echo (receive)  pin
    pinMode(AR_ULTRASONIC_TRIGGER_PIN, OUTPUT);  // Define the ultrasound trigger (send) pin

#ifdef SPEED_SENSORS_INSTALLED
    // Attach the interrupts for the speed sensors to their respective ISRs.
    attachInterrupt(digitalPinToInterrupt (AR_LEFT_MOTOR_INTERRUPT), ISR_countA, RISING);  // Increase counter A when speed sensor pin goes High
    attachInterrupt(digitalPinToInterrupt (AR_RIGHT_MOTOR_INTERRUPT), ISR_countB, RISING);  // Increase counter B when speed sensor pin goes High
#endif
}

void SmartCar::turnHead(uint8_t angle)
{
    m_Servo.write(angle);
    delay(500);
}

uint8_t SmartCar::readHeadAngle()
{
    return m_Servo.read();
}

int SmartCar::ping()
{
  delay(100);  // Wait 100ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  int cm = m_Sonar->ping_cm();   //Send ping, get ping distance in centimeters (cm).
  if (cm==0)
  {
    cm=250;
  }
  return cm;
}

int SmartCar::turnHeadAndPing(uint8_t angle)
{
    turnHead(angle);

    return ping();
}

void SmartCar::driveForward(uint8_t mspeed)
{
  setForward();

  drive(mspeed, mspeed);
}

void SmartCar::driveBackward(uint8_t mspeed)
{
  setBackward();

  drive(mspeed, mspeed);
}

// Function to drive forward with time.
void SmartCar::driveForwardTime(uint32_t time, uint8_t mspeed)
{
  setForward();

  drive(mspeed, mspeed);

  delay(time);

  drive(SPEED_STOP, SPEED_STOP);
}

// Function to drive backward with time.
void SmartCar::driveBackwardTime(uint32_t time, uint8_t mspeed)
{
  setBackward();

  drive(mspeed, mspeed);

  delay(time);

  drive(SPEED_STOP, SPEED_STOP);
}

// Function to turn left with time.
void SmartCar::turnLeftTime(uint32_t time, uint8_t mspeed)
{
  setLeft();

  drive(mspeed, mspeed);

  delay(time);

  drive(SPEED_STOP, SPEED_STOP);
}

// Function to turn right with time.
void SmartCar::turnRightTime(uint32_t time, uint8_t mspeed)
{
  setRight();

  drive(mspeed, mspeed);

  delay(time);

  drive(SPEED_STOP, SPEED_STOP);
}

#ifdef SPEED_SENSORS_INSTALLED
// Function to drive forward with distance.
void SmartCar::driveForwardDistance(uint32_t distance, int mspeed)
{
  setForward();

  int steps = cmToSlots(distance);

  counter_A = 0;  //  reset counter A to zero
  counter_B = 0;  //  reset counter B to zero

  // Go forward until step value is reached
  while (steps > counter_A && steps > counter_B) {

    if (steps > counter_A) {
      analogWrite(AR_LEFT_MOTOR_ENABLE_PIN, mspeed);
    } else {
      analogWrite(AR_LEFT_MOTOR_ENABLE_PIN, 0);
    }
    if (steps > counter_B) {
      analogWrite(AR_RIGHT_MOTOR_ENABLE_PIN, mspeed);
    } else {
      analogWrite(AR_RIGHT_MOTOR_ENABLE_PIN, 0);
    }
  }

  drive(SPEED_STOP, SPEED_STOP);

  counter_A = 0;  //  reset counter A to zero
  counter_B = 0;  //  reset counter B to zero
}

// Function to drive backward with distance.
void SmartCar::driveBackwardDistance(uint32_t distance, int mspeed)
{
  setBackward();

  int steps = cmToSlots(distance);

  counter_A = 0;  //  reset counter A to zero
  counter_B = 0;  //  reset counter B to zero

  // Go in reverse until step value is reached
  while (steps > counter_A && steps > counter_B) {

    if (steps > counter_A) {
      analogWrite(AR_LEFT_MOTOR_ENABLE_PIN, mspeed);
    } else {
      analogWrite(AR_LEFT_MOTOR_ENABLE_PIN, 0);
    }
    if (steps > counter_B) {
      analogWrite(AR_RIGHT_MOTOR_ENABLE_PIN, mspeed);
    } else {
      analogWrite(AR_RIGHT_MOTOR_ENABLE_PIN, 0);
    }
  }

  drive(SPEED_STOP, SPEED_STOP);

  counter_A = 0;  //  reset counter A to zero
  counter_B = 0;  //  reset counter B to zero
}

// Function to turn left with degrees.
void SmartCar::turnLeftDegrees(uint32_t degrees, int mspeed)
{
  counter_A = 0;  //  reset counter A to zero
  counter_B = 0;  //  reset counter B to zero

  setLeft();

  int steps = degreesToSlots(degrees);

  // Go until step value is reached
  while (steps > counter_A && steps > counter_B) {

    if (steps > counter_A) {
      analogWrite(AR_LEFT_MOTOR_ENABLE_PIN, mspeed);
    } else {
      analogWrite(AR_LEFT_MOTOR_ENABLE_PIN, 0);
    }
    if (steps > counter_B) {
      analogWrite(AR_RIGHT_MOTOR_ENABLE_PIN, mspeed);
    } else {
      analogWrite(AR_RIGHT_MOTOR_ENABLE_PIN, 0);
    }
  }

  drive(SPEED_STOP, SPEED_STOP);

  counter_A = 0;  //  reset counter A to zero
  counter_B = 0;  //  reset counter B to zero
}

// Function to turn right with degrees.
void SmartCar::turnRightDegrees(uint32_t degrees, int mspeed)
{
  setRight();

  int steps = degreesToSlots(degrees);

  counter_A = 0;  //  reset counter A to zero
  counter_B = 0;  //  reset counter B to zero

  // Go until step value is reached
  while (steps > counter_A && steps > counter_B) {

    if (steps > counter_A) {
      analogWrite(AR_LEFT_MOTOR_ENABLE_PIN, mspeed);
    } else {
      analogWrite(AR_LEFT_MOTOR_ENABLE_PIN, SPEED_STOP);
    }
    if (steps > counter_B) {
      analogWrite(AR_RIGHT_MOTOR_ENABLE_PIN, mspeed);
    } else {
      analogWrite(AR_RIGHT_MOTOR_ENABLE_PIN, SPEED_STOP);
    }
  }

  drive(SPEED_STOP, SPEED_STOP);

  counter_A = 0;  //  reset counter A to zero
  counter_B = 0;  //  reset counter B to zero
}
#endif

void SmartCar::stop()
{
  analogWrite(AR_LEFT_MOTOR_ENABLE_PIN, SPEED_STOP);
  analogWrite(AR_LEFT_MOTOR_ENABLE_PIN, SPEED_STOP);

  digitalWrite(AR_LEFT_MOTOR_INPUT_ONE_PIN, LOW);
  digitalWrite(AR_LEFT_MOTOR_INPUT_TWO_PIN, LOW);
  digitalWrite(AR_RIGHT_MOTOR_INPUT_THREE_PIN, LOW);
  digitalWrite(AR_RIGHT_MOTOR_INPUT_FOUR_PIN, LOW);
}

void SmartCar::clear()
{
    m_Sonar = new NewPing(AR_ULTRASONIC_TRIGGER_PIN, AR_ULTRASONIC_ECHO_PIN, ULTRASONIC_MAX_DISTANCE);
}

// Function to convert from centimeters to slots in the rotary encoders.
int SmartCar::cmToSlots(float cm)
{
    float circumference = (WHEEL_DIAMETER * PI) / DISTANCE_DIVISOR; // Calculate wheel circumference in cm
    float cm_step = circumference / SPEED_ENCODER_SLOTS;  // CM per Step

    float f_result = cm / cm_step;  // Calculate result as a float
    return (int) f_result; // Convert to an integer (note this is NOT rounded)
}

// Function to convert from degrees to slots in the rotary encoders.
int SmartCar::degreesToSlots(uint32_t degrees)
{
  // Use the formula for the arc of a circle to determine the distance to spin.
  return cmToSlots(2 * PI * HALF_AXLE_TRACK * (degrees / 360));
}

void SmartCar::setForward()
{
  // Set left motor to forward.
  digitalWrite(AR_LEFT_MOTOR_INPUT_ONE_PIN, HIGH);
  digitalWrite(AR_LEFT_MOTOR_INPUT_TWO_PIN, LOW);

  // Set right motor to forward.
  digitalWrite(AR_RIGHT_MOTOR_INPUT_THREE_PIN, HIGH);
  digitalWrite(AR_RIGHT_MOTOR_INPUT_FOUR_PIN, LOW);
}

void SmartCar::setBackward()
{
  // Set left motor to reverse.
  digitalWrite(AR_LEFT_MOTOR_INPUT_ONE_PIN, LOW);
  digitalWrite(AR_LEFT_MOTOR_INPUT_TWO_PIN, HIGH);

  // Set right motor to reverse.
  digitalWrite(AR_RIGHT_MOTOR_INPUT_THREE_PIN, LOW);
  digitalWrite(AR_RIGHT_MOTOR_INPUT_FOUR_PIN, HIGH);
}


void SmartCar::setLeft()
{
  // Set left motor to reverse.
  digitalWrite(AR_LEFT_MOTOR_INPUT_ONE_PIN, LOW);
  digitalWrite(AR_LEFT_MOTOR_INPUT_TWO_PIN, HIGH);

  // Set right motor to forward.
  digitalWrite(AR_RIGHT_MOTOR_INPUT_THREE_PIN, HIGH);
  digitalWrite(AR_RIGHT_MOTOR_INPUT_FOUR_PIN, LOW);
}

void SmartCar::setRight()
{
  // Set left motor to forward.
  digitalWrite(AR_LEFT_MOTOR_INPUT_ONE_PIN, HIGH);
  digitalWrite(AR_LEFT_MOTOR_INPUT_TWO_PIN, LOW);

  // Set right motor to reverse.
  digitalWrite(AR_RIGHT_MOTOR_INPUT_THREE_PIN, LOW);
  digitalWrite(AR_RIGHT_MOTOR_INPUT_FOUR_PIN, HIGH);
}

void SmartCar::drive(uint8_t leftSpeed, uint8_t rightSpeed)
{
  analogWrite(AR_LEFT_MOTOR_ENABLE_PIN, leftSpeed);
  analogWrite(AR_RIGHT_MOTOR_ENABLE_PIN, rightSpeed);
}