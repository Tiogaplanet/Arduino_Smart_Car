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
#define ULTRASONIC_MAX_DISTANCE 200

// Define serial baud rate.
#define BAUD_RATE 9600

// Actual wheel diameter is 66.10 millimeters.
#define WHEEL_DIAMETER 6610
#define DISTANCE_DIVISOR 1000



// Define an assert mechanism that can be used to log and halt when the user is found to be calling the API incorrectly.
#define ROBOT_ASSERT(EXPRESSION) if (!(EXPRESSION)) robotAssert(__LINE__);

static void robotAssert(uint32_t lineNumber)
{
    Serial.print(F("Robot Assert: arduino_smart_car.cpp:"));
        Serial.println(lineNumber);

    while (1)
    {
    }
}

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



SmartCar::SmartCar()
{
    clear();
}

SmartCar::~SmartCar()
{

}

void SmartCar::begin()
{
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

    // Attach the interrupts for the speed sensors to their respective ISRs.
    attachInterrupt(digitalPinToInterrupt (AR_LEFT_MOTOR_INTERRUPT), ISR_countA, RISING);  // Increase counter A when speed sensor pin goes High
    attachInterrupt(digitalPinToInterrupt (AR_RIGHT_MOTOR_INTERRUPT), ISR_countB, RISING);  // Increase counter B when speed sensor pin goes High
}

void SmartCar::turnHead(uint8_t angle)
{
    m_Servo.write(angle);
}

uint32_t SmartCar::readHeadAngle()
{
    return m_Servo.read();
}

uint32_t SmartCar::ping()
{
	return m_Sonar->ping_cm();
}

uint32_t SmartCar::turnHeadAndPing(uint8_t angle)
{
	turnHead(angle);
	delay(500);
	return m_Sonar->ping_cm();
}

// Function to drive forward.
void SmartCar::driveForward(uint32_t distance, int mspeed)
{
    int steps = cmToSlots(distance);

  counter_A = 0;  //  reset counter A to zero
  counter_B = 0;  //  reset counter B to zero

  // Set Motor A forward
  digitalWrite(AR_LEFT_MOTOR_INPUT_ONE_PIN, HIGH);
  digitalWrite(AR_LEFT_MOTOR_INPUT_TWO_PIN, LOW);

  // Set Motor B forward
  digitalWrite(AR_RIGHT_MOTOR_INPUT_THREE_PIN, HIGH);
  digitalWrite(AR_RIGHT_MOTOR_INPUT_FOUR_PIN, LOW);

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

  // Stop when done
  analogWrite(AR_LEFT_MOTOR_ENABLE_PIN, 0);
  analogWrite(AR_RIGHT_MOTOR_ENABLE_PIN, 0);
  counter_A = 0;  //  reset counter A to zero
  counter_B = 0;  //  reset counter B to zero

}

// Function to drive backward.
void SmartCar::driveBackward(uint32_t distance, int mspeed)
{
    int steps = cmToSlots(distance);

  counter_A = 0;  //  reset counter A to zero
  counter_B = 0;  //  reset counter B to zero

  // Set Motor A reverse
  digitalWrite(AR_LEFT_MOTOR_INPUT_ONE_PIN, LOW);
  digitalWrite(AR_LEFT_MOTOR_INPUT_TWO_PIN, HIGH);

  // Set Motor B reverse
  digitalWrite(AR_RIGHT_MOTOR_INPUT_THREE_PIN, LOW);
  digitalWrite(AR_RIGHT_MOTOR_INPUT_FOUR_PIN, HIGH);

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

  // Stop when done
  analogWrite(AR_LEFT_MOTOR_ENABLE_PIN, 0);
  analogWrite(AR_RIGHT_MOTOR_ENABLE_PIN, 0);
  counter_A = 0;  //  reset counter A to zero
  counter_B = 0;  //  reset counter B to zero
}

// Function to turn left.
void SmartCar::turnLeft(uint32_t distance, int mspeed)
{
    int steps = cmToSlots(distance);

  counter_A = 0;  //  reset counter A to zero
  counter_B = 0;  //  reset counter B to zero

  // Set Motor A forward
  digitalWrite(AR_LEFT_MOTOR_INPUT_ONE_PIN, HIGH);
  digitalWrite(AR_LEFT_MOTOR_INPUT_TWO_PIN, LOW);

  // Set Motor B reverse
  digitalWrite(AR_RIGHT_MOTOR_INPUT_THREE_PIN, LOW);
  digitalWrite(AR_RIGHT_MOTOR_INPUT_FOUR_PIN, HIGH);

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

  // Stop when done
  analogWrite(AR_LEFT_MOTOR_ENABLE_PIN, 0);
  analogWrite(AR_RIGHT_MOTOR_ENABLE_PIN, 0);
  counter_A = 0;  //  reset counter A to zero
  counter_B = 0;  //  reset counter B to zero
}

// Function to turn right.
void SmartCar::turnRight(uint32_t distance, int mspeed)
{
    int steps = cmToSlots(distance);

  counter_A = 0;  //  reset counter A to zero
  counter_B = 0;  //  reset counter B to zero

  // Set Motor A reverse
  digitalWrite(AR_LEFT_MOTOR_INPUT_ONE_PIN, LOW);
  digitalWrite(AR_LEFT_MOTOR_INPUT_TWO_PIN, HIGH);

  // Set Motor B forward
  digitalWrite(AR_RIGHT_MOTOR_INPUT_THREE_PIN, HIGH);
  digitalWrite(AR_RIGHT_MOTOR_INPUT_FOUR_PIN, LOW);

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

  // Stop when done
  analogWrite(AR_LEFT_MOTOR_ENABLE_PIN, 0);
  analogWrite(AR_RIGHT_MOTOR_ENABLE_PIN, 0);
  counter_A = 0;  //  reset counter A to zero
  counter_B = 0;  //  reset counter B to zero
}

void SmartCar::stop()
{
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
    int result;  // Final calculation result
    float circumference = (WHEEL_DIAMETER * PI) / DISTANCE_DIVISOR; // Calculate wheel circumference in cm
    float cm_step = circumference / SPEED_ENCODER_SLOTS;  // CM per Step

    float f_result = cm / cm_step;  // Calculate result as a float
    result = (int) f_result; // Convert to an integer (note this is NOT rounded)

    return result;  // End and return result
}