/*
  This file is an initial placeholder pending development of the rest of the library.

  Robot Car with Speed Sensor Demonstration
  RobotCarSpeedSensorDemo.ino
  Demonstrates use of Hardware Interrupts
  to control motors on Robot Car

  DroneBot Workshop 2017
  http://dronebotworkshop.com
*/

#include <arduino_smart_car.h>

// Setup the ultrasonic sensor.
#define TRIGGER_PIN  4
#define ECHO_PIN     10
#define MAX_DISTANCE 200
NewPing DistanceSensor(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// Setup the servo for the ultrasonic sensor.
#define SERVO_PIN 12
#define CENTER 90
#define RIGHT 1
#define LEFT 179
Servo servo;

// Setup the drive motors.
#define STOP 0
#define QUARTER_SPEED 64
#define THIRD_SPEED 85
#define HALF_SPEED 128
#define TWO_THIRDS_SPEED 170
#define FULL_SPEED 255
#define THREE_QUARTERS_SPEED 191
// Left motor.
int enA = 5;
int in1 = 6;
int in2 = 7;
// Right motor.
int enB = 11;
int in3 = 8;
int in4 = 9;

// Setup the interripts for the speed sensors on the motors.
const byte MOTOR_A = 3;  // Motor 2 Interrupt Pin - INT 1 - Right Motor
const byte MOTOR_B = 2;  // Motor 1 Interrupt Pin - INT 0 - Left Motor

// Constant for steps in disk
const float stepcount = 20.00;  // 20 slots in disk, change if different

// Constant for wheel diameter
const float wheeldiameter = 66.10; // Wheel diameter in millimeters, change if different

// Integers for pulse counters
volatile int counter_A = 0;
volatile int counter_B = 0;

void setup()
{
  Serial.begin(9600);

  // Enable the servo.
  servo.attach(SERVO_PIN);

  // Set all the motor control pins to outputs.
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Attach the interrupts for the speed sensors to their respective ISRs.
  attachInterrupt(digitalPinToInterrupt (MOTOR_A), ISR_countA, RISING);  // Increase counter A when speed sensor pin goes High
  attachInterrupt(digitalPinToInterrupt (MOTOR_B), ISR_countB, RISING);  // Increase counter B when speed sensor pin goes High

  // Test Motor Movement  - Experiment with your own sequences here.
  MoveForward(CMtoSteps(50), 255);  // Forward half a metre at 255 speed
  delay(1000);  // Wait one second
  MoveReverse(10, 255);  // Reverse 10 steps at 255 speed
  delay(1000);  // Wait one second
  MoveForward(10, 150);  // Forward 10 steps at 150 speed
  delay(1000);  // Wait one second
  MoveReverse(CMtoSteps(25.4), 200);  // Reverse 25.4 cm at 200 speed
  delay(1000);  // Wait one second
  SpinRight(20, 255);  // Spin right 20 steps at 255 speed
  delay(1000);  // Wait one second
  SpinLeft(60, 175);  // Spin left 60 steps at 175 speed
  delay(1000);  // Wait one second
  MoveForward(1, 255);  // Forward 1 step at 255 speed

  // Get distances.
  ask_pin_F();            // Read in front of the distance

  ask_pin_R();

  ask_pin_F();            // Read in front of the distance

  ask_pin_L();

  // Center the servo before looping.
  servo.write(CENTER);
}


void loop()
{
  // Put whatever you want here!


}

// All the capability below will eventually move into the library.

// Interrupt Service Routines

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

// Function to convert from centimeters to steps
int CMtoSteps(float cm) {

  int result;  // Final calculation result
  float circumference = (wheeldiameter * PI) / 10; // Calculate wheel circumference in cm
  float cm_step = circumference / stepcount;  // CM per Step

  float f_result = cm / cm_step;  // Calculate result as a float
  result = (int) f_result; // Convert to an integer (note this is NOT rounded)

  return result;  // End and return result

}

// Function to Move Forward
void MoveForward(int steps, int mspeed)
{
  counter_A = 0;  //  reset counter A to zero
  counter_B = 0;  //  reset counter B to zero

  // Set Motor A forward
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  // Set Motor B forward
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  // Go forward until step value is reached
  while (steps > counter_A && steps > counter_B) {

    if (steps > counter_A) {
      analogWrite(enA, mspeed);
    } else {
      analogWrite(enA, 0);
    }
    if (steps > counter_B) {
      analogWrite(enB, mspeed);
    } else {
      analogWrite(enB, 0);
    }
  }

  // Stop when done
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  counter_A = 0;  //  reset counter A to zero
  counter_B = 0;  //  reset counter B to zero

}

// Function to Move in Reverse
void MoveReverse(int steps, int mspeed)
{
  counter_A = 0;  //  reset counter A to zero
  counter_B = 0;  //  reset counter B to zero

  // Set Motor A reverse
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  // Set Motor B reverse
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  // Go in reverse until step value is reached
  while (steps > counter_A && steps > counter_B) {

    if (steps > counter_A) {
      analogWrite(enA, mspeed);
    } else {
      analogWrite(enA, 0);
    }
    if (steps > counter_B) {
      analogWrite(enB, mspeed);
    } else {
      analogWrite(enB, 0);
    }
  }

  // Stop when done
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  counter_A = 0;  //  reset counter A to zero
  counter_B = 0;  //  reset counter B to zero

}

// Function to Spin Right
void SpinRight(int steps, int mspeed)
{
  counter_A = 0;  //  reset counter A to zero
  counter_B = 0;  //  reset counter B to zero

  // Set Motor A reverse
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  // Set Motor B forward
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  // Go until step value is reached
  while (steps > counter_A && steps > counter_B) {

    if (steps > counter_A) {
      analogWrite(enA, mspeed);
    } else {
      analogWrite(enA, 0);
    }
    if (steps > counter_B) {
      analogWrite(enB, mspeed);
    } else {
      analogWrite(enB, 0);
    }
  }

  // Stop when done
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  counter_A = 0;  //  reset counter A to zero
  counter_B = 0;  //  reset counter B to zero

}

// Function to Spin Left
void SpinLeft(int steps, int mspeed)
{
  counter_A = 0;  //  reset counter A to zero
  counter_B = 0;  //  reset counter B to zero

  // Set Motor A forward
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  // Set Motor B reverse
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  // Go until step value is reached
  while (steps > counter_A && steps > counter_B) {

    if (steps > counter_A) {
      analogWrite(enA, mspeed);
    } else {
      analogWrite(enA, 0);
    }
    if (steps > counter_B) {
      analogWrite(enB, mspeed);
    } else {
      analogWrite(enB, 0);
    }
  }

  // Stop when done
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  counter_A = 0;  //  reset counter A to zero
  counter_B = 0;  //  reset counter B to zero

}

void ask_pin_F()   // Measure the distance ahead
{
  servo.write(CENTER);
  delay(1000);
  Serial.print("Forward distance: ");
  Serial.print(DistanceSensor.ping_cm());
  Serial.println("cm");
  delay(1000);
}
void ask_pin_L()   // Measure the distance on the left
{
  servo.write(LEFT);
  delay(1000);
  Serial.print("Left distance: ");
  Serial.print(DistanceSensor.ping_cm());
  Serial.println("cm");
  delay(1000);
}
void ask_pin_R()   // Measure the distance on the right
{
  servo.write(RIGHT);
  delay(1000);
  Serial.print("Right distance: ");
  Serial.print(DistanceSensor.ping_cm());
  Serial.println("cm");
  delay(1000);
}

void demoThirdSpeed()
{
  // this function will run the motors in both directions at a fixed speed
  // turn on motor A
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  // Set motor speed to THIRD_SPEED.
  analogWrite(enA, THIRD_SPEED);
  // turn on motor B
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  // Set motor speed to THIRD_SPEED.
  analogWrite(enB, THIRD_SPEED);
  delay(2000);
  // now change motor directions
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  delay(2000);
  // now turn off motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
