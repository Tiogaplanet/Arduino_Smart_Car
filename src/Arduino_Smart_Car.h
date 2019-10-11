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
#ifndef Arduino_Smart_Car_h
#define Arduino_Smart_Car_h

#include <NewPing.h>
#include <Servo.h>

// Are the speed sensors installed?  If not, comment this definition.
//#define SPEED_SENSORS_INSTALLED

// A few defines for servo angles.
#define SWEEP_CENTER 96 // Play with this value to account for slightly off-center hardware assembly.
#define SWEEP_RIGHT  SWEEP_CENTER - 90
#define SWEEP_LEFT   SWEEP_CENTER + 90

// Let's setup a few "standard" speeds for each motor.  These are pulse width modulation (PWM) values.
#define SPEED_STOP    0
#define SPEED_STALL  70
#define SPEED_SLOW   80
#define SPEED_FAST  163
#define SPEED_PLAID 255



class SmartCar
{
public:
    // Constructor/Destructors.
    SmartCar();
    ~SmartCar();

    void begin();

    // Servo and ultrasonic operations.
    void turnHead(uint8_t angle);
    uint8_t readHeadAngle();
    int ping();
    int turnHeadAndPing(uint8_t angle);

    // Movement operations.
    void driveForward(uint8_t mspeed);
    void driveBackward(uint8_t mspeed);
    void driveForwardTime(uint32_t time, uint8_t mspeed);
    void driveBackwardTime(uint32_t time, uint8_t mspeed);
    void turnLeftTime(uint32_t time, uint8_t mspeed);
    void turnRightTime(uint32_t time, uint8_t mspeed);
#ifdef SPEED_SENSORS_INSTALLED
    void driveForwardDistance(uint32_t distance, uint8_t mspeed);
    void driveBackwardDistance(uint32_t distance, uint8_t mspeed);
    void turnLeftDegrees(uint32_t degrees, uint8_t mspeed);
    void turnRightDegrees(uint32_t degrees, uint8_t mspeed);
#endif
    void stop();

protected:
    void clear();
    int cmToSlots(float cm);
    int degreesToSlots(uint32_t degrees);
    void setForward();
    void setBackward();
    void setLeft();
    void setRight();
    void drive(uint8_t leftSpeed, uint8_t rightSpeed);

    Servo m_Servo;
    NewPing *m_Sonar;
};

#endif
