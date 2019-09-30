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

// A few defined servo angles.
#define CENTER 90
#define RIGHT  1
#define LEFT   180

// This number may need to be changed if your encoder has a different number of slots.
#define SPEED_ENCODER_SLOTS 20


class SmartCar
{
public:
    // Constructor/Destructors.
    SmartCar();
    ~SmartCar();

    void begin();

    // Servo and ultrasonic operations.
    void turnHead(uint8_t angle);
    uint32_t readHeadAngle();
	uint32_t ping();
	
	// Servo and ultrasonic combinations.
	uint32_t turnHeadAndPing(uint8_t angle);
	
    // Movement operations.
    void driveForward(uint32_t distance, int mspeed);
    void driveBackward(uint32_t distance, int mspeed);
    void turnLeft(uint32_t distance, int mspeed);
    void turnRight(uint32_t distance, int mspeed);
    void stop();

protected:
    void clear();
    int cmToSlots(float cm);

    Servo m_Servo;
    NewPing *m_Sonar;
};

#endif
