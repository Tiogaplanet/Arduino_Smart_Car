#ifndef Arduino_Smart_Car_h
#define Arduino_Smart_Car_h

#include <NewPing.h>
#include <Servo.h>


#define CENTER 90
#define RIGHT  1
#define LEFT   179

// This number may need to be changed if your encoder has a different number of slots.
#define SPEED_ENCODER_SLOTS 20



class SmartCar
{
public:
    // Constructor/Destructors.
    SmartCar();
    ~SmartCar();

    void begin();

    // Servo operations.
    void turnHead(uint8_t angle);
    uint32_t readHeadAngle();
    uint32_t readDistance();
    uint32_t readDistance(uint8_t angle);

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

};

#endif
