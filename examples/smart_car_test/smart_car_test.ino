/*
   Credit to HDA Robotics at https://create.arduino.cc/projecthub/hda-robotics/project-1-2wd-obstacle-avoiding-robot-390ef8
*/
#include <arduino_smart_car.h>

SmartCar car;

void setup()
{
  car.begin();

  car.turnHead(SWEEP_CENTER);
}

void loop()
{
  int distanceRight = 0;
  int distanceLeft = 0;

  delay(50);

  if (car.ping() <= 25)
  {
    car.stop();
    delay(300);
    car.driveBackwardTime(550, SPEED_SLOW);
    delay(300);
    distanceRight = car.turnHeadAndPing(SWEEP_RIGHT);
    car.turnHead(SWEEP_CENTER);
    delay(300);
    distanceLeft = car.turnHeadAndPing(SWEEP_LEFT);
    car.turnHead(SWEEP_CENTER);
    delay(300);

    if (distanceRight >= distanceLeft)
    {
      car.turnRightTime(550, SPEED_SLOW);
      car.stop();
    }
    else
    {
      car.turnLeftTime(550, SPEED_SLOW);
      car.stop();
    }

  }
  else
  {
    car.driveForward(SPEED_SLOW);
  }
}
