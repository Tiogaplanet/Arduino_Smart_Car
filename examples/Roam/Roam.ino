/* Copyright (C) 2019  Samuel Trasare (https://github.com/tiogaplanet)

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
/* Example used in following API documentation:
    turnHead()
    turnHeadAndPing()
    ping()
    driveForward()
    driveBackwardTime()
    turnRightTime()
    turnLeftTime()
    stop()
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
