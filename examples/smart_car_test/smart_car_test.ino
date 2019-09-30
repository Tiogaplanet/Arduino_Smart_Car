#include <arduino_smart_car.h>

SmartCar car;

void setup() {
  // put your setup code here, to run once:
  car.begin();
}

void loop() {
  // put your main code here, to run repeatedly:

  Serial.print("Ping center: ");
  Serial.print(car.turnHeadAndPing(CENTER));
  Serial.println(" cm");
  delay(1000);

  Serial.print("Ping left: ");
  Serial.print(car.turnHeadAndPing(LEFT));
  Serial.println(" cm");
  delay(1000);

  Serial.print("Ping center: ");
  Serial.print(car.turnHeadAndPing(CENTER));
  Serial.println(" cm");
  delay(1000);

  Serial.print("Ping right: ");
  Serial.print(car.turnHeadAndPing(RIGHT));
  Serial.println(" cm");
  delay(1000);
}
