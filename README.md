# Arduino Smart Car
An Arduino IDE library for the two-wheeled Arduino robot chassis sold by VKmaker.

![Two-wheeled Arduino-based robot by VKmaker](https://github.com/Tiogaplanet/Arduino_Smart_Car/raw/master/images/VKmaker02.jpg)

There are many inexpensive robot kits available based on the Arduino Uno.  This project shares my experiences in building and coding the two-wheeled kit sold by VKmaker.

While the kit contains almost everything you need to build a working robot, I strongly recommend you also buy and install a pair of LM393-based speed sensors.  Otherwise, it's going to be quite a challenge to keep the robot driving straight, especially on hardwood floors.  This library supports the kit the way VKmaker sells it but it also includes support for speed sensors.

There are other minor hardware modifications you will need to make to the original kit which are detailed in the [wiki](https://github.com/Tiogaplanet/Arduino_Smart_Car/wiki).

## Acknowledgement
*   A special thanks goes out to the [DroneBot Workshop](https://dronebotworkshop.com) whose many helpful articles and videos enabled this project.
*   Thanks goes out to [HDA Robotics](https://create.arduino.cc/projecthub/hda-robotics/project-1-2wd-obstacle-avoiding-robot-390ef8) whose sketch provided the foundation of this library.

## Installation
1.  The Arduino_Smart_Car library is intended for use with the Arduino IDE.  Installation is the same as for other libraries.  Download the zip and select `Sketch->Include Library->Add .ZIP Library...`.  Browse to the downloaded zip file and the Arduino IDE will do the rest.

## Kit Assembly and Library API
A very thorough guide to using this library and the minor hardware changes needed to enable it are provided in the [wiki](https://github.com/Tiogaplanet/Arduino_Smart_Car/wiki).

## Contributing
This project is intended to share my experiences building an Arduino-based robot kit.  However, your suggestions for improvement and your contributions are highly encouraged!  Please see [CONTRIBUTING.md](https://github.com/Tiogaplanet/Arduino_Smart_Car/blob/master/CONTRIBUTING.md) for more information.
