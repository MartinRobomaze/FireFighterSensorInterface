# FireFighterSensorInterface
FireFighter ESP32 Firmware. It communicates with [FireFighter-Go](https://github.com/MartinRobomaze/FireFighter-Go), reads data from sensors and controls motors.  
## Compiling and uplolading
This project uses [PlatformIO](https://platformio.org/) for building and uploading the project. It can be installed using this [guide](https://platformio.org/install/cli).
1. Install project dependencies using `pio pkg install`
2. Compile and upload the project to ESP32 using `pio run -t upload`
