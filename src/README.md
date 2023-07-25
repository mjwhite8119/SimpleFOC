# SimpleFOC Tests
This project is setup to do testing of the SimpleFOC library. The `main` branch uses the *SimpleFOCShield* together with an Arduino UNO.  The `SimpleFOCMini-ESP32` branch uses the *SimpleFOCMini* with an ESP32.  Testing on the Arduino UNO is limited to the size of the flash memory so most of the tests will be done on the ESP32.

Tests use a gimbal motor at 7.5V with 7 pole-pairs.  

## Test 1 Open Loop Velocity Control 
Simple open loop velocity control using `motor.move()`.

## Test 2 Commander Interface
Use the Commander Serial Terminal interface to communicate with the code.  I just send the velocity in by typing **T2** (or whatever velocity) into the Serial Monitor. Added `monitor_filters = send_on_enter` to the `platformio.ini` file, but not sure if this is required.

## Test 3 Position Open Loop
Setup the following commands:
- V - Velocity in radians per second
- P - Position in radians.
- M - Set mode (0 = velocity, 1 = position)
- L - Set voltage limit
- S - Set speed limit