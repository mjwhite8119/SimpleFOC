# SimpleFOC Tests
This project is setup to do testing of the SimpleFOC library. The `main` branch uses the *SimpleFOCShield* together with an Arduino UNO.  The `SimpleFOCMini-ESP32` branch uses the *SimpleFOCMini* with an ESP32.  Testing on the Arduino UNO is limited to the size of the flash memory so most of the tests will be done on the ESP32.

Tests use a gimbal motor at 7.5V with 7 pole-pairs.  This was determined by the specs and also verified using the test in the `polepair-test` branch.

Monitoring variables:
    monitor_variables = _MON_TARGET | _MON_VOLT_Q | _MON_VEL | _MON_ANGLE;

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

## Test 4 - Torque Closed Loop

## Test 5 - Velocity Closed Loop
Got to these parameters without it oscillating:
    motor.PID_velocity.P = 0.2f;
    motor.PID_velocity.I = 4.0f;
    motor.PID_velocity.D = 0;


## Test 6 - Position Closed Loop
Started by getting the parameters for the `motor.initFOC(6.18, Direction::CW);` by using the following lines:

    Serial.println(motor.zero_electric_angle);
    Serial.println(motor.sensor_direction);

For the gimbal motor this gave me 6.18 and 1 respectively.    
For the gimbal motor I set `motor.P_angle.P = 8`.  

## Issues
The motor will suddenly just spin.  No control can stop it other than disconnection the power.  Still not sure how to tune `motor.voltage_limit` and `motor.velocity_limit`.