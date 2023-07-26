/**
 *
 * Position/angle motion control example
 * Steps:
 * 1) Configure the motor and magnetic sensor
 * 2) Run the code
 * 3) Set the target angle (in radians) from serial terminal
 *
 */
#include <SimpleFOC.h>

// magnetic sensor instance - SPI
// MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, 10);
// magnetic sensor instance - MagneticSensorI2C
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// magnetic sensor instance - analog output
// MagneticSensorAnalog sensor = MagneticSensorAnalog(A1, 14, 1020);

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7); // Gimbal motor
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);

// angle set point variable
float target = 0;

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target, cmd); }

void setup() {

  // initialise magnetic sensor hardware
  sensor.init();
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 8;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);

  // choose FOC modulation (optional)
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // Set motion control loop to be used. Pick only one of these
  // motor.controller = MotionControlType::torque;
  // motor.controller = MotionControlType::velocity;
  motor.controller = MotionControlType::angle;

  // contoller configuration
  // default parameters in defaults.h

  // velocity PI controller parameters
  motor.PID_velocity.P = 0.2f;
  motor.PID_velocity.I = 4.0f;
  motor.PID_velocity.D = 0;

  // velocity low pass filtering time constant
  // default 5ms - try different values to see what is the best.
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.01f;

  // maximal voltage to be set to the motor
  motor.voltage_limit = 3;

  // angle P controller
  motor.P_angle.P = 8;
  // maximal velocity of the position control
  motor.velocity_limit = 2;

  // use monitoring with serial
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC(6.18, Direction::CW);

  // add target command T
  // command.add('T', doTarget, "target voltage");
  // command.add('T', doTarget, "target velocity");
  command.add('T', doTarget, "target angle");

  // Serial.println(motor.zero_electric_angle);
  // Serial.println(motor.sensor_direction);

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target angle using serial terminal (Degrees):"));
  // Serial.println(F("Set the target voltage using serial terminal:"));
  // Serial.println(F("Set the target velocity using serial terminal (Radians):"));
  _delay(1000);
}

int count = 0;
void loop() {

  // motor.PID_velocity.I = target;

  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz
  motor.loopFOC();

  // if (count > 1000) {
  //   Serial.print(motor.PID_velocity.P);Serial.print(" : ");Serial.println(motor.PID_velocity.I);
  //   count = 0;
  // }
  // count++;
  
  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor.move(target * (_PI/180)); // Use degrees instead of radians
  // motor.move(target);

  // motor.useMonitoring(Serial);
  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  motor.monitor();

  // user communication
  command.run();
}