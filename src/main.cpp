// Open loop motor control example
#include <SimpleFOC.h>


// BLDC motor & driver instance
// BLDCMotor motor = BLDCMotor(pole pair number);
BLDCMotor motor = BLDCMotor(7); // Gimbal motor
// BLDCDriver3PWM driver = BLDCDriver3PWM(pwmA, pwmB, pwmC, Enable(optional));
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);

//target variables
float target_velocity = 0;
float target_position = 0;
// Velocity 0, Position 1
float mode = 0; 

// instantiate the commander
Commander command = Commander(Serial);
void doTargetVelocity(char* cmd) { command.scalar(&target_velocity, cmd); }
void doTargetPosition(char* cmd) { command.scalar(&target_position, cmd); }
void doMode(char* cmd) { command.scalar(&mode, cmd); Serial.print("Mode ");Serial.println(mode);}

void doVoltageLimit(char* cmd) { command.scalar(&motor.voltage_limit, cmd); }
void doVelocityLimit(char* cmd) { command.scalar(&motor.velocity_limit, cmd); }

void moveMode() {
  if (mode == 0) {
    motor.controller = MotionControlType::velocity_openloop;
    motor.move(target_velocity);
  }
  else {
    motor.controller = MotionControlType::angle_openloop;
    motor.move(target_position);
  } 
}

void setup() {

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 8;
  // limit the maximal dc voltage the driver can set
  // as a protection measure for the low-resistance motors
  // this value is fixed on startup
  driver.voltage_limit = 6;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);

  // limiting motor movements
  // limit the voltage to be set to the motor
  // start very low for high resistance motors
  // current = voltage / resistance, so try to be well under 1Amp
  motor.voltage_limit = 1;   // [V]
 
  // open loop control config
  motor.controller = MotionControlType::velocity_openloop;
  // motor.controller = MotionControlType::angle_openloop;

  // init motor hardware
  motor.init();

  // add target command T
  command.add('V', doTargetVelocity, "target velocity");
  command.add('P', doTargetPosition, "target angle");
  command.add('M', doMode, "mode");

  command.add('L', doVoltageLimit, "voltage limit");
  command.add('S', doVelocityLimit, "velocity limit");


  Serial.begin(115200);
  Serial.println("Motor ready!");
  if (mode == 0) {
    Serial.println("Set target velocity [rad/s]");
  } else {
    Serial.println("Set target position [rad]");
  }
  
  _delay(1000);
}

void loop() {

  // open loop velocity movement
  // using motor.voltage_limit and motor.velocity_limit
  // motor.move(target_velocity);
  // motor.move(target_position);
  moveMode();

  // user communication
  command.run();
}
