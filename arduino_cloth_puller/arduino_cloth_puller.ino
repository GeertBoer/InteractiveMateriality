#include <SimpleFOC.h>

// magnetic sensor instance - SPI
MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, PA4);

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(PC6, PC7, PC8, PB15);

void setup() {
  // initialise magnetic sensor hardware
  sensor.init();
  // link the motor to the sensor
  motor.linkSensor(&sensor);
  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);
  // choose FOC modulation (optional)
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  // set motion control loop to be used
  motor.controller = MotionControlType::angle;
  // velocity PI controller parameters
  motor.PID_velocity.P = 0.2f;
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 0;
  // maximal voltage to be set to the motor
  motor.voltage_limit = 12;
  // velocity low pass filtering time constant
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.01f;
  // angle P controller
  motor.P_angle.P = 20;
  // maximal velocity of the position control
  motor.velocity_limit = 20;
  // use monitoring with serial
  Serial.begin(115200);
  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

}

bool dir = true;

unsigned long oldmillis = 0;
float wait = 1000;
int ctr = 0;
void loop() {
  motor.loopFOC();
  sensor.update();

  if (dir) {
    motor.move(-6.0);
  }
  else {
    motor.move(3.0);
  }

  if ((millis() - oldmillis) > wait) {
    Serial.println(sensor.getAngle());
    dir = !dir;
    oldmillis = millis();
    ctr++;
  }
}
