/* Adapted SmartKnob calibration program for my own motor as a standalone program within the Arduino IDE
*/

#include <SimpleFOC.h>
#include "Arduino.h"
#include <vector>
#include "SimpleFOCDrivers.h"
#include "encoders/MT6701/MagneticSensorMT6701SSI.h"

// magnetic sensor instance - SPI
#define SENSOR1_CS 5  // some digital pin that you're using as the nCS pin
MagneticSensorMT6701SSI sensor(SENSOR1_CS);

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(1);
BLDCDriver6PWM driver = BLDCDriver6PWM(32, 33, 25, 26, 27, 14); //THE U, V, AND W PHASE PINS FROM THE MAIN PROGRAM

int runCheck;
float a = 0.0;

void setup() {
  // put your setup code here, to run once:
  runCheck = 0;
  _delay(1000);
  Serial.begin(115200);
  
  //Initialize magnetic sensor
  sensor.init();

  // link driver
  driver.voltage_limit = 5;
  driver.voltage_power_supply = 10;

  driver.init();
  
  motor.linkDriver(&driver);
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // choose FOC modulation
  //motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::angle_openloop;
  motor.pole_pairs = 1;

  //initialize motor
  motor.init();
  motor.initFOC(0, Direction::CW);
  //motor.initFOC();
}


void MotorCalibrate() {
    while (runCheck < 1) {
      // SimpleFOC is supposed to be able to determine this automatically (if you omit params to initFOC), but
      // it seems to have a bug (or I've misconfigured it) that gets both the offset and direction very wrong!
      // So this value is based on experimentation.
      // TODO: dig into SimpleFOC calibration and find/fix the issue

      _delay(3000);
      Serial.println("Starting calibration, please DO NOT TOUCH MOTOR until complete!");

      // #### Determine direction motor rotates relative to angle sensor
      for (uint8_t i = 0; i < 200; i++) {
          sensor.update();
          motor.move(a);
          delay(1);
      }
      Serial.println("Sensor moved to 0");
      float start_sensor = sensor.getAngle();
      Serial.println(start_sensor);

      for (a; a < (3 * _2PI); a += 0.01) {
          sensor.update();
          motor.move(a);
          delay(1);
      }

      for (uint8_t i = 0; i < 200; i++) {
          sensor.update();
          delay(1);
      }
      float end_sensor = sensor.getAngle();
      Serial.println(end_sensor);


      motor.voltage_limit = 4;
      motor.move(a);

      Serial.println("");

      // TODO: check for no motor movement!

      Serial.println("Sensor measures positive for positive motor rotation:");
      if (end_sensor > start_sensor) {
          Serial.println("YES, Direction=CW");
          motor.initFOC(0, Direction::CW);
      } else {
          Serial.println("NO, Direction=CCW");
          motor.initFOC(0, Direction::CCW);
      }


      // #### Determine pole-pairs
      // Rotate 20 electrical revolutions and measure mechanical angle traveled, to calculate pole-pairs
      uint8_t electrical_revolutions = 20;
      //snprintf(buf_, sizeof(buf_), "Going to measure 20 electrical revolutions...", electrical_revolutions);
      Serial.println("Going to measure 20 electrical revolutions...");
      //print_buf(buf_);
      motor.voltage_limit = 4;
      motor.move(a);
      Serial.println("Going to electrical zero...");
      float destination = a + _2PI;
      for (a; a < destination; a += 0.03) {
          sensor.update();
          motor.move(a);
          delay(1);
      }
      Serial.println("pause..."); // Let momentum settle...
      for (uint16_t i = 0; i < 1000; i++) {
          sensor.update();
          delay(1);
      }
      Serial.print("Measuring... Sensor Angle @ electrical zero = ");
      Serial.println(sensor.getAngle());

      start_sensor = motor.sensor_direction * sensor.getAngle();
      destination = a + electrical_revolutions * _2PI;
      for (a; a < destination; a += 0.03) {
          sensor.update();
          motor.move(a);
          delay(1);
      }
      for (uint16_t i = 0; i < 1000; i++) {
          sensor.update();
          motor.move(a);
          delay(1);
      }
      // after this movement direction, we should have arrive at 24 total rotations!
      end_sensor = motor.sensor_direction * sensor.getAngle();
      motor.voltage_limit = 0;
      Serial.print("Prior 'a' (target) = ");
      Serial.println(a);
      Serial.println(sensor.getAngle());

      motor.move(a);

      Serial.print("Current 'a' (target) = ");
      Serial.println(a);
      Serial.println(sensor.getAngle());

      Serial.println("Evaluation for loop = ");
      Serial.println(motor.shaft_angle);
      Serial.println(motor.target);
      Serial.println(fabsf(motor.shaft_angle - motor.target));

      if (fabsf(motor.shaft_angle - motor.target) > 1 * PI / 180) {
          Serial.println("ERROR: motor did not reach target!");
          //while(1) {}
      }

      float electrical_per_mechanical = electrical_revolutions * _2PI / (end_sensor - start_sensor);
      Serial.print("Electrical angle / mechanical angle (i.e. pole pairs) = ");
      Serial.println(electrical_per_mechanical);

      //int measured_pole_pairs = (int)round(electrical_per_mechanical);          
      //***********************This is where one should apply the Pole Pairs from the calibration program, and I have simply overwritten it with the known pole pairs I have from prior calibrations. This number is a hardware thing and will never change for any one motor design.******************
      int measured_pole_pairs = 7;
      Serial.print("Pole pairs set to: ");
      Serial.println(measured_pole_pairs);

      delay(1000);

      // #### Determine mechanical offset to electrical zero
      // Measure mechanical angle at every electrical zero for several revolutions
      motor.voltage_limit = 5;
      motor.move(a);
      float offset_x = 0;
      float offset_y = 0;
      float destination1 = (floor(a / _2PI) + measured_pole_pairs / 2.) * _2PI;
      float destination2 = (floor(a / _2PI)) * _2PI;
      Serial.println("Start Destination 1 Move");
      for (a; a < destination1; a += 0.4) {
          motor.move(a);
          delay(100);
          for (uint8_t i = 0; i < 100; i++) {
              sensor.update();
              delay(3);
          }
          float real_electrical_angle = _normalizeAngle(a);
          float measured_electrical_angle = _normalizeAngle( (float)(motor.sensor_direction * measured_pole_pairs) * sensor.getMechanicalAngle()  - 0);

          float offset_angle = measured_electrical_angle - real_electrical_angle;
          offset_x += cosf(offset_angle);
          offset_y += sinf(offset_angle);

          /*Serial.print(degrees(real_electrical_angle));
          Serial.print(", ");
          Serial.print(degrees(measured_electrical_angle)); 
          Serial.print(", ");  
          Serial.println(degrees(_normalizeAngle(offset_angle))); 
          */
      }
      Serial.println("Start Destination 2 Move");
      for (a; a > destination2; a -= 0.4) {
          motor.move(a);
          delay(100);
          for (uint8_t i = 0; i < 100; i++) {
              sensor.update();
              delay(3);
          }
          float real_electrical_angle = _normalizeAngle(a);
          float measured_electrical_angle = _normalizeAngle( (float)(motor.sensor_direction * measured_pole_pairs) * sensor.getMechanicalAngle()  - 0);

          float offset_angle = measured_electrical_angle - real_electrical_angle;
          offset_x += cosf(offset_angle);
          offset_y += sinf(offset_angle);

          /*Serial.print(", ");
          Serial.print(degrees(measured_electrical_angle)); 
          Serial.print(", ");  
          Serial.println(degrees(_normalizeAngle(offset_angle)));   
          */
      }

      Serial.println(a);
      motor.voltage_limit = 0;
      motor.move(a);

      float avg_offset_angle = atan2f(offset_y, offset_x);


      // #### Apply settings
      // TODO: save to non-volatile storage
      motor.pole_pairs = measured_pole_pairs;
      motor.zero_electric_angle = avg_offset_angle + _3PI_2;
      motor.voltage_limit = 3;
      motor.controller = MotionControlType::torque;

      //Serial.println("\n\nRESULTS:\n Update these constants at the top of " __FILE__);
      Serial.print("  ZERO_ELECTRICAL_OFFSET: ");
      Serial.println(motor.zero_electric_angle);
      //print_buf(buf_);
      if (motor.sensor_direction == Direction::CW) {
          Serial.println("  FOC_DIRECTION: Direction::CW");
      } else {
          Serial.println("  FOC_DIRECTION: Direction::CCW");
      }
      Serial.print("  MOTOR_POLE_PAIRS: ");
      Serial.println(motor.pole_pairs);
      //print_buf(buf_);
      runCheck++;
      a = 0.0;
      delay(2000);
    }
}



void loop() {
  // put your main code here, to run repeatedly:
  MotorCalibrate();
}
