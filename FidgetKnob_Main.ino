/*  A custom example of SmartKnob (and therefore SimpleFOC software) that contains all critical code to run the Fidget Knob within one .ino file.
    This program is subdivided into relevant classes for setup, configuration, data output/monitoring, and the actual haptic sensation.

    PINS:
    Obviously, your setup may differ from my own as it relates to the exact pin assignments for different components.
    I have tried to collect all relevant pin assignment variables at the top of the file TO BE CHANGED by each user and implementation.
    Other variables, such as maximum supply and driver voltage, can be changed within the program.
    
    DATA:
    This code also monitors key variables and outputs them to the serial terminal. These variables are internally averaged and filtered to create less output samples.    
    Further, we are not using built in 'monitoring' function as it is harder to customize the output and maneuver variables to our liking.
    The data output to the serial terminal is read by a separate Processing script. The original design logged files over a SPI SD card, but this was incompatible with the Fidget Knob haptic feedback.

    
*/
#include <SimpleFOC.h>
#include "Arduino.h"
#include <analogWrite.h>
#include <vector>
#include "SimpleFOCDrivers.h"
#include "encoders/MT6701/MagneticSensorMT6701SSI.h"
#include <sys/time.h>

// KEY VARIABLES TO BE CHANGED BY USER
static int configNumber = 9; //Total number of Fidget Knob mode configurations!

// ********************HAPTIC KNOB SETUP Variables ************************
// Hardware-specific motor calibration constants.
// Run calibration program (______________.name) at startup, then update these constants with the calibration results.
// NOTE: The calibration softwares are notoriously finnicky, please run it multiple times and take the average. Also free feel to adjust these values and try multiple offsets.
// One can also discover their motor pole pair number by following this tutorial from SimpleFOC: https://docs.simplefoc.com/example_from_scratch. Match 1 rotation per 1 second in Step 2 to accurately set pole pairs.
const float ZERO_ELECTRICAL_OFFSET = 0.80;
static const Direction FOC_DIRECTION = Direction::CW;
static const int MOTOR_POLE_PAIRS = 7;
// ####

// PIN NUMBERS
//Magentic Sensor Pins
static const int NCS_PIN = 5; // some digital pin that you're using as the nCS pin

// Motor Driver Board Pins
static const int UH_PIN = 32;
static const int UL_PIN = 33;
static const int VH_PIN = 25;
static const int VL_PIN = 26;
static const int WH_PIN = 27;
static const int WL_PIN = 14;

// Button Pin
static const int buttonPin = 22;

// LED Pins
const int ledRed = 13;         // the PWM pin the Red LED is attached to
const int ledBlue = 4;         // the PWM pin the Blue LED is attached to
const int ledGreen = 12;         // the PWM pin the Green LED is attached to

// ** RGB LED Setup Variables
const int colorRed[3] = {200,20,0};
const int colorOrange[3] = {150,210,48};
const int colorYellow[3] = {100,250,25};
const int colorGreen[3] = {20,250,50};
const int colorCyan[3] = {30,220,250};
const int colorBlue[3] = {0,70,250};
const int colorMagenta[3] = {120,50,150};
const int colorPurple[3] = {110,0,250};
const int colorLightWhite[3] = {15, 25, 20};
const int colorBlack[3] = {0,0,0};
int activeColor[3];
//int lightIncrement = 0;

//************** This is the end of the main chunk of variables you should be adjusting for setup and such. For mode configuration, see the "Configuration" function.*********

// RTC Stuff
time_t now;
struct tm timeInfo;
struct timeval tv;
String unixTime;

//Data Output Setups
//Storage - For easier processing and output within a single "print" statement (important to ensure quick processing of data monitoring loop so we don't mess up knob function)
char dataString[45];  // holds the data to be written to the file
String endChar = "*"; 
// Variables for later averaging and printing to serial (and thereby document)
float motorAngleOutput;
float motorVelocityOutput;
float motorPhaseA;
float motorPhaseB;
float motorPhaseC;
int counter;
int countMax;
int timeFlag;

// Device Mode and Haptics Configuration
int configSelect = 1;
float config[7];
int readBuffer = 0;
int pinState = 0;
int formerPinState = 0;

static const float DEAD_ZONE_DETENT_PERCENT = 0.2;
static const float DEAD_ZONE_RAD = 1 * _PI / 180;

static const float IDLE_VELOCITY_EWMA_ALPHA = 0.001;
static const float IDLE_VELOCITY_RAD_PER_SEC = 0.05;
static const uint32_t IDLE_CORRECTION_DELAY_MILLIS = 500;
static const float IDLE_CORRECTION_MAX_ANGLE_RAD = 5 * PI / 180;
static const float IDLE_CORRECTION_RATE_ALPHA = 0.0005;

// magnetic sensor instance - SPI
MagneticSensorMT6701SSI sensor(NCS_PIN);

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);
BLDCDriver6PWM driver = BLDCDriver6PWM(UH_PIN, UL_PIN, VH_PIN, VL_PIN, WH_PIN, WL_PIN);

// Set initial 2-detent mode
float current_detent_center = motor.shaft_angle;
float StartupMode[] = { 0, 0, (10 * PI / 180), 0, 1, 1.1, 1};  //free spinning with no detents
  // **STRUCTURE OF DETENT PARAMETER ARRAY**
  // int32_t num_positions; 0
  // int32_t position; 1
  // float position_width_radians; 2
  // float detent_strength_unit; 3
  // float endstop_strength_unit; 4
  // float snap_point; 5
  // float array identifier 6

float idle_check_velocity_ewma = 0;
uint32_t last_idle_start = 0;
uint32_t last_publish = 0;

void setup() {
  _delay(1000);
  // use monitoring with serial for motor init
  // monitoring port
  Serial.begin(115200);
  _delay(2000);

   // Configure LED Pins
  pinMode(ledRed, OUTPUT);
  pinMode(ledBlue, OUTPUT);
  pinMode(ledGreen, OUTPUT);

  // The below checks if the main button is depressed while the device boots up. If it is, we set the default time stamp and do not wait for serial connection from Processing.
  pinState = !digitalRead(buttonPin); 
  if (pinState == 1) {    //enables an independent mode of operation not tied to Processing and datalogging.
    unixTime = 1;
    //Flash blue LED to show we selected the independent mode
    for (int j = 0; j < 4; j++) {
      analogWrite(ledRed, (255-colorBlue[0]));
      analogWrite(ledGreen, (255-colorBlue[1]));
      analogWrite(ledBlue, (255-colorBlue[2]));
      delay(50);
      analogWrite(ledRed, (255));
      analogWrite(ledGreen, (255));
      analogWrite(ledBlue, (255));
      delay(50);
    }
    formerPinState = pinState;
  } 
  else {                  //initializes proper timestamp as received from Processing data logging
    int timeRead = 0;
    while (!Serial.available()) {
      delay(300);
      Serial.println("Serial unavailable");
    }
    while (Serial.available() > 0 && timeRead == 0) {
      unixTime = Serial.readString();
      Serial.println(unixTime);
      timeRead = 1;
    }
  }
 
  // CONFIGURATE HARDWARE LINKS
  // initialise magnetic sensor hardware
  sensor.init();
  // link the motor to the sensor
  motor.linkSensor(&sensor);
  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 10; //Set to your power supply voltage if you are using something different than prescribed. No need to supply more than 10V, really.
  driver.init();
  // link driver
  motor.linkDriver(&driver);

  // CONFIG FOC CONTROLS AND MODULATION
  // choose FOC modulation
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  // set control loop type to be used
  //motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::torque;

  // INITIALIZE FOC MOTOR
  // initialise motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC(ZERO_ELECTRICAL_OFFSET, FOC_DIRECTION);
  // set the inital target value
  motor.target = 2;
  Serial.println("Just initialized the FOC");

  // chosen variables to include in monitoring, were the standard library to be used.
  motor.monitor_variables = _MON_TARGET | _MON_VOLT_Q | _MON_VEL | _MON_ANGLE;  // default _MON_TARGET | _MON_VOLT_Q | _MON_VEL | _MON_ANGLE
  motor.monitor_downsample = 100;                                               // default 10, downsample output to serial

  _delay(2000);
  // **Here you can adjust the number of samples that are taken and averaged per each data output. Currently 50 samples (at 1000Hz) are included in each data output.
  counter = 0;
  countMax = 50;
  motorAngleOutput = 0.0;
  motorVelocityOutput = 0.0;
  motorPhaseA = 0.0;
  motorPhaseB = 0.0;
  motorPhaseC = 0.0;

  motor.voltage_limit = 5; //If your motor is drawing too much current, lower the voltage limit (and tune the PID parameters)
  motor.velocity_limit = 10000;

  // THese PID parameters can be tuned using the SimpleFOC studio (and a separate program) or just adjusted manually and recompiled. As that is time consuming, utilize SimpleFOC Studio if you will be doing lots of tuning.
  // SimpleFOC Studio: https://docs.simplefoc.com/studio 
  motor.PID_velocity.P = 4;
  motor.PID_velocity.I = 0.01;
  motor.PID_velocity.D = 0.001;
  motor.PID_velocity.output_ramp = 10000;
  motor.PID_velocity.limit = 6;
  motor.LPF_velocity.Tf = 0.001;

  motor.pole_pairs = MOTOR_POLE_PAIRS;
  motor.move(0); // initial zeroing move, before setting the mode
  
  // RTC Stuff - this is all to enable a real-time clock for accurate data matching with real-world occurences, if that interests you. Needs some work still, but functions with inital, manually seeded start time (below).
  int unixTimeConvert = unixTime.toInt();
  tv.tv_sec =	unixTimeConvert;  // enter UTC UNIX time (get it from https://www.unixtimestamp.com ) ***THIS CAN BE UPDATED BEFORE STARTING THE DEVICE TO TRACK THE REAL TIME. Future work will make this automatic/from system.
  settimeofday(&tv, NULL);

  // Set timezone to France (Europe/Paris or your own timezone, if you want)
  setenv("TZ", "CET-1CEST,M3.5.0/2,M10.5.0/ 3", 1); // https://www.gnu.org/software/libc/manual/html_node/TZ-Variable.html
  tzset();

  time(&now);
  localtime_r(&now, &timeInfo);
  Serial.print("The current time in Amsterdam is ");
  Serial.println(&timeInfo, "%H:%M:%S");
  //Serial.println(timeInfo.tm_sec);   //KEY FOR HOW TO ACCESS THE TIME INFO STRUCTURE
  //Serial.println(timeInfo.tm_min);
  //Serial.println(timeInfo.tm_hour);
  delay(1000);
  
  // setup button pin
  pinMode(buttonPin, INPUT);

  //populate configuration of the first haptic mode
  for (int h = 0; h < 7; h++) {
    config[h] = StartupMode[h];  //unbounded, no detents
  }


    // Update derivative factor of torque controller based on detent width.
    // If the D factor is large on coarse detents, the motor ends up making noise because the P&D factors amplify the noise from the sensor.
    // This is a piecewise linear function so that fine detents (small width) get a higher D factor and coarse detents get a small D factor.
    // Fine detents need a nonzero D factor to artificially create "clicks" each time a new value is reached (the P factor is small
    // for fine detents due to the smaller angular errors, and the existing P factor doesn't work well for very small angle changes (easy to
    // get runaway due to sensor noise & lag)).
    // TODO: consider eliminating this D factor entirely and just "play" a hardcoded haptic "click" (e.g. a quick burst of torque in each
    // direction) whenever the position changes when the detent width is too small for the P factor to work well.
    const float derivative_lower_strength = config[3] * 0.08;
    const float derivative_upper_strength = config[3] * 0.02;
    const float derivative_position_width_lower = radians(3);
    const float derivative_position_width_upper = radians(8);
    const float raw = derivative_lower_strength + (derivative_upper_strength - derivative_lower_strength) / (derivative_position_width_upper - derivative_position_width_lower) * (config[2] - derivative_position_width_lower);
    motor.PID_velocity.D = constrain(
      raw,
      min(derivative_lower_strength, derivative_upper_strength),
      max(derivative_lower_strength, derivative_upper_strength));

    Serial.print("This is the new D");
    Serial.println(motor.PID_velocity.D);   
    _delay(2500);

    //Flash green LED to show ready!
    for (int j = 0; j < 4; j++) {
      analogWrite(ledRed, (255-colorGreen[0]));
      analogWrite(ledGreen, (255-colorGreen[1]));
      analogWrite(ledBlue, (255-colorGreen[2]));
      delay(50);
      analogWrite(ledRed, (255));
      analogWrite(ledGreen, (255));
      analogWrite(ledBlue, (255));
      delay(50);
    }
    Serial.println("Ready!");
    delay(500);
    analogWrite(ledRed, (255-colorRed[0]));
    analogWrite(ledGreen, (255-colorRed[1]));
    analogWrite(ledBlue, (255-colorRed[2]));
}

void Configuration() {
  configSelect += readBuffer;
  //Serial.println("Entering Configuration Change");
  // Select configuration from button presses (my addition)
  if (configSelect <= configNumber) {
    // SETTING MODE
    //Serial.print("Got new config: ");
    // *****This is what each variable means for 
    // int32_t num_positions; 0
    // int32_t position; 1
    // float position_width_radians; 2
    // float detent_strength_unit; 3
    // float endstop_strength_unit; 4
    // float snap_point; 5
    // float array identifier 6
    //Serial.println(configSelect);
    //motor.move(ZERO_ELECTRICAL_OFFSET);
    current_detent_center = motor.shaft_angle;
    #if SK_INVERT_ROTATION
    current_detent_center = -motor.shaft_angle;
    #endif
    if (configSelect == 1) {
      float UnboundedNoDetentMode[] = { 0, 0, (10 * PI / 180), 0, 1, 1.1, 1}; //unbounded, no detents
      activeColor[0] = colorRed[0];                                           // set the active color array to contain the matching color
      activeColor[1] = colorRed[1];
      activeColor[2] = colorRed[2];      
      for (int i = 0; i < 7; i++) {
        config[i] = UnboundedNoDetentMode[i];  
      }
    } 
    else if (configSelect == 2) {
      float UnboundedWeakDetentMode[] = { 0, 0, (10 * _PI / 180), 0.2, 1, 1.1, 2 };  // unbounded rotation with weak detents
      activeColor[0] = colorOrange[0];
      activeColor[1] = colorOrange[1];
      activeColor[2] = colorOrange[2];
      for (int j = 0; j < 7; j++) {
        config[j] = UnboundedWeakDetentMode[j];
      }
    } 
    else if (configSelect == 3) {
      float UnboundedStrongDetentMode[] = { 0, 0, (10 * _PI / 180), 1, 1, 1.1, 3 };  //unbounded rotation with strong detents
      activeColor[0] = colorYellow[0];
      activeColor[1] = colorYellow[1];
      activeColor[2] = colorYellow[2];
      for (int k = 0; k < 7; k++) {
        config[k] = UnboundedStrongDetentMode[k];
      }
    } 
    else if (configSelect == 4) {
      float OnOffStrongMode[] = { 2, 0, (60 * _PI / 180), 1, 1, 0.55, 4 };  //single strong detent
      activeColor[0] = colorGreen[0];
      activeColor[1] = colorGreen[1];
      activeColor[2] = colorGreen[2];
      for (int l = 0; l < 7; l++) {
        config[l] = OnOffStrongMode[l];
      }
    } 
    else if (configSelect == 5) {
      float MultipleStrongMode[] = { 13, 0, (60 * _PI / 180), 1, 1, 0.55, 5 };  //multiple strong detents
      activeColor[0] = colorCyan[0];
      activeColor[1] = colorCyan[1];
      activeColor[2] = colorCyan[2];
      for (int l = 0; l < 7; l++) {
        config[l] = MultipleStrongMode[l];
      }
    } 
    else if (configSelect == 6) {
      float CenterModeSmall[] = { 1, 0, (30 * _PI / 180), 0.01, 0.7, 1.1, 6 };  //return to center small range
      activeColor[0] = colorBlue[0];
      activeColor[1] = colorBlue[1];
      activeColor[2] = colorBlue[2];
      for (int m = 0; m < 7; m++) {
        config[m] = CenterModeSmall[m];
      }
    } 
    /*else if (configSelect == 7) {
      float CenterModeLarge[] = { 1, 0, (180 * _PI / 180), 0.01, 0.7, 1.1, 7 };  //return to center large range
      for (int n = 0; n < 7; n++) {
        config[n] = CenterModeLarge[n];
      }
    } */
    else if (configSelect == 7) {
      float FineDetentMode[] = { 0, 0, (2 * _PI / 180), 1, 1, 1.0, 8 };  //fine values with detents
      activeColor[0] = colorMagenta[0];
      activeColor[1] = colorMagenta[1];
      activeColor[2] = colorMagenta[2];
      for (int p = 0; p < 7; p++) {
        config[p] = FineDetentMode[p];
      }
    } 
    else if (configSelect == 8) {
      float CoarseStrongMode[] = { 0, 0, (8.225806452 * _PI / 180), 2, 1, 1.1, 9 };  //coarse values with strong detents
      activeColor[0] = colorPurple[0];
      activeColor[1] = colorPurple[1];
      activeColor[2] = colorPurple[2];
      for (int q = 0; q < 7; q++) {
        config[q] = CoarseStrongMode[q];
      }
    } 
    else if (configSelect == 9) {
      static float CoarseWeakMode[] = { 0, 0, (10 * _PI / 180), 0.4, 1, 0.8, 10 };  //coarse values with weak detents
      activeColor[0] = colorLightWhite[0];
      activeColor[1] = colorLightWhite[1];
      activeColor[2] = colorLightWhite[2];
      for (int r = 0; r < 7; r++) {
        config[r] = CoarseWeakMode[r];
      }
    } 
    else {
      Serial.println("uh oh spaghetti o, we have a problem! configSelect does not equal 1 to 9");
    }

    // Update derivative factor of torque controller based on detent width.
    // If the D factor is large on coarse detents, the motor ends up making noise because the P&D factors amplify the noise from the sensor.
    // This is a piecewise linear function so that fine detents (small width) get a higher D factor and coarse detents get a small D factor.
    // Fine detents need a nonzero D factor to artificially create "clicks" each time a new value is reached (the P factor is small
    // for fine detents due to the smaller angular errors, and the existing P factor doesn't work well for very small angle changes (easy to
    // get runaway due to sensor noise & lag)).
    const float derivative_lower_strength = config[3] * 0.08;
    const float derivative_upper_strength = config[3] * 0.02;
    const float derivative_position_width_lower = radians(3);
    const float derivative_position_width_upper = radians(8);
    const float raw = derivative_lower_strength + (derivative_upper_strength - derivative_lower_strength) / (derivative_position_width_upper - derivative_position_width_lower) * (config[2] - derivative_position_width_lower);
    motor.PID_velocity.D = constrain(
      raw,
      min(derivative_lower_strength, derivative_upper_strength),
      max(derivative_lower_strength, derivative_upper_strength));
    readBuffer = 0;
    delay(2);
    //Change the LED color (write)
    analogWrite(ledRed, (255-activeColor[0]));
    analogWrite(ledGreen, (255-activeColor[1]));
    analogWrite(ledBlue, (255-activeColor[2]));
    delay(2);
  } 
  else if (configSelect >= (configNumber + 1)) {
    configSelect = 1;
    readBuffer = 0;
    //Serial.println("resetting configSelect");
    Configuration();
  } 
  else {
    Serial.println("error with the configSelect counter within the configuration function");
  }
}

void CustomMonitoring() {
  // ************** My data monitoring code**************

  if (counter < countMax) {
    motorAngleOutput += motor.shaft_angle;
    motorVelocityOutput += motor.shaft_velocity;
    motorPhaseA += motor.Ua;
    motorPhaseB += motor.Ub;
    motorPhaseC += motor.Uc;
    counter++;
  } 
  else if (counter == countMax) {
    float angAvg = motorAngleOutput / countMax;
    float velAvg = motorVelocityOutput / countMax;
    float uaAvg = motorPhaseA / countMax;
    float ubAvg = motorPhaseB / countMax;
    float ucAvg = motorPhaseC / countMax;

    //dtostrf(float_value, min_width, num_digits_after_decimal, where_to_store_string)
    char angleConvert[9];
    char velocityConvert[9];
    char uaConvert[7];
    char ubConvert[7];
    char ucConvert[7];
    dtostrf(angAvg, 7, 3, angleConvert);
    dtostrf(velAvg, 7, 3, velocityConvert);
    dtostrf(uaAvg, 5, 2, uaConvert);
    dtostrf(ubAvg, 5, 2, ubConvert);
    dtostrf(ucAvg, 5, 2, ucConvert);
    //endChar = "*";
    sprintf(dataString, "%02d:%02d:%02d,%d,%s,%s,%s,%s,%s,%d,%d", timeInfo.tm_hour, timeInfo.tm_min, timeInfo.tm_sec, millis(), angleConvert, velocityConvert, uaConvert, ubConvert, ucConvert, configSelect, timeFlag);
    Serial.println(dataString);

    //reset variables
    motorAngleOutput = 0;
    motorVelocityOutput = 0;
    motorPhaseA = 0;
    motorPhaseB = 0;
    motorPhaseC = 0;
    counter = 0;
  } 
  else {
    Serial.println("Big error uh oh oh boy, a counter has messed up!!");
  }
}

//The main motor task
void MotorTask() {
  // Must call this to initiate the entire FOC process
  motor.loopFOC();
  // If we are not moving and we're close to the center (but not exactly there), slowly adjust the centerpoint to match the current position
  idle_check_velocity_ewma = motor.shaft_velocity * IDLE_VELOCITY_EWMA_ALPHA + idle_check_velocity_ewma * (1 - IDLE_VELOCITY_EWMA_ALPHA);
  if (fabsf(idle_check_velocity_ewma) > IDLE_VELOCITY_RAD_PER_SEC) {
    last_idle_start = 0;
  } 
  else {
    if (last_idle_start == 0) {
      last_idle_start = millis();
    }
  }
  if (last_idle_start > 0 && millis() - last_idle_start > IDLE_CORRECTION_DELAY_MILLIS && fabsf(motor.shaft_angle - current_detent_center) < IDLE_CORRECTION_MAX_ANGLE_RAD) {
    current_detent_center = motor.shaft_angle * IDLE_CORRECTION_RATE_ALPHA + current_detent_center * (1 - IDLE_CORRECTION_RATE_ALPHA);
  }

  // Check where we are relative to the current nearest detent; update our position if we've moved far enough to snap to another detent
  float angle_to_detent_center = motor.shaft_angle - current_detent_center;
  #if SK_INVERT_ROTATION
  angle_to_detent_center = -motor.shaft_angle - current_detent_center;
  #endif
  if (angle_to_detent_center > config[2] * config[5] && (config[0] <= 0 || config[1] > 0)) {
    current_detent_center += config[2];
    angle_to_detent_center -= config[2];
    config[1]--;
  } 
  else if (angle_to_detent_center < -config[2] * config[5] && (config[0] <= 0 || config[1] < config[0] - 1)) {
    current_detent_center -= config[2];
    angle_to_detent_center += config[2];
    config[1]++;
  }

  float dead_zone_adjustment = constrain(
    angle_to_detent_center,
    fmaxf(-config[2] * DEAD_ZONE_DETENT_PERCENT, -DEAD_ZONE_RAD),
    fminf(config[2] * DEAD_ZONE_DETENT_PERCENT, DEAD_ZONE_RAD));

  bool out_of_bounds = config[0] > 0 && ((angle_to_detent_center > 0 && config[1] == 0) || (angle_to_detent_center < 0 && config[1] == config[0] - 1));
  motor.PID_velocity.limit = 10;  //out_of_bounds ? 10 : 3;
  motor.PID_velocity.P = out_of_bounds ? config[4] * 4 : config[3] * 4;


  // Apply motor torque based on our angle to the nearest detent (detent strength, etc is handled by the PID_velocity parameters)
  if (fabsf(motor.shaft_velocity) > 60) {
    // Don't apply torque if velocity is too high (helps avoid positive feedback loop/runaway)
    motor.move(0);
  } 
  else {
    float torque = motor.PID_velocity(-angle_to_detent_center + dead_zone_adjustment);
    #if SK_INVERT_ROTATION
    torque = -torque;
    #endif
    motor.move(torque);
  }

  // Output to the serial command line, only if Monitoring isnt being used!
  /*if (millis() - last_publish > 5) {
    int32_t current_position = config[1];
    float sub_position_unit = -angle_to_detent_center / config[2];
    bool has_config = true;
    Serial.print(current_position);
    Serial.print(", ");
    Serial.print(sub_position_unit);
    Serial.print(", ");
    Serial.println(config[6]);
    last_publish = millis();
  }*/

  //motor.monitor();
  //Serial.println("End of motortask function");
  delay(1);
}

void loop() {

  // Determine if a button has been pressed to switch modes
  pinState = !digitalRead(buttonPin);
  if (pinState != formerPinState && pinState == 1) {
    readBuffer += pinState;
    Configuration();
    //Serial.println(pinState);
    formerPinState = pinState;
  } 
  else {
    formerPinState = pinState;
  }

  //CALL THE MOTOR TASK HERE!
  MotorTask();
  //Serial.println("Looping Iteration!");

  // CALL THE CUSTOM MONITORING TASK HERE
  CustomMonitoring();

  //Output to LEDs here
  //analogWrite(ledRed, (255-activeColor[0]));
  //analogWrite(ledGreen, (255-activeColor[1]));
  //analogWrite(ledBlue, (255-activeColor[2]));

  // Call Command Interface
  // user communication
  // commander.run();
}