#include <math.h>


//L298 CHIP-1 PINS
const int ENA_1 = 9;  //PWM
const int IN1_1 = 12;
const int IN2_1 = 11;
const int IN3_1 = 13;
const int IN4_1 = 8;
const int ENB_1 = 10;  //PWM

//L298 CHIP-2 PINS
const int ENA_2 = 6;  //PWM
const int IN1_2 = 7;
const int IN2_2 = 5;
const int IN3_2 = 4;
const int IN4_2 = 2;
const int ENB_2 = 3;  //PWM

//POTENTIOMETER PIN
const int pot_pin = A0;

//LED PINS STATUS AND DIRECTION
const int s_green_pin = A5;
const int s_yellow_pin = A4;
const int s_red_pin = A3;

const int d_blue_pin = A1;
const int d_amber_pin = A2;

//Other variables
int motor_speed = 0;
int prevP_Value = 0;
int hover_power = 128;      //50% pwm
int upThrust_power = 204;   // 80% pwm
int downThrust_power = 77;  //30% pwm

const float target_height = 0.39;  // Target height (0.5M) in meters.
bool target_height_reached = false;
bool total_time_recorded = false;
bool shut_down = false;
bool error_correction = false;
float target_height_reached_time = 0;

unsigned long previousTime = 0;
unsigned long first_height_reached_time = 0;
unsigned long hover_time = 0;
float maximum_error = 0;
float average_error = 0;
float Total_time = 0;
float Height_error = 0;
float angle = 0.00;
float target_height_angle = 0;
float drone_height = 0.00;
float first_height_reached = 0.00;



void setup() {
  Serial.begin(9600);

  pinMode(pot_pin, INPUT);
  //L298 CHIP 1
  pinMode(ENA_1, OUTPUT);
  pinMode(IN1_1, OUTPUT);
  pinMode(IN2_1, OUTPUT);
  pinMode(IN3_1, OUTPUT);
  pinMode(IN4_1, OUTPUT);
  pinMode(ENB_1, OUTPUT);
  //L298 CHIP 2
  pinMode(ENA_2, OUTPUT);
  pinMode(IN1_2, OUTPUT);
  pinMode(IN2_2, OUTPUT);
  pinMode(IN3_2, OUTPUT);
  pinMode(IN4_2, OUTPUT);
  pinMode(ENB_2, OUTPUT);

  //SETTING ANALOG PINS AS DIGITAL PINS
  pinMode(s_green_pin, OUTPUT);
  pinMode(s_yellow_pin, OUTPUT);
  pinMode(s_red_pin, OUTPUT);
  pinMode(d_blue_pin, OUTPUT);   //upward direction indicator
  pinMode(d_amber_pin, OUTPUT);  //Downward direction indicator

  //Initialise other led to be off
  digitalWrite(s_yellow_pin, LOW);
  digitalWrite(s_red_pin, LOW);
  digitalWrite(d_blue_pin, LOW);
  digitalWrite(d_amber_pin, LOW);

  //Light green led for 0.5 sec to show system has started
  digitalWrite(s_green_pin, HIGH);
  delay(500);
  digitalWrite(s_green_pin, LOW);
  motor_speed = downThrust_power;

  Serial.println("1. System Started.");
  Serial.println("2. System Initiated.");
  Serial.println("3. Controller Starting.");
  Serial.println(" ");
  Serial.println("Time,      Power,    Angle,    Height,    Error");

  //Call drive_motors function to set its speed to 0 (OFF)
  drive_motors();
}

void loop() {
  if (shut_down) return;
  if (target_height > 0.491) {  //Target height should not ecxeed hypotenuse for asin() function to work.
    Serial.print("ERROR! TARGET HEIGHT SHOULD NOT EXCEED HYPOTENUSE 0.491M");
    shut_down = true;
  }

  unsigned long currentTime = millis();                 //---------------------------------------------------------------------------------------------
  if (currentTime - previousTime > 39 && !shut_down) {  //Non_blocking condition for 25Hz.
    previousTime = currentTime;

    int p_Value = analogRead(pot_pin);  // Read potentiometer ADC value.

    //Limit potentiometer values
    if (p_Value < 212) p_Value = 212;
    if (p_Value > 410) p_Value = 410;


    if (!error_correction) {
      angle = ((float)p_Value - 410) * (-68.5 - 0) / (212 - 410);
      //angle = ((float)p_Value - 410) * (0 - (-68.5)) / (212 - 410);  //Angle in degrees
      float angle_radians = (angle * PI / 180);   // Convert angle degrees to radians.
      drone_height = 0.491 * sin(angle_radians);  //Realtime drone height in meters.
      Height_error = abs(drone_height) - target_height;
    }


    //CONTROLL POWER TO THE MOTORS
    if (p_Value > prevP_Value) {  // 70% power when potentiometer value is increasing (UP thrust > drone weight), UPWARD MOTION.
      motor_speed = upThrust_power;
      digitalWrite(d_amber_pin, LOW);  //Downward direction indicator
      digitalWrite(d_blue_pin, HIGH);  //Upward direction indicator
      first_height_reached_time = currentTime;
      //Serial.print("Upward motion");
    } else if (p_Value < prevP_Value) {  // 30% power when potentiometer value is decreasing (UP thrust < drone weight), DOWNWARD MOTION
      motor_speed = downThrust_power;
      digitalWrite(d_amber_pin, HIGH);  //Downward direction indicator
      digitalWrite(d_blue_pin, LOW);    //Upward direction indicator
      first_height_reached_time = currentTime;
    } else {
      motor_speed = hover_power;       // 50% power (UP thrust = drone weight), Assumed simulated power for drone HOVER.
      digitalWrite(d_amber_pin, LOW);  //Downward direction indicator
      digitalWrite(d_blue_pin, LOW);   //Upward direction indicator
    }
    prevP_Value = p_Value;


    // LIGHT APPROPRIATE STATUS LED AND ERROR CORRECTIOn ========================================================================
    if (abs(Height_error) <= 0.02) {
      // TARGET HEIGHT REACHED (within +-20mm)
      digitalWrite(s_red_pin, LOW);
      digitalWrite(s_yellow_pin, LOW);
      digitalWrite(s_green_pin, HIGH);
      target_height_reached = true;
      motor_speed = hover_power;
      //Ensure total time to reach target height is recorded ones the target height is reached
      if (!total_time_recorded) {
        target_height_reached_time = currentTime;
        total_time_recorded = true;
      }
      digitalWrite(d_amber_pin, LOW);  //Downward direction indicator
      digitalWrite(d_blue_pin, LOW);   //Upward direction indicator

    } else if (Height_error < 0.02) {
      // ABOVE TARGET HEIGHT
      digitalWrite(s_red_pin, HIGH);
      digitalWrite(s_yellow_pin, LOW);
      digitalWrite(s_green_pin, LOW);
      target_height_reached = false;
      total_time_recorded = false;
      maximum_error = Height_error;
      first_height_reached = drone_height;
      if (!target_height_reached && first_height_reached && currentTime - first_height_reached_time >= 2000 && p_Value < 410) {
        get_angleFor_target_height();
        float error_angle = target_height_angle - abs(angle);
        angle = angle + (-error_angle);
        float angle_radians = (angle * PI / 180);  // Convert angle degrees to radians.
        drone_height = 0.491 * sin(angle_radians);
        Height_error = abs(drone_height) - target_height;
        motor_speed = downThrust_power;  //Reduce motor power to decend.
        error_correction = true;

        digitalWrite(d_amber_pin, HIGH);  //Downward direction indicator
        digitalWrite(d_blue_pin, LOW);    //Upward direction indicator
      }

    } else {
      // BELOW TARGET HEIGHT
      digitalWrite(s_red_pin, LOW);
      digitalWrite(s_yellow_pin, HIGH);
      digitalWrite(s_green_pin, LOW);
      target_height_reached = false;
      total_time_recorded = false;
      maximum_error = Height_error;
      first_height_reached = drone_height;
      if (!target_height_reached && first_height_reached && currentTime - first_height_reached_time >= 2000 && p_Value > 212) {
        get_angleFor_target_height();
        float error_angle =  abs(angle) - target_height_angle;
        angle = angle + error_angle;
        float angle_radians = (angle * PI / 180);  // Convert angle degrees to radians.
        drone_height = 0.491 * sin(angle_radians);
        Height_error = abs(drone_height) - target_height;
        motor_speed = upThrust_power;  //Increase motor power to ascend.
        error_correction = true;

        digitalWrite(d_amber_pin, LOW);  //Downward direction indicator
        digitalWrite(d_blue_pin, HIGH);    //Upward direction indicator
      }
    }



    drive_motors();                                           // call this function to update speed on the motors.
    int power_percentage = map(motor_speed, 0, 255, 0, 100);  //Map motor speed to a scale of (0 - 100%).


    //PRINT BLACKBOX DATA ON THE SERIAL.  target_height_angle
    Serial.print(currentTime / 1000.00);  //convert miliseconds to seconds /1000
    Serial.print(",      ");
    Serial.print(power_percentage);
    Serial.print(",      ");
    Serial.print(angle);
    Serial.print(",      ");
    Serial.print(drone_height);
    Serial.print(",      ");
    Serial.print(Height_error);
    Serial.print("\n ");

    if (target_height_reached && (currentTime - target_height_reached_time >= 5000)) {
      stop();
      shut_down = true;
    }
  }
}

void drive_motors() {
  //Checks motor speed
  analogWrite(ENA_1, motor_speed);  //Motor 1
  analogWrite(ENB_1, motor_speed);  //Motor 2
  analogWrite(ENA_2, motor_speed);  //Motor 3
  analogWrite(ENB_2, motor_speed);  //Motor 4

  //Foward Direction
  digitalWrite(IN1_1, LOW);
  digitalWrite(IN2_1, HIGH);  //Motor 1
  digitalWrite(IN3_1, LOW);
  digitalWrite(IN4_1, HIGH);  //Motor 2
  digitalWrite(IN1_2, LOW);
  digitalWrite(IN2_2, HIGH);  //Motor 3
  digitalWrite(IN3_2, LOW);
  digitalWrite(IN4_2, HIGH);  //Motor 4
}
//Function to Get maximum angle required for the set target height
void get_angleFor_target_height() {
  float hypotenuse = 0.491;
  float opposite_height = target_height;
  float ratio = opposite_height / hypotenuse;
  float angle_radians = asin(ratio);
  target_height_angle = angle_radians * 180.0 / PI;
}

//Function to shutdown the drone
void stop() {
  digitalWrite(s_yellow_pin, HIGH);
  digitalWrite(s_red_pin, HIGH);
  digitalWrite(s_green_pin, HIGH);

  Serial.println("4. Shuting down...");
  Serial.print("Time taken to reach Maximum height: ");
  Serial.print(target_height_reached_time / 1000.00);
  Serial.println("s");
  Serial.print("Current Angle: ");
  Serial.print(angle);
  Serial.println("°");
  Serial.println("");
  Serial.print("First Height reached: ");
  Serial.print(first_height_reached);
  Serial.println("m");
  Serial.print("Height reached After correction: ");
  Serial.print(drone_height);
  Serial.println("m");
  Serial.println("");
  Serial.print("Maximum error encountered: ");
  Serial.print(maximum_error);
  Serial.println("m");
  Serial.print("Error after correction: ");
  Serial.print(Height_error);
  Serial.println("m");

  delay(1000);

  digitalWrite(s_yellow_pin, LOW);
  digitalWrite(s_red_pin, LOW);
  digitalWrite(s_green_pin, LOW);

  digitalWrite(d_amber_pin, LOW);  //Downward direction indicator
  digitalWrite(d_blue_pin, LOW);

  digitalWrite(IN1_1, LOW);
  digitalWrite(IN2_1, LOW);  //Motor 1
  digitalWrite(IN3_1, LOW);
  digitalWrite(IN4_1, LOW);  //Motor 2
  digitalWrite(IN1_2, LOW);
  digitalWrite(IN2_2, LOW);  //Motor 3
  digitalWrite(IN3_2, LOW);
  digitalWrite(IN4_2, LOW);  //Motor 4
}
