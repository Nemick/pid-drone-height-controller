#include <math.h>


//L298 PINS
const int Enable_pin = 10;
const int Direction_pin = 9;

//POTENTIOMETER PIN
const int pot_pin = A0;

//LED PINS STATUS AND DIRECTION
const int GREEN_LED = 2;
const int YELLOW_LED = 3;
const int RED_LED = 4;


// State flags
bool shut_down = false;
bool targetHeight_reached = false;
bool totalTime_recorded = false;
bool shutDown = false;
bool errorCorrection = false;
unsigned long previousTime = 0;
unsigned long firstHeight_reached_time = 0;

unsigned long lastTime = 0;

int pot_ADC = 0;
int motor_speed = 0;
int hover_power = 100;


float targetHeight = 0.43;  //Should no exeed (Hypotenuse) 0.491M for asin() to compute.
float pot_Angle = 0.00;
float droneHeight = 0.00;
float targetHeight_reached_time = 0;
float firstHeight_reached = 0.00;
float maxError = 0;
float TimeElapsed = 0;
float previous_error = 0;
float Height_error = 0;
float integral = 0;
float derivertive = 0;

float KP = 150;
float KI = 2;
float KD = 80;


void setup() {
  Serial.begin(9600);

  pinMode(pot_pin, INPUT);
  pinMode(Enable_pin, OUTPUT);
  pinMode(Direction_pin, OUTPUT);

  //SETTING ANALOG PINS AS DIGITAL PINS
  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);

  //Initialise other led to be off
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(RED_LED, LOW);

  //Light green led for 0.5 sec to show system has started
  digitalWrite(GREEN_LED, HIGH);
  delay(500);
  digitalWrite(GREEN_LED, LOW);

  Serial.println("1. System Started.");
  Serial.println("2. System Initiated.");
  Serial.println("3. Controller Starting.");
  Serial.println(" ");
  Serial.println("Time,      Power,    Angle,    Height,    Error");
}

void loop() {
  if (shut_down) return;

  unsigned long currentTime = millis();                 //---------------------------------------------------------------------------------------------
  if (currentTime - previousTime > 39 && !shut_down) {  //Non_blocking condition for 25Hz.
    previousTime = currentTime;

    read_sensor();       //READ POTENTIOMETER
    calculate_height();  //CALCULATE HEIGHT
    motor_controlError_correction();

    int power_percentage = map(motor_speed, 0, 255, 0, 100);  //Map motor speed to a scale of (0 - 100%).

    //PRINT BLACKBOX DATA ON THE SERIAL.  target_height_angle
    Serial.print(currentTime / 1000.00);  //convert miliseconds to seconds /1000
    Serial.print(",      ");
    Serial.print(motor_speed);
    Serial.print(",      ");
    Serial.print(pot_Angle);
    Serial.print(",      ");
    Serial.print(droneHeight);
    Serial.print(",      ");
    Serial.print(Height_error);
    Serial.println("");


    if (targetHeight_reached && (currentTime - targetHeight_reached_time >= 5000)) {
      ShutDown_procedure();
      shut_down = true;
    }
  }
}

//FUNCTION TO READ POTENTIOMETER----------------------------------------------------------
void read_sensor() {
  unsigned long currentTime = millis();
  int raw_value = analogRead(pot_pin);
  // Enforce ADC value limits
  raw_value = constrain(raw_value, 212, 410);
  // Map ADC value to angle in degrees
  pot_Angle = ((float)raw_value - 410) * (-68.5) / (212 - 410);
}

//CALCULATE REALTIME DRONE HEIGHT
void calculate_height() {
  float angle_rad = pot_Angle * PI / 180;          // Convert angle to radians
  droneHeight = 0.491 * sin(angle_rad);            // Calculate drone height
  Height_error = abs(droneHeight) - targetHeight;  // Calculate height error
}

//CONTROLL MOTOR AND CORRECT ERROR
void motor_controlError_correction() {
  unsigned long currentTime = millis();
  // === STATUS LED AND ERROR CORRECTION LOGIC ====================================================
  bool isAtTargetHeight = abs(Height_error) <= 0.02;

  if (isAtTargetHeight) {
    // Within ±20mm of target height
    digitalWrite(RED_LED, LOW);
    digitalWrite(YELLOW_LED, LOW);
    digitalWrite(GREEN_LED, HIGH);


    targetHeight_reached = true;
    // Record time only once when target is reached
    if (!totalTime_recorded) {
      targetHeight_reached_time = currentTime;
      totalTime_recorded = true;
    }

  } else {
    // Not at target height
    targetHeight_reached = false;
    totalTime_recorded = false;
    maxError = Height_error;
    firstHeight_reached = droneHeight;

    //bool time_elapsed = currentTime - firstHeight_reached_time >= 2000;


    if (Height_error < -0.02) {
      // ABOVE the target height
      digitalWrite(RED_LED, HIGH);
      digitalWrite(YELLOW_LED, LOW);
      digitalWrite(GREEN_LED, LOW);
      digitalWrite(Direction_pin, LOW);
    } else {
      // BELOW the target height
      digitalWrite(RED_LED, LOW);
      digitalWrite(YELLOW_LED, HIGH);
      digitalWrite(GREEN_LED, LOW);
      digitalWrite(Direction_pin, HIGH);
    }
  }

  if (abs(Height_error) > 0.02) {
    TimeElapsed = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;

    integral += abs(Height_error) * TimeElapsed;
    derivertive = (abs(Height_error) - previous_error) / TimeElapsed;
    motor_speed = hover_power + (KP * abs(Height_error) + KI * integral + KD * derivertive);
    motor_speed = constrain(motor_speed, 0, 255);

    previous_error = abs(Height_error);
  }
}

//DRIVE THE MOTOR
void drive_motor() {
  analogWrite(Enable_pin, (int)motor_speed);
}

//Function to shutdown the drone
void ShutDown_procedure() {
  // Turn on all status LEDs
  digitalWrite(YELLOW_LED, HIGH);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, HIGH);

  // Output shutdown telemetry
  Serial.println("4. Shuting down...");
  Serial.println("----------------------------------");
  Serial.print("Time to max height: ");
  Serial.print(targetHeight_reached_time / 1000.0);
  Serial.println(" s");

  Serial.print("Current  pot_Angle: ");
  Serial.print(pot_Angle);
  Serial.println("°");

  Serial.println();
  Serial.print("First height: ");
  Serial.print(firstHeight_reached);
  Serial.println(" m");

  Serial.print("Corrected height: ");
  Serial.print(droneHeight);
  Serial.println(" m");

  Serial.println();
  Serial.print("Max error: ");
  Serial.print(maxError);
  Serial.println(" m");

  Serial.print("Corrected error: ");
  Serial.print(Height_error);
  Serial.println(" m");

  Serial.println("----------------------------------");

  delay(1000);  // Brief pause before shutting off everything

  // Turn off status LEDs
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
}
