/****************************************************************************
 * Robot Koi Fish Main Code
 * Fundamentals of Robotics Koi Fish Team
 ****************************************************************************/

#include<Servo.h>
#include<SoftwareSerial.h>

// Define constants
#define PRINT true

#define LOGIC_BATTERY_PIN A1
#define MAIN_BATTERY_PIN A0

#define PUMP_ENABLE_PIN 4
#define PUMP_SPEED_PIN 5
#define WATER_SENSOR_PIN 7
#define ESTOP_RELAY_PIN 8
#define SERVO_YAW_PIN 9
#define SERVO_PITCH_PIN 10
#define LED_PIN 13

#define BATT_MAX 1000             //9 V
#define BATT_MIN 0                //6 V

byte yaw_limits[] = {20, 165};    //Servo limits 20, 165
byte pitch_limits[] = {21, 165};  //Servo limits 21, 165

// Define servos
Servo servo_yaw;
Servo servo_pitch;


void setup() { // ----------S----------S----------S----------S----------S----------S----------S----------S----------S
  // Set pin modes
  pinMode(PUMP_ENABLE_PIN, OUTPUT);
  pinMode(WATER_SENSOR_PIN, INPUT);
  pinMode(ESTOP_RELAY_PIN, OUTPUT);
  servo_yaw.attach(SERVO_YAW_PIN);
  servo_pitch.attach(SERVO_PITCH_PIN);
  pinMode(LED_PIN, OUTPUT);

  // Open serial comms
  Serial.begin(9600);

  // Activate motor power
  digitalWrite(ESTOP_RELAY_PIN, HIGH);
}

void loop() {
  // Wait for XBee verification to start tests
    println("Running");
    if (gotInput(121)) {
      println("MSG: Estop");
      stop();
    }
}

bool gotInput(int asciiVal) {
  if (Serial.available()) {
    println("Serial Available.");
    int r = Serial.read();
    if (r == asciiVal) {
      return true;
    } else {
      return false;
    }
  }
  else {
    return false;
  }
}

/*
 * function eStop()
 * DESC: Estops the robot
 * ARGS: none
 * RTNS: none
 */
void stop() {
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(ESTOP_RELAY_PIN, LOW);
  println("MSG: Estop activated");
  while (true) {
    // Type y to re-activate
    if (gotInput(121)) {
      println("MSG: Estop de-activated");
      return;
    }
  }
}

void print(char *text) {
  #ifdef PRINT
  Serial.print(text);
  #endif
}

void println(char *text) {
  #ifdef PRINT
  Serial.println(text);
  #endif
}




