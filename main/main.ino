/****************************************************************************
   Robot Koi Fish Main Code
   Fundamentals of Robotics Koi Fish Team
 ****************************************************************************/

#include<Servo.h>
#include<SoftwareSerial.h>
#include<SPI.h>
#include<Pixy.h>

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
#define CLOSE_THRESH 15000

// Define variables
int target_sig = 1;
byte yaw_limits[] = {20, 165};    //Servo limits 20, 165
byte pitch_limits[] = {21, 165};  //Servo limits 21, 165
float yaw_servo_pos = 165.0/20.0;

// Define servos and cam
Pixy pixy;
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
  pixy.init();

  // Open serial comms
  Serial.begin(9600);

  // Activate motor power
  digitalWrite(ESTOP_RELAY_PIN, HIGH);

  while (true) {
    if (gotInput(121)) {
      println("MSG: Starting robot");
      return;
    }
  }
}

void loop() { // ----------L----------L----------L----------L----------L----------L----------L----------L----------L----------L
  // Wait for XBee verification to start tests
  if (gotInput(121)) {
    println("MSG: Estop");
    stop();
  }
  else {
    // Sense ---------------------------
    uint16_t blocks = pixy.getBlocks();

    // Think ---------------------------
    // Get the largest block
    if (blocks) {
      int max_area = 0;
      int max_x = 0;
      int max_y = 0;
      for (int j = 0; j < blocks; j++) {
        if (pixy.blocks[j].signature == target_sig) {
          // Save the largest block
          if (pixy.blocks[j].width * pixy.blocks[j].height > max_area) {
            max_area = pixy.blocks[j].width * pixy.blocks[j].height;
            max_x = pixy.blocks[j].x;
            max_y = pixy.blocks[j].y;
          }
        }

      }
      Serial.print("area: ");
      Serial.print(max_area);
      Serial.print(" ,x: ");
      Serial.print(max_x);
      Serial.print(" ,y: ");
      Serial.println(max_y);

      if((max_area != 0) && (max_x != 0) && (max_y !=0)) {
        // Calculate the output tsteering angle
        yaw_servo_pos = (140.0 / 313.0) * max_x + 20.0;
      }
      if (max_area > CLOSE_THRESH) {
        stop();
      }

      // Act  ----------------------------
//      Serial.print("Writing ");
//      Serial.print(servo_pos);
//      Serial.println(" to the servo.");

      servo_yaw.write(yaw_servo_pos);
    }
  }
}

bool gotInput(int asciiVal) {
  if (Serial.available()) {
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
   function eStop()
   DESC: Estops the robot
   ARGS: none
   RTNS: none
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




