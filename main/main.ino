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
#define ESTOP_RELAY_PIN 8
#define SERVO_YAW_PIN 9
#define SERVO_PITCH_PIN 10
#define LED_PIN 7

#define BATT_MAX 1000             //9 V
#define BATT_MIN 0                //6 V
#define CLOSE_THRESH 25000
#define LED_DELAY 500

// Our States
#define STOPPED 0
#define FORWARD 1
#define CIRCLE 2
#define SEARCH 3


// Define variables
int target_sig = 1;
byte yaw_limits[] = {20, 165};    //Servo limits 20, 165
byte pitch_limits[] = {21, 165};  //Servo limits 21, 165
float yaw_servo_pos = 165.0 / 20.0;

// Define servos and cam
Pixy pixy;
Servo servo_yaw;
Servo servo_pitch;

// An object detected by the pixicam
struct PixiBlock {
  float area;
  float x;
  float y;
};


void setup() { // ----------S----------S----------S----------S----------S----------S----------S----------S----------S
  // Set pin modes
  pinMode(PUMP_ENABLE_PIN, OUTPUT);
  pinMode(ESTOP_RELAY_PIN, OUTPUT);
  servo_yaw.attach(SERVO_YAW_PIN);
  servo_pitch.attach(SERVO_PITCH_PIN);
  pinMode(LED_PIN, OUTPUT);
  pixy.init();

  // Open serial comms
  Serial.begin(9600);

  // Activate motor power
  digitalWrite(ESTOP_RELAY_PIN, HIGH);

  Serial.println("Initialization finished. Press 'y' to continue.");
  while (true) {
    if (gotInput(121)) {
      Serial.println("MSG: Starting robot");
      return;
    }
  }
}
int state = FORWARD;
void loop() { // ----------L----------L----------L----------L----------L----------L----------L----------L----------L----------L
  int nextState;
  if (state == STOPPED) {
    nextState = stoppedState();
  }
  else if (state == FORWARD) {
    nextState = forwardState();
  }
  else if (state == SEARCH) {
    nextState = searchState();
  }
  state = nextState;
}

int stoppedState() {
  // Type y to re-activate
  if (gotInput(121)) {
    Serial.println("MSG: Estop de-activated");
    digitalWrite(ESTOP_RELAY_PIN, HIGH);
    return FORWARD;
  }
  else {
    digitalWrite(LED_PIN, HIGH);
    digitalWrite(ESTOP_RELAY_PIN, LOW);
    return STOPPED;
  }
}

int forwardState() {
  // Wait for XBee verification to start tests
  if (gotInput(121)) {
    Serial.println("MSG: Recieved Serial Stop Command");
    return STOPPED;
  }
  else {
    // Sense ---------------------------
    uint16_t blocks = pixy.getBlocks();

    // Think ---------------------------
    bool ledState = isLedOn();

    if (blocks) {
      PixiBlock b = getLargestBlock(blocks);
      if ((b.area != 0) && (b.x != 0) && (b.y != 0)) {
        // Calculate the output tsteering angle
        yaw_servo_pos = (140.0 / 313.0) * b.x + 20.0;
      }
      if (b.area > CLOSE_THRESH) {
          Serial.println("MSG: Object too close.");
          return STOPPED;
      }
    }
    else {
      Serial.println("Cannot find object. beginning to search.");
      return SEARCH;
    }

    // Act  ----------------------------
    //      Serial.Serial.print("Writing ");
    //      Serial.Serial.print(servo_pos);
    //      Serial.println(" to the servo.");

    digitalWrite(LED_PIN, ledState);
    servo_yaw.write(yaw_servo_pos);
    return FORWARD;
  }
}

int searchState() {
  if (gotInput(121)) {
    Serial.println("MSG: Recieved Serial Stop Command");
    return STOPPED;
  }
  else {
    // Sense ---------------------------
    uint16_t blocks = pixy.getBlocks();

    // Think ---------------------------
    bool ledState = isLedOn();

    // Act ----------------------------
    digitalWrite(LED_PIN, ledState);

    if (blocks) {
      Serial.println("Found Buoy. Resuming forward state.");
      return FORWARD;
    } else {
      servo_yaw.write(40);
      return SEARCH;
    }
  }
}



// Gets the largest block detected by the pixicam
PixiBlock getLargestBlock(uint16_t blocks) {
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
  Serial.print(String(max_area));
  Serial.print(" ,x: ");
  Serial.print(String(max_x));
  Serial.print(" ,y: ");
  Serial.println(String(max_y));

  PixiBlock b = {max_area, max_x, max_y};
  return b;
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

bool isLedOn() {
  int t = millis() % (2*LED_DELAY);
  if (t < LED_DELAY) {
    return true;
  }
  else {
    return false;
  }
}
