/****************************************************************************
   Robot Koi Fish Main Code
   Fundamentals of Robotics Koi Fish Team
 ****************************************************************************/

#include<Servo.h>
#include<SoftwareSerial.h>
#include<SPI.h>
#include<Pixy.h>

// Define constants

#define LOGIC_BATTERY_PIN A0
#define MAIN_BATTERY_PIN A1
#define TEMPERATURE_SENSOR A2
#define WATER_SENSOR A3

#define RESET_RELAY_PIN 2
#define HALL_EFFECT_PIN 3
#define PUMP_ENABLE_PIN 4
#define PUMP_SPEED_PIN 5
#define SERVO_YAW_PIN 6
#define LED_PIN 7
#define ESTOP_RELAY_PIN 8
#define SERVO_PITCH_PIN 9

#define BATT_MAX 1000             //9 V
#define BATT_MIN 0                //6 V
#define CLOSE_THRESH 3000
#define LED_DELAY 500

// Our States
#define STOPPED 0
#define FORWARD 1
#define CIRCLE 2
#define SEARCH 3
#define TURN_AWAY 4
#define TURN_TO 5

#define N 20


// Define variables
int target_sig = 1;
float yaw_servo_pos = 165.0 / 20.0;

// Define servos and cam
Pixy pixy;
Servo servo_yaw;
Servo servo_pitch;

// An object detected by the pixicam
struct PixiBlock {
  long area;
  int x;
  int y;

  PixiBlock(long area0, int x0, int y0) {
    area = area0;
    x = x0;
    y = y0;
  }

  PixiBlock() {
    area = 0;
    x = 0;
    y = 0;
  }
};

// Stores the N most recent block values for our 3 targets
PixiBlock prev_blocks[3][N] = {{}};


void setup() { // ----------S----------S----------S----------
  // Set pin modes
  pinMode(PUMP_ENABLE_PIN, OUTPUT);
  pinMode(ESTOP_RELAY_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  servo_yaw.attach(SERVO_YAW_PIN);
  servo_pitch.attach(SERVO_PITCH_PIN);
  pixy.init();

  // Open serial comms
  Serial.begin(9600);

  Serial.println("Initialization finished. Press 'y' to continue.");
  while (true) {
    if (gotInput(121)) {
      Serial.println("MSG: Starting robot");
      return;
    }
  }
}

int state = FORWARD;
void loop() { // ----------L----------L----------L----------L----------L
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
  else if (state == TURN_AWAY) {
    nextState = turnAwayState();
  }
  else if (state == TURN_TO) {
    nextState = turnToState();
  }

  state = nextState;

}

// ----------STOPPED----------STOPPED----------STOPPED
int stoppedState() {
  // Type y to re-activate
  if (gotInput(121)) {
    Serial.println("STOPPED: Estop de-activated");
    digitalWrite(ESTOP_RELAY_PIN, HIGH);
    return FORWARD;
  }
  else {
    digitalWrite(LED_PIN, HIGH);
    digitalWrite(ESTOP_RELAY_PIN, LOW);
    return STOPPED;
  }
}

// ----------FORWARD----------FORWARD----------FORWARD
int forwardState() {
  // Check for Serial Stop Command
  if (gotInput(121)) {
    Serial.println("FORWARD: Recieved Serial Stop Command");
    return STOPPED;
  }
  else {
    // Sense ---------------------------
    uint16_t blocks = pixy.getBlocks();

    // Think ---------------------------
    bool ledState = isLedOn();

    if (blocks) {
      PixiBlock b_new = getLargestBlock(blocks, getTarget());
      push(prev_blocks[getTarget()], b_new, N);
      PixiBlock b = getAverageBlock(prev_blocks[getTarget()], N);
      // Serial.println(b.area);
      if ((b.area != 0) && (b.x != 0) && (b.y != 0)) {
        // Calculate the output tsteering angle
        yaw_servo_pos = (140.0 / 313.0) * b.x + 20.0;
      }

      // If buoy in-cam size passes threshold, get next buoy or mission complete
      if (b.area > CLOSE_THRESH) {
          Serial.println("FORWARD: Object too close. Starting to turn away.");
          return TURN_AWAY;
      }
    }
    else {
      push(prev_blocks[0], PixiBlock(), N);
      push(prev_blocks[1], PixiBlock(), N);
      push(prev_blocks[2], PixiBlock(), N);
    }

    // for (int j = 0; j<N; j+=1) {
    //   Serial.print(prev_blocks[getTarget()][j].area);
    //   Serial.print(' ');
    // }
    // Serial.println(' ');

    // Act  ----------------------------
    //      Serial.Serial.print("Writing ");
    //      Serial.Serial.print(servo_pos);
    //      Serial.println(" to the servo.");
    digitalWrite(LED_PIN, ledState);
    return FORWARD;
  }

}

// Turns away until the current target is no langer in view
int turnAwayState() {
  // Wait for XBee verification to start tests
  if (gotInput(121)) {
    Serial.println("TURN_AWAY: Recieved Serial Stop Command");
    return STOPPED;
  }
  else {
    // Sense ---------------------------
    uint16_t blocks = pixy.getBlocks();

    // Think ---------------------------
    bool ledState = isLedOn();

    // Act -----------------
    digitalWrite(LED_PIN, ledState);

    // If we see our block, push value onto block history
    if (blocks) {
      PixiBlock b_new = getLargestBlock(blocks, getTarget());
      push(prev_blocks[getTarget()], b_new, N);
    } else { // Otherwise, push 0's onto block history
      push(prev_blocks[0], PixiBlock(), N);
      push(prev_blocks[1], PixiBlock(), N);
      push(prev_blocks[2], PixiBlock(), N);
    }

    PixiBlock b = getAverageBlock(prev_blocks[getTarget()], N);
    // Serial.println(b.area);
    if (b.area > 0) {
      // Keep turning
      servo_yaw.write(40);
      return TURN_AWAY;
    }
    else {
      Serial.print("TURN_AWAY: No longer see buoy #");
      Serial.print(getTarget());
      Serial.println(". Turn until we see this buoy again.");
      return TURN_TO;

    }

  }
}


int turnToState() {
  if (gotInput(121)) {
    Serial.println("TURN_TO: Recieved Serial Stop Command");
    return STOPPED;
  }
  else {
    // Sense ---------------------------
    uint16_t blocks = pixy.getBlocks();

    // Think ---------------------------
    bool ledState = isLedOn();

    // Act ----------------------------
    digitalWrite(LED_PIN, ledState);

    // If we see our block, push value onto block history
    if (blocks) {
      PixiBlock b_new = getLargestBlock(blocks, getTarget());
      push(prev_blocks[getTarget()], b_new, N);
    } else { // Otherwise, push 0's onto block history
      push(prev_blocks[0], PixiBlock(), N);
      push(prev_blocks[1], PixiBlock(), N);
      push(prev_blocks[2], PixiBlock(), N);
    }
    PixiBlock b = getAverageBlock(prev_blocks[getTarget()], N);
    // Serial.println(b.area);
    if (b.area > 0) {
      bool done = toNextTarget();
      if (done) {
        Serial.println("TURN_TO: Done!");
        return STOPPED;
      }
      else {
        Serial.println("TURN_TO: Found Buoy. Searching for next buoy.");
        return SEARCH;
      }
    }
    else {
      servo_yaw.write(40);
      return TURN_TO;
    }
  }
}

// ----------SEARCH----------SEARCH----------SEARCH
int searchState() {
  if (gotInput(121)) {
    Serial.println("SEARCH: Recieved Serial Stop Command");
    return STOPPED;
  }
  else {
    // Sense ---------------------------
    uint16_t blocks = pixy.getBlocks();

    // Think ---------------------------
    bool ledState = isLedOn();

    // Act ----------------------------
    digitalWrite(LED_PIN, ledState);

    // If we see our block, push value onto block history
    if (blocks) {
      PixiBlock b_new = getLargestBlock(blocks, getTarget());
      push(prev_blocks[getTarget()], b_new, N);
    } else { // Otherwise, push 0's onto block history
      push(prev_blocks[0], PixiBlock(), N);
      push(prev_blocks[1], PixiBlock(), N);
      push(prev_blocks[2], PixiBlock(), N);
    }
    PixiBlock b = getAverageBlock(prev_blocks[getTarget()], N);
    // Serial.println(b.area);

    if (b.area > 0) {
      Serial.print("SEARCH: Found Buoy #");
      Serial.print(getTarget());
      Serial.println(". Resuming forward state.");
      return FORWARD;
    } else {
      servo_yaw.write(40);
      return SEARCH;
    }
  }
}

// Helper Functions -------------------------------------------------


// pushes an item onto the beginning of an array, shifting each other element down.
void push(PixiBlock arr[], PixiBlock new_el, int n) {
  PixiBlock last = new_el;
  PixiBlock temp;
  for (int i = 0; i < n; i=i+1) {
    // This should insert the new element at beginning
    // And move the rest down
    temp = arr[i];
    arr[i] = last;
    last = temp;
  }
}

// Averages the areas and positions of an array of pixiblocks of length n
PixiBlock getAverageBlock(PixiBlock arr[], int n) {
  long avg_area = 0;
  int avg_x = 0;
  int avg_y = 0;
  for (int j = 0; j<N; j+=1) {
    avg_area += arr[j].area;
    avg_x += arr[j].x;
    avg_y += arr[j].y;
  }
  avg_area = avg_area / n;
  avg_x = avg_x / n;
  avg_y = avg_y / n;
  return PixiBlock(avg_area, avg_x, avg_y);
}


// Gets the largest block detected by the pixicam
PixiBlock getLargestBlock(uint16_t blocks, int target) {
  long max_area = 0;
  int max_x = 0;
  int max_y = 0;
  for (int j = 0; j < blocks; j++) {
    if (pixy.blocks[j].signature == target) {
      // Save the largest block
      if (pixy.blocks[j].width * pixy.blocks[j].height > max_area) {
        max_area = pixy.blocks[j].width * pixy.blocks[j].height;
        max_x = pixy.blocks[j].x;
        max_y = pixy.blocks[j].y;
      }
    }
  }

  PixiBlock b = {max_area, max_x, max_y};

  if ((b.area != 0) && (b.x != 0) && (b.y != 0)) {
    // Calculate the output tsteering angle
    // Serial.print("area: ");
    // Serial.print(b.area);
    // Serial.print(" ,x: ");
    // Serial.print(b.x);
    // Serial.print(" ,y: ");
    // Serial.println(b.y);
  }

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
