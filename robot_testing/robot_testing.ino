/****************************************************************************
 * Robot Koi Fish Testing Code
 * Fundamentals of Robotics Koi Fish Team
 * Connor Novak: connor@students.olin.edu
 * Version 1.1
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

SoftwareSerial XBee(2, 3); // RX, TX


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
  XBee.begin(9600);
}

void loop() { // ----------L----------L----------L----------L----------L----------L----------L----------L----------L


//  // Wait for XBee verification to start tests
//  XBee.write("TST: Send 'y' to begin testing phase");
//  while (true) {
//    if (XBee.available() && XBee.read() == 121) {
//        #ifdef PRINT
//        Serial.println("MSG: Starting tests");
//        #endif
//    }
//  }
  

  
  delay(1000);

  // Test running LED
  if (testLED(LED_PIN) != 0) {
    #ifdef PRINT
    Serial.println("ERR: LED test failed");
    #endif
    eStop();
  }

  // Test main battery voltage monitor
  if (testVoltage(MAIN_BATTERY_PIN, BATT_MAX, BATT_MIN) != 0) {
    #ifdef PRINT
    Serial.println("ERR: Main battery voltage test failed");
    #endif
    eStop();
  }

  // Test logic battery voltage monitor
  if (testVoltage(LOGIC_BATTERY_PIN, BATT_MAX, BATT_MIN) != 0) {
    #ifdef PRINT
    Serial.println("ERR: Logic battery voltage test failed");
    #endif
    eStop();
  }

  // Test Estop relay
  if (testEStop(ESTOP_RELAY_PIN) != 0) {
    #ifdef PRINT
    Serial.println("ERR: Estop relay test failed");
    #endif
    eStop();
  }

  // Test yaw servo
  #ifdef PRINT
  Serial.println("MSG: Testing yaw servo");
  #endif
  if (testServo(&servo_yaw, yaw_limits) != 0) {
    #ifdef PRINT
    Serial.println("ERR: Yaw servo test failed");
    #endif
    eStop();
  }

  // Test pitch servo
  #ifdef PRINT
  Serial.println("MSG: Testing pitch servo");
  #endif
  if (testServo(&servo_pitch, pitch_limits) != 0) {
    #ifdef PRINT
    Serial.println("ERR: Pitch servo test failed");
    #endif
    eStop();
  }

  // Test bilge pump
  if (testPump(PUMP_ENABLE_PIN, PUMP_SPEED_PIN) != 0) {
    #ifdef PRINT
    Serial.println("ERR: Pump test failed");
    #endif
    eStop();
  }

  // Test Water Sensor
  if (testWaterSensor(WATER_SENSOR_PIN) != 0) {
    #ifdef PRINT
    Serial.println("ERR: Water sensor test failed");
    #endif
    eStop();
  }
  
  while(true) {
    
    #ifdef PRINT
    Serial.println("MSG: Tests complete!");
    delay(10000);
    #endif
    
  }
}

// ----------F----------F----------F----------F----------F----------F----------F----------F----------F----------F


/*
 * function testLED()
 * DESC: Tests running LED
 * ARGS: byte representing LED pin
 * RTNS: 0 if no error, else 1
 */
int testLED(byte pin) {
  
  digitalWrite(pin, HIGH);
  #ifdef PRINT
  Serial.println("MSG: Running LED to HIGH");
  #endif
  
  delay(2000);
  
  digitalWrite(pin, LOW);
  #ifdef PRINT
  Serial.println("MSG: Running LED to LOW");
  #endif

  delay(2000);
  return 0;
}


/*
 * function testVoltage()
 * DESC: Tests voltage monitor
 * ARGS: byte representing voltage monitor pin
 * RTNS: 0 if no error, else 1
 */
int testVoltage(byte pin, byte max_v, byte min_v) {
  int reading = analogRead(pin);

  if (min_v <= reading <= max_v) {
    #ifdef PRINT
    Serial.print("MSG: Battery voltage is ");
    Serial.println(reading);
    #endif
    return 0;
  }
  else {
    #ifdef PRINT
    Serial.println("ERR: Battery voltage is outside constraints.");
    #endif
    return 1;
  }
}


/*
 * function testEStop()
 * DESC: Tests estop relay
 * ARGS: bytes representing relay pin
 * RTNS: 0 if no error, else 1
 */
int testEStop(byte pin) {
    
  digitalWrite(pin, HIGH);
  #ifdef PRINT
  Serial.println("MSG: Estop relay closed; motor light on");
  #endif
  
  delay(2000);
  
  digitalWrite(pin, LOW);
  #ifdef PRINT
  Serial.println("MSG: Estop relay opened; motor light off");
  #endif

  delay(2000);
  return 0;
}


/*
 * function testServo()
 * DESC: Tests servo
 * ARGS: Servo to test, limits at which to test servo
 * RTNS: 0 if no error, else 1
 */
int testServo(Servo* servo, byte limits[]) {

  // Turn on servo power
  digitalWrite(ESTOP_RELAY_PIN, HIGH);

  // Send servo to low limit
  servo->write(limits[0]);
  #ifdef PRINT
  Serial.println("MSG: Sending servo to lower limit");
  #endif
  delay(1000);

  // Send servo to high limit
  servo->write(limits[1]);
  #ifdef PRINT
  Serial.println("MSG: Sending servo to upper limit");
  #endif
  delay(1000);

  // Turn off servo power
  digitalWrite(ESTOP_RELAY_PIN, LOW);
  return 0;
}


/*
 * function testPump()
 * DESC: Tests bilge pump
 * ARGS: pump pin
 * RTNS: 0 if no error, else 1
 */
int testPump(byte enable_pin, byte speed_pin) {

  //Turn on pump power
  digitalWrite(ESTOP_RELAY_PIN, HIGH);

  //Turn on pump
  #ifdef PRINT
  Serial.println("MSG: Turning on pump");
  #endif
  digitalWrite(enable_pin, HIGH);
  analogWrite(speed_pin, 255);
  delay(2000);
  
  //Turn off pump
  #ifdef PRINT
  Serial.println("MSG: Turning off pump");
  #endif
  analogWrite(speed_pin, 0);
  digitalWrite(enable_pin, LOW);
  
  //Turn off pump power
  digitalWrite(ESTOP_RELAY_PIN, LOW);
}


/*
 * function testWaterSensor()
 * DESC: Tests water sensor
 * ARGS: water sensor pin
 * RTNS: 0 if no error, else 1
 */
 int testWaterSensor(int pin) {

    #ifdef PRINT
  Serial.println("MSG: Reading water sensor");
  #endif
  
  // Take reading from sensor
  boolean reading = digitalRead(pin);
  
  // If reading is okay, return 0, else 1
  if (!reading) {
    #ifdef PRINT
    Serial.print("MSG: Water sensor reading is ");
    Serial.println(reading);
    #endif
    return 0;
  }
  else {
    return 1;
  }
 }
 
 
/*
 * function eStop()
 * DESC: Estops the robot
 * ARGS: none
 * RTNS: none
 */
void eStop() {
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(ESTOP_RELAY_PIN, LOW);
  #ifdef PRINT
  Serial.println("MSG: Estop activated");
  #endif
  while (true) {
    
  }
}
