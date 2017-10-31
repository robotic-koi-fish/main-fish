/*********************************************************************************
Main Control Loop
Team Coi Fish 
Members: Jonah Spear, Emily Kohler, Cristina Segar, Connor Novak, Jacob Regenstein
**********************************************************************************/

// Libraries
#include<Servo.h> //Library for running Servos

// Define constants
#define DEBUG true

// Define global variables
  // Sensor Readings
  // Motor Settings

// Define servos

// Set up robot code, runs once on start up
void setup() {
    // Set pin modes
    // Open serial comms
    // Run testing function
    // Load mission
}

// Robot Control Loop, runs until e-stop condition
void loop() {
    
}

// Control Functions ("Borrow other stuff" - Dave)

void sense()
{
  // read sensors, update global reading vars
    // temperature sensor (opt)
    // water sensor (opt)
    // main battery voltage
    // logic battery voltage
    // pixycam
}

void think()
{
  // use sense
  // mission tells us which buoy
  // returns motor values
}

void act()
{
  //uses think values to run motors
}

void spin()
{
  // uses sense, think and act to turn and find next bouy
}
  

void test() {
  
}


/*
 * function eStop()
 * DESC: Estops the robot
 * ARGS: none
 * RTNS: none
 */
void eStop() {
  digitalWrite(LED_PIN, HIGH);              // Sets running LED to ON
  digitalWrite(ESTOP_RELAY_PIN, LOW);       // Cuts main battery line

  // Sends message if necessary
  #ifdef PRINT
  Serial.println("MSG: Estop activated");
  #endif

  // Waits until restart
  while (true) {}
}

