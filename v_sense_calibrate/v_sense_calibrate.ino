/***********************************************************************
 * Voltage Sensor Calibration
 * Fundamentals of Robotics Team Koi Fish
 * Connor Novak
 * V 1.0
 **********************************************************************/
#define VOLTAGE_METER_PIN A0

// Declare data storage arrays
int sensor_vals[10];
int voltage_vals[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

void setup() { //----------S----------S----------S----------S----------S

  //Open serial comms
  Serial.begin(9600);
  
}

void loop() { //----------L----------L----------L----------L----------L
  
  // Loop over every voltage in voltage_vals
  for (int v = 0; v < sizeof(voltage_vals)/sizeof(int); v++) {
    
    // store dataCollect() result in data array
    sensor_vals[v] = dataCollect(voltage_vals[v], VOLTAGE_METER_PIN);
    
  }
  // Print out results to serial port
  Serial.print("MSG: Voltage data - ");
  printArray(voltage_vals, sizeof(voltage_vals)/sizeof(int));
  Serial.print("MSG: Sensor data - ");
  printArray(sensor_vals, sizeof(sensor_vals)/sizeof(int));
  
  // enter while loop so as not to re-run program
  while(true) {
    delay(10);
  }
  
}

//----------F----------F----------F----------F----------F----------F

/*
 * function dataCollect()
 * DESC: collects and returns sensor reading at given voltage
 * ARGS: int voltage value, byte sensor pin
 * RTNS: data value
 */
int dataCollect(int voltage, byte pin) {

  //Prompt user to input specified voltage
  Serial.print("TYP: Please apply ");
  Serial.print(voltage);
  Serial.println("V to the battery lead and enter 'y' when done");
  
  bool user_data_received = false;

  // loop until user presses y-key (ASCII 121)
  while (!user_data_received) {
    if (Serial.available()) {
      if (Serial.read() == 121) {
        Serial.println("MSG: Taking data");
        user_data_received = true;
      }
      else {
        Serial.println("ERR: Incorrect input; try again");
      }
    }
  }
  
  // take sensor value
  int value = analogRead(VOLTAGE_METER_PIN);
  delay(50);
  
  // return sensor value
  Serial.print("MSG: Sensed value of ");
  Serial.println(value);
  return value;
}


/*
 * function printArray()
 * DESC: prints given array
 * ARGS: integer array to print, integer length of array
 * RTNS: none
 */
void printArray(int to_print[], int print_len) {

  // Loop over items in array
  for (int i = 0; i < print_len; i++) {

    //Print new entry
    Serial.print(to_print[i]);

    // Print comma if appropriate
    if (i < print_len - 1) {
      Serial.print(", ");
    }
  }
  
  // Print newline character
  Serial.println(" ");
}

