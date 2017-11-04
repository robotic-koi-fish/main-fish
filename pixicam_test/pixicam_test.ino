// Moves a servo based on the position of an object

#include<Servo.h>
#include <SPI.h>  
#include <Pixy.h>

#define SERVO_PIN 9

// Global Variables -----------------
Pixy pixy;
Servo servo;
int target_sig = 1; // The object we are tracking

void setup()
{
  Serial.begin(9600);
  Serial.println("Starting...\n");
  
  pixy.init();
  Serial.println("Pixicam started.");
  servo.attach(SERVO_PIN);
  Serial.println("Servo initialized.");
}

void loop()
{ 
  char buf[32]; 
  
  // Sense ---------------------------
  uint16_t blocks = pixy.getBlocks();

  // Think ---------------------------
  // Get the largest block
  if (blocks) {
    int max_area = 0; 
    int max_x = 0;
    int max_y = 0;
    sprintf(buf, "Detected %d:\n", blocks);
    for (int j=0; j<blocks; j++) {
        if (pixy.blocks[j].signature == target_sig) {
          // Save the largest block
          if (pixy.blocks[j].width * pixy.blocks[j].height > max_area) {
            max_area = pixy.blocks[j].width * pixy.blocks[j].height;
            max_x = pixy.blocks[j].x;
            max_y = pixy.blocks[j].y;
          }
        }
        
    }
//    Serial.print("area: ");
//    Serial.print(max_area);
//    Serial.print(" ,x: ");
//    Serial.print(max_x);
//    Serial.print(" ,y: ");
//    Serial.println(max_y);

    // Calculate the output thrust
    float servo_pos = (140.0/313.0)*max_x + 20.0;
    // We don't want to move the servo excessively, so round output to nearest 5
    int round_to = 5;
    int final_pos = round_to * int(servo_pos/round_to);
    
    // Act  ----------------------------
    Serial.print("Writing ");
    Serial.print(final_pos);
    Serial.println(" to the servo.");
    
    servo.write(final_pos);
  }

}

