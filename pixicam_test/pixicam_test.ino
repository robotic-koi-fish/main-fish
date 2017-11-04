// Go towards a buoy
   
#include <SPI.h>  
#include <Pixy.h>

// Global Variables -----------------
Pixy pixy;
int target_sig = 1; // The object we are tracking


void setup()
{
  Serial.begin(9600);
  Serial.println("Starting...\n");

  pixy.init();
  Serial.println("Pixicam started.");
}

void loop()
{ 
  static int i = 0;
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
    Serial.print("area: ");
    Serial.print(max_area);
    Serial.print(" ,x: ");
    Serial.print(max_x);
    Serial.print(" ,y: ");
    Serial.println(max_y);
  }

  // Calculate the output thrust

  // Act  ----------------------------
  // Set the power of the thrusters

}

