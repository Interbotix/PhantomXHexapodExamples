
/*************************************************
 * Hexapod Leashed 
 *   Control a PhantomX Hexapod using a leash via
 *   an analog joystick.
 *   This version of the code has removed all 
 *   ArbotiX-Commander controls.
 *
 *
 *  This Sketch is based of the PhantomX Hexapod Commander Sketch
 *  https://github.com/trossenrobotics/HexapodMKIICommander
 *
 *************************************************/


#include <ax12.h>
#include <BioloidController.h>
#include "nuke.h"

#include <Wire.h>
#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.

#define DEADBAND 10
#define GOAL 100

// Define one or the other depending upon which servo type you are using.
#define AX12_HEXAPOD
//#define AX18_HEXAPOD

int multiplier;

#define RIPPLE_SPEED    1
#define AMBLE_SPEED     5
#define TRIPOD_SPEED    5

//anlog input pins
#define xAnalog 0
#define yAnalog 1


//speed definition for AX12 version
#ifdef AX12_HEXAPOD
#define TOP_SPEED      10
#endif

//speed definition for AX18 version
#ifdef AX18_HEXAPOD
#define TOP_SPEED      12
#endif

int reading = 0;
int goalDifferece = 0;

int tempSpeed = 0;
void setup()
{
  
  Wire.begin(); // join i2c busbus
  Serial.begin(9600); // start serial communication at 9600bps
  Serial.println("test");
  // setup IK
  setupIK();

  //Select which gait to use
  gaitSelect(TRIPOD);
  //Gait List 
  //RIPPLE_SMOOTH
  //RIPPLE
  //AMBLE_SMOOTH
  //AMBLE
  //TRIPOD
  
  //multiplier effects the crawler's speed 1 - slowest 10 = highest  
  multiplier = 10;
  //Change the speed to mirror the gait
  //RIPPLE and AMBLE speeds should be limited to 1/4 respectivley
  //TRIPOD can go from 5 to TOP_SPEED
  
  
  delay (1000);  //wait after gait is setup
  float voltage = (ax12GetRegister (1, AX_PRESENT_VOLTAGE, 1)) / 10.0;  //read the voltage from servo #1 - divide the value from the register by 10 to conver to volts
  //if the voltage falls below 10 then loop forver to stop the cralwer
  if (voltage < 10.0)
  {
    while(1)
    {
    Serial.println("Voltage Error");
    }
  }
  // stand up slowly
  bioloid.poseSize = 18;
  bioloid.readPose();
  doIK();
  bioloid.interpolateSetup(1000);
  while(bioloid.interpolating > 0){
    bioloid.interpolateStep();
    delay(3);
  }
  
  
  for(int k = 1 ; k <= 18; k++)
  {
    //ax12SetRegister(k, 24, 0);
    ax12SetRegister(k, 25, 1);
  }
  
  

}

void loop()
{
  
  //Serial.println("yes"); // print the reading
    
    
  Wire.beginTransmission((int)LIDARLite_ADDRESS); // transmit to LIDAR-Lite
  Wire.write((int)RegisterMeasure); // sets register pointer to  (0x00)  
  Wire.write((int)MeasureValue); // sets register pointer to  (0x00)  
  Wire.endTransmission(); // stop transmitting
    
  delay(20); // Wait 20ms for transmit
  
  Wire.beginTransmission((int)LIDARLite_ADDRESS); // transmit to LIDAR-Lite
  Wire.write((int)RegisterHighLowB); // sets register pointer to (0x8f)
  Wire.endTransmission(); // stop transmitting
  
  delay(20); // Wait 20ms for transmit
  
  Wire.requestFrom((int)LIDARLite_ADDRESS, 2); // request 2 bytes from LIDAR-Lite
  
  if(2 <= Wire.available()) // if two bytes were received
  {
    reading = Wire.read(); // receive high byte (overwrites previous reading)
    reading = reading << 8; // shift high byte to be high 8 bits
    reading |= Wire.read(); // receive low byte as lower 8 bits
    goalDifferece = reading - GOAL;
    
    
  }

  if(reading > (GOAL + DEADBAND) )
  {
    
    tempSpeed = map((goalDifferece), 10, 100, 20,127);
    tempSpeed = min(tempSpeed, 127);
  }
  
  if(reading < (GOAL - DEADBAND))
  {
     tempSpeed = map((goalDifferece), -100, 10, -127, -20);
     tempSpeed = max(tempSpeed, -127);
    
  }
  if((reading < (GOAL + DEADBAND)) && (reading > (GOAL-DEADBAND)))
  {
      tempSpeed = 0;
   
  } 
     Xspeed = (tempSpeed * multiplier)/2;
   
    Serial.print(goalDifferece); // print the reading
    Serial.print(" - "); // print the reading
    Serial.print(tempSpeed); // print the reading
    Serial.print(" - "); // print the reading
    Serial.println(Xspeed); // print the reading
    
    
  //Serial.print(GOAL-reading);
  //Serial.println(Yspeed);
 

  // if our previous interpolation is complete, recompute the IK
  if(bioloid.interpolating == 0){
    doIK();
    bioloid.interpolateSetup(tranTime);
  }
  // update joints
  bioloid.interpolateStep();
}
