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

#include <SPI.h>  
#include <Pixy.h>

Pixy pixy;

// Define one or the other depending upon which servo type you are using.
#define AX12_HEXAPOD
//#define AX18_HEXAPOD

int multiplier;

#define RIPPLE_SPEED    1
#define AMBLE_SPEED     4
#define TRIPOD_SPEED    5

//anlog input pins
#define xAnalog 0
#define yAnalog 1

#define TILT 20


//speed definition for AX12 version
#ifdef AX12_HEXAPOD
#define TOP_SPEED      10
#endif

//speed definition for AX18 version
#ifdef AX18_HEXAPOD
#define TOP_SPEED      12
#endif


#define TILT_MAX 712 
#define TILT_MIN 312
int tilt;

int pixyCenterY;
int pixyCenter;
long pixySize;
long avgSizeArr[5];
int avgSizePointer = 0;
long avgSize;
long lastSeen;
void setup()
{
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
  multiplier = TRIPOD_SPEED;
  //Change the speed to mirror the gait
  //RIPPLE and AMBLE speeds should be limited to 1/4 respectivley
  //TRIPOD can go from 5 to TOP_SPEED
  
  
  delay (1000);  //wait after gait is setup
  float voltage = (ax12GetRegister (1, AX_PRESENT_VOLTAGE, 1)) / 10.0;  //read the voltage from servo #1 - divide the value from the register by 10 to conver to volts
  //if the voltage falls below 10 then loop forver to stop the cralwer
  if (voltage < 10.0)
    while(1);

  // stand up slowly
  bioloid.poseSize = 18;
  bioloid.readPose();
  doIK();
  bioloid.interpolateSetup(1000);
  while(bioloid.interpolating > 0){
    bioloid.interpolateStep();
    delay(3);
  }
  tilt = 512;
  SetPosition(TILT,tilt);
  Serial.begin(9600);
       Serial.print("startup");
pixy.init();
}

void loop()
{
  static int i = 0;
  int j;
  uint16_t blocks;
  char buf[32]; 
  
  blocks = pixy.getBlocks();
  
  if (blocks)
  {
      
    lastSeen = millis();
     pixyCenter = pixy.blocks[0].x - 160;
     pixyCenterY = pixy.blocks[0].y - 100;
     pixySize = pixy.blocks[0].height *  pixy.blocks[0].width;
     avgSizeArr[avgSizePointer]= pixySize;
     avgSizePointer = (avgSizePointer + 1)%5;
     avgSize = (avgSizeArr[0]+avgSizeArr[1]+avgSizeArr[2]+avgSizeArr[3]+avgSizeArr[4])/5;
     
     Serial.print(avgSizeArr[0]);
     Serial.print(" ");
     
     Serial.print(avgSizeArr[1]);
     Serial.print(" ");
     
     Serial.print(avgSizeArr[2]);
     Serial.print(" ");
     
     Serial.print(avgSizeArr[3]);
     Serial.print(" ");
     
     Serial.print(avgSizeArr[4]);
     Serial.println(" ");
  
  
     
     
     Serial.println(avgSize);
     
     
     //Xspeed = -52.72*log(avgSize)+539.5;
   // Xspeed = 300;
 
 
   if (avgSize > 25000)
   {
    Xspeed = 0; 
   }
   
   /*
   else if (avgSize > 10000)
   {
    Xspeed = 10; 
   }
   
   else if (avgSize > 5000)
   {
    Xspeed = 100; 
   }
   
   else if (avgSize > 1000)
   {
    Xspeed = 200; 
   }
   
   else if (avgSize > 500)
   {
    Xspeed = 225; 
   }
   
   else if (avgSize > 100)
   {
    Xspeed = 250; 
   }
   
   else if (avgSize > 50)
   {
    Xspeed = 300; 
   }*/
   else
   {
    Xspeed = 300; 
     
   }
 
  }
  
  
  else
  {
    if(lastSeen - millis() > 1500);
    {
    pixyCenter = 0;   
    pixyCenterY = 0;
      
    Xspeed = 0;
    Rspeed = 0;
    }
   }
    
    
  if(pixyCenterY > 10)
  {
    tilt  = tilt -(pixyCenterY/6);
    //tilt  = tilt +(10);
    //tilt = 512 + (pixyCenterY/2);
    //Serial.print("G:");
    //Serial.println(pixyCenterY);
    
   }
   
   else if(pixyCenterY < -10)
   {
     tilt  = tilt -(pixyCenterY/6);
     //tilt  = tilt +(-10);
     // tilt = 512 + (pixyCenterY/2);
   // Serial.print("L:");
     //Serial.println(pixyCenterY);
    }
    
    else
    {
      //Xspeed = 0;
      // tilt = 512;
    }

   // Serial.print("Tilt");
   // Serial.println(tilt);

    
    
    if(pixyCenter > 10)
    {
      Rspeed = -pixyCenter/100.0;
    }
    else if(pixyCenter < -10)
    {
      Rspeed = -pixyCenter/100.0;
     
    }
    else
    {
      Rspeed = 0;
    }
    
    // Yspeed = pixyCenter;
    
//    
//    //deadband
//    //If the right_V value is above 5/ below -5, then set the XSpeed value
//    if((right_V) > 5 || (right_V < -5) )
//    {
//      Xspeed = (multiplier*right_V)/2; //setting the XSpeed value will make the cralwer walk back/forward
//    }
//    //otherwise set the XSpeed to 0
//    if((right_H) > 5 || (right_H < -5) )
//    {
//      Yspeed = (multiplier*right_H)/2;
//    }
//    //otherwise set the YSpeed to 0
//    else
//    {
//      Yspeed = 0;
//    } 
 
   tilt = max(tilt, TILT_MIN);  //use the max() function to make sure the value never falls below TILT_MIN 
     tilt = min(tilt, TILT_MAX);  //use the min() function to make sute the value never goes above TILT_MAX 
     
    SetPosition(TILT,tilt);
     
     
  // if our previous interpolation is complete, recompute the IK
  if(bioloid.interpolating == 0){
    doIK();
    bioloid.interpolateSetup(tranTime);
  }
  // update joints
  bioloid.interpolateStep();
}
