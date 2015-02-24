
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


//speed definition for AX12 version
#ifdef AX12_HEXAPOD
#define TOP_SPEED      10
#endif

//speed definition for AX18 version
#ifdef AX18_HEXAPOD
#define TOP_SPEED      12
#endif

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
  multiplier = AMBLE_SPEED;
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
  
      SetPosition(20,512); //set the position of servo # 1 to '0'

}

void loop(){
 
    //Normally the commander will send a value from -128 to 128
    //this line reads an analog value and then converts it to the -128 to 128 scale    
    
    int right_V = (((1023-analogRead(xAnalog))-512)/4);
    int right_H= (((1023-analogRead(yAnalog))-512)/4);
    
  
    //deadband
    //If the right_V value is above 5/ below -5, then set the XSpeed value
    if((right_V) > 5 || (right_V < -5) )
    {
      Xspeed = (multiplier*right_V)/2; //setting the XSpeed value will make the cralwer walk back/forward
    }
    //otherwise set the XSpeed to 0
    else
    {
      Xspeed = 0;
    }
    
    if((right_H) > 5 || (right_H < -5) )
    {
      Yspeed = (multiplier*right_H)/2;
    }
    //otherwise set the YSpeed to 0
    else
    {
      Yspeed = 0;
    } 
 

  // if our previous interpolation is complete, recompute the IK
  if(bioloid.interpolating == 0){
    doIK();
    bioloid.interpolateSetup(tranTime);
  }
  // update joints
  bioloid.interpolateStep();
}
