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

#include <IRremote.h>

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

int RECV_PIN = 3;

IRrecv irrecv(RECV_PIN);

decode_results results;


void setup()
{
    irrecv.enableIRIn(); // Start the receiver


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
}



void dump2(decode_results *results) {
  int count = results->rawlen;

  if(results->value == 0x8679708F)
  {

  Xspeed = 150;
  Yspeed   = 0;
  
  }
  else if(results->value == 0x86798877)
  {


  Xspeed = -150;
  Yspeed = 0;
  
  }
  else if(results->value == 0x8679B04F)
  {



  Xspeed = 0;
  Yspeed = 150 ; 
  }
  else if(results->value == 0x867908F7)
  {

  Xspeed = 0;
  Yspeed = -150  ;
  
  }
  
  else if(results->value == 0x8679F00F)
  {

  Xspeed = 0;
  Yspeed = 0  ;
  
  }
  
  
  
}



void loop(){
  
  if (irrecv.decode(&results)) {
    Serial.println(results.value, HEX);
    dump2(&results);
    irrecv.resume(); // Receive the next value
  }


 

  // if our previous interpolation is complete, recompute the IK
  if(bioloid.interpolating == 0){
    doIK();
    bioloid.interpolateSetup(tranTime);
  }
  // update joints
  bioloid.interpolateStep();
}


