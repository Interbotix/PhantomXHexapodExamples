#include <Servo.h>

/*
 * Auto-Generated by NUKE!
 *   http://arbotix.googlecode.com
 */

#include <ax12.h>
#include <BioloidController.h>
#include <Commander.h>
#include "nuke.h"

// Define one or the other depending upon which servo type you are using.
#define AX12_HEXAPOD
//#define AX18_HEXAPOD

Commander command = Commander();
int multiplier;
Servo pan;
Servo tilt;

Servo nerfA;
Servo nerfB;
Servo nerfC;
Servo nerfD;

int fireA, fireB, fireC, fireD, fireAll;

int panValue = 90;
int tiltValue = 90;

int vericalMapped;
int horizontalMapped;

#define RIPPLE_SPEED    1
#define AMBLE_SPEED     3
#define TRIPOD_SPEED    5

#ifdef AX12_HEXAPOD
#define TOP_SPEED      10
#endif

#ifdef AX18_HEXAPOD
#define TOP_SPEED      12
#endif


int speed = 5;

void setup(){
  pinMode(0,OUTPUT);
  pinMode(7,OUTPUT);
  // setup IK
  setupIK();
  gaitSelect(AMBLE_SMOOTH);
  // setup serial
  Serial.begin(38400);

  // wait, then check the voltage (LiPO safety)
  delay (1000);
  float voltage = (ax12GetRegister (1, AX_PRESENT_VOLTAGE, 1)) / 10.0;
  Serial.print ("System Voltage: ");
  Serial.print (voltage);
  Serial.println (" volts.");
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
  multiplier = AMBLE_SPEED;
  delay(1000);
  nerfA.attach(12);
  nerfB.attach(13);
  nerfC.attach(14);
  nerfD.attach(15);
  
  nerfA.write(105);
  nerfB.write(105);
  nerfC.write(105);
  nerfD.write(105);
       
  
}

void loop(){
  // take commands
  if(command.ReadMsgs() > 0){
    
    if(command.buttons&BUT_RT)
    {
      
      if(command.buttons&BUT_R1)
      { 
        if(fireAll == 0)
              {
        nerfA.write(113);
        nerfB.write(113);
        nerfC.write(113);
        nerfD.write(113);
        delay (1000);
        nerfA.write(105);
        nerfB.write(105);
        nerfC.write(105);
        nerfD.write(105);
        delay (1000);
        fireAll = 1;
              }
      }
      
      if(command.buttons&BUT_L6)
      { 
        if(fireA == 0)
              {
        nerfA.write(113);
        
        delay (1000);
        nerfA.write(105);
        
        delay (1000);
        fireA = 1;
              }
      }
      
      
      if(command.buttons&BUT_L5)
      { 
        if(fireB == 0)
              {
        nerfB.write(113);
        
        delay (1000);
        nerfB.write(105);
        
        delay (1000);
        fireB = 1;
              }
      }
      
      
      if(command.buttons&BUT_L4)
      { 
        if(fireC == 0)
              {
        nerfC.write(113);
        
        delay (1000);
        nerfC.write(105);
        
        delay (1000);
        fireC = 1;
              }
      }
      
   
      
      if(command.buttons&BUT_R3)
      { 
        if(fireD == 0)
              {
        nerfD.write(113);
        
        delay (1000);
        nerfD.write(105);
        
        delay (1000);
        fireD = 1;
              }
      }
      
   
          
      
      
      
      
    }
    
    else
    {
      
      digitalWrite(0,HIGH-digitalRead(0));
      // select gaits
      if(command.buttons&BUT_R1){ 
        gaitSelect(RIPPLE_SMOOTH); 
        multiplier=RIPPLE_SPEED;
      }
      if(command.buttons&BUT_R2){ 
        gaitSelect(AMBLE_SMOOTH); 
        multiplier=AMBLE_SPEED;
      }
      if(command.buttons&BUT_R3){ 
        gaitSelect(RIPPLE); 
        multiplier=RIPPLE_SPEED;
      }
      if(command.buttons&BUT_L4){ 
        gaitSelect(AMBLE); 
        multiplier=AMBLE_SPEED;
      }
      if(command.buttons&BUT_L5){ 
        gaitSelect(TRIPOD); 
        multiplier=TRIPOD_SPEED;
      }
      if(command.buttons&BUT_L6){ 
        gaitSelect(TRIPOD); 
        multiplier=TOP_SPEED;
      }
      
      
      
      // set movement speed
      if((command.walkV) > 15 || (command.walkV < -15) ){
        Xspeed = (multiplier*command.walkV)/2;
      }
      else
      {
        Xspeed = 0;
      }
      
      if((command.walkH) > 15 || (command.walkH < -15) ){   
      Yspeed = (multiplier*command.walkH)/2;
      }
      else
      {
       Yspeed = 0;
      }
      
      if((command.lookH) > 15 || (command.lookH < -15) ){
      Rspeed = -(command.lookH)/100.0;
      }
      else
      {
        Rspeed = 0;
      }
      
    }
    
// Use the phoenix code if you want pretty body rotation. :)    
//
//    if((command.buttons&BUT_LT) > 0){
//      bodyRotY = (((float)command.lookV))/300.0;
//      bodyRotZ = ((float)command.lookH)/300.0;  
//      bodyRotX = ((float)command.walkH)/300.0;  
//      Rspeed = 0;
//      Xspeed = 0;
//      Yspeed = 0;  
//    }

 }

  // if our previous interpolation is complete, recompute the IK
  if(bioloid.interpolating == 0){
    doIK();
    bioloid.interpolateSetup(tranTime);
  }
  // update joints
  bioloid.interpolateStep();
}