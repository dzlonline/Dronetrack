//************************************************************************
//  HCD flying from DroneTrack
//  (C) DZL December2013
//************************************************************************

#include <HCD.h>

HCD drone0;

//Drone ID's (pick 4 random numbers)
unsigned char ID0[]={
  0x16,0x01,0x55,0x11};

void setup()
{
  Serial.begin(19200);
}

unsigned long timer=0;

unsigned char inbuf[10]={0,0,0,0,0,0,0,0,0,0};
unsigned char inptr=0;

void loop()
{

  if(Serial.available())
  {
    unsigned char c=Serial.read();
    if(c==255)
    {
      if(inbuf[4]&0x01)
        drone0.reconnect(ID0);

      if(inbuf[4]&0x02)
      {
        drone0.bind(ID0);
      }

      inptr=0;      
    
    }
    else
    {
      if(inptr<10)
        inbuf[inptr++]=c;
    }
  }

  if(millis()>=timer)
  {

    if(drone0.inactive())
        drone0.bind(ID0);
 
    timer+=20;
    drone0.update(inbuf[0],inbuf[1],0x40,inbuf[2],inbuf[3],0x40,0x40,0);
  }
}
































