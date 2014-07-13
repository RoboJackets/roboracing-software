#include "Message.h"

//sets speed and heading. if they are successfully set, return 0, otherwise returns an error code
int Message::getMessage(int& theSpeed, int& theHeading)
{
  int incomingByte = Serial.read();
  if(incomingByte==181)
  {
    int bytes[3];
    for(int i=0;i<2;i++)
    {
      if (waitForNextByte()==-1)
        return -1;
      bytes[i] =Serial.read();
    }

    if (bytes[2]!=182)
      return -2;
    
    if (bytes[0]>180 || bytes[1]>180)
      return -3;
    
    theSpeed = bytes[0]-90; //convert back to -90 to 90 scale
    theHeading = bytes[1]-90; //convert back to -90 to 90 scale
    //Serial.println("New message received");
    return 0;
  }
}

int Message::getMessage2(int& theSpeed, int& theHeading)
{
  int message[4];
  message[0] = Serial.read();
  
  //Serial.print("first byte received: ");
  //Serial.print(message[0]);
  //Serial.println();
  
  if(message[0]==181)
  {
    for(int i=0;i<3;i++)
    {
      if (waitForNextByte()==-1)
        return -1;
      message[i+1] =Serial.read();
    }
    
/*
    Serial.print ("New message received ");
    
    for(int i=0;i<4;i++)
    {
        Serial.print(message[i]);
        Serial.print(" ");
    }
    Serial.println();
*/

    if (message[3]!=182)
      return -2;
    
    if (message[1]>180 || message[2]>180)
      return -3;
    
    theSpeed = message[1]-90; //convert back to -90 to 90 scale
    theHeading = message[2]-90; //convert back to -90 to 90 scale

    return 0;
  }
}


//Waits for next byte of info to arrive, returns 0 once it has or -1 if it times out

int Message::waitForNextByte()
{
  unsigned int timeout = 50;
  unsigned long start = millis();
  unsigned long time;
  while(!Serial.available())
  {
    time = millis();
    if((time-start)>timeout)
    {
      return -1;
    }
  }
  return 0;
}
