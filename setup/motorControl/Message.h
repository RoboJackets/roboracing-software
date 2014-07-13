#ifndef Message_h
#define Message_h

#include "Arduino.h"


class Message
{
  public:
    int getMessage(int& theSpeed, int& theHeading);
    int getMessage2(int& theSpeed, int& theHeading);
    
 private:
   int waitForNextByte(void);
   
};
#endif
