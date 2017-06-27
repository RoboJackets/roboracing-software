#ifndef _QUATERNIONFILTERS_H_
#define _QUATERNIONFILTERS_H_

/*
 * Lifted directly from SparkFun's Arduino library for the MPU9250
 * https://github.com/sparkfun/SparkFun_MPU-9250_Breakout_Arduino_Library
 */

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy,
                              float gz, float mx, float my, float mz,
                              float deltat);
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy,
                            float gz, float mx, float my, float mz,
                            float deltat);
const float * getQ();

#endif // _QUATERNIONFILTERS_H_
