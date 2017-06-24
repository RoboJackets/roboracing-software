#include <signal.h>
#include <ros/ros.h>
#include <mraa.hpp>
#include <iostream>
#include "mpu9250.h"

using namespace std;

constexpr uint8_t MPU9250_ADDRESS = 0b1101000;
constexpr uint8_t MPU9250_WHO_AM_I_REGISTER = 117;
constexpr uint8_t MPU9250_WHO_AM_I_VALUE = 0x71;
constexpr uint8_t MPU9250_ACCEL_XOUT_H_REG = 59;
constexpr uint8_t MPU9250_ACCEL_XOUT_L_REG = 60;

volatile bool running = true;

void signal_handler(int) {
    running = false;
}

void set_SIGINT_handler() {
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = signal_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
}

int main(int argc, char **argv) {

    set_SIGINT_handler();

//    ros::init(argc, argv,  "mpu9250_for_joule");

//    ros::NodeHandle handle;

/*
 * TODO look at https://communities.intel.com/thread/110551
 */

    MPU9250 imu;
    
    if(!imu.whoAmIPassed()) {
        cerr << "Failed to read Who-Am-I value from IMU." << endl;
        return 1;
    }
    
    array<float, 6> selfTestResults;
    imu.runSelfTest(selfTestResults);
    
    if(any_of(selfTestResults.begin(), selfTestResults.end(),
           [](const auto &e){ return e > 14.0; })) {
        cerr << "WARNING: SelfTest results outside of acceptable range." << endl;
        for(const auto &val : selfTestResults) {
            cerr << val << ", ";
        }
        cerr << endl;
    }
    
    imu.calibrate();

    imu.initialize();

    imu.initializeMagnetometer();

    imu.getAres();
    imu.getGres();
    imu.getMres();

    array<int16_t, 3> accel_data;
    array<int16_t, 3> gyro_data;
    array<int16_t, 3> mag_data;

    array<float, 3> accel_real;
    array<float, 3> gyro_real;
    array<float, 3> mag_real;

    while(running) {
        imu.readAccelData(accel_data);
        imu.readGyroData(gyro_data);

        transform(accel_data.begin(), accel_data.end(), accel_real.begin(),
                  [&imu](const auto &e){ return e * imu.aRes;});

        transform(gyro_data.begin(), gyro_data.end(), gyro_real.begin(),
                  [&imu](const auto &e){ return e * imu.gRes;});

        cout << accel_real[0] << " " << accel_real[1] << " " << accel_real[2] << "\t" << gyro_real[0] << " " << gyro_real[1] << " " << gyro_real[2] << endl;

        usleep(500'000);
    }

    return 0;
}

