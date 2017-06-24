#ifndef MPU9250_H
#define MPU9250_H
#include <mraa.hpp>
#include <array>

/* Joule Pinout
 * http://www.intel.com/content/www/us/en/support/boards-and-kits/intel-joule-kits/000022494.html
 * Sparkfun Arduino Library
 * https://github.com/sparkfun/SparkFun_MPU-9250_Breakout_Arduino_Library/blob/master/src/MPU9250.cpp
 * Datasheet
 * https://cdn.sparkfun.com/assets/learn_tutorials/5/5/0/MPU9250REV1.0.pdf
 */

class MPU9250 {
public:
    MPU9250(uint8_t i2c_bus = 0);
    
    inline bool whoAmIPassed() {
        return _whoAmIPassed;
    }
    
    void runSelfTest(std::array<float,6 > &results);
    
    void calibrate();
    
    void initialize();

    void initializeMagnetometer();

    void getAres();
    void getGres();
    void getMres();

    void readAccelData(std::array<int16_t, 3> &data);

    void readGyroData(std::array<int16_t, 3> &data);

    void readMagData(std::array<int16_t, 3> &data);

    enum Ascale {
        AFS_2G = 0,
        AFS_4G,
        AFS_8G,
        AFS_16G
    };

    enum Gscale {
        GFS_250DPS = 0,
        GFS_500DPS,
        GFS_1000DPS,
        GFS_2000DPS
    };

    enum Mscale {
        MFS_14BITS = 0,
        MFS_16BITS
    };

    enum M_MODE {
        M_8HZ = 0x02,
        M_100HZ = 0x06
    };

    float aRes;
    float gRes;
    float mRes;

private:
    mraa::I2c _i2c;
    
    bool _whoAmIPassed = false;
    
    std::array<float, 3> gyroBias;
    std::array<float, 3> accelBias;
    std::array<float, 3> magBias;
    std::array<float, 3> magScale;

    uint8_t _Gscale = GFS_250DPS;
    uint8_t _Ascale = AFS_2G;
    uint8_t _Mscale = MFS_16BITS;
    uint8_t _Mmode = M_8HZ;

    std::array<float, 3> factoryMagCalibration;
    std::array<float, 3> factoryMagBias;
    
    // Useful constants
    static constexpr uint8_t MPU9250_ADDRESS = 0b1101000;
    static constexpr uint8_t MPU9250_WHO_AM_I_VAL = 0x71;
    static constexpr uint8_t READ_FLAG = 0x80;
    static constexpr uint8_t AK8963_ADDRESS = 0x0C;
    static constexpr uint8_t AK8963_WHO_AM_I_VAL = 0x48;
    
    /* Useful MPU9250 registers
     * See MPU9250 Register map
     * https://cdn.sparkfun.com/assets/learn_tutorials/5/5/0/MPU-9250-Register-Map.pdf
     */
    static constexpr uint8_t SELF_TEST_X_GYRO = 0;
    static constexpr uint8_t SELF_TEST_Y_GYRO = 0;
    static constexpr uint8_t SELF_TEST_Z_GYRO = 0;
    static constexpr uint8_t SELF_TEST_X_ACCEL = 0;
    static constexpr uint8_t SELF_TEST_Y_ACCEL = 0;
    static constexpr uint8_t SELF_TEST_Z_ACCEL = 0;
    static constexpr uint8_t XG_OFFSET_H = 19;
    static constexpr uint8_t XG_OFFSET_L = 20;
    static constexpr uint8_t YG_OFFSET_H = 21;
    static constexpr uint8_t YG_OFFSET_L = 22;
    static constexpr uint8_t ZG_OFFSET_H = 23;
    static constexpr uint8_t ZG_OFFSET_L = 24;
    static constexpr uint8_t SMPLRT_DIV = 25;
    static constexpr uint8_t CONFIG = 26;
    static constexpr uint8_t GYRO_CONFIG = 27;
    static constexpr uint8_t ACCEL_CONFIG = 28;
    static constexpr uint8_t ACCEL_CONFIG_2 = 29;
    static constexpr uint8_t FIFO_EN = 35;
    static constexpr uint8_t I2C_MST_CTRL = 36;
    static constexpr uint8_t INT_PIN_CFG = 55;
    static constexpr uint8_t INT_ENABLE = 56;
    static constexpr uint8_t ACCEL_XOUT_H = 59;
    static constexpr uint8_t ACCEL_XOUT_L = 60;
    static constexpr uint8_t ACCEL_YOUT_H = 61;
    static constexpr uint8_t ACCEL_YOUT_L = 62;
    static constexpr uint8_t ACCEL_ZOUT_H = 63;
    static constexpr uint8_t ACCEL_ZOUT_L = 64;
    static constexpr uint8_t TEMP_OUT_H = 65;
    static constexpr uint8_t TEMP_OUT_L = 66;
    static constexpr uint8_t GYRO_XOUT_H = 67;
    static constexpr uint8_t GYRO_XOUT_L = 68;
    static constexpr uint8_t GYRO_YOUT_H = 69;
    static constexpr uint8_t GYRO_YOUT_L = 70;
    static constexpr uint8_t GYRO_ZOUT_H = 71;
    static constexpr uint8_t GYRO_ZOUT_L = 72;
    static constexpr uint8_t USER_CTRL = 106;
    static constexpr uint8_t PWR_MGMT_1 = 107;
    static constexpr uint8_t PWR_MGMT_2 = 108;
    static constexpr uint8_t FIFO_COUNTH = 114;
    static constexpr uint8_t FIFO_R_W = 116;
    static constexpr uint8_t MPU9250_WHO_AM_I = 117;
    static constexpr uint8_t XA_OFFSET_H = 119;
    static constexpr uint8_t XA_OFFSET_L = 120;
    static constexpr uint8_t YA_OFFSET_H = 122;
    static constexpr uint8_t YA_OFFSET_L = 123;
    static constexpr uint8_t ZA_OFFSET_H = 125;
    static constexpr uint8_t ZA_OFFSET_L = 126;

    /* Useful AK8963 registers
    * See AK8963 Register map
    * https://cdn.sparkfun.com/assets/learn_tutorials/5/5/0/MPU-9250-Register-Map.pdf
    */
    static constexpr uint8_t AK8963_WIA = 0x00;
    static constexpr uint8_t AK8963_INFO = 0x01;
    static constexpr uint8_t AK8963_ST1 = 0x02;
    static constexpr uint8_t AK8963_XOUT_L = 0x03;
    static constexpr uint8_t AK8963_XOUT_H = 0x04;
    static constexpr uint8_t AK8963_YOUT_L = 0x05;
    static constexpr uint8_t AK8963_YOUT_H = 0x06;
    static constexpr uint8_t AK8963_ZOUT_L = 0x07;
    static constexpr uint8_t AK8963_ZOUT_H = 0x08;
    static constexpr uint8_t AK8963_ST2 = 0x09;
    static constexpr uint8_t AK8963_CNTL = 0x0A;
    static constexpr uint8_t AK8963_ASTC = 0x0C;
    static constexpr uint8_t AK8963_I2CDIS = 0x0F;
    static constexpr uint8_t AK8963_ASAX = 0x10;
    static constexpr uint8_t AK8963_ASAY = 0x11;
    static constexpr uint8_t AK8963_ASAZ = 0x12;
};

#endif // MPU9250_H
