#include "mpu9250.h"
#include <iostream>
#include <algorithm>

using namespace std;

MPU9250::MPU9250(uint8_t i2c_bus)
    : _i2c(i2c_bus)
{
    _i2c.address(MPU9250_ADDRESS);

    // Check MPU9250 WhoAmI register to ensure communication is working
    auto whoAmIVal = _i2c.readReg(MPU9250_WHO_AM_I);
    if(whoAmIVal != MPU9250_WHO_AM_I_VAL) {
        cerr << "Recieved unexpected value (" << static_cast<int>(whoAmIVal) << ") from MPU9250's \"Who Am I\" register." << endl;
        return;
    }

    // Put MPU9250 auxiliary I2C port into bypass (pass-through) mode
    // This allows direct communication to the AK8963 magnetometer
    auto current_config = _i2c.readReg(INT_PIN_CFG);
    current_config |= 0b00000010;
    _i2c.writeReg(INT_PIN_CFG, current_config);

    // Check AK8963 WhoAmI register to ensure communication is working
    _i2c.address(AK8963_ADDRESS);
    whoAmIVal = _i2c.readReg(AK8963_WIA);
    if(whoAmIVal != AK8963_WHO_AM_I_VAL) {
        cerr << "Recieved unexpected value (" << static_cast<int>(whoAmIVal) << ") from AK8963's \"Who Am I\" register." << endl;
        return;
    }
    
    _whoAmIPassed = true;

    getAres();
    getGres();
    getMres();
}

void MPU9250::getAccel(double &x, double &y, double &z) {
    array<int16_t, 3> raw_data;
    readAccelData(raw_data);
    x = raw_data[0] * aRes;
    y = raw_data[1] * aRes;
    z = raw_data[2] * aRes;
}

void MPU9250::getGyro(double &x, double &y, double &z) {
    array<int16_t, 3> raw_data;
    readGyroData(raw_data);
    x = raw_data[0] * gRes;
    y = raw_data[1] * gRes;
    z = raw_data[2] * gRes;
}

void MPU9250::getMag(double &x, double &y, double &z) {
    array<int16_t, 3> raw_data;
    readMagData(raw_data);
    x = raw_data[0] * mRes;
    y = raw_data[1] * mRes;
    z = raw_data[2] * mRes;
}

void MPU9250::getTemp(double &temperature) {
    uint16_t raw_data;
    readTempData(raw_data);
    temperature = (raw_data / 333.87f) + 21.0f;
}

void MPU9250::runSelfTest(array<float, 6> &results)
{
    array<uint8_t, 6> rawData = {0, 0, 0, 0, 0, 0};
    array<uint8_t, 6> factorySelfTestValues;
    array<uint32_t, 3> gAvg = {0, 0, 0};
    array<uint32_t, 3> aAvg = {0, 0, 0};
    array<uint32_t, 3> aSTAvg = {0, 0, 0};
    array<uint32_t, 3> gSTAvg = {0, 0, 0};
    array<float, 6> factoryTrim;
    
    // Set gyro sample rate to 1 kHz
    _i2c.address(MPU9250_ADDRESS);
    _i2c.writeReg(SMPLRT_DIV, 0x00);
    // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
    _i2c.address(MPU9250_ADDRESS);
    _i2c.writeReg(CONFIG, 0x02);
    // Set full scale range for the gyro to 250 dps
    _i2c.address(MPU9250_ADDRESS);
    _i2c.writeReg(GYRO_CONFIG, 1);
    // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
    _i2c.address(MPU9250_ADDRESS);
    _i2c.writeReg(ACCEL_CONFIG_2, 0x02);
    // Set full scale range for the accelerometer to 2 g
    _i2c.address(MPU9250_ADDRESS);
    _i2c.writeReg(ACCEL_CONFIG, 1);
    
    // Get average current values of gyro and acclerometer
    for (int i = 0; i < 200; i++)
    {
        // Read the six raw data registers into data array
        _i2c.address(MPU9250_ADDRESS);
        _i2c.readBytesReg(ACCEL_XOUT_H, rawData.data(), rawData.size());
        // Turn the MSB and LSB into a signed 16-bit value
        aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;
        aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
        aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

        // Read the six raw data registers sequentially into data array
        _i2c.address(MPU9250_ADDRESS);
        _i2c.readBytesReg(GYRO_XOUT_H, rawData.data(), rawData.size());
        // Turn the MSB and LSB into a signed 16-bit value
        gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;
        gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
        gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
    }
    
    transform(gAvg.begin(), gAvg.end(), gAvg.begin(),
              [](auto e){ return e/200; });
    transform(aAvg.begin(), aAvg.end(), aAvg.begin(), 
              [](auto e){ return e/200; });
    
    // Configure the accelerometer for self-test
    // Enable self test on all three axes and set accelerometer range to +/- 2 g
    _i2c.address(MPU9250_ADDRESS);
    _i2c.writeReg(ACCEL_CONFIG, 0xE0);
    // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
    _i2c.address(MPU9250_ADDRESS);
    _i2c.writeReg(GYRO_CONFIG, 0xE0);
    
    // Delay a while to let the device stabilize
    usleep(25'000);
    
    // Get average self-test values of gyro and acclerometer
    for (int i = 0; i < 200; i++)
    {
        // Read the six raw data registers into data array
        _i2c.address(MPU9250_ADDRESS);
        _i2c.readBytesReg(ACCEL_XOUT_H, rawData.data(), rawData.size());
        // Turn the MSB and LSB into a signed 16-bit value
        aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;
        aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
        aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

        // Read the six raw data registers sequentially into data array
        _i2c.address(MPU9250_ADDRESS);
        _i2c.readBytesReg(GYRO_XOUT_H, rawData.data(), rawData.size());
        // Turn the MSB and LSB into a signed 16-bit value
        gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;
        gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
        gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
    }
    
    transform(gSTAvg.begin(), gSTAvg.end(), gSTAvg.begin(),
              [](auto e){ return e/200; });
    transform(aSTAvg.begin(), aSTAvg.end(), aSTAvg.begin(), 
              [](auto e){ return e/200; });
    
    // Configure the gyro and accelerometer for normal operation
    _i2c.address(MPU9250_ADDRESS);
    _i2c.writeReg(ACCEL_CONFIG, 0x00);
    _i2c.address(MPU9250_ADDRESS);
    _i2c.writeReg(GYRO_CONFIG, 0x00);
    
    // Delay a while to let the device stabilize
    usleep(25'000);  
    
    // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
    // X-axis accel self-test results
    _i2c.address(MPU9250_ADDRESS);
    factorySelfTestValues[0] = _i2c.readReg(SELF_TEST_X_ACCEL);
    // Y-axis accel self-test results
    _i2c.address(MPU9250_ADDRESS);
    factorySelfTestValues[1] = _i2c.readReg(SELF_TEST_Y_ACCEL);
    // Z-axis accel self-test results
    _i2c.address(MPU9250_ADDRESS);
    factorySelfTestValues[2] = _i2c.readReg(SELF_TEST_Z_ACCEL);
    // X-axis gyro self-test results
    _i2c.address(MPU9250_ADDRESS);
    factorySelfTestValues[3] = _i2c.readReg(SELF_TEST_X_GYRO);
    // Y-axis gyro self-test results
    _i2c.address(MPU9250_ADDRESS);
    factorySelfTestValues[4] = _i2c.readReg(SELF_TEST_Y_GYRO);
    // Z-axis gyro self-test results
    _i2c.address(MPU9250_ADDRESS);
    factorySelfTestValues[5] = _i2c.readReg(SELF_TEST_Z_GYRO);
    
    // Retrieve factory self-test value from self-test code reads
    // FT[Xa] factory trim calculation
    factoryTrim[0] = 2620.0f*(pow(1.01f ,((float)factorySelfTestValues[0] - 1.0f) ));
    // FT[Ya] factory trim calculation
    factoryTrim[1] = 2620.0f*(pow(1.01f ,((float)factorySelfTestValues[1] - 1.0f) ));
    // FT[Za] factory trim calculation
    factoryTrim[2] = 2620.0f*(pow(1.01f ,((float)factorySelfTestValues[2] - 1.0f) ));
    // FT[Xg] factory trim calculation
    factoryTrim[3] = 2620.0f*(pow(1.01f ,((float)factorySelfTestValues[3] - 1.0f) ));
    // FT[Yg] factory trim calculation
    factoryTrim[4] = 2620.0f*(pow(1.01f ,((float)factorySelfTestValues[4] - 1.0f) ));
    // FT[Zg] factory trim calculation
    factoryTrim[5] = 2620.0f*(pow(1.01f ,((float)factorySelfTestValues[5] - 1.0f) ));
    
    // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim
    // of the Self-Test Response
    // To get percent, must multiply by 100
    for (int i = 0; i < 3; i++)
    {
        // Report percent differences
        results[i] = 100.0f * ((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i]
          - 100.0f;
        // Report percent differences
        results[i+3] = 100.0f * ((float)(gSTAvg[i] - gAvg[i])) / factoryTrim[i+3]
          - 100.0f;
    }
}

void MPU9250::calibrate()
{
    array<uint8_t, 12> data;
    uint16_t packet_count;
    uint16_t fifo_count;
    array<int32_t, 3> gyro_bias = {0, 0, 0};
    array<int32_t, 3> accel_bias = {0, 0, 0};
    
    // reset device
    // Write a one to bit 7 reset bit; toggle reset device
    _i2c.address(MPU9250_ADDRESS);
    _i2c.writeReg(PWR_MGMT_1, READ_FLAG);
    usleep(100'000);
    
    // get stable time source; Auto select clock source to be PLL gyroscope
    // reference if ready else use the internal oscillator, bits 2:0 = 001
    _i2c.address(MPU9250_ADDRESS);
    _i2c.writeReg(PWR_MGMT_1, 0x01);
    _i2c.address(MPU9250_ADDRESS);
    _i2c.writeReg(PWR_MGMT_2, 0x00);
    usleep(200'000);
    
    // Configure device for bias calculation
    // Disable all interrupts
    _i2c.address(MPU9250_ADDRESS);
    _i2c.writeReg(INT_ENABLE, 0x00);
    // Disable FIFO
    _i2c.address(MPU9250_ADDRESS);
    _i2c.writeReg(FIFO_EN, 0x00);
    // Turn on internal clock source
    _i2c.address(MPU9250_ADDRESS);
    _i2c.writeReg(PWR_MGMT_1, 0x00);
    // Disable I2C master
    _i2c.address(MPU9250_ADDRESS);
    _i2c.writeReg(I2C_MST_CTRL, 0x00);
    // Disable FIFO and I2C master modes
    _i2c.address(MPU9250_ADDRESS);
    _i2c.writeReg(USER_CTRL, 0x00);
    // Reset FIFO and DMP
    _i2c.address(MPU9250_ADDRESS);
    _i2c.writeReg(USER_CTRL, 0x00);
    usleep(15'000);
    
    // Configure MPU6050 gyro and accelerometer for bias calculation
    // Set low-pass filter to 188 Hz
    _i2c.address(MPU9250_ADDRESS);
    _i2c.writeReg(CONFIG, 0x01);
    // Set sample rate to 1 kHz
    _i2c.address(MPU9250_ADDRESS);
    _i2c.writeReg(SMPLRT_DIV, 0x00);
    // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    _i2c.address(MPU9250_ADDRESS);
    _i2c.writeReg(GYRO_CONFIG, 0x00);
    // Set accelerometer full-scale to 2 g, maximum sensitivity
    _i2c.address(MPU9250_ADDRESS);
    _i2c.writeReg(ACCEL_CONFIG, 0x00);
    
    uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
    uint16_t  accelsensitivity = 16384; // = 16384 LSB/g
    
    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    _i2c.address(MPU9250_ADDRESS);
    _i2c.writeReg(USER_CTRL, 0x40); // Enable FIFO
    // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
    _i2c.address(MPU9250_ADDRESS);
    _i2c.writeReg(FIFO_EN, 0x78);
    // accumulate 40 samples in 40 milliseconds = 480 bytes
    usleep(40'000);
    
    // At end of sample accumulation, turn off FIFO sensor read
    // Disable gyro and accelerometer sensors for FIFO
    _i2c.address(MPU9250_ADDRESS);
    _i2c.writeReg(FIFO_EN, 0x00);
    // Read FIFO sample count
    _i2c.address(MPU9250_ADDRESS);
    _i2c.readBytesReg(FIFO_COUNTH, data.data(), 2);
    fifo_count = ((uint16_t)data[0] << 8) | data[1];
    // How many sets of full gyro and accelerometer data for averaging
    packet_count = fifo_count/12;
    
    for (int i = 0; i < packet_count; i++)
    {
        int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
        // Read data for averaging
        _i2c.address(MPU9250_ADDRESS);
        _i2c.readBytesReg(FIFO_R_W, data.data(), 12);
        // Form signed 16-bit integer for each sample in FIFO
        accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  );
        accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  );
        accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  );
        gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  );
        gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  );
        gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]);

        // Sum individual signed 16-bit biases to get accumulated signed 32-bit
        // biases.
        accel_bias[0] += (int32_t) accel_temp[0];
        accel_bias[1] += (int32_t) accel_temp[1];
        accel_bias[2] += (int32_t) accel_temp[2];
        gyro_bias[0]  += (int32_t) gyro_temp[0];
        gyro_bias[1]  += (int32_t) gyro_temp[1];
        gyro_bias[2]  += (int32_t) gyro_temp[2];
    }
    // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[0] /= (int32_t) packet_count;
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;
    
    // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    if (accel_bias[2] > 0L)
    {
        accel_bias[2] -= (int32_t) accelsensitivity;
    }
    else
    {
        accel_bias[2] += (int32_t) accelsensitivity;
    }
    
    // Construct the gyro biases for push to the hardware gyro bias registers,
    // which are reset to zero upon device startup.
    // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input
    // format.
    data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF;
    // Biases are additive, so change sign on calculated average gyro biases
    data[1] = (-gyro_bias[0]/4)       & 0xFF;
    data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
    data[3] = (-gyro_bias[1]/4)       & 0xFF;
    data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
    data[5] = (-gyro_bias[2]/4)       & 0xFF;
    
    // Push gyro biases to hardware registers
    _i2c.address(MPU9250_ADDRESS);
    _i2c.writeReg(XG_OFFSET_H, data[0]);
    _i2c.address(MPU9250_ADDRESS);
    _i2c.writeReg(XG_OFFSET_L, data[1]);
    _i2c.address(MPU9250_ADDRESS);
    _i2c.writeReg(YG_OFFSET_H, data[2]);
    _i2c.address(MPU9250_ADDRESS);
    _i2c.writeReg(YG_OFFSET_L, data[3]);
    _i2c.address(MPU9250_ADDRESS);
    _i2c.writeReg(ZG_OFFSET_H, data[4]);
    _i2c.address(MPU9250_ADDRESS);
    _i2c.writeReg(ZG_OFFSET_L, data[5]);
    
    // Output scaled gyro biases for display in the main program
    gyroBias[0] = (float) gyro_bias[0]/(float) gyrosensitivity;
    gyroBias[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
    gyroBias[2] = (float) gyro_bias[2]/(float) gyrosensitivity;
    
    // Construct the accelerometer biases for push to the hardware accelerometer
    // bias registers. These registers contain factory trim values which must be
    // added to the calculated accelerometer biases; on boot up these registers
    // will hold non-zero values. In addition, bit 0 of the lower byte must be
    // preserved since it is used for temperature compensation calculations.
    // Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.

    // A place to hold the factory accelerometer trim biases
    int32_t accel_bias_reg[3] = {0, 0, 0};
    // Read factory accelerometer trim values
    _i2c.address(MPU9250_ADDRESS);
    _i2c.readBytesReg(XA_OFFSET_H, data.data(), 2);
    accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
    _i2c.address(MPU9250_ADDRESS);
    _i2c.readBytesReg(YA_OFFSET_H, data.data(), 2);
    accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
    _i2c.address(MPU9250_ADDRESS);
    _i2c.readBytesReg(ZA_OFFSET_H, data.data(), 2);
    accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
    
    // Define mask for temperature compensation bit 0 of lower byte of
    // accelerometer bias registers
    uint32_t mask = 1uL;
    // Define array to hold mask bit for each accelerometer bias axis
    array<uint8_t, 3> mask_bit = {0, 0, 0};

    for (int i = 0; i < 3; i++)
    {
        // If temperature compensation bit is set, record that fact in mask_bit
        if ((accel_bias_reg[i] & mask))
        {
          mask_bit[i] = 0x01;
        }
    }
    
    // Construct total accelerometer bias, including calculated average
    // accelerometer bias from above
    // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g
    // (16 g full scale)
    accel_bias_reg[0] -= (accel_bias[0]/8);
    accel_bias_reg[1] -= (accel_bias[1]/8);
    accel_bias_reg[2] -= (accel_bias[2]/8);

    data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    data[1] = (accel_bias_reg[0])      & 0xFF;
    // preserve temperature compensation bit when writing back to accelerometer
    // bias registers
    data[1] = data[1] | mask_bit[0];
    data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    data[3] = (accel_bias_reg[1])      & 0xFF;
    // Preserve temperature compensation bit when writing back to accelerometer
    // bias registers
    data[3] = data[3] | mask_bit[1];
    data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    data[5] = (accel_bias_reg[2])      & 0xFF;
    // Preserve temperature compensation bit when writing back to accelerometer
    // bias registers
    data[5] = data[5] | mask_bit[2];

    // Apparently this is not working for the acceleration biases in the MPU-9250
    // Are we handling the temperature correction bit properly?
    // Push accelerometer biases to hardware registers
    _i2c.address(MPU9250_ADDRESS);
    _i2c.writeReg(XA_OFFSET_H, data[0]);
    _i2c.address(MPU9250_ADDRESS);
    _i2c.writeReg(XA_OFFSET_L, data[1]);
    _i2c.address(MPU9250_ADDRESS);
    _i2c.writeReg(YA_OFFSET_H, data[2]);
    _i2c.address(MPU9250_ADDRESS);
    _i2c.writeReg(YA_OFFSET_L, data[3]);
    _i2c.address(MPU9250_ADDRESS);
    _i2c.writeReg(ZA_OFFSET_H, data[4]);
    _i2c.address(MPU9250_ADDRESS);
    _i2c.writeReg(ZA_OFFSET_L, data[5]);

    // Output scaled accelerometer biases for display in the main program
    accelBias[0] = (float)accel_bias[0]/(float)accelsensitivity;
    accelBias[1] = (float)accel_bias[1]/(float)accelsensitivity;
    accelBias[2] = (float)accel_bias[2]/(float)accelsensitivity;
}

void MPU9250::initialize()
{
    calibrate();
    initializeAccelAndGyro();
    initializeMagnetometer();
}

void MPU9250::initializeMagnetometer() {
    array<uint8_t, 3> rawData;
    // Power down magnetometer
    _i2c.address(AK8963_ADDRESS);
    _i2c.writeReg(AK8963_CNTL, 0x00);
    usleep(10'000);
    // Enter Fuse ROM access mode
    _i2c.address(AK8963_ADDRESS);
    _i2c.writeReg(AK8963_CNTL, 0x0F);
    usleep(10'000);

    // Read the x-, y-, and z-axis calibration values
    _i2c.address(AK8963_ADDRESS);
    _i2c.readBytesReg(AK8963_ASAX, rawData.data(), 3);

    // Populate sensitivity adjustment values
    transform(rawData.begin(), rawData.end(), factoryMagCalibration.begin(),
              [](const auto &raw){
                  return static_cast<float>((raw-128)/256. + 1.);
              });

    // Power down magnetometer
    _i2c.address(AK8963_ADDRESS);
    _i2c.writeReg(AK8963_CNTL, 0x00);

    usleep(10'000);

    /* Configure the magnetometer for continuous read and highest resultion.
     * Set Mscale bit 4 to 1 (0) to enable 16 (14) bit resultion in CNTL
     * register, and enable continuous mode data acquisition Mmode (bits [3:0]),
     * 0010 for 8 Hz and 0110 for 100 Hz sample rates.
     */
    _i2c.address(AK8963_ADDRESS);
    _i2c.writeReg(AK8963_CNTL, _Mscale << 4 | _Mmode);

    usleep(10'000);
}

void MPU9250::initializeAccelAndGyro() {
    // wake up device
    // Clear sleep mode bit (6), enable all sensors
    _i2c.address(MPU9250_ADDRESS);
    _i2c.writeReg(PWR_MGMT_1, 0x00);
    usleep(100'000); // Wait for all registers to reset

    // Get stable time source
    // Auto select clock source to be PLL gyroscope reference if ready else
    _i2c.address(MPU9250_ADDRESS);
    _i2c.writeReg(PWR_MGMT_1, 0x01);
    usleep(200'000);

    // Configure Gyro and Thermometer
    // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz,
    // respectively;
    // minimum delay time for this setting is 5.9 ms, which means sensor fusion
    // update rates cannot be higher than 1 / 0.0059 = 170 Hz
    // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
    // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!),
    // 8 kHz, or 1 kHz
    _i2c.address(MPU9250_ADDRESS);
    _i2c.writeReg(CONFIG, 0x03);

    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    // Use a 200 Hz rate; a rate consistent with the filter update rate
    // determined inset in CONFIG above.
    _i2c.address(MPU9250_ADDRESS);
    _i2c.writeReg(SMPLRT_DIV, 0x04);

    // Set gyroscope full scale range
    // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are
    // left-shifted into positions 4:3

    // get current GYRO_CONFIG register value
    _i2c.address(MPU9250_ADDRESS);
    uint8_t c = _i2c.readReg(GYRO_CONFIG);
    // c = c & ~0xE0; // Clear self-test bits [7:5]
    c = c & ~0x02; // Clear Fchoice bits [1:0]
    c = c & ~0x18; // Clear AFS bits [4:3]
    c = c | _Gscale << 3; // Set full scale range for the gyro
    // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of
    // GYRO_CONFIG
    // c =| 0x00;
    // Write new GYRO_CONFIG value to register
    _i2c.address(MPU9250_ADDRESS);
    _i2c.writeReg(GYRO_CONFIG, c);

    // Set accelerometer full-scale range configuration
    // Get current ACCEL_CONFIG register value
    _i2c.address(MPU9250_ADDRESS);
    c = _i2c.readReg(ACCEL_CONFIG);
    // c = c & ~0xE0; // Clear self-test bits [7:5]
    c = c & ~0x18;  // Clear AFS bits [4:3]
    c = c | _Ascale << 3; // Set full scale range for the accelerometer
    // Write new ACCEL_CONFIG register value
    _i2c.address(MPU9250_ADDRESS);
    _i2c.writeReg(ACCEL_CONFIG, c);

    // Set accelerometer sample rate configuration
    // It is possible to get a 4 kHz sample rate from the accelerometer by
    // choosing 1 for accel_fchoice_b bit [3]; in this case the bandwidth is
    // 1.13 kHz
    // Get current ACCEL_CONFIG2 register value
    _i2c.address(MPU9250_ADDRESS);
    _i2c.readReg(ACCEL_CONFIG_2);
    c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
    c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
    // Write new ACCEL_CONFIG2 register value
    _i2c.address(MPU9250_ADDRESS);
    _i2c.writeReg(ACCEL_CONFIG_2, c);
    // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
    // but all these rates are further reduced by a factor of 5 to 200 Hz because
    // of the SMPLRT_DIV setting

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH
    // until interrupt cleared, clear on read of INT_STATUS, and enable
    // I2C_BYPASS_EN so additional chips can join the I2C bus and all can be
    // controlled by the Arduino as master.
    _i2c.address(MPU9250_ADDRESS);
    _i2c.writeReg(INT_PIN_CFG, 0x22);
    // Enable data ready (bit 0) interrupt
    _i2c.address(MPU9250_ADDRESS);
    _i2c.writeReg(INT_ENABLE, 0x01);
    usleep(100'000);
}

void MPU9250::getAres() {
    switch(_Ascale) {
        case AFS_2G:
            aRes = 2.0f / 32768.0f;
            break;
        case AFS_4G:
            aRes = 4.0f / 32768.0f;
            break;
        case AFS_8G:
            aRes = 8.0f / 32768.0f;
            break;
        case AFS_16G:
            aRes = 16.0f / 32768.0f;
            break;
    }
}

void MPU9250::getGres() {
    switch(_Gscale) {
        case GFS_250DPS:
            gRes = 250.0f / 32768.0f;
            break;
        case GFS_500DPS:
            gRes = 500.0f / 32768.0f;
            break;
        case GFS_1000DPS:
            gRes = 1000.0f / 32768.0f;
            break;
        case GFS_2000DPS:
            gRes = 2000.0f / 32768.0f;
            break;
    }
}

void MPU9250::getMres() {
    switch(_Mscale) {
        case MFS_14BITS:
            mRes = 10.0f * 4912.0f / 8190.0f;
            break;
        case MFS_16BITS:
            mRes = 10.0f * 4912.0f / 32760.0f;
            break;
    }
}

void MPU9250::readAccelData(std::array<int16_t, 3> &data) {
    array<uint8_t, 6> rawData;

    _i2c.address(MPU9250_ADDRESS);
    _i2c.readBytesReg(ACCEL_XOUT_H, rawData.data(), 6);

    data[0] = static_cast<int16_t>(rawData[0] << 8) | rawData[1];
    data[1] = static_cast<int16_t>(rawData[2] << 8) | rawData[3];
    data[2] = static_cast<int16_t>(rawData[4] << 8) | rawData[5];
}

void MPU9250::readGyroData(std::array<int16_t, 3> &data) {
    array<uint8_t, 6> rawData;

    _i2c.address(MPU9250_ADDRESS);
    _i2c.readBytesReg(GYRO_XOUT_H, rawData.data(), 6);

    data[0] = static_cast<int16_t>(rawData[0] << 8) | rawData[1];
    data[1] = static_cast<int16_t>(rawData[2] << 8) | rawData[3];
    data[2] = static_cast<int16_t>(rawData[4] << 8) | rawData[5];
}

void MPU9250::readMagData(std::array<int16_t, 3> &data) {
    array<uint8_t, 7> rawData;

    // Wait for magnetometer data ready bit to be set
    _i2c.address(AK8963_ADDRESS);
    if(_i2c.readReg(AK8963_ST1) & 0x01) {

        // Read the six raw data registers and ST2 sequentially into data array
        _i2c.address(AK8963_ADDRESS);
        _i2c.readBytesReg(AK8963_XOUT_L, rawData.data(), 7);

        auto c = rawData[6];

        if(!(c & 0x08)) {
            data[0] = static_cast<int16_t>(rawData[1] << 8) | rawData[0];
            data[1] = static_cast<int16_t>(rawData[3] << 8) | rawData[2];
            data[2] = static_cast<int16_t>(rawData[5] << 8) | rawData[4];
        }
    }
}

void MPU9250::readTempData(uint16_t &data) {
    array<uint8_t, 2> rawData;

    _i2c.address(MPU9250_ADDRESS);
    _i2c.readBytesReg(TEMP_OUT_H,rawData.data(),2);

    data  = static_cast<uint16_t>(rawData[0] << 8) | rawData[1];
}
