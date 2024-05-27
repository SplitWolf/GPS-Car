#include "HMC5883L_Driver.h"
#include "Wire.h"

namespace HMC5883L {

    Driver::Driver(uint8_t sCL_PIN, uint8_t sDA_PIN)
    {
        m_sCL_PIN = sCL_PIN;
        m_sDA_PIN = sDA_PIN;
        m_initilized = false;
        m_configA = {
            hmc5883lDataRate_t::HMC5883L_DATE_RATE_15_Hz,
            hmc5883lMeasurementMode_t::HMC5883L_MODE_NORMAL,
            hmc5883lSamplesAveraged_t::HMC5883L_SAMPLES_AVGD_8,
        };
    }

    bool Driver::begin() {
        Wire.setSCL(m_sCL_PIN);
        Wire.setSDA(m_sDA_PIN);
        Wire.begin();
        this->writeDefaultConfig();
        m_initilized = true;
        //TODO: Check stuff
        return true;
    }

    void Driver::setGain(hm5883lGain_t gain)
    {
        writeDevice(hm5883lRegisters_t::HMC5883L_REGISTER_CRB, gain);
    }

    void Driver::setMode(hm5883lOperatingMode_t mode, bool highSpeed)
    {
        uint8_t highSpeedValue = highSpeed << 7;
        writeDevice(hm5883lRegisters_t::HMC5883L_REGISTER_MR, mode | highSpeedValue);
    }

    void Driver::setDataRate(hmc5883lDataRate_t rate)
    {
        uint8_t currentConfig = readDeivce(hm5883lRegisters_t::HMC5883L_REGISTER_CRA);
        uint8_t DR_MASK = 0x7 << 2;
        uint8_t newConfig = (currentConfig & ~DR_MASK) |  rate;
        writeDevice(hm5883lRegisters_t::HMC5883L_REGISTER_CRA, newConfig);
    }

    void Driver::setSamples(hmc5883lSamplesAveraged_t samples)
    {
        uint8_t currentConfig = readDeivce(hm5883lRegisters_t::HMC5883L_REGISTER_CRA);
        uint8_t SMP_MASK = 0x3 << 5;
        uint8_t newConfig = (currentConfig & ~SMP_MASK) |  samples;
        writeDevice(hm5883lRegisters_t::HMC5883L_REGISTER_CRA, newConfig);
    }

    bool Driver::isDataReady() {
        int rdy = readDeivce(hm5883lRegisters_t::HMC5883L_REGISTER_STR);
        return (rdy & 0x1) == 1;
    }

    void Driver::waitForDataReady() {
        while(true) {
            if(isDataReady()) {
                break;
            }
            delay(2);
        }
    }

    XYZ_Data Driver::readData() {
        int16_t x = 0, y = 0, z = 0;
        writeDevice(hm5883lRegisters_t::HMC5883L_REGISTER_OUT_X_H,-1);
        Wire.requestFrom(HMC5883L_DEVICE_ADDRESS, 6);
        if(6<=Wire.available()){
            x =  Wire.read() << 8; //X msb
            x |= Wire.read();      //X lsb
            z =  Wire.read() << 8; //Z msb
            z |= Wire.read();      //Z lsb
            y =  Wire.read() << 8; //Y msb
            y |= Wire.read();      //Y lsb
        }

        return XYZ_Data { x, y, z };
    }

    // void Driver::outputMotionCalData(XYZ_Data &accel_data, XYZ_Data &gyro_data)
    // {
    //     XYZ_Data mag_data = readData();
    //     Serial.print("Raw:");
    //     Serial.print(accel_data.x);
    //     Serial.print(',');
    //     Serial.print(accel_data.y);
    //     Serial.print(',');
    //     Serial.print(accel_data.z);
    //     Serial.print(',');
    //     Serial.print(gyro_data.x);
    //     Serial.print(',');
    //     Serial.print(gyro_data.y);
    //     Serial.print(',');
    //     Serial.print(gyro_data.z);
    //     Serial.print(',');
    //     Serial.print(mag_data.x);
    //     Serial.print(',');
    //     Serial.print(mag_data.y);
    //     Serial.print(',');
    //     Serial.print(mag_data.z);
    //     Serial.println();
    //     // Serial.printf("Raw:%d,%d,%d,%d,%d,%d,%d,%d,%d\n",accel_data.x,accel_data.y,accel_data.z,gyro_data.x,gyro_data.y,gyro_data.z,mag_data.x,mag_data.y,mag_data.z);
    // }

    void Driver::selfTest()
    {
        //TODO: Update to acutally report test results
        writeDevice(hm5883lRegisters_t::HMC5883L_REGISTER_CRA, HMC5883L_SAMPLES_AVGD_8 | HMC5883L_DATE_RATE_15_Hz | HMC5883L_MODE_POS_BAIS);
        writeDevice(hm5883lRegisters_t::HMC5883L_REGISTER_CRB, hm5883lGain_t::HMC5883L_GAIN_4_7);

        writeDevice(hm5883lRegisters_t::HMC5883L_REGISTER_MR, hm5883lOperatingMode_t::HMC5883L_MODE_CONTINOUS);

        waitForDataReady();

        XYZ_Data data = this->readData();
    }

    void Driver::setMeasurmentMode(hmc5883lMeasurementMode_t mode)
    {
        uint8_t currentConfig = readDeivce(hm5883lRegisters_t::HMC5883L_REGISTER_CRA);
        uint8_t MEA_MASK = 0x3;
        uint8_t newConfig = (currentConfig & ~MEA_MASK) |  mode;
        writeDevice(hm5883lRegisters_t::HMC5883L_REGISTER_CRA, newConfig);
    }

    void Driver::writeDefaultConfig()
    {
        writeConfigA();
        setGain(hm5883lGain_t::HMC5883L_GAIN_4_7);
        setMode(hm5883lOperatingMode_t::HMC5883L_MODE_CONTINOUS, false);
    }

    void Driver::writeConfigA() {
        writeDevice(hm5883lRegisters_t::HMC5883L_REGISTER_CRA, m_configA.samples | m_configA.dateRate | m_configA.mode);
    }

    void Driver::writeDevice(hm5883lRegisters_t reg, int data)
    {
        Wire.beginTransmission(HMC5883L_DEVICE_ADDRESS);
        Wire.send(reg);
        if(data != -1) {
            Wire.send(data);
        }
        Wire.endTransmission();
    }

    uint8_t Driver::readDeivce(hm5883lRegisters_t reg)
    {
        uint8_t data = 0;
        Wire.beginTransmission(HMC5883L_DEVICE_ADDRESS);
        Wire.send(reg);
        Wire.endTransmission();
        Wire.requestFrom(HMC5883L_DEVICE_ADDRESS,1);
        while(Wire.available()) {
            data = Wire.read();
        }
        return data;
    }
};
