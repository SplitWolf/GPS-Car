#include "HMC5883L_Driver.h"
#include "Wire.h"

HMC5883L::HMC5883L(uint8_t sCL_PIN, uint8_t sDA_PIN)
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

bool HMC5883L::begin() {
    Wire.setSCL(m_sCL_PIN);
    Wire.setSDA(m_sDA_PIN);
    Wire.begin();
    this->writeDefaultConfig();
    m_initilized = true;
    //TODO: Check stuff
    return true;
}

void HMC5883L::setGain(hm5883lGain_t gain)
{
    writeDevice(hm5883lRegisters_t::HMC5883L_REGISTER_CRB, gain);
}

void HMC5883L::setMode(hm5883lOperatingMode_t mode, bool highSpeed)
{
    uint8_t highSpeedValue = highSpeed << 7;
    writeDevice(hm5883lRegisters_t::HMC5883L_REGISTER_MR, mode | highSpeedValue);
}

void HMC5883L::setDataRate(hmc5883lDataRate_t rate)
{
    m_configA.dateRate = rate;
    writeConfigA();
}

void HMC5883L::setSamples(hmc5883lSamplesAveraged_t samples)
{
    m_configA.samples = samples;
    writeConfigA();
}

bool HMC5883L::isDataReady() {
    int rdy = readDeivce(hm5883lRegisters_t::HMC5883L_REGISTER_STR);
    return (rdy & 0x1) == 1;
}

void HMC5883L::waitForDataReady() {
    while(true) {
        if(isDataReady()) {
            break;
        }
        delay(2);
    }
}

HMC5883L_Data HMC5883L::readData() {
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

    return HMC5883L_Data { x, y, z };
}

void HMC5883L::selfTest()
{
    //TODO: Update to acutally report test results
    hm5883lConfigA prev_config = m_configA;

    m_configA.samples = hmc5883lSamplesAveraged_t::HMC5883L_SAMPLES_AVGD_8;
    m_configA.dateRate = hmc5883lDataRate_t::HMC5883L_DATE_RATE_15_Hz;
    m_configA.mode = hmc5883lMeasurementMode_t::HMC5883L_MODE_POS_BAIS;    

    writeConfigA();
    //TODO: Maybe store previous settings?
    setGain(hm5883lGain_t::HMC5883L_GAIN_4_7);
    setMode(hm5883lOperatingMode_t::HMC5883L_MODE_CONTINOUS, false);
    waitForDataReady();

    HMC5883L_Data data = this->readData();

    m_configA = prev_config;
    writeConfigA();
}

void HMC5883L::setMeasurmentMode(hmc5883lMeasurementMode_t mode)
{
    m_configA.mode = mode;
    writeConfigA();
}

void HMC5883L::writeDefaultConfig()
{
    writeConfigA();
    setGain(hm5883lGain_t::HMC5883L_GAIN_4_7);
    setMode(hm5883lOperatingMode_t::HMC5883L_MODE_CONTINOUS, false);
}

void HMC5883L::writeConfigA() {
    writeDevice(hm5883lRegisters_t::HMC5883L_REGISTER_CRA, m_configA.samples | m_configA.dateRate | m_configA.mode);
}

void HMC5883L::writeDevice(hm5883lRegisters_t reg, int data)
{
    Wire.beginTransmission(HMC5883L_DEVICE_ADDRESS);
    Wire.send(reg);
    if(data != -1) {
        Wire.send(data);
    }
    Wire.endTransmission();
}

uint8_t HMC5883L::readDeivce(hm5883lRegisters_t reg)
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
