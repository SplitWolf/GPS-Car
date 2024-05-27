#include <stdint.h>

namespace HMC5883L 
{
    // Device Address
    #define HMC5883L_DEVICE_ADDRESS 0x1E

    // Device enum constants
    #pragma region CONSTANTS

    // Limits with gain = 4.7 / 390
    #define HMC5883L_TEST_LIMIT_LOW = 243
    #define HMC5883L_TEST_LIMIT_HIGH = 575

    typedef enum  {
        HMC5883L_REGISTER_CRA = 0x00,
        HMC5883L_REGISTER_CRB = 0x01,
        HMC5883L_REGISTER_MR = 0x02,
        HMC5883L_REGISTER_OUT_X_H = 0x03,
        HMC5883L_REGISTER_OUT_X_L = 0x04,
        HMC5883L_REGISTER_OUT_Z_H = 0x05,
        HMC5883L_REGISTER_OUT_Z_L = 0x06,
        HMC5883L_REGISTER_OUT_Y_H = 0x07,
        HMC5883L_REGISTER_OUT_Y_L = 0x08,
        HMC5883L_REGISTER_STR = 0x09, // SR1: Lock, SR0: RDY
        HMC5883L_REGISTER_IRA = 0x0A, // ASCII Value: H
        HMC5883L_REGISTER_IRB = 0x0B, // ASCII Value: 4
        HMC5883L_REGISTER_IRC = 0x0C, // ASCII Value: 3
    } hm5883lRegisters_t;


    typedef enum {                  // Gain: Lsb
        HMC5883L_GAIN_0_8_8 = 0x00,// 1370
        HMC5883L_GAIN_1_3 = 0x20,  // Default 1090
        HMC5883L_GAIN_1_9 = 0x40,  // 820
        HMC5883L_GAIN_2_5 = 0x60,  // 660
        HMC5883L_GAIN_4_0 = 0x80,  // 440
        HMC5883L_GAIN_4_7 = 0xA0,  // 390
        HMC5883L_GAIN_5_6 = 0xC0,  // 330
         HMC5883L_GAIN_8_1 = 0xE,  // 230
    } hm5883lGain_t;

    typedef enum {
        HMC5883L_DATE_RATE_0_75_Hz = 0x00,
        HMC5883L_DATE_RATE_1_5_Hz = 0x04,
        HMC5883L_DATE_RATE_3_Hz = 0x08,
        HMC5883L_DATE_RATE_7_5_Hz = 0x0C,
        HMC5883L_DATE_RATE_15_Hz = 0x10, // Default
        HMC5883L_DATE_RATE_30_Hz = 0x14,
        HMC5883L_DATE_RATE_75_Hz = 0x18,
    } hmc5883lDataRate_t;

    typedef enum  {
        HMC5883L_SAMPLES_AVGD_1 = 0x00, // Default
        HMC5883L_SAMPLES_AVGD_2 = 0x20,
        HMC5883L_SAMPLES_AVGD_4 = 0x40,
        HMC5883L_SAMPLES_AVGD_8 = 0x60, 
    } hmc5883lSamplesAveraged_t;

    typedef enum {
        HMC5883L_MODE_NORMAL = 0x00, // Default
        HMC5883L_MODE_POS_BAIS = 0x01,
        HMC5883L_MODE_NEG_BAIS = 0x02,
    } hmc5883lMeasurementMode_t;

    typedef enum {
        HMC5883L_MODE_CONTINOUS = 0x00,
        HMC5883L_MODE_SINGLE = 0x01
    } hm5883lOperatingMode_t;

    struct hm5883lConfigA {
        hmc5883lDataRate_t dateRate;
        hmc5883lMeasurementMode_t mode;
        hmc5883lSamplesAveraged_t samples;
    };

    #pragma endregion CONSTANTS

    //TODO: Parse this with bool function
    #define HMC5883L_HIGH_SPEED_I2C = 0x1 << 7

    struct XYZ_Data
    {
        int16_t x;
        int16_t y;
        int16_t z;
    };

    class Driver
    {        
    public:
        Driver(uint8_t sCL_PIN, uint8_t sDA_PIN);
        // TODO: Do I need the destructor?
        // ~Driver();

        // TODO: Allow input of magnetic declination
        // TODO: Perform Hard Iron Calibration
        bool begin();

        bool isDataReady();
        void waitForDataReady(); // Blocks until data is ready
        XYZ_Data readData();

        // void outputMotionCalData(XYZ_Data& accel_data, XYZ_Data& gyro_data);

        void setGain(hm5883lGain_t gain);
        void setMode(hm5883lOperatingMode_t mode, bool highSpeed);
        void setDataRate(hmc5883lDataRate_t rate);
        void setSamples(hmc5883lSamplesAveraged_t samples);

        //
        void selfTest();

    private:
        void setMeasurmentMode(hmc5883lMeasurementMode_t mode);
        void writeDefaultConfig();
        void writeConfigA();
        void writeDevice(hm5883lRegisters_t reg, int data);
        uint8_t readDeivce(hm5883lRegisters_t reg);
    private:
        uint8_t m_sCL_PIN;
        uint8_t m_sDA_PIN;
        hm5883lConfigA m_configA;
        bool m_initilized;
    };
};