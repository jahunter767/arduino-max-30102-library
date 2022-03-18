#ifndef MAX_30102_h
#define MAX_30102_h

/*
#ifdef __AVR__
#include <avr/pgmspace.h>
#endif
#if (ARDUINO >= 100)
#include "Arduino.h"
#endif
*/

#include "lib/i2c/I2C.h"

#define MAX_30102_FIFO_ALMOST_FULL_INT        0x80
#define MAX_30102_NEW_FIFO_DATA_RDY_INT       0x40
#define MAX_30102_ALC_OVF_INT                 0x20
#define MAX_30102_DIE_TEMP_RDY_INT            0x02
#define MAX_30102_PWR_RDY_INT                 0x01

#define MAX_30102_FIFO_CFG_SAMPLE_AVG_1       0x00
#define MAX_30102_FIFO_CFG_SAMPLE_AVG_2       0x20
#define MAX_30102_FIFO_CFG_SAMPLE_AVG_4       0x40
#define MAX_30102_FIFO_CFG_SAMPLE_AVG_8       0x60
#define MAX_30102_FIFO_CFG_SAMPLE_AVG_16      0x80
#define MAX_30102_FIFO_CFG_SAMPLE_AVG_32      0xA0

#define MAX_30102_ADC_SCALE_2048_NA           0x00
#define MAX_30102_ADC_SCALE_4096_NA           0x20
#define MAX_30102_ADC_SCALE_8192_NA           0x40
#define MAX_30102_ADC_SCALE_16384_NA          0x60

#define MAX_30102_SAMPLE_RATE_50              0x00
#define MAX_30102_SAMPLE_RATE_100             0x04
#define MAX_30102_SAMPLE_RATE_200             0x08
#define MAX_30102_SAMPLE_RATE_400             0x0C
#define MAX_30102_SAMPLE_RATE_800             0x10
#define MAX_30102_SAMPLE_RATE_1000            0x14
#define MAX_30102_SAMPLE_RATE_1600            0x18
#define MAX_30102_SAMPLE_RATE_3200            0x1C

#define MAX_30102_PULSE_WIDTH_69_US           0x00      //ADC Resolution 15
#define MAX_30102_PULSE_WIDTH_118_US          0x01      //ADC Resolution 16
#define MAX_30102_PULSE_WIDTH_215_US          0x02      //ADC Resolution 17
#define MAX_30102_PULSE_WIDTH_411_US          0x03      //ADC Resolution 18

#define MAX_30102_MULTI_LED_SLOT_1_3_EN_RED   0x01
#define MAX_30102_MULTI_LED_SLOT_1_3_EN_IR    0x02
#define MAX_30102_MULTI_LED_SLOT_2_4_EN_RED   0x10
#define MAX_30102_MULTI_LED_SLOT_2_4_EN_IR    0x20


/*
#define MAX_30102_DEFAULT_ADDRESS             0x57
#define MAX_30102_FIFO_SIZE                   32
#define MAX_30102_FIFO_SIZE_IN_BYTES          192
#define MAX_30102_READING_BYTE_LENGTH         3

#define MAX_30102_FIFO_WRITE_PTR_REG          0x04
#define MAX_30102_FIFO_WRITE_PTR_MASK         0xE0

#define MAX_30102_FIFO_OVF_COUNTER_REG        0x05
#define MAX_30102_FIFO_OVF_COUNTER_MASK       0xE0

#define MAX_30102_FIFO_READ_PTR_REG           0x06
#define MAX_30102_FIFO_READ_PTR_MASK          0xE0

#define MAX_30102_FIFO_DATA_REG               0x07
#define MAX_30102_FIFO_MSB_MASK               0xFC      //MSB - Most Significant Bits

#define MAX_30102_FIFO_CFG_REG                0x08
#define MAX_30102_FIFO_CFG_ROLL_EN_MASK       0xEF

#define MAX_30102_FIFO_CFG_SMP_AVE_MASK       0x1F

#define MAX_30102_MODE_CFG_REG                0x09
#define MAX_30102_MODE_CFG_SHUTDOWN_MASK      0x7F
#define MAX_30102_MODE_CFG_RESET_MASK         0xBF
#define MAX_30102_MODE_CFG_LED_CTRL_MASK      0xF8
#define MAX_30102_MODE_CFG_HEART_RATE_MODE    0x02
#define MAX_30102_MODE_CFG_SPO2_MODE          0x03
#define MAX_30102_MODE_CFG_MULTI_LED_MODE     0x07

#define MAX_30102_SPO2_CFG_REG                0x0A
#define MAX_30102_SPO2_CFG_ADC_RANGE_MASK     0x9F
#define MAX_30102_SPO2_CFG_SAMPLE_RATE_MASK   0xE3
#define MAX_30102_SPO2_CFG_PULSE_WIDTH_MASK   0xFC

#define MAX_30102_LED_PULSE_AMP_REG_1         0x0C
#define MAX_30102_LED_PULSE_AMP_REG_2         0x0D

#define MAX_30102_MULTI_LED_CFG_REG_1         0x11
#define MAX_30102_MULTI_LED_SLOT_1_MASK       0xF8
#define MAX_30102_MULTI_LED_SLOT_2_MASK       0x8F

#define MAX_30102_MULTI_LED_CFG_REG_2         0x12
#define MAX_30102_MULTI_LED_SLOT_3_MASK       0xF8
#define MAX_30102_MULTI_LED_SLOT_4_MASK       0x8F

#define MAX_30102_TEMP_INT_REG                0x1F
#define MAX_30102_TEMP_FRACTION_REG           0x20
#define MAX_30102_TEMP_FRACTION_MASK          0xF0

#define MAX_30102_TEMP_CFG_REG                0x21
#define MAX_30102_TEMP_ENABLE_MASK            0xFE
*/

struct max30102Readings{
    uint32_t red[32];
    uint32_t ir[32];
    int8_t currRedSample = -1, currIrSample = -1;
    float temp;
};

class MAX_30102 {
    private:
        //-----------------------------Constants-----------------------------//

        static const uint8_t MAX_30102_DEFAULT_ADDRESS            = 0x57;
        static const uint8_t MAX_30102_FIFO_SIZE                  = 32;
        static const uint8_t MAX_30102_FIFO_SIZE_IN_BYTES         = 192;
        static const uint8_t MAX_30102_READING_BYTE_LENGTH        = 3;

        static const uint8_t MAX_30102_INTERRUPT_STATUS_1_REG     = 0x00;
        static const uint8_t MAX_30102_INTERRUPT_STATUS_2_REG     = 0x01;
        static const uint8_t MAX_30102_INTERRUPT_ENABLE_1_REG     = 0x02;
        static const uint8_t MAX_30102_INTERRUPT_ENABLE_2_REG     = 0x03;

        static const uint8_t MAX_30102_FIFO_WRITE_PTR_REG         = 0x04;
        static const uint8_t MAX_30102_FIFO_WRITE_PTR_MASK        = 0xE0;

        static const uint8_t MAX_30102_FIFO_OVF_COUNTER_REG       = 0x05;
        static const uint8_t MAX_30102_FIFO_OVF_COUNTER_MASK      = 0xE0;

        static const uint8_t MAX_30102_FIFO_READ_PTR_REG          = 0x06;
        static const uint8_t MAX_30102_FIFO_READ_PTR_MASK         = 0xE0;

        static const uint8_t MAX_30102_FIFO_DATA_REG              = 0x07;
        static const uint8_t MAX_30102_FIFO_MSB_MASK              = 0xFC;      //MSB - Most Significant Bits

        static const uint8_t MAX_30102_FIFO_CFG_REG               = 0x08;
        static const uint8_t MAX_30102_FIFO_CFG_SMP_AVE_MASK      = 0x1F;
        static const uint8_t MAX_30102_FIFO_CFG_ROLL_EN_MASK      = 0xEF;
        static const uint8_t MAX_30102_FIFO_ALMOST_FULL_MASK      = 0xF0;

        static const uint8_t MAX_30102_MODE_CFG_REG               = 0x09;
        static const uint8_t MAX_30102_MODE_CFG_SHUTDOWN_MASK     = 0x7F;
        static const uint8_t MAX_30102_MODE_CFG_RESET_MASK        = 0xBF;
        static const uint8_t MAX_30102_MODE_CFG_LED_CTRL_MASK     = 0xF8;
        static const uint8_t MAX_30102_MODE_CFG_HEART_RATE_MODE   = 0x02;
        static const uint8_t MAX_30102_MODE_CFG_SPO2_MODE         = 0x03;
        static const uint8_t MAX_30102_MODE_CFG_MULTI_LED_MODE    = 0x07;

        static const uint8_t MAX_30102_SPO2_CFG_REG               = 0x0A;
        static const uint8_t MAX_30102_SPO2_CFG_ADC_RANGE_MASK    = 0x9F;
        static const uint8_t MAX_30102_SPO2_CFG_SAMPLE_RATE_MASK  = 0xE3;
        static const uint8_t MAX_30102_SPO2_CFG_PULSE_WIDTH_MASK  = 0xFC;

        static const uint8_t MAX_30102_LED_PULSE_AMP_REG_1        = 0x0C;
        static const uint8_t MAX_30102_LED_PULSE_AMP_REG_2        = 0x0D;

        static const uint8_t MAX_30102_MULTI_LED_CFG_REG_1        = 0x11;
        static const uint8_t MAX_30102_MULTI_LED_SLOT_1_MASK      = 0xF8;
        static const uint8_t MAX_30102_MULTI_LED_SLOT_2_MASK      = 0x8F;

        static const uint8_t MAX_30102_MULTI_LED_CFG_REG_2        = 0x12;
        static const uint8_t MAX_30102_MULTI_LED_SLOT_3_MASK      = 0xF8;
        static const uint8_t MAX_30102_MULTI_LED_SLOT_4_MASK      = 0x8F;

        static const uint8_t MAX_30102_TEMP_INT_REG               = 0x1F;
        static const uint8_t MAX_30102_TEMP_FRACTION_REG          = 0x20;
        static const uint8_t MAX_30102_TEMP_FRACTION_MASK         = 0xF0;

        static const uint8_t MAX_30102_TEMP_CFG_REG               = 0x21;
        static const uint8_t MAX_30102_TEMP_ENABLE_MASK           = 0xFE;

        //-----------------------------Atributes-----------------------------//

        I2C* bus;
        uint8_t _address, currentReadPtr, numActiveSlots;
        uint8_t slots[4], IRFilterDivisions[4] = {2, 16, 16, 32};
        unsigned long lastPulse;
        long avgIRArray[4];
        float avgBPM;

        //-----------------------------Methods-----------------------------//

        // FIFO Configuration
        void clearFIFO();
        uint8_t getReadPtr();
        uint8_t getWritePtr();
        uint8_t FIFOAvailable();

        // Reading Operations
        uint32_t getNextReading(uint8_t* byteArray, uint8_t index);
        //uint32_t getPrevSample(uint8_t* byteArray, uint8_t index);
        void storeRedReading(max30102Readings* readings, uint32_t value);
        void storeIrReading(max30102Readings* readings, uint32_t value);
        void readSlot(max30102Readings* readings, uint8_t* byteArray, uint8_t index, uint8_t slot);
        void readSample(max30102Readings* readings, uint8_t* byteArray, uint8_t index);

    public:
        MAX_30102();

        // Pseudo Constructors
        void new_MAX_30102();
        void new_MAX_30102(uint8_t address);

        // Power Management
        void shutdownMAX_30102();
        void reset();
        void wakeUp();

        // Interrupt Management
        void enableInterrupt(uint8_t interrupt);
        uint8_t getInterruptFlags();

        // FIFO Configuration cont'd
        void setAveraging(uint8_t mode);
        void enableFIFORollover();
        void setFIFOAlmostFullThreshold(uint8_t spaceRemaining);

        // Mode Configuration
        void enableHeartRateMode();
        void enableSpO2Mode();
        void enableMultiLEDMode(uint8_t s1, uint8_t s2, uint8_t s3, uint8_t s4);

        // Additional Configuration
        void setADCRange(uint8_t range);
        void setSampleRate(uint8_t sampleRate);
        void setPulseWidth(uint8_t pulseWidth);
        void configureLEDAmplitude(uint8_t redAmplitude, uint8_t irAmplitude);

        void readLEDs(max30102Readings* readings);
        void readTemp(max30102Readings* temperature);

        float computeBPM(max30102Readings* samples);
};
#endif /* MAX_30102_h */
