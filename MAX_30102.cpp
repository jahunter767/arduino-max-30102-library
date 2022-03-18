#include "MAX_30102.h"
#include "lib/EMA.h"

//************************Constructors************************//
MAX_30102::MAX_30102(){}

// Pseudo Constructors
void MAX_30102::new_MAX_30102(){
    this->_address = MAX_30102_DEFAULT_ADDRESS;
    this->bus = new I2C();
    this->bus->new_I2C(this->_address);
    this->currentReadPtr = 0;
    MAX_30102::enableHeartRateMode();
} // End Constructor

void MAX_30102::new_MAX_30102(uint8_t address){
    this->_address = address;
    this->bus = new I2C();
    this->bus->new_I2C(this->_address);
    this->currentReadPtr = 0;
    MAX_30102::enableHeartRateMode();
} // End Constructor

//************************Private************************//

void MAX_30102::clearFIFO(){
    bus->maskedWriteByte(MAX_30102_FIFO_WRITE_PTR_REG, MAX_30102_FIFO_WRITE_PTR_MASK, 0);
    bus->maskedWriteByte(MAX_30102_FIFO_OVF_COUNTER_REG, MAX_30102_FIFO_OVF_COUNTER_MASK, 0);
    bus->maskedWriteByte(MAX_30102_FIFO_READ_PTR_REG, MAX_30102_FIFO_READ_PTR_MASK, 0);
} // End-clearFIFO

uint8_t MAX_30102::getReadPtr(){
    return (bus->readByte(MAX_30102_FIFO_READ_PTR_REG) & ~MAX_30102_FIFO_READ_PTR_MASK);
} // End-getReadPtr

uint8_t MAX_30102::getWritePtr(){
    return (bus->readByte(MAX_30102_FIFO_WRITE_PTR_REG) & ~MAX_30102_FIFO_WRITE_PTR_MASK);
} // End-getWritePtr

// Returns how much unread samples are stored in the FIFO buffer
uint8_t MAX_30102::FIFOAvailable(){
    int8_t temp;
    this->currentReadPtr = MAX_30102::getReadPtr();
    temp = MAX_30102::getWritePtr() - this->currentReadPtr;
    if (temp < 0) temp += MAX_30102_FIFO_SIZE;
    return temp;
} // End-FIFOAvailable

//-----------Reading Operations-----------//

uint32_t MAX_30102::getNextReading(uint8_t* byteArray, uint8_t index){
    uint32_t temp1, temp2;
    
    temp1 = byteArray[index] & ~MAX_30102_FIFO_MSB_MASK;
    temp1 = temp1 << 16;
    temp2 += temp1;
        
    temp1 = byteArray[index+1];
    temp1 = temp1 << 8;
    temp2 += temp1;

    temp2 += byteArray[index+2];
    return temp2;
} // End-getNextSample

/*
uint32_t MAX_30102::getPrevSample(uint8_t* byteArray, uint8_t index){
    uint32_t temp1, temp2;
    
    temp2 += byteArray[index];
    
    temp1 = byteArray[index-1];
    temp1 = temp1 << 8;
    temp2 += temp1;

    temp1 = byteArray[index-2] & ~MAX_30102_FIFO_MSB_MASK;
    temp1 = temp1 << 16;
    temp2 += temp1;
        
    return temp2;
} // End-getPrevSample
*/

void MAX_30102::storeRedReading(max30102Readings* readings, uint32_t value){
    readings->currRedSample += 1;
    readings->currRedSample %= MAX_30102_FIFO_SIZE;
    readings->red[readings->currRedSample] = value;
} // End-storeRedReading

void MAX_30102::storeIrReading(max30102Readings* readings, uint32_t value){
    readings->currIrSample += 1;
    readings->currIrSample %= MAX_30102_FIFO_SIZE;
    readings->ir[readings->currIrSample] = value;
} // End-storeIrReading

void MAX_30102::readSlot(max30102Readings* readings, uint8_t* byteArray, uint8_t index, uint8_t slot){
    uint32_t sample = MAX_30102::getNextReading(byteArray, index);

    if ((slot == (uint8_t) MAX_30102_MULTI_LED_SLOT_1_3_EN_RED) || (slot == (uint8_t) MAX_30102_MULTI_LED_SLOT_2_4_EN_RED)){
        MAX_30102::storeRedReading(readings, sample);
    }else if ((slot == (uint8_t) MAX_30102_MULTI_LED_SLOT_1_3_EN_IR) || (slot == (uint8_t) MAX_30102_MULTI_LED_SLOT_2_4_EN_IR)){
        MAX_30102::storeIrReading(readings, sample);
    } // End-If
} // End-readSlot

void MAX_30102::readSample(max30102Readings* readings, uint8_t* byteArray, uint8_t index){
    uint8_t slot, next = index;
    for (slot = 0; slot < this->numActiveSlots; slot++){
        MAX_30102::readSlot(readings, byteArray, next, this->slots[slot]);
        next += MAX_30102_READING_BYTE_LENGTH;
    } // End-For
} // End-readSample

//************************Public************************//

//-----------Power Management-----------//

void MAX_30102::shutdownMAX_30102(){
    bus->maskedWriteByte(MAX_30102_MODE_CFG_REG, MAX_30102_MODE_CFG_SHUTDOWN_MASK, ~MAX_30102_MODE_CFG_SHUTDOWN_MASK);
} // End-shutdown

// Reset all configuration, threshold, and data registers to POR values
void MAX_30102::reset(){
    bus->maskedWriteByte(MAX_30102_MODE_CFG_REG, MAX_30102_MODE_CFG_RESET_MASK, ~MAX_30102_MODE_CFG_RESET_MASK);
    delay(250);
} // End-resetMAX

void MAX_30102::wakeUp(){
    bus->maskedWriteByte(MAX_30102_MODE_CFG_REG, MAX_30102_MODE_CFG_SHUTDOWN_MASK, 0);
} // End-wakeUp

//-----------Interrupt Management-----------//
void MAX_30102::enableInterrupt(uint8_t interrupt){
    if (interrupt == MAX_30102_DIE_TEMP_RDY_INT){
        bus->maskedWriteByte(MAX_30102_INTERRUPT_ENABLE_2_REG, ~interrupt, interrupt);
    }else{
        bus->maskedWriteByte(MAX_30102_INTERRUPT_ENABLE_1_REG, ~interrupt, interrupt);
    } // End-If
} // End-enableInterrupt

uint8_t MAX_30102::getInterruptFlags(){
    uint8_t intStatus1 = bus->readByte(MAX_30102_INTERRUPT_STATUS_1_REG);
    uint8_t intStatus2 = bus->readByte(MAX_30102_INTERRUPT_STATUS_2_REG);
    uint8_t flags = (intStatus1 | intStatus2) & 0xE3;
    return flags;
} // End-getInterruptFlags

//-----------FIFO Configuration-----------//

void MAX_30102::setAveraging(uint8_t mode){
    bus->maskedWriteByte(MAX_30102_FIFO_CFG_REG, MAX_30102_FIFO_CFG_SMP_AVE_MASK, mode);
} // End-setAveraging

void MAX_30102::enableFIFORollover(){
    bus->maskedWriteByte(MAX_30102_FIFO_CFG_REG, MAX_30102_FIFO_CFG_ROLL_EN_MASK, ~MAX_30102_FIFO_CFG_ROLL_EN_MASK);
} // End-enableFIFORollover

void MAX_30102::setFIFOAlmostFullThreshold(uint8_t spaceRemaining){
    bus->maskedWriteByte(MAX_30102_FIFO_CFG_REG, MAX_30102_FIFO_ALMOST_FULL_MASK, spaceRemaining);
} // End-setAlmostFullThreshold

//-----------Mode Configuration-----------//

void MAX_30102::enableHeartRateMode(){
    bus->maskedWriteByte(MAX_30102_MODE_CFG_REG, MAX_30102_MODE_CFG_LED_CTRL_MASK, MAX_30102_MODE_CFG_HEART_RATE_MODE);
    this->numActiveSlots = 1;
    this->slots[0] = (uint8_t) MAX_30102_MULTI_LED_SLOT_1_3_EN_RED;
    this->slots[1] = 0;
    this->slots[2] = 0;
    this->slots[3] = 0;
} // end-enableHeartRateMode

void MAX_30102::enableSpO2Mode(){
    bus->maskedWriteByte(MAX_30102_MODE_CFG_REG, MAX_30102_MODE_CFG_LED_CTRL_MASK, MAX_30102_MODE_CFG_SPO2_MODE);
    this->numActiveSlots = 2;
    this->slots[0] = (uint8_t) MAX_30102_MULTI_LED_SLOT_1_3_EN_RED;
    this->slots[1] = (uint8_t) MAX_30102_MULTI_LED_SLOT_2_4_EN_IR;
    this->slots[2] = 0;
    this->slots[3] = 0;
} // End-setSpO2Mode

void MAX_30102::enableMultiLEDMode(uint8_t s1, uint8_t s2, uint8_t s3, uint8_t s4){
    this->numActiveSlots = 0;
    bus->maskedWriteByte(MAX_30102_MODE_CFG_REG, MAX_30102_MODE_CFG_LED_CTRL_MASK, MAX_30102_MODE_CFG_MULTI_LED_MODE);
    this->slots[0] = s1;
    bus->maskedWriteByte(MAX_30102_MULTI_LED_CFG_REG_1, MAX_30102_MULTI_LED_SLOT_1_MASK, s1);
    this->slots[1] = s2;
    bus->maskedWriteByte(MAX_30102_MULTI_LED_CFG_REG_1, MAX_30102_MULTI_LED_SLOT_2_MASK, s2);
    this->slots[2] = s3;
    bus->maskedWriteByte(MAX_30102_MULTI_LED_CFG_REG_2, MAX_30102_MULTI_LED_SLOT_1_MASK, s3);
    this->slots[3] = s4;
    bus->maskedWriteByte(MAX_30102_MULTI_LED_CFG_REG_2, MAX_30102_MULTI_LED_SLOT_1_MASK, s4);

    if (s1 != 0) {this->numActiveSlots += 1;} // End-If
    if (s2 != 0) {this->numActiveSlots += 1;} // End-If
    if (s3 != 0) {this->numActiveSlots += 1;} // End-If
    if (s4 != 0) {this->numActiveSlots += 1;} // End-If
} // End-enableMultiLEDMode

//-----------Additional Configuration-----------//

void MAX_30102::setADCRange(uint8_t range){
    bus->maskedWriteByte(MAX_30102_SPO2_CFG_REG, MAX_30102_SPO2_CFG_ADC_RANGE_MASK, range);
} // End-setADCRange

void MAX_30102::setSampleRate(uint8_t sampleRate){
    bus->maskedWriteByte(MAX_30102_SPO2_CFG_REG, MAX_30102_SPO2_CFG_SAMPLE_RATE_MASK, sampleRate);
} // End-setSampleRate

void MAX_30102::setPulseWidth(uint8_t pulseWidth){
    bus->maskedWriteByte(MAX_30102_SPO2_CFG_REG, MAX_30102_SPO2_CFG_PULSE_WIDTH_MASK, pulseWidth);
} // End-setPulseWidth

void MAX_30102::configureLEDAmplitude(uint8_t redAmplitude, uint8_t irAmplitude){
    bus->writeByte(MAX_30102_LED_PULSE_AMP_REG_1, redAmplitude);
    bus->writeByte(MAX_30102_LED_PULSE_AMP_REG_2, irAmplitude);
} // End-configureLEDAmplitude

//----------------------------------------------//

// Updates the structure with the average and instantaneous IR and Red readings
void MAX_30102::readLEDs(max30102Readings* readings){
    uint8_t numSamples, stepSize, numBytes, count;
    
    // Compute the number of bytes to be read
    numSamples = MAX_30102::FIFOAvailable();
    stepSize = this->numActiveSlots * MAX_30102_READING_BYTE_LENGTH;
    numBytes = numSamples * stepSize;

    // Ensuring that numbytes does not exceed the size of the FIFO buffer
    // as well as preventing reading incomplete samples
    if (numBytes > (uint8_t) MAX_30102_FIFO_SIZE_IN_BYTES) {numBytes = MAX_30102_FIFO_SIZE_IN_BYTES;} // End-If
    numBytes = numBytes - (numBytes % stepSize);

    // Clearing current readings
    readings->currRedSample = -1;
    readings->currIrSample = -1;
    
    // Reads bytes from FIFO buffer
    uint8_t rawReadings[numBytes];
    bus->readByteStream(MAX_30102_FIFO_DATA_REG, numBytes, rawReadings);
    for (count = 0; count < numBytes; count+=stepSize){
        MAX_30102::readSample(readings, rawReadings, count);
    } // End-For

    //Update readPtr
    this->currentReadPtr += numSamples;
    bus->writeByte(MAX_30102_FIFO_READ_PTR_REG, currentReadPtr);
} // End-readRed

// Updates the structure with the temperature in celcius
void MAX_30102::readTemp(max30102Readings* temperature){
    bus->maskedWriteByte(MAX_30102_TEMP_CFG_REG, MAX_30102_TEMP_ENABLE_MASK, ~MAX_30102_TEMP_ENABLE_MASK);
    temperature->temp = bus->readByte(MAX_30102_TEMP_INT_REG) + 0.0625*(bus->readByte(MAX_30102_TEMP_FRACTION_REG) & ~MAX_30102_TEMP_FRACTION_MASK);
} // End-readTemp

float MAX_30102::computeBPM(max30102Readings* samples){
    float bpm, bpmMeanSquareDev;
    long meanSquareIR, rmsIR, diff;
    uint8_t count;
    if (samples->ir[samples->currIrSample] > 25000){
        for (count = 0; count < (samples->currIrSample + 1); count++){
            cascadedEMA(this->avgIRArray, samples->ir[count], this->IRFilterDivisions, 4);
            diff = samples->ir[samples->currIrSample] - this->avgIRArray[3];
            diff *= diff;
            meanSquareIR = integerEMA(meanSquareIR, diff, 16);
            rmsIR = sqrt(meanSquareIR);

            if (!this->beat && (this->avgIRArray[0] <= (this->avgIRArray[3] - rmsIR))){
                bpm = 60000/(millis() - this->lastPulse); // 60000 ms = 1 minute

                // Checks for valid BPM, ie 25 < bpm < 225
                if (bpm <= 25){
                    // Heart rate likely too slow to be valid.
                    // Absorption of the IR likely changed from patient moving
                    // resulting in a shift in the average value.
                    // Due to the use of an exponential moving average to filter
                    // high frequency noise and get an average DC value the
                    // thresholds used to detect a pulse are more resistant to change

                    this->lastPulse = millis(); // Change lastPulse to ignore undetected pulses
                    return -1;
                }else if(bpm > 225){
                    // Heart rate likely too fast to be valid. Possibly detected
                    // spike from external noise such as an increase in the ambient
                    // light or the patient moving. Hence wait for actual pulse.
                    return -1;
                }else{
                    // Valid pulse most likely detected
                    diff = bpm - this->avgBPM;
                    diff *= diff;
                    bpmMeanSquareDev = floatingPtEMA(bpmMeanSquareDev, diff, 6);
                    bpmStandardDev = sqrt(bpmMeanSquareDev);

                    // Filter extra spikes that are within a the valid range
                    if ((bpm < (this->avgBPM + bpmStandardDev)) && (bpm > (this->avgBPM - bpmStandardDev))){
                        this->lastPulse = millis();
                        this->beat = true;
                        this->avgBPM = floatingPtEMA(this->avgBPM, bpm, 6);
                    } // End-If
                    return bpm;
                } // End-If
            }else if (this->avgIRArray[0] > (this->avgIRArray[3] - rmsIR)){
                this->beat = false;
                return -1;
            } // End-If
        } // End-For
    }else{
        // Clears the bpm if no finger detected for 3 seconds
        if ((millis() - this->lastPulse) > 3000){this->avgBPM = -1;} // End-If
        return -1;
    } // End-If
} // End-computeBPM
