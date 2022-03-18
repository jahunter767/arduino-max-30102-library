#include "EMA.h"

long integerEMA(long currAvg, long nextSample, uint8_t divisions){
    return currAvg + (nextSample - currAvg)/divisions;
} // End-exponetialMovingAverage

float floatingPtEMA(float currAvg, float nextSample, uint8_t divisions){
    return currAvg + (nextSample - currAvg)/divisions;
} // End-exponetialMovingAverage

void cascadedEMA(long* currAvg, long nextSample, uint8_t* divisions, uint8_t filterCount){
    int count, numDivs = sizeof(divisions)/sizeof(divisions[0]);
    int numAvgs = sizeof(currAvg)/sizeof(long);
    long temp;

    // Computes a series of moving averages and updates the array
    temp = integerEMA(currAvg[0], nextSample, divisions[0]);
    currAvg[0] = temp;
    for (count = 1; count < filterCount; count++){
        temp = integerEMA(currAvg[count], currAvg[count-1], divisions[count%numDivs]);
        currAvg[count] = temp;
    } // End-For
} // End-cascadedEMA
