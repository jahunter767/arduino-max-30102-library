#ifndef EMA_h
#define EMA_h

long integerEMA(long currAvg, long nextSample, uint8_t divisions);
float floatingPtEMA(float currAvg, float nextSample, uint8_t divisions);
void cascadedEMA(long* currAvg, long nextSample, uint8_t* divisions, uint8_t filterCount);

#endif /* EMA_h */
