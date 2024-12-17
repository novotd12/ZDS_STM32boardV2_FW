#ifndef INC_FIR_H_
#define INC_FIR_H_

#include "main.h"

void setNtaps(int16_t taps);
void setCoefs(int16_t n, int32_t val);
int32_t updateFIR(int32_t val);

#endif
