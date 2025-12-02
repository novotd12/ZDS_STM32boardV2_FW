#ifndef MOVING_AVERAGE_H
#define MOVING_AVERAGE_H

#include <stdint.h>

#define MOVING_AVG_MAX_SIZE 4096

typedef struct {
    float buffer[MOVING_AVG_MAX_SIZE];  // staticky alokovaný kruhový buffer
    uint16_t size;                      // aktivní počet vzorků
    uint16_t index;                     // pozice pro zápis
    uint16_t count;                     // počet platných vzorků (při rozběhu)
    float sum;                          // běžící součet
} MovingAverage_t;

void MovingAverage_Init(MovingAverage_t *ma, uint16_t size);
float MovingAverage_Update(MovingAverage_t *ma, float new_sample);
float MovingAverage_GetValue(MovingAverage_t *ma);
void MovingAverage_SetSize(MovingAverage_t *ma, uint16_t new_size);

#endif // MOVING_AVERAGE_H
