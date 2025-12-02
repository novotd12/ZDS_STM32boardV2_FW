#include "MA.h"

void MovingAverage_Init(MovingAverage_t *ma, uint16_t size){
    if (size == 0 || size > MOVING_AVG_MAX_SIZE)
        size = MOVING_AVG_MAX_SIZE;

    ma->size = size;
    ma->index = 0;
    ma->count = 0;
    ma->sum = 0.0f;

    for (uint16_t i = 0; i < size; i++) {
        ma->buffer[i] = 0.0f;
    }
}

float MovingAverage_Update(MovingAverage_t *ma, float new_sample){

    // Odečti starou hodnotu z běžícího součtu
    ma->sum -= ma->buffer[ma->index];

    // Nahraď novou hodnotou
    ma->buffer[ma->index] = new_sample;

    // Přičti novou hodnotu
    ma->sum += new_sample;

    // Posuň index (circular)
    ma->index++;
    if (ma->index >= ma->size)
        ma->index = 0;

    // Zvyš počet vzorků do dosažení plné kapacity
    if (ma->count < ma->size)
        ma->count++;

    // Vrať aktuální průměr
    return (ma->sum / ma->count);
}

float MovingAverage_GetValue(MovingAverage_t *ma)
{
    if (!ma || ma->count == 0) return 0.0f;
    return (ma->sum / ma->count);
}

void MovingAverage_SetSize(MovingAverage_t *ma, uint16_t new_size)
{
    if (!ma) return;
    if (new_size == 0 || new_size > MOVING_AVG_MAX_SIZE)
        new_size = MOVING_AVG_MAX_SIZE;

    // Přepočítej součet podle nového okna
    float new_sum = 0.0f;
    uint16_t valid_samples = (ma->count < new_size) ? ma->count : new_size;

    // Projdeme poslední validní vzorky od nejnovějšího zpět
    for (uint16_t i = 0; i < valid_samples; i++) {
        int16_t pos = (ma->index + ma->size - 1 - i);
        if (pos >= ma->size) pos -= ma->size;
        if (pos < 0) pos += ma->size;
        new_sum += ma->buffer[pos];
    }

    // Aktualizace parametrů
    ma->size = new_size;
    ma->count = valid_samples;
    ma->sum = new_sum;
    if (ma->index >= new_size)
        ma->index = 0;

    // Vyčisti zbytek bufferu
    for (uint16_t i = valid_samples; i < new_size; i++) {
        ma->buffer[i] = 0.0f;
    }
}
