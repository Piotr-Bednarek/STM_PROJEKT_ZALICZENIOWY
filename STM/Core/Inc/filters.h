/**
 ******************************************************************************
 * @file    filters.h
 * @author  Piotr Bednarek Jan Andrzejewski Mateusz Banaszak
 * @date    Jan 8, 2026
 * @brief   Nagłówek biblioteki filtrów cyfrowych.
 ******************************************************************************
 */

#ifndef INC_FILTERS_H_
#define INC_FILTERS_H_

#include "stm32f7xx_hal.h"

#define MEDIAN_WINDOW_SIZE 3  ///< Minimalne opóźnienie - tylko 3 próbki do eliminacji "szpilek"

typedef struct {
	float buffer[MEDIAN_WINDOW_SIZE];
	uint8_t index;
	uint8_t count;
} MedianFilter_t;

void MedianFilter_Init(MedianFilter_t *filter);
float MedianFilter_Apply(MedianFilter_t *filter, float v);

typedef struct {
	float alpha;       ///< Współczynnik wygładzania (0-1, mniejszy = większe wygładzanie)
	float filtered_value;
	uint8_t initialized;
} EMA_Filter_t;

void EMA_Init(EMA_Filter_t *ema, float alpha);
float EMA_Update(EMA_Filter_t *ema, float new_value);


typedef struct {
	float min_alpha;
	float max_alpha;
	float threshold;
	float value;
	uint8_t first_run;
} AdaptiveEMA_t;

void AdaptiveEMA_Init(AdaptiveEMA_t *ema, float min_alpha, float max_alpha, float threshold);
float AdaptiveEMA_Filter(AdaptiveEMA_t *ema, float measurement);

typedef struct {
    float min_cutoff; ///< Minimalna częstotliwość odcięcia (Hz)
    float beta;       ///< Współczynnik prędkości (responsywności)
    float d_cutoff;   ///< Częstotliwość odcięcia dla pochodnej (Hz)
    float x_prev;     ///< Poprzednia wartość
    float dx_prev;    ///< Poprzednia pochodna
    uint32_t t_prev;  ///< Poprzedni znacznik czasu (ms)
    int first_run;    ///< Flaga pierwszego uruchomienia
} OneEuroFilter_t;

void OneEuro_Init(OneEuroFilter_t *f, float min_cutoff, float beta, float d_cutoff);
float OneEuro_Update(OneEuroFilter_t *f, float x, uint32_t t_now);

#endif /* INC_FILTERS_H_ */
