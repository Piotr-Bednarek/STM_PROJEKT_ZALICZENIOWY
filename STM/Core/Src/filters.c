/**
 ******************************************************************************
 * @file    filters.c
 * @author  Piotr Bednarek Jan Andrzejewski Mateusz Banaszak
 * @date    Jan 8, 2026
 * @brief   Implementacja filtrów cyfrowych (Mediana, EMA, 1-Euro).
 *
 * Biblioteka zawiera implementację popularnych filtrów wygładzających dane
 * z czujników. Zawiera:
 *  - Filtr Medianowy (usuwanie szumów impulsowych)
 *  - Filtr EMA (Wykładnicza Średnia Krocząca - dolnoprzepustowy)
 *  - Adaptacyjny Filtr EMA (zmienna stała czasowa)
 *  - Filtr 1-Euro (zminimalizowany lag i jitter)
 ******************************************************************************
 */

#include "filters.h"
#include <stdlib.h> 

/**
 * @brief Inicjalizuje filtr medianowy.
 *        Zeruje bufor, licznik próbek i indeks.
 * @param filter Wskaźnik do struktury MedianFilter_t.
 */
void MedianFilter_Init(MedianFilter_t *filter) {
	filter->index = 0;
	filter->count = 0;
	for (int i = 0; i < MEDIAN_WINDOW_SIZE; i++) {
		filter->buffer[i] = 0.0f;
	}
}

/**
 * @brief Aplikuje filtr medianowy do nowej próbki.
 *        Funkcja dodaje próbkę do bufora cyklicznego i zwraca medianę z aktualnych próbek.
 * @param filter Wskaźnik do struktury MedianFilter_t.
 * @param v Nowa wartość próbki (pomiar).
 * @return Przefiltrowana wartość (mediana).
 */
float MedianFilter_Apply(MedianFilter_t *filter, float v) {
	filter->buffer[filter->index] = v;
	filter->index = (filter->index + 1) % MEDIAN_WINDOW_SIZE;
	if (filter->count < MEDIAN_WINDOW_SIZE) {
		filter->count++;
	}

	float sorted[MEDIAN_WINDOW_SIZE];
	for (int i = 0; i < filter->count; i++) {
		sorted[i] = filter->buffer[i];
	}

	for (int i = 0; i < filter->count - 1; i++) {
		for (int j = 0; j < filter->count - i - 1; j++) {
			if (sorted[j] > sorted[j + 1]) {
				float temp = sorted[j];
				sorted[j] = sorted[j + 1];
				sorted[j + 1] = temp;
			}
		}
	}

	if (filter->count % 2 == 0) {
		return (sorted[filter->count / 2 - 1] + sorted[filter->count / 2]) / 2.0f;
	} else {
		return sorted[filter->count / 2];
	}
}

/**
 * @brief Inicjalizuje prosty filtr dolnoprzepustowy (EMA).
 * @param ema Wskaźnik do struktury EMA_Filter_t.
 * @param alpha Współczynnik wygładzania (0.0 - 1.0). Mniejsza wartość = mocniejsze filtrowanie.
 */
void EMA_Init(EMA_Filter_t *ema, float alpha) {
	ema->alpha = alpha;
	ema->filtered_value = 0.0f;
	ema->initialized = 0;
}

/**
 * @brief Aktualizuje filtr EMA nową wartością.
 *        Wzór: Out = alpha * In + (1 - alpha) * PrevOut.
 * @param ema Wskaźnik do struktury EMA_Filter_t.
 * @param new_value Nowa wartość próbki.
 * @return Przefiltrowana wartość.
 */
float EMA_Update(EMA_Filter_t *ema, float new_value) {
	if (!ema->initialized) {
		ema->filtered_value = new_value;
		ema->initialized = 1;
		return new_value;
	}
	ema->filtered_value = ema->alpha * new_value + (1.0f - ema->alpha) * ema->filtered_value;
	return ema->filtered_value;
}

/**
 * @brief Inicjalizuje adaptacyjny filtr EMA.
 *        Pozwala na dynamiczną zmianę współczynnika alpha w zależności od błędu (zmiany sygnału).
 * @param ema Wskaźnik do struktury AdaptiveEMA_t.
 * @param min_alpha Minimalny współczynnik alpha (dla małych zmian - silne filtrowanie).
 * @param max_alpha Maksymalny współczynnik alpha (dla dużych zmian - szybka reakcja).
 * @param threshold Próg błędu, powyżej którego alpha rośnie w stronę max_alpha.
 */
void AdaptiveEMA_Init(AdaptiveEMA_t *ema, float min_alpha, float max_alpha, float threshold) {
	ema->min_alpha = min_alpha;
	ema->max_alpha = max_alpha;
	ema->threshold = threshold;
	ema->value = 0.0f;
	ema->first_run = 1;
}

/**
 * @brief Aplikuje adaptacyjny filtr EMA.
 *        Jeśli zmiana sygnału jest duża, filtr reaguje szybciej (większe alpha).
 *        Jeśli sygnał jest stabilny, filtr wygładza mocniej (mniejsze alpha).
 * @param ema Wskaźnik do struktury AdaptiveEMA_t.
 * @param measurement Nowy pomiar.
 * @return Przefiltrowana wartość.
 */
float AdaptiveEMA_Filter(AdaptiveEMA_t *ema, float measurement) {
	if (ema->first_run) {
		ema->value = measurement;
		ema->first_run = 0;
		return measurement;
	}
	float error = (measurement > ema->value) ? (measurement - ema->value) : (ema->value - measurement);
	float factor = error / ema->threshold;
	if (factor > 1.0f)
		factor = 1.0f;
	float alpha = ema->min_alpha + (ema->max_alpha - ema->min_alpha) * factor;
	ema->value = alpha * measurement + (1.0f - alpha) * ema->value;
	return ema->value;
}

/**
 * @brief Inicjalizuje filtr 1-Euro.
 *        Zaawansowany filtr dolnoprzepustowy minimalizujący opóźnienia (lag) przy szybkim ruchu
 *        oraz drgania (jitter) przy powolnym ruchu.
 * @param f Wskaźnik do struktury OneEuroFilter_t.
 * @param min_cutoff Minimalna częstotliwość odcięcia (Hz) dla stabilnego sygnału.
 * @param beta Współczynnik prędkości - jak mocno zwiększać cutoff przy szybkim ruchu.
 * @param d_cutoff Częstotliwość odcięcia dla obliczania pochodnej (Hz).
 */
void OneEuro_Init(OneEuroFilter_t *f, float min_cutoff, float beta, float d_cutoff) {
    f->min_cutoff = min_cutoff;
    f->beta = beta;
    f->d_cutoff = d_cutoff;
    f->x_prev = 0.0f;
    f->dx_prev = 0.0f;
    f->t_prev = 0;
    f->first_run = 1;
}

/**
 * @brief Aktualizuje filtr 1-Euro.
 * @param f Wskaźnik do struktury OneEuroFilter_t.
 * @param x Nowa wartość wejściowa.
 * @param t_now Aktualny znacznik czasu (w milisekundach), np. HAL_GetTick().
 * @return Przefiltrowana wartość.
 */
float OneEuro_Update(OneEuroFilter_t *f, float x, uint32_t t_now) {
    if (f->first_run) {
        f->x_prev = x;
        f->dx_prev = 0.0f;
        f->t_prev = t_now;
        f->first_run = 0;
        return x;
    }

    float dt = (t_now - f->t_prev) / 1000.0f; 
    if (dt <= 0.0f) return f->x_prev;
    
    float dx = (x - f->x_prev) / dt;

    float rc_d = 1.0f / (2.0f * 3.14159f * f->d_cutoff);
    float alpha_d = 1.0f / (1.0f + rc_d / dt);
    float dx_hat = dx * alpha_d + f->dx_prev * (1.0f - alpha_d);

    float cutoff = f->min_cutoff + f->beta * (dx_hat > 0 ? dx_hat : -dx_hat); 

    float rc = 1.0f / (2.0f * 3.14159f * cutoff);
    float alpha = 1.0f / (1.0f + rc / dt);
    float x_hat = x * alpha + f->x_prev * (1.0f - alpha);

    f->x_prev = x_hat;
    f->dx_prev = dx_hat;
    f->t_prev = t_now;

    return x_hat;
}
