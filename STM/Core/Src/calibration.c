/**
 ******************************************************************************
 * @file    calibration.c
 * @author  Piotr Bednarek Jan Andrzejewski Mateusz Banaszak
 * @date    Jan 8, 2026
 * @brief   Implementacja logiki kalibracji belki.
 *
 * Plik zawiera funkcje do obsługi 5-punktowej kalibracji belki.
 * Pozwala na mapowanie surowych odczytów z czujnika odległości na rzeczywistą
 * pozycję na belce w mm. Używa interpolacji liniowej pomiędzy punktami kalibracyjnymi.
 ******************************************************************************
 */

#include "calibration.h"


static CalPoint_t cal_table[5] = {
	{ 50.0f,   0.0f },
	{ 90.0f,   75.0f },
	{ 175.0f,  150.0f },
	{ 270.0f,  225.0f },
	{ 310.0f,  290.0f }
};

static volatile uint8_t calibration_ready = 0;
static volatile uint8_t cal_points_received = 0;

/**
 * @brief Inicjalizuje moduł kalibracji.
 *        Resetuje flagi stanu kalibracji. Tablica `cal_table` posiada domyślne wartości startowe.
 */
void Calibration_Init(void) {
	calibration_ready = 0;
	cal_points_received = 0;
}

/**
 * @brief Aktualizuje pojedynczy punkt kalibracyjny.
 *        Zapisuje nową parę (surowy odczyt, rzeczywista pozycja) w tablicy kalibracyjnej.
 *        Automatycznie sprawdza, czy odebrano już wszystkie wymagane punkty.
 * @param index Indeks punktu (0-4).
 * @param raw_val Surowa wartość odczytana z czujnika.
 * @param actual_pos Rzeczywista pozycja fizyczna na belce (w mm).
 * @return 1 w przypadku sukcesu, 0 jeśli indeks jest nieprawidłowy.
 */
uint8_t Calibration_UpdatePoint(uint8_t index, float raw_val, float actual_pos) {
	if (index >= 5) return 0;

	cal_table[index].raw_val = raw_val;
	cal_table[index].actual_pos = actual_pos;

	cal_points_received |= (1 << index);

	if (cal_points_received == 0x1F) {
		calibration_ready = 1;
	}

	return 1;
}

/**
 * @brief Sprawdza status kalibracji.
 * @return 1 jeśli kalibracja jest kompletna (wszystkie punkty odebrane), 0 w przeciwnym razie.
 */
uint8_t Calibration_IsReady(void) {
	return calibration_ready;
}

/**
 * @brief Zwraca maskę bitową odebranych punktów.
 *        Bit 0 odpowiada punktowi 0, bit 1 punktowi 1 itd.
 * @return Maska bitowa otrzymanych punktów (0x1F oznacza komplet).
 */
uint8_t Calibration_GetReceivedPointsMask(void) {
	return cal_points_received;
}

/**
 * @brief Pobiera wskaźnik do punktu kalibracyjnego.
 *        Służy głównie do celów diagnostycznych.
 * @param index Indeks punktu (0-4).
 * @return Wskaźnik do struktury CalPoint_t lub NULL w przypadku błędu.
 */
CalPoint_t* Calibration_GetPoint(uint8_t index) {
	if (index >= 5) return NULL;
	return &cal_table[index];
}

/**
 * @brief Interpoluje surowy odczyt na rzeczywistą odległość.
 *        Znajduje odpowiedni przedział kalibracyjny i wykonuje interpolację liniową.
 *        Jeśli wartość wykracza poza zakres kalibracji, jest przycinana do krawędzi (clamping).
 * @param raw_input Surowy odczyt z czujnika.
 * @return Obliczona rzeczywista pozycja w mm.
 */
#include <math.h>

/**
 * @brief Interpoluje surowy odczyt na rzeczywistą odległość.
 *        Znajduje odpowiedni przedział kalibracyjny i wykonuje interpolację liniową.
 *        Jeśli wartość wykracza poza zakres kalibracji, jest przycinana do krawędzi (clamping).
 * @param raw_input Surowy odczyt z czujnika.
 * @return Obliczona rzeczywista pozycja w mm.
 */
float Calibration_Interpolate(float raw_input) {
    int descending = (cal_table[0].raw_val > cal_table[4].raw_val);
    
    if (descending) {
        if (raw_input >= cal_table[0].raw_val) return cal_table[0].actual_pos;
        if (raw_input <= cal_table[4].raw_val) return cal_table[4].actual_pos;
    } else {
        if (raw_input <= cal_table[0].raw_val) return cal_table[0].actual_pos;
        if (raw_input >= cal_table[4].raw_val) return cal_table[4].actual_pos;
    }

	for (int i = 0; i < 4; i++) {
        float r1 = cal_table[i].raw_val;
        float r2 = cal_table[i+1].raw_val;
        
        float min_r = (r1 < r2) ? r1 : r2;
        float max_r = (r1 < r2) ? r2 : r1;
        
		if (raw_input >= min_r && raw_input <= max_r) {
			float range_x = r2 - r1;
			float range_y = cal_table[i+1].actual_pos - cal_table[i].actual_pos;

			if (fabsf(range_x) < 0.001f) return cal_table[i].actual_pos;

			float ratio = (raw_input - r1) / range_x;
			return cal_table[i].actual_pos + (ratio * range_y);
		}
	}
    
    if (fabsf(raw_input - cal_table[0].raw_val) < fabsf(raw_input - cal_table[4].raw_val))
        return cal_table[0].actual_pos;
    else
        return cal_table[4].actual_pos;
}
