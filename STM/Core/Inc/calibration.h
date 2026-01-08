/**
 ******************************************************************************
 * @file    calibration.h
 * @author  Piotr Bednarek Jan Andrzejewski Mateusz Banaszak
 * @date    Jan 8, 2026
 * @brief   Nagłówek biblioteki kalibracji.
 ******************************************************************************
 */

#ifndef INC_CALIBRATION_H_
#define INC_CALIBRATION_H_

#include "stm32f7xx_hal.h"


typedef struct {
	float raw_val;
	float actual_pos;
} CalPoint_t;



/**
 * @brief Inicjalizuje tablicę kalibracyjną domyślnymi wartościami.
 */
void Calibration_Init(void);

/**
 * @brief Aktualizuje konkretny punkt kalibracyjny w tablicy.
 * @param index Indeks punktu (0-4).
 * @param raw_val Surowa wartość z czujnika.
 * @param actual_pos Rzeczywista pozycja fizyczna (mm).
 * @return 1 jeśli aktualizacja udana, 0 jeśli indeks poza zakresem.
 */
uint8_t Calibration_UpdatePoint(uint8_t index, float raw_val, float actual_pos);

/**
 * @brief Sprawdza czy kalibracja jest kompletna (wszystkie punkty odebrane).
 * @return 1 jeśli gotowa, 0 w przeciwnym razie.
 */
uint8_t Calibration_IsReady(void);

/**
 * @brief Zwraca maskę bitową odebranych punktów kalibracyjnych.
 * @return Maska bitową (bity 0-4).
 */
uint8_t Calibration_GetReceivedPointsMask(void);

/**
 * @brief Pobiera wskaźnik do konkretnego punktu (do debugowania).
 * @param index Indeks punktu.
 * @return Wskaźnik do CalPoint_t lub NULL jeśli błąd.
 */
CalPoint_t* Calibration_GetPoint(uint8_t index);

/**
 * @brief Interpoluje surowy odczyt na rzeczywistą odległość przy użyciu tabeli kalibracyjnej.
 * @param raw_input Surowy odczyt z czujnika.
 * @return Interpolowana (rzeczywista) odległość w mm.
 */
float Calibration_Interpolate(float raw_input);

#endif /* INC_CALIBRATION_H_ */
