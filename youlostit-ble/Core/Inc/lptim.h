/*
 * lptim.h
 *
 *  Created on: Mar 13, 2025
 *      Author: alexis
 */

#ifndef INC_LPTIM_H_
#define INC_LPTIM_H_

/* Include the type definitions for the timer peripheral */
#include <stm32l475xx.h>

void lptim1_init(LPTIM_TypeDef* timer);
void lptim1_reset(LPTIM_TypeDef* timer);
void lptim1_set_ms(LPTIM_TypeDef* timer, uint32_t period_ms);

#endif /* INC_LPTIM_H_ */
