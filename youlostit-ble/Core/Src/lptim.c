/*
 * lptim1.c
 *
 *  Created on: Mar 13, 2025
 *      Author: alexis
 */

#include "lptim.h"

void lptim1_init(LPTIM_TypeDef* timer)
{
    // Enable clock for LPTIM1
    RCC->APB1ENR1 |= RCC_APB1ENR1_LPTIM1EN;

    RCC->CSR |= RCC_CSR_LSION;

    // Wait for LSI to be ready
    while ((RCC->CSR & RCC_CSR_LSIRDY) == 0) {}

    // Select clock source (use LSI for low power)
    RCC->CCIPR &= ~RCC_CCIPR_LPTIM1SEL;
    RCC->CCIPR |= (RCC_CCIPR_LPTIM1SEL_0); // Select LSI (32 kHz)

    // Reset LPTIM1
    RCC->APB1RSTR1 |= RCC_APB1RSTR1_LPTIM1RST;
    RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_LPTIM1RST;

    // Configure LPTIM1 for low-power counting
    LPTIM1->CR = 0;
    LPTIM1-> CFGR = 0;

    // Set prescaler (divide by 32 for ~1ms ticks from 32kHz LSI)
    LPTIM1->CFGR |= LPTIM_CFGR_PRESC_2;

    // Enable LPTIM1
    LPTIM1->CR |= LPTIM_CR_ENABLE;

    // Enable interrupt on ARR match
    LPTIM1->IER |= LPTIM_IER_ARRMIE;
    NVIC_EnableIRQ(LPTIM1_IRQn);
    NVIC_SetPriority(LPTIM1_IRQn, 0);

    LPTIM1->CR |= LPTIM_CR_CNTSTRT;
}

void lptim1_set_ms(LPTIM_TypeDef* timer, uint32_t period_ms)
{
    // Set Auto-Reload value for desired period (assuming 1ms tick from prescaler)
    LPTIM1->ARR = period_ms - 1;

    LPTIM1->CMP = 0;
}
