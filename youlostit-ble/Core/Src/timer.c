/*
 * timer.c
 *
 *  Created on: Oct 5, 2023
 *      Author: schulman
 */

#include "timer.h"


void timer_init(TIM_TypeDef* timer)
{
		// TODO implement this
		RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;

		// Step 1: Stop the timer and reset it to clear any previous states
	    timer->CR1 &= ~TIM_CR1_CEN;  // Clear the CEN bit to stop the timer
	    timer->CNT = 0;              // Reset the counter to 0
	    timer->SR = 0;

	    // Step 2: Configure the timer to auto-reload when the max value is reached
	    timer->ARR = 0xFFFF;         // Set the Auto-Reload Register (ARR) to the max value (16-bit max)

	    // Step 3: Enable the timer interrupt
	    timer->DIER |= TIM_DIER_UIE;  // Enable the Update Interrupt (UIE) bit
	    NVIC_EnableIRQ(TIM2_IRQn);    // Enable the interrupt in the NVIC (interrupt controller)
	    NVIC_SetPriority(TIM2_IRQn, 1); // Set the priority of the interrupt (optional)

	    // Step 4: Start the timer
	    timer->CR1 |= TIM_CR1_CEN;    // Set the CEN bit to enable the timer
}

void timer_reset(TIM_TypeDef* timer)
{
  // TODO implement this
	// Reset the timer's counter value, but don't reset the whole timer
	    timer->CNT = 0;  // Set the counter back to 0

}

void timer_set_ms(TIM_TypeDef* timer, uint16_t period_ms)
{
		// TODO implement this
		// Step 1: Calculate the timer's frequency based on the system clock
	    uint32_t system_clock = 8000000;  // Assuming system clock is 4 MHz (adjust based on actual clock)

	    // Step 2: Determine the prescaler value and the period for milliseconds
	    uint32_t prescaler = (system_clock / 1000) - 1;  // To get 1 ms ticks
	    uint32_t period = period_ms;  // Period in milliseconds

	    // Step 3: Set the prescaler and auto-reload register for the timer
	    timer->PSC = prescaler;  // Set the prescaler to divide the clock
	    timer->ARR = period - 1; // Set the period value for how many ticks it should wait

	    // Step 4: Enable the timer (done in the initialization)
}

