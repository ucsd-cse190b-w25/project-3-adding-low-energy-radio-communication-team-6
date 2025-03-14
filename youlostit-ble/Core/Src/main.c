/* USER CODE BEGIN Header */

/**
  ******************************************************************************
  * @file    main.c
  * @brief   Main program body
  ******************************************************************************
  */

/* USER CODE END Header */
#include "ble.h"
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

/* Include memory map of our MCU */
#include <stm32l475xx.h>

/* Include LED driver */
#include "leds.h"

/* Include timer driver */
#include "timer.h"

/* Include i2c driver */
#include "i2c.h"

/* Include lsm6dsl driver */
#include "lsm6dsl.h"

#include "lptim.h"

#define STUDENT_ID 0x0C1F   // 3103 16-bit student ID
#define PREAMBLE 0x99       // 8-bit preamble (10011001 in binary)
#define LSM6DSL_ADDR 0x6A
#define WHO_AM_I_REG 0x0F

int dataAvailable = 0;

SPI_HandleTypeDef hspi3;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);

// Define timing intervals
// #define LED_TOGGLE_INTERVAL 50     // LED toggle every 50 ms
#define MESSAGE_SEND_INTERVAL 1  // Send message every 1 * 10000ms = 10s
#define LOST_MODE_THRESHOLD 6  // Lost mode after 6 * 10000ms = 60s

volatile uint32_t inactivity_counter = 0;
volatile uint32_t message_counter = 0;
volatile uint8_t lost_mode_triggered = 0;
volatile uint32_t lost_seconds_counter = 0;

enum state { FOUND, LOST };
enum state currentState = FOUND;

int16_t x_prev = 0, y_prev = 0, z_prev = 0;

// Function to detect movement
int isMoving() {
    int16_t x, y, z;
    lsm6dsl_read_xyz(&x, &y, &z);

    int16_t x_diff = abs(x - x_prev);
    int16_t y_diff = abs(y - y_prev);
    int16_t z_diff = abs(z - z_prev);

    x_prev = x;
    y_prev = y;
    z_prev = z;

    return (x_diff > 500 || y_diff > 500 || z_diff > 500);
}

// Redefine the libc _write() function so you can use printf in your code
int _write(int file, char *ptr, int len) {
    int i = 0;
    for (i = 0; i < len; i++) {
        ITM_SendChar(*ptr++);
    }
    return len;
}


// Send BLE lost message
void sendLostMessage() {
    unsigned char message[30]; // Adjust size as necessary
    snprintf((char*)message, sizeof(message), "IMLOSIN %lus missing", lost_seconds_counter * 10);
    updateCharValue(NORDIC_UART_SERVICE_HANDLE, READ_CHAR_HANDLE, 0, strlen((char*)message), message);
}


// LPTIM1 Interrupt Handler (Runs every 10000ms)
void LPTIM1_IRQHandler(void) {
    if (LPTIM1->ISR & LPTIM_ISR_ARRM) { // Check if autoreload match occurred
    	LPTIM1->ICR |= LPTIM_ICR_ARRMCF; // Clear the interrupt flag

        // Toggle LEDs every 10 seconds
        // GPIOA->ODR ^= GPIO_ODR_OD5;  // Toggle LED 1 (PA5)
        // GPIOB->ODR ^= GPIO_ODR_OD14; // Toggle LED 2 (PB14)

        if (currentState == FOUND) {
            inactivity_counter++;
            if (inactivity_counter >= LOST_MODE_THRESHOLD) {
                currentState = LOST;
                lost_mode_triggered = 1;  // First BLE message immediately
                message_counter = 0;
                lost_seconds_counter = 0;  // Reset lost seconds counter
            }
        } else if (currentState == LOST) {
            lost_seconds_counter++; // Increment seconds counter in LOST mode
            message_counter++;
            if (message_counter >= MESSAGE_SEND_INTERVAL) {
                sendLostMessage();
                message_counter = 0;
            }
        }
        //LPTIM1->CR |= LPTIM_CR_CNTSTRT;
    }
}

uint8_t nonDiscoverable = 0; // FOUND

// Handle state transitions
void handleState() {
    if (isMoving()) {
        inactivity_counter = 0;
        if (currentState == LOST) {
            currentState = FOUND;
            disconnectBLE();
            setDiscoverability(0);
            lost_mode_triggered = 0;
        }
    }

    if (currentState == LOST && lost_mode_triggered) {
        sendLostMessage();
        setDiscoverability(1);
        lost_mode_triggered = 0;
    }
}

int main(void) {
    HAL_Init();
    SystemClock_Config();

    i2c_init();
    lsm6dsl_init();
    leds_init();
    lptim1_init(LPTIM1);

    lptim1_set_ms(LPTIM1, 20000);

    // Enable LPTIM1 interrupt in NVIC
    //LPTIM1_IRQHandler();
    NVIC_SetPriority(LPTIM1_IRQn, 1);
    HAL_NVIC_EnableIRQ(LPTIM1_IRQn);

    MX_GPIO_Init();
    MX_SPI3_Init();

    // Reset BLE module
    HAL_GPIO_WritePin(BLE_RESET_GPIO_Port, BLE_RESET_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(BLE_RESET_GPIO_Port, BLE_RESET_Pin, GPIO_PIN_SET);

    ble_init();
    HAL_Delay(10);
    setDiscoverability(0);

    while (1) {
        handleState();

             if (currentState == FOUND) {
            	 // Disable all peripherals on APB1 (except I2C2 and TIM2)
            	 RCC->APB1ENR1 &= ~(RCC_APB1ENR1_OPAMPEN | RCC_APB1ENR1_DAC1EN | RCC_APB1ENR1_TIM2EN |
            	                    RCC_APB1ENR1_PWREN | RCC_APB1ENR1_CAN1EN | RCC_APB1ENR1_I2C3EN |
            	                    RCC_APB1ENR1_I2C1EN | RCC_APB1ENR1_UART5EN | RCC_APB1ENR1_UART4EN |
            	                    RCC_APB1ENR1_USART3EN | RCC_APB1ENR1_USART2EN | RCC_APB1ENR1_SPI2EN |
            	                    RCC_APB1ENR1_WWDGEN | RCC_APB1ENR1_TIM7EN | RCC_APB1ENR1_TIM6EN |
            	                    RCC_APB1ENR1_TIM5EN | RCC_APB1ENR1_TIM4EN | RCC_APB1ENR1_TIM3EN);

            	 //standbyBle();

            	 RCC->APB1ENR2 = 0;
            	 RCC->APB2ENR = 0;
            	 RCC->AHB2ENR = 0;

            	 // Disable I2C clock
            	 RCC->APB1ENR1 &= ~RCC_APB1ENR1_I2C2EN;

            	 // Disable SPI clock
            	 RCC->APB1ENR1 &= ~RCC_APB1ENR1_SPI3EN;

                 // If in FOUND state and not performing tasks, enter light sleep mode
            	 SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk; // Enable deep sleep

                 HAL_SuspendTick();  // Suspend system tick before sleeping
                 __WFI();            // Wait for interrupt (enters sleep mode)
                 HAL_ResumeTick();   // Resume system tick after waking up

                 // Re-enable peripheral clocks after waking up
                 RCC->APB1ENR1 |= (RCC_APB1ENR1_I2C2EN | RCC_APB1ENR1_SPI3EN);

                 // Re-enable LPTIM1 clock if needed after wake-up
                 //RCC->APB1ENR1 |= RCC_APB1ENR1_LPTIM1EN;
                 //LPTIM1->CR |= LPTIM_CR_ENABLE;  // Ensure it's enabled

                 // Re-enable GPIO clocks
                 RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN |
                                  RCC_AHB2ENR_GPIOCEN | RCC_AHB2ENR_GPIODEN |
                                  RCC_AHB2ENR_GPIOEEN);
             }

        if (!nonDiscoverable && HAL_GPIO_ReadPin(BLE_INT_GPIO_Port, BLE_INT_Pin)) {
            catchBLE();
        }
    }
}


/**
  * @brief System Clock Configuration
  * @attention This changes the System clock frequency, make sure you reflect that change in your timer
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the RCC Oscillators according to the specified parameters
      * in the RCC_OscInitTypeDef structure.
      */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = 0;
    // This line changes system clock frequency
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_7;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void) {
    /* USER CODE BEGIN SPI3_Init 0 */
    /* USER CODE END SPI3_Init 0 */

    /* USER CODE BEGIN SPI3_Init 1 */
    /* USER CODE END SPI3_Init 1 */
    /* SPI3 parameter configuration */
    hspi3.Instance = SPI3;
    hspi3.Init.Mode = SPI_MODE_MASTER;
    hspi3.Init.Direction = SPI_DIRECTION_2LINES;
    hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi3.Init.NSS = SPI_NSS_SOFT;
    hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi3.Init.CRCPolynomial = 7;
    hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
    if (HAL_SPI_Init(&hspi3) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI3_Init 2 */
    /* USER CODE END SPI3_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* USER CODE BEGIN MX_GPIO_Init_1 */
    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIO_LED1_GPIO_Port, GPIO_LED1_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(BLE_CS_GPIO_Port, BLE_CS_Pin, GPIO_PIN_SET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(BLE_RESET_GPIO_Port, BLE_RESET_Pin, GPIO_PIN_SET);

    /*Configure GPIO pin : BLE_INT_Pin */
    GPIO_InitStruct.Pin = BLE_INT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BLE_INT_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : GPIO_LED1_Pin BLE_RESET_Pin */
    GPIO_InitStruct.Pin = GPIO_LED1_Pin | BLE_RESET_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : BLE_CS_Pin */
    GPIO_InitStruct.Pin = BLE_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(BLE_CS_GPIO_Port, &GPIO_InitStruct);

    /* EXTI interrupt init */
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
