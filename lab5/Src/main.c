/**
  *
  *
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
void _Error_Handler(char * file, int line);

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

//Initialize all four LEDs
void init_leds(void)
{
  //Initialize red LED, PC6
  GPIOC->MODER |= GPIO_MODER_MODER6_0; //General purpose output
  GPIOC->OTYPER &= ~GPIO_OTYPER_OT_6; // Push-pull
  GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEEDR6_0; //Low speed
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR6_0 | GPIO_PUPDR_PUPDR6_1); //No pull up or down

  //Initialize blue LED, PC7
  GPIOC->MODER |= GPIO_MODER_MODER7_0; //General purpose output
  GPIOC->OTYPER &= ~GPIO_OTYPER_OT_7; // Push-pull
  GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEEDR7_0; //Low speed
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR7_0 | GPIO_PUPDR_PUPDR7_1); //No pull up or down

  //Initialize orange LED, PC8
  GPIOC->MODER |= GPIO_MODER_MODER8_0; //General purpose output
  GPIOC->OTYPER &= ~GPIO_OTYPER_OT_8; // Push-pull
  GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEEDR8_0; //Low speed
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR8_0 | GPIO_PUPDR_PUPDR8_1); //No pull up or down

  //Initialize green LED, PC9
  GPIOC->MODER |= GPIO_MODER_MODER9_0; //General purpose output
  GPIOC->OTYPER &= ~GPIO_OTYPER_OT_9; // Push-pull
  GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEEDR9_0; //Low speed
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR9_0 | GPIO_PUPDR_PUPDR9_1); //No pull up or down

  //Set all LEDs off
  GPIOC->BSRR |= GPIO_BSRR_BR_6;
  GPIOC->BSRR |= GPIO_BSRR_BR_7;
  GPIOC->BSRR |= GPIO_BSRR_BR_8;
  GPIOC->BSRR |= GPIO_BSRR_BR_9;
}

int main(void)
{
  SystemClock_Config();

  //Enable clock on GPIOB and GPIOC
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

  init_leds();
  
  //Set PB11 to alternate function, open-drain output, I2C2_SDA
  GPIOB->MODER |= GPIO_MODER_MODER11_1;
  GPIOB->OTYPER |= GPIO_OTYPER_OT_11;
  GPIOB->AFR[1] |= 0x00001000;
  //Set PB13 to alternate function, open-drain output, I2C2_SCL
  GPIOB->MODER |= GPIO_MODER_MODER13_1;
  GPIOB->OTYPER |= GPIO_OTYPER_OT_13;
  GPIOB->AFR[1] |= 0x00500000;
  //Set PB14 to output mode, push-pull output, set pin high
  GPIOB->MODER |= GPIO_MODER_MODER14_0;
  GPIOB->OTYPER &= ~GPIO_OTYPER_OT_14;
  GPIOB->ODR |= GPIO_ODR_14;
  //Set PC0 to output mode, push-pull output, set pin high
  GPIOC->MODER |= GPIO_MODER_MODER0_0;
  GPIOC->OTYPER &= ~GPIO_OTYPER_OT_0;
  GPIOC->ODR |= GPIO_ODR_0;

  //Enable I2C2 in RCC
  RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
  //Set TIMINGR to 100 kHz standard-mode - section 5.5.2 and fig 5.4
  //PRESC 1
  //SCLL 0x13
  //SCLH 0xF
  //SDADEL 0x2
  //SCLDEL 0x4
  I2C2->TIMINGR |= 0x10420F13;
  //Enable I2C using PE bit in CR1
  I2C2->CR1 |= I2C_CR1_PE;
  //Set L3GD20 slave address = 0x69
  I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
  I2C2->CR2 |= ((1 << 16) | (0x69 << 1));
  //Set number of bytes to transmit = 1
  //set RD_WRN bit to 0 to indicate write
  //set the START bit
  //I2C2->CR2 |= 0x00010069;
  HAL_Delay(1);
  I2C2->CR2 |= I2C_CR2_START;

  //Wait until either TXIS or NACKF flags are set
  while (!(I2C2->ISR & I2C_ISR_NACKF) && !(I2C2->ISR & I2C_ISR_TXIS)) {}
  if (I2C2->ISR & I2C_ISR_NACKF)
  {
    //Failed
    GPIOC->ODR |= GPIO_ODR_6;
  }
  else
  {
    //success
    GPIOC->ODR |= GPIO_ODR_9;
    //Write addres of WHO_AM_I register into I2C TXDR
    I2C2->TXDR = 0x0F;
    //Wait until TC flag set
    while (!(I2C2->ISR & I2C_ISR_TC)) {}
    //Reload CR2 with same parameters but set RD_WRN bit to 1 to read 
    //set start bit again to perform restart
    I2C2->CR2 |= 0x0010469;
    I2C2->CR2 |= I2C_CR2_START;
    //wait until either RXNE or NACKF flags set
    while (!(I2C2->ISR & I2C_ISR_NACKF) && !(I2C2->ISR & I2C_ISR_RXNE)) {}
    if (I2C2->ISR & I2C_ISR_NACKF)
    {
      //Failed
      GPIOC->ODR |= GPIO_ODR_8;
    }
    else
    {
      //wait until TC
      while (!(I2C2->ISR & I2C_ISR_TC)) {}
      //Check RXDR register to see if it matches 0xD3
      if (I2C2->RXDR & 0xD3)
      {
        GPIOC->ODR |= GPIO_ODR_7;
      }
      //Set STOP bit in CR2 to release I2C bus
      I2C2->CR2 |= I2C_CR2_STOP;
    }
  }
  
  while (1)
  {
  }

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
