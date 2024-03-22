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

//Initialize USART3 for debugging
void init_uart(void)
{
  //Set pin PC4 for USART TX
  GPIOC->MODER |= GPIO_MODER_MODER4_1; //Alternate function
  GPIOC->OTYPER &= ~GPIO_OTYPER_OT_4; // Push-pull
  GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEEDR4_0; //Low speed
  GPIOC->PUPDR |= GPIO_PUPDR_PUPDR4_0; //Pull up
  //Set pin PC5 for USART RX
  GPIOC->MODER |= GPIO_MODER_MODER5_1; //Alternate function
  GPIOC->OTYPER &= ~GPIO_OTYPER_OT_5; // Push-pull
  GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEEDR5_0; //Low speed
  GPIOC->PUPDR |= GPIO_PUPDR_PUPDR5_0; //Pull up
  //-GPIOC->AFR[0] &= 0xFFFF44FF;
  GPIOC->AFR[0] |= 0x00110000;

  //Enable clock to USART3
  RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
  //Set word length to 8 bits
  USART3->CR1 &= ~USART_CR1_M1;
  USART3->CR1 &= ~USART_CR1_M0;
  //Set baud rate to 115200 bits/sec
  USART3->BRR = 0x45;
  //Set stop bits to 1
  USART3->CR2 &= ~USART_CR2_STOP_0;
  USART3->CR2 &= ~USART_CR2_STOP_1;
  //Set no parity
  USART3->CR1 &= ~USART_CR1_PCE;
  //Enable transmitter and receiver
  USART3->CR1 |= USART_CR1_TE;
  USART3->CR1 |= USART_CR1_RE;
  //Enable USART peripheral
  USART3->CR1 |= USART_CR1_UE;  
}

//Transmits one character over UART
void transmit_char(char c)
{
  //Wait until USART transmit register is empty
  while (!(USART3->ISR & USART_ISR_TC)) {}
  //Write character to transmit register
  USART3->TDR = c;
}

//Converts 16 bit signed int to chars for uart debugging
void transmit_val(int16_t val)
{
  uint8_t num[5];

  if (val < 0)
  {
    val = ~val;
    val += 1;
    transmit_char('-');
  }
  else
    transmit_char(' ');
    
  for (int i = 0; i < 5; i++)
  {
    num[i] = val % 10;
    val /= 10;
  }
  for (int i = 4; i >=0; i--)
    transmit_char(num[i]+48);

  transmit_char('\n');
  transmit_char('\r');
}

int16_t read_axis(char axis)
{
  //Set gyro I2C address to X or Y axis
  uint16_t axis_address;
  int16_t low = 0, high = 0, axis_value = 0;
  if (axis == 'x')
    axis_address = 0xA8;
  else if (axis == 'y')
    axis_address = 0xAA;
  else
    return 0;

  //Configure I2C to write to gyro
  I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
  I2C2->CR2 |= ((1 << 16) | (0x69 << 1));
  I2C2->CR2 &= ~I2C_CR2_RD_WRN;
  I2C2->CR2 |= I2C_CR2_START;
  
  //Wait until either TXIS or NACKF flags are set
  while (!(I2C2->ISR & I2C_ISR_NACKF) && !(I2C2->ISR & I2C_ISR_TXIS)) {}
  if (!(I2C2->ISR & I2C_ISR_NACKF))
  {
    //Write addres of axis data register into I2C TXDR
    I2C2->TXDR = axis_address;
    //Wait until TC flag set
    while (!(I2C2->ISR & I2C_ISR_TC)) {}
    //Reconfigure I2C to read 2 bytes
    I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));  
    I2C2->CR2 |= ((2 << 16) | (0x69 << 1));
    I2C2->CR2 |= I2C_CR2_RD_WRN;
    //set start bit again to perform restart
    I2C2->CR2 |= I2C_CR2_START;
    //wait until either RXNE or NACKF flags set
    while (!(I2C2->ISR & I2C_ISR_NACKF) && !(I2C2->ISR & I2C_ISR_RXNE)) {}
    if (!(I2C2->ISR & I2C_ISR_NACKF))
    {
      //Read first byte
      low = I2C2->RXDR;
    }
    else
      return 0;
    //wait until either RXNE or NACKF flags set
    while (!(I2C2->ISR & I2C_ISR_NACKF) && !(I2C2->ISR & I2C_ISR_RXNE)) {}
    if (!(I2C2->ISR & I2C_ISR_NACKF))
    {
      //Read second byte
      high = I2C2->RXDR;
    }
    else
      return 0;
    //Wait until transmission complete
    while (!(I2C2->ISR & I2C_ISR_TC)) {}
    //Set STOP bit in CR2 to release I2C bus
    I2C2->CR2 |= I2C_CR2_STOP;
  }
  else
    return 0;

  axis_value = high << 8;
  axis_value |= low;
  return axis_value;
}

int main(void)
{
  SystemClock_Config();

  //Enable clock on GPIOB and GPIOC
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

  init_leds();
  init_uart();

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
  I2C2->CR2 |= I2C_CR2_START;

  //Wait until either TXIS or NACKF flags are set
  while (!(I2C2->ISR & I2C_ISR_NACKF) && !(I2C2->ISR & I2C_ISR_TXIS)) {}
  if (I2C2->ISR & I2C_ISR_NACKF)
  {
    //Failed
    GPIOC->ODR |= GPIO_ODR_6; //Turn on red LED
  }
  else
  {
    //success
    GPIOC->ODR |= GPIO_ODR_9; //Turn on green LED
    //Write addres of WHO_AM_I register into I2C TXDR
    I2C2->TXDR = 0x0F;
    //Wait until TC flag set
    while (!(I2C2->ISR & I2C_ISR_TC)) {}
    //Reload CR2 with same parameters but set RD_WRN bit to 1 to read 
    I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));  
    I2C2->CR2 |= ((1 << 16) | (0x69 << 1));
    I2C2->CR2 |= I2C_CR2_RD_WRN;
    //set start bit again to perform restart
    I2C2->CR2 |= I2C_CR2_START;
    //wait until either RXNE or NACKF flags set
    while (!(I2C2->ISR & I2C_ISR_NACKF) && !(I2C2->ISR & I2C_ISR_RXNE)) {}
    if (I2C2->ISR & I2C_ISR_NACKF)
    {
      //Failed
      GPIOC->ODR |= GPIO_ODR_8; //Turn on orange LED
    }
    else
    {
      //wait until TC
      while (!(I2C2->ISR & I2C_ISR_TC)) {}
      //Check RXDR register to see if it matches 0xD3
      if (I2C2->RXDR == 0xD3)
      {
        GPIOC->ODR |= GPIO_ODR_7; //Turn on blue LED
      }
      //Set STOP bit in CR2 to release I2C bus
      I2C2->CR2 |= I2C_CR2_STOP;
    }
  }

  //Wait 1 second and turn off LEDs
  HAL_Delay(1000);
  GPIOC->ODR &= ~GPIO_ODR_6;
  GPIOC->ODR &= ~GPIO_ODR_7;
  GPIOC->ODR &= ~GPIO_ODR_8;
  GPIOC->ODR &= ~GPIO_ODR_9;

  //Writing to gyro
  I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
  I2C2->CR2 |= ((2 << 16) | (0x69 << 1));
  I2C2->CR2 &= ~I2C_CR2_RD_WRN;
  I2C2->CR2 |= I2C_CR2_START;
  
  //Wait until either TXIS or NACKF flags are set
  while (!(I2C2->ISR & I2C_ISR_NACKF) && !(I2C2->ISR & I2C_ISR_TXIS)) {}
  if (!(I2C2->ISR & I2C_ISR_NACKF))
  {
    //Success
    //Writing to control register 1
    I2C2->TXDR = 0x20;
    //Wait until TC flag set
    while (!(I2C2->ISR & I2C_ISR_NACKF) && !(I2C2->ISR & I2C_ISR_TXIS)) {}
    //Enable X and Y gyro axes and set sensor into normal mode
    I2C2->TXDR = 0x0B;
    //Wait until TC flag set
    while (!(I2C2->ISR & I2C_ISR_TC)) {}
    //Set STOP bit to release I2C bus
    I2C2->CR2 |= I2C_CR2_STOP;
  }
  else
    GPIOC->ODR |= GPIO_ODR_6; //Turn on red LED
  
  int16_t gyro_x, gyro_y;

  while (1)
  {
    HAL_Delay(100);
    //Read X value
    gyro_x = read_axis('x');
    HAL_Delay(1);
    //Read Y value
    gyro_y = read_axis('y');

    if (gyro_x > 2000)
    {
      GPIOC->ODR |= GPIO_ODR_9;     
    }
    else if (gyro_x < -2000)
    {
      GPIOC->ODR |= GPIO_ODR_8;
    }
    else
    {
      GPIOC->ODR &= ~GPIO_ODR_8;
      GPIOC->ODR &= ~GPIO_ODR_9;
    }

    if (gyro_y > 2000)
    {
      GPIOC->ODR |= GPIO_ODR_6;     
    }
    else if (gyro_y < -2000)
    {
      GPIOC->ODR |= GPIO_ODR_7;
    }
    else
    {
      GPIOC->ODR &= ~GPIO_ODR_6;
      GPIOC->ODR &= ~GPIO_ODR_7;
    } 
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