/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f446xx.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
// Variable for clearing ADDR bit
uint8_t cl;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void GPIO_Init(void);
void I2C_Init(void);
void I2C_Start(void);
void I2C_Write(uint8_t var);
void I2C_Send_Addr(uint8_t Addr);
void I2C_Stop(void);
void LCD_Init(void);
void LCD_Write_Cmd(uint8_t Device_Addr, uint8_t Slave_Reg_Addr, uint8_t data);
void LCD_Write_Data(uint8_t Device_Addr, uint8_t Slave_Reg_Addr, uint8_t data);
void LCD_Cursor(int r, int c);
void TIM4_ms_Delay(uint16_t delay);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  TIM4_ms_Delay(100);// give the LCD time to finish its own POR
  GPIO_Init();
  I2C_Init();
  LCD_Init();
  LCD_Cursor(0, 2);  // position cursor at row 0, col 2
  char *msg = "I LOVE BUBU";
  for (int i = 0;  i < strlen(msg);  i++) {
      LCD_Write_Data(LCD_ADDR, DATA_REG, msg[i]);
  }
  TIM4_ms_Delay(1000); // Delay of 1s
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */



/* USER CODE BEGIN 4 */
void GPIO_Init()
{
	RCC->AHB1ENR  |= RCC_AHB1ENR_GPIOBEN;    // enable GPIOB clock
	GPIOB->MODER &= ~(GPIO_MODER_MODER6_Msk); //Clears PB6 MODER first
	GPIOB->MODER |= GPIO_MODER_MODER6_1; //Sets the PB6 MODER to alternate function
	GPIOB->MODER &= ~(GPIO_MODER_MODER7_Msk); //Repeat for PB7
	GPIOB->MODER |= GPIO_MODER_MODER7_1;
	GPIOB->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED6_Msk; //Clear OSPEEDR
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED6_Msk; //Sets PB6 to High Speed
	GPIOB->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED7_Msk; //Repeat for PB7
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED7_Msk;
	GPIOB->OTYPER |= GPIO_OTYPER_OT6_Msk; //Open drain PB6
	GPIOB->OTYPER |= GPIO_OTYPER_OT7_Msk; //Open drain PB7
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD6_Msk; //Clears PB6 PUPDR
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD6_1; //Sets PB6 to Pull-Up
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD7_Msk; //Repeat for PB7
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD7_1;
	GPIOB->AFR[0] &= ~GPIO_AFRL_AFSEL6_Msk; //Clears AFRL for PB6
	GPIOB->AFR[0] |= GPIO_AFRL_AFSEL6_2; //sets AF4 for PB6
	GPIOB->AFR[0] &= ~GPIO_AFRL_AFSEL7_Msk; //Repeats for PB7
	GPIOB->AFR[0] |= GPIO_AFRL_AFSEL7_2;
}
/* USER CODE END 4 */
void I2C_Init()
{
	RCC->APB1ENR  |= RCC_APB1ENR_I2C1EN;     // enable I2C1 clock
	I2C1->CR1 |= I2C_CR1_SWRST_Msk; //Software reset for I2C1
	I2C1->CR1 &= ~I2C_CR1_SWRST_Msk;
	I2C1->CR2 &= ~I2C_CR2_FREQ_Msk; //Clears FREQ reg
	I2C1->CR2 |= 16UL << 0; //Sets I2C clock at 16 MHz
	I2C1->OAR1 |= 1UL << 14; //OAR1 software needs to be kept at 1 by software.
	I2C1->CCR = 0x50; //sets SCL at 100 Hz(80 decimal)
	I2C1->TRISE = 17;     // TRISE for 1000 ns @ 16 MHz
	I2C1->CR1  |= I2C_CR1_PE;    // enable I²C

}
void I2C_Start()
{
	I2C1->CR1 |= I2C_CR1_ACK; //Enables ACK bit
	I2C1->CR1 |= I2C_CR1_START; //Start bit generation
	while(!(I2C1->SR1 & I2C_SR1_SB)) {} //bit mask to wait until start bit detected
}
void I2C_Write(uint8_t var)
{
    while(!(I2C1->SR1 & I2C_SR1_TXE)){} //polling the TXE to see if data reg is empty
    I2C1->DR = var; //if empty put var into DR
    while(!(I2C1->SR1 & I2C_SR1_BTF)){} //polls the byte transfer finished


}
void I2C_Send_Addr(uint8_t Addr){
    //Put the address to be sent into the I2C_DR register
    I2C1->DR = Addr;

    // ADDR bit is polled in the I2C_SR1 register for end of
    // address transmission
    while(!(I2C1->SR1 & I2C_SR1_ADDR)){}

    // Variable that will read I2C_SR1 and I2C_SR2
    // for clearing the ADDR bit
    cl = (I2C1->SR1 | I2C1->SR2);
}
void I2C_Stop(){
    // Stop Communication after current byte transfer
    I2C1->CR1 |= 1UL<<9;
}
void LCD_Write_Cmd(uint8_t Device_Addr,uint8_t Slave_Reg_Addr, uint8_t data)
{
	uint8_t dh, dl, d1, d2, d3, d4;
	dh = data & 0xF0; //extracts only upper 4 bits
	dl = (data<<4) & 0xF0; //extracts lower 4 bits
	d1 = dh | 0x0C; //sets EN=1, RS=0, R/W=0 send the upper 4 bits
	d2 = dh | 0x08; //sets EN=0, RS=0, set back lights on
	d3 = dl | 0x0C; //same for lower 4 bits
	d4 = dl | 0x08;
	I2C_Start();
	I2C_Send_Addr(Device_Addr);
	I2C_Write(Slave_Reg_Addr);
	I2C_Write(d1);
	TIM4_ms_Delay(2); // Wait for 2 ms for the command to take action
	I2C_Write(d2);
	I2C_Write(d3);
	TIM4_ms_Delay(2); // Wait for 2 ms for the command to take action
	I2C_Write(d4);
	I2C_Stop();
}
void LCD_Write_Data(uint8_t Device_Addr, uint8_t Slave_Reg_Addr, uint8_t data)
{
	uint8_t dh, dl, d1, d2, d3, d4;
	dh = data & 0xF0; //extracts only upper 4 bits
	dl = (data<<4) & 0xF0; //extracts lower 4 bits
	d1 = dh | 0x0D; //Same as before for d1 but RS needs to be 1
	d2 = dh | 0x09;
	d3 = dl | 0x0D; //same for lower 4 bits
	d4 = dl | 0x09;
	I2C_Start();
	I2C_Send_Addr(Device_Addr);
	I2C_Write(Slave_Reg_Addr);
	I2C_Write(d1);
	TIM4_ms_Delay(2); // Wait for 2 ms for the command to take action
	I2C_Write(d2);
	I2C_Write(d3);
	TIM4_ms_Delay(2); // Wait for 2 ms for the command to take action
	I2C_Write(d4);
	I2C_Stop();
}
void LCD_Cursor(int r, int c){
    if (r==1){
        c |= 0xC0;
        LCD_Write_Cmd(LCD_ADDR, INST_REG, c);
    }
    else{
        c |= 0x80;
        LCD_Write_Cmd(LCD_ADDR, INST_REG, c);
    }
}
void LCD_Init(){
    // 1. Initializing the LCD in 4-bit mode
    TIM4_ms_Delay(50);
    LCD_Write_Cmd(LCD_ADDR,INST_REG,0x30);
    TIM4_ms_Delay(5);

    LCD_Write_Cmd(LCD_ADDR,INST_REG,0x30);
    TIM4_ms_Delay(1);

    LCD_Write_Cmd(LCD_ADDR,INST_REG,0x30);
    TIM4_ms_Delay(10);

    LCD_Write_Cmd(LCD_ADDR,INST_REG,0x20); // Set the LCD in 4-bit Mode
    TIM4_ms_Delay(10);

    // 2. Initializing the Display

    // Function Set (DL=0 for 4-bit mode; N=1 for 2-line display;
    // F=0 for 5x8 characters)
    LCD_Write_Cmd(LCD_ADDR,INST_REG,0x28);
    TIM4_ms_Delay(5);

    // Display Control (D=0;C=0;B=0 - Display is off)
    LCD_Write_Cmd(LCD_ADDR,INST_REG,0x08);
    TIM4_ms_Delay(5);

    // Clear the display
    LCD_Write_Cmd(LCD_ADDR,INST_REG,0x01);
    TIM4_ms_Delay(5);

    TIM4_ms_Delay(1); // Wait for some time

    // correct: 0x06 = I/D=1 (increment), S=0 (no shift)
    LCD_Write_Cmd(LCD_ADDR, INST_REG, 0x06);
    TIM4_ms_Delay(5);

    // Display Control (D=1;C=0;B=0 - Cursor blinks)
    LCD_Write_Cmd(LCD_ADDR,INST_REG,0x0F);
    TIM4_ms_Delay(5);
}
void TIM4_ms_Delay(uint16_t delay){
    RCC->APB1ENR |= 1<<2; //Start the clock for the timer peripheral
    TIM4->PSC = 16000-1; //Setting the clock frequency to 1kHz.
    TIM4->ARR = (delay); // Total period of the timer
    TIM4->CNT = 0;
    TIM4->CR1 |= 1; //Start the Timer
    while(!(TIM4->SR & TIM_SR_UIF)){} //Polling the update interrupt flag
    TIM4->SR &= ~(0x0001); //Reset the update interrupt flag
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
