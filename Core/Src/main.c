#include "math.h"
#include "main.h"

#define RAD_TO_DEG 57.295779513082320876798154814105
#define MPU6050_ADDRESS  0x68
enum Mode { MODE_BALANCE, MODE_DRIVE };
enum Mode current_mode = MODE_BALANCE;

// Khai báo chân
// PWM: PA8 (TIM1_CH1), PA9 (TIM1_CH2)
// Ði?u khi?n: PB12 (DIR1), PB13 (EN1), PB14 (DIR2), PB15 (EN2)
/* -----------------Private variables ------------------------------*/
uint16_t cal_int;
uint32_t start,last;

double  speed;
double acc_x,acc_y,acc_z;
double temperature;
double gyro_roll,gyro_pitch,gyro_yaw;

double gyro_rate;
double angle_pitch;
float acc_angle;

float Kp = 1.0, Ki = 0.25, Kd = 100; // H? s? PID (c?n ch?nh cho xe c? th?)
float error = 0, last_error = 0, integral = 0, derivative = 0, output = 0;
const float desired_angle = 0.0; // Góc cân b?ng mong mu?n (th?ng d?ng)

double gyro_roll_cal,gyro_pitch_cal,gyro_yaw_cal;
double accel_roll_cal,accel_pitch_cal,accel_yaw_cal;


const float max_output = 999; // Gi?i h?n PWM d?ng co
const float max_integral = 100.0; // Ch?ng windup

double filter_pitch = 0.0;

char rx;
float angle_offset = 0.0f;

/*--------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
void gyro_setup(void);
void PID_Caculate(void);
float constrain(float value, float min, float max);
void calibrate_gyro(void);
void stop(void);
void motors(void);
void TIM1_PWM_Init(void);
void GPIO_Init(void);
void gyro_setup(void);
void gyro_signalen(void);
void USART2_IRQHandler(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
	TIM1_PWM_Init();
	GPIO_Init();
	gyro_setup();
	HAL_Delay(3000);
	calibrate_gyro();
	    // B?t d?ng co
  GPIOB->BRR = (1<<13) | (1<<15);  // EN1 và EN2 LOW (b?t)
  while (1)
  {
    if (USART2->SR & USART_SR_RXNE)
    {
        rx = USART2->DR;

        if (rx == 'F') {
            angle_offset = -7.0;
        }
        else if (rx == 'B') {
            angle_offset = 5.0;
        }
        else if (rx == 'S') {
            angle_offset = 0.0f;
        }
    }
		start = HAL_GetTick();
		if (start - last >= 5) {
				last = start;
				gyro_signalen();
				filter_pitch = 0.8 * filter_pitch + 0.2 * gyro_pitch;
				gyro_rate = (filter_pitch - gyro_pitch_cal) / 65.5;

				acc_angle = atan2(-(acc_x - accel_roll_cal) / 4096, acc_z / 4096) * RAD_TO_DEG;
				angle_pitch = 0.995 * (angle_pitch + gyro_rate * 0.005) + 0.005 * acc_angle;
				error = angle_pitch - angle_offset;
				PID_Caculate();
				motors();
		}
  }
}
void USART2_IRQHandler(void)
{

  HAL_UART_IRQHandler(&huart2);
}
void calibrate_gyro(void) {
    for (cal_int = 0; cal_int < 2000; cal_int++) {
        if (cal_int % 25 == 0) {
            GPIOC->ODR ^= GPIO_PIN_13;  // ??o tr?ng th?i ch?n PC14
        }
        gyro_signalen();  // ??c gi? tr? t? MPU6050
        gyro_roll_cal += gyro_roll;
        gyro_pitch_cal += gyro_pitch;
        gyro_yaw_cal += gyro_yaw;
				accel_roll_cal += acc_x;
				accel_pitch_cal += acc_y;
				accel_yaw_cal += acc_z;
				HAL_Delay(4);
    }
    GPIOC->BSRR = GPIO_PIN_13; // red_led(HIGH);
    gyro_roll_cal /= 2000;
    gyro_pitch_cal /= 2000;
    gyro_yaw_cal /= 2000;
		accel_roll_cal /= 2000;
		accel_pitch_cal /= 2000;
		accel_yaw_cal /= 2000;
}
void gyro_signalen(void)
{
    uint8_t data[14];  // B? d?m luu d? li?u t? MPU6050

    // ??c 14 byte t? d?a ch? 0x3B c?a MPU6050
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDRESS << 1, 0x3B, I2C_MEMADD_SIZE_8BIT, data, 14, HAL_MAX_DELAY);

    // Chuy?n d?i d? li?u t? byte sang s? nguy?n 16-bit
    acc_x = (int16_t)(data[0] << 8 | data[1]);
    acc_y = (int16_t)(data[2] << 8 | data[3]);
    acc_z = (int16_t)(data[4] << 8 | data[5]);
    temperature = (int16_t)(data[6] << 8 | data[7]);
    gyro_roll = (int16_t)(data[8] << 8 | data[9]);
    gyro_pitch = (int16_t)(data[10] << 8 | data[11]);
    gyro_yaw = (int16_t)(data[12] << 8 | data[13]);

}
void gyro_setup(void) {
    uint8_t data[2];

    // C?u h?nh Thanh ghi PWR_MGMT_1 (0x6B) - B?t MPU6050
    data[0] = 0x6B;
    data[1] = 0x00;
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDRESS << 1, data, 2, HAL_MAX_DELAY);

    // C?u h?nh Thanh ghi GYRO_CONFIG (0x1B) - 500dps full scale
    data[0] = 0x1B;
    data[1] = 0x08;
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDRESS << 1, data, 2, HAL_MAX_DELAY);

    // C?u h?nh Thanh ghi ACCEL_CONFIG (0x1C) - ?8g full scale range
    data[0] = 0x1C;
    data[1] = 0x10;
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDRESS << 1, data, 2, HAL_MAX_DELAY);

    // ??c l?i GYRO_CONFIG d? ki?m tra
    uint8_t check;
    data[0] = 0x1B;
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDRESS << 1, data, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDRESS << 1, &check, 1, HAL_MAX_DELAY);
    
    if (check != 0x08) {
        GPIOC->BSRR = GPIO_PIN_13; 
        while (1) HAL_Delay(10);
    }
    // C?u h?nh Thanh ghi CONFIG (0x1A) - B? l?c th?ng th?p ~43Hz
    data[0] = 0x1A;
    data[1] = 0x03;
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDRESS << 1, data, 2, HAL_MAX_DELAY);

}
void TIM1_PWM_Init(void) {
    // 1. B?t clock TIM1, GPIOA, GPIOB
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN;

    // 2. C?u hình PA8 (CH1) và PA9 (CH2) là Alternate Function Push-Pull
    GPIOA->CRH &= ~(GPIO_CRH_CNF8 | GPIO_CRH_MODE8 | GPIO_CRH_CNF9 | GPIO_CRH_MODE9);
    GPIOA->CRH |= GPIO_CRH_CNF8_1 | GPIO_CRH_MODE8_1 | GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9_1;
    
    // 3. C?u hình TIM1 PWM
    TIM1->PSC = 8 - 1;      // Prescaler: 72MHz/6 -> 12MHz
    TIM1->ARR = 1000 - 1;    // Period: 12000 ticks -> 12kHz PWM
    
    // C?u hình kênh 1 (PA8)
    TIM1->CCR1 = 0;        // Duty 50% (500/1000)
    TIM1->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;  // PWM mode 1
    
    // C?u hình kênh 2 (PA9)
    TIM1->CCR2 = 0;        // Duty 30% (300/1000)
    TIM1->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
    
    // B?t output
    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM1->BDTR |= TIM_BDTR_MOE;
    TIM1->CR1 |= TIM_CR1_CEN;
}
void GPIO_Init(void) {
    // C?u hình PB12-PB15 là output push-pull
    GPIOB->CRH &= ~(GPIO_CRH_CNF12 | GPIO_CRH_MODE12 | 
                    GPIO_CRH_CNF13 | GPIO_CRH_MODE13 |
                    GPIO_CRH_CNF14 | GPIO_CRH_MODE14 |
                    GPIO_CRH_CNF15 | GPIO_CRH_MODE15);
    
    GPIOB->CRH |= (GPIO_CRH_MODE12_0 | GPIO_CRH_MODE13_0 |
                   GPIO_CRH_MODE14_0 | GPIO_CRH_MODE15_0);  // Output 2MHz
    
		RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

		GPIOC->CRH &= ~(0xF << 20);         // Clear bits [23:20] cho PC13
		GPIOC->CRH |=  (0x2 << 20);         // MODE13 = 10 (output 2MHz), CNF13 = 00
    // T?t d?ng co m?c d?nh (EN active LOW)
    GPIOB->ODR |= (1<<13) | (1<<15);  // EN1 và EN2 HIGH (t?t)
}

void PID_Caculate(void)
{

//  error = angle_pitch - desired_angle;
  
  // Thành ph?n P
  float P = Kp * error;
  
  // Thành ph?n I (có ch?ng windup)
  integral += error * 0.005;
  integral = constrain(integral, -max_integral, max_integral);
  float I = Ki * integral;
  
  // Thành ph?n D
  derivative = (error - last_error) / 0.005;
  float D = Kd * derivative;
  last_error = error;
  
  // 3. T?ng h?p output
  output = P + I + D;
  output = constrain(output, -max_output, max_output);
}

void stop(void)
{
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	GPIOB->BRR = (1<<13) | (1<<15);  // EN1 và EN2 LOW (b?t)
}
void motors(void) {
    // B?t driver (EN = LOW)
    GPIOB->BRR = (1<<13) | (1<<15);

    // Ð?t hu?ng d?a trên d?u c?a speed
    if (error >= 3.5) 
		{

        GPIOB->BRR = (1<<12);  // DIR1 = LOW (quay thu?n)
        GPIOB->BSRR = (1<<14);  // DIR2 = LOW (quay thu?n)
			  TIM1->CCR1 = fabs(output);
				TIM1->CCR2 = fabs(output);
    } 
		else if(error <= -3.5)
		{

        GPIOB->BSRR = (1<<12); // DIR1 = HIGH (quay ngu?c)
        GPIOB->BRR = (1<<14); // DIR2 = HIGH (quay ngu?c)
			  TIM1->CCR1 = fabs(output);
				TIM1->CCR2 = fabs(output);
    }
		else if(error >= 20)
		{

				GPIOB->BRR = (1<<12);  // DIR1 = LOW (quay thu?n)
        GPIOB->BSRR = (1<<14);  // DIR2 = LOW (quay thu?n)
			  TIM1->CCR1 = max_output;
				TIM1->CCR2 = max_output;
		}
		else if(error <= -20)
		{

        GPIOB->BSRR = (1<<12); // DIR1 = HIGH (quay ngu?c)
        GPIOB->BRR = (1<<14); // DIR2 = HIGH (quay ngu?c)
			  TIM1->CCR1 = max_output;
				TIM1->CCR2 = max_output;
    }
		else
		{
			stop();
		}

}

float constrain(float value, float min, float max) {
  if (value < min) return min;
  else if (value > max) return max;
  else return value;
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
