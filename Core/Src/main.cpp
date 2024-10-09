/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "adc.h"
#include "cordic.h"
#include "fdcan.h"
#include "i2c.h"
#include "opamp.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "../../UserLib/pwm.hpp"
#include "../../UserLib/pid.hpp"
#include "../../UserLib/math.hpp"
#include "../../UserLib/programable_LED.hpp"
#include "../../UserLib/LED_pattern.hpp"
#include "../../UserLib/encoder.hpp"
#include "../../UserLib/cordic.hpp"
#include "../../UserLib/fdcan_control.hpp"

#include <stdio.h>
#include <memory>

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern "C"{
	int _write(int file, char *ptr, int len) {
		HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len,100);
		return len;
	}
}

auto PWM_U = SabaneLib::LEDPWMHard{&htim1,TIM_CHANNEL_2};
auto PWM_V = SabaneLib::LEDPWMHard{&htim1,TIM_CHANNEL_3};
auto PWM_W = SabaneLib::LEDPWMHard{&htim1,TIM_CHANNEL_1};

auto table = SabaneLib::MotorMath::SinTable<12>{};

auto atan_enc = SabaneLib::ContinuableEncoder(16,1000.f);
auto enc = SabaneLib::AS5600State(&hi2c1,1000,true);

auto cordic = SabaneLib::MotorMath::FastMathCordic{CORDIC};

auto position_pid = SabaneLib::PIDBuilder(1000.0f).set_gain(0.000'01f, 0.000'007f, 0.0f).set_limit(0.1f).build();

auto can = SabaneLib::FdCanComm{&hfdcan1,
	std::make_unique<SabaneLib::RingBuffer<SabaneLib::CanFrame,5> >(),
	std::make_unique<SabaneLib::RingBuffer<SabaneLib::CanFrame,5> >(),
	FDCAN_RX_FIFO0,
	FDCAN_FILTER_TO_RXFIFO0,
	FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
	FDCAN_FLAG_RX_FIFO0_NEW_MESSAGE
};

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
	HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
	can.rx_interrupt_task();
}
void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes){
	can.tx_interrupt_task();
}


volatile uint16_t adc_val[4]={0};
volatile uint16_t enc_val[2]={0};
volatile uint16_t vref_val = 0;
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc){
      if(hadc == &hadc1){
    	  adc_val[0]=ADC1->JDR1;
    	  enc_val[0]=ADC1->JDR2;
    	  enc_val[1]=ADC1->JDR3;
    	  vref_val = ADC1->JDR4;

      }else if(hadc == &hadc2){
    	  adc_val[1]=ADC2->JDR1;
    	  adc_val[2]=ADC2->JDR2;
    	  //enc_val[1]=ADC1->JDR3;
      }
}


float power_rad = 0.0f;
float power = 0.05;
float target_mm = 0.0f;
constexpr float angle_to_rad = 2.0f*M_PI/(float)(1<<12);
int32_t atan_enc_bias = 0;

void motor_move(float pwm){
	::power_rad = SabaneLib::MotorMath::q15_to_rad(::atan_enc.get_angle()) + (M_PI/2.0f)*(pwm>0.0f?1.0f:-1.0f);
	::power = abs(pwm);

	PWM_U.out(table.cos(::power_rad                                  )*0.4f*::power + 0.5f);
	PWM_V.out(table.cos(::power_rad - static_cast<float>(2*M_PI)/3.0f)*0.4f*::power + 0.5f);
	PWM_W.out(table.cos(::power_rad + static_cast<float>(2*M_PI)/3.0f)*0.4f*::power + 0.5f);
}

q15_t qsin;
q15_t qcos;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim17){
		enc.read_start();

		::qsin =  (static_cast<q15_t>(enc_val[0])-static_cast<q15_t>(2142));//-static_cast<q15_t>((1.7/3.2)*(0xFFF)));
		::qcos = -(static_cast<q15_t>(enc_val[1])-static_cast<q15_t>(2154));//-static_cast<q15_t>((1.7/3.2)*(0xFFF)));

		::cordic.start_atan2(static_cast<q15_t>(::qcos*16),static_cast<q15_t>(::qsin*16));
		while(not ::cordic.is_avilable());
		::atan_enc.update(cordic.read_ans());

		constexpr float mm_to_q15rad = static_cast<float>(0xFFFF) / 30.0f;
		float target_angle = ::target_mm * mm_to_q15rad;
		::motor_move(::position_pid(target_angle,::atan_enc.get_angle()-atan_enc_bias));
	}
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
	::enc.i2c_rx_interrupt_task();
}

float adc_to_current(uint16_t adc_val){
	constexpr float rl = (2.2*1.5)/(2.2+1.5); //Âü∫Êùø‰∏äÔøΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?ÂúßÊäµÊäóÔøΩ????øΩ?øΩ??øΩ?øΩ???øΩ?øΩ??øΩ?øΩ?
	constexpr float v_bias = rl/(rl+22)*3.3;  //„Éê„Ç§„Ç¢„ÇπÈõªÂúß
	constexpr float amp_gain_inv = 1.0f/7.0f; //cube mx„ÅßË®≠ÂÆö„Åô„Çã„Ç™„Éö„Ç¢„É≥„ÉóÔøΩ????øΩ?øΩ??øΩ?øΩ„Ç≤„Ç§„É≥„ÅÆ???øΩ?øΩ??øΩ?øΩ?Êï∞
	constexpr float shant_r_inv = 1.0f/0.005f;         //„Ç∑„É£„É≥„ÉàÊäµÊäóÔøΩ????øΩ?øΩ??øΩ?øΩÂÄ§
	float v = adc_val*3.3/static_cast<float>(1<<12);

	return ((amp_gain_inv + 1.0f)*v_bias - amp_gain_inv*v)*shant_r_inv;
}

namespace LSMParam{
	enum class Shaft{
		X,
		Y
	};

	enum class Config{
		POS,
		POWER,
		GAIN_P,
		GAIN_I,
		GAIN_D
	};
}

float data_select(LSMParam::Shaft xy,SabaneLib::ByteReader &r){
	auto data_x = r.read<float>();
	auto data_y = r.read<float>();

	switch(xy){
	case LSMParam::Shaft::X:
		return data_x.has_value() ? data_x.value() : 0.0f;
	case LSMParam::Shaft::Y:
		return data_y.has_value() ? data_y.value() : 0.0f;
	}
}

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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_OPAMP1_Init();
  MX_OPAMP2_Init();
  MX_OPAMP3_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM17_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_I2C1_Init();
  MX_CORDIC_Init();
  MX_TIM2_Init();
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);

  HAL_OPAMP_Start(&hopamp1);
  HAL_OPAMP_Start(&hopamp2);
  HAL_OPAMP_Start(&hopamp3);

  HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);

  table.generate([](float rad)->float{
	  cordic.start_sincos(rad);
	  while(not cordic.is_avilable());
	  return cordic.get_sincos().sin;
  });
  //table.generate();

  HAL_GPIO_WritePin(CAN_R_GPIO_Port,CAN_R_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(CAN_SHDN_GPIO_Port,CAN_SHDN_Pin,GPIO_PIN_RESET);
  can.start();
  can.set_filter_free(0);

    PWM_U.start();
	PWM_V.start();
	PWM_W.start();
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);

	HAL_TIM_Base_Start_IT(&htim17);

	HAL_ADC_Start(&hadc1);
	HAL_ADCEx_InjectedStart_IT(&hadc1);
	HAL_ADC_Start(&hadc2);
	HAL_ADCEx_InjectedStart_IT(&hadc2);

	HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);

	enc.read_start();

	HAL_Delay(10);

	const int32_t enc_bias = enc.get_angle();

	::qsin =  (static_cast<q15_t>(enc_val[0])-static_cast<q15_t>((1.7/3.2)*(0xFFF)));
	::qcos = -(static_cast<q15_t>(enc_val[1])-static_cast<q15_t>((1.7/3.2)*(0xFFF)));

	cordic.start_atan2(static_cast<q15_t>(::qcos * 16),static_cast<q15_t>(::qsin * 16));
	while(not cordic.is_avilable());
	q15_t cordic_result = cordic.read_ans();

	constexpr float enc_to_mm = 8.0f*M_PI/static_cast<float>(1<<12);
	constexpr float q15rad_to_mm = 30.0f / static_cast<float>(0xFFFF);

  q31_t i = 0;

  target_mm = 0.0f;
  ::position_pid.set_limit(0.0f);
  while(HAL_GPIO_ReadPin(SW_GPIO_Port,SW_Pin));
  printf("start\r\n");
  ::atan_enc_bias = atan_enc.update(cordic_result);
  ::position_pid.set_limit(0.3f);
  HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);

  auto my_axis = LSMParam::Shaft::Y;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(::can.rx_available()){
		  SabaneLib::CanFrame rx_frame;
		  ::can.rx(rx_frame);
		  auto reader = rx_frame.reader();

		  switch(static_cast<LSMParam::Config>(rx_frame.id)){
		  case LSMParam::Config::POS:
			  ::target_mm = data_select(my_axis,reader);
			  break;
		  case LSMParam::Config::POWER:
			  ::position_pid.set_limit(data_select(my_axis,reader));
			  break;
		  case LSMParam::Config::GAIN_P:
			  ::position_pid.set_p_gain(data_select(my_axis,reader));
			  break;
		  case LSMParam::Config::GAIN_I:
			  ::position_pid.set_i_gain(data_select(my_axis,reader));
			  break;
		  case LSMParam::Config::GAIN_D:
			  ::position_pid.set_d_gain(data_select(my_axis,reader));
			  break;
		  default:
			  break;
		  }

	  }else{
	  }
	  HAL_Delay(10);

//	  target_mm = 20.0f;
//	  HAL_Delay(500);
//	  target_mm = 70.0f;
//	  HAL_Delay(500);
//	  target_mm = 130.0f;
//	  HAL_Delay(500);

//	  enc.read_start();
//
//	  float enc_mm = (enc.get_angle()-enc_bias)*enc_to_mm;
//	  float atan_mm = (atan_enc.get_angle()-atan_enc_bias)*q15rad_to_mm;
//
//	  printf("%4.3f,%4.3f,%4.3f,%4.3f,%d,%d\r\n",
//			  enc_mm,
//			  atan_mm,
//			 enc_mm-atan_mm,
//			  SabaneLib::MotorMath::q15_to_rad(atan_enc.get_angle()) / M_PI,
//			  ::qcos,
//			  ::qsin
//	  );



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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 18;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
