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
#include "can.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_delay.h"
#include "INA266.h"
#include "user_can.h"
#include "PID.h"
#include "user_can.h"
#include "PowerConrtol.h"
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define PowerControl ENABLE
#define MotorControl ENABLE
#define Detect__Mode ENABLE

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
Robermaster robermaster = RobermasterC;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float current = 0;
float voltage = 0;
float power = 0;

uint16_t detaTime;
uint16_t lastTime;
uint16_t currentTime;

int16_t setV[MOTOR_NUM];
int16_t setV_SLope[MOTOR_NUM];
int16_t setCurrent[MOTOR_NUM];
uint32_t setAngle[MOTOR_NUM] = {3000, 3000};
PID pid_V[MOTOR_NUM];
PID BufferPid;
CascadePID pid_ANGLE[MOTOR_NUM];
uint16_t ID[MOTOR_NUM];
AutoFox_INA226 ina226;
SingleMotor motors[MOTOR_NUM];
int state = 0;
extern CONTROL_MODE control_mode;
extern fp32 RealBuffer;
fp32 powerPredict;

float paramVector[3][1] = {0};
float transVector[3][3] = {0.000025, 0, 0,
                           0, 0.000025, 0,
                           0, 0, 0.000025};
float kainVector[3][1] = {0};
float Xsample[3][1] = {0};
float Ysample[1][1] = {0};
float lamdaPowerAutoUpdate = 0.9999;

float TRANS_M_X[3][1] = {0};
float XT[1][3] = {0};
float XT_M_TRANS[1][3] = {0};
float XT_M_TRANS_M_X[1][1] = {0};
float XT_M_PARAM[1][1] = {0};
float deta_PARAM[3][1] = {0};

float lastParamVector[3][1] = {0};
float lastTrans[3][3] = {0};

float KAIN_M_XT[3][3] = {0};
float KAIN_M_XT_M_TRANS[3][3] = {0};

arm_matrix_instance_f32 paramMatrix;
arm_matrix_instance_f32 transMatrix;
arm_matrix_instance_f32 kainMatrix;
arm_matrix_instance_f32 XsampleMatrix;
arm_matrix_instance_f32 YsampleMatrix;
arm_matrix_instance_f32 TRANS_M_XMatrix;
arm_matrix_instance_f32 XTMatrix;
arm_matrix_instance_f32 XT_M_TRANSMatrix;
arm_matrix_instance_f32 XT_M_TRANS_M_XMatrix;
arm_matrix_instance_f32 XT_M_PARAMMatrix;
arm_matrix_instance_f32 deta_PARAMMatrix;
arm_matrix_instance_f32 lastParamMatrix;
arm_matrix_instance_f32 lastTransMatrix;
arm_matrix_instance_f32 KAIN_M_XTMatrix;
arm_matrix_instance_f32 KAIN_M_XT_M_TRANSMatrix;
extern float a;
void PowerControl_AutoUpdateParamInit()
{
  paramVector[0][0] = 1.534e-07;
  paramVector[1][0] = a;
  paramVector[2][0] = 0.78;
  arm_mat_init_f32(&paramMatrix, 3, 1, (float *)paramVector);
  arm_mat_init_f32(&transMatrix, 3, 3, (float *)transVector);
  arm_mat_init_f32(&kainMatrix, 3, 1, (float *)kainVector);
  arm_mat_init_f32(&XsampleMatrix, 3, 1, (float *)Xsample);
  arm_mat_init_f32(&YsampleMatrix, 1, 1, (float *)Ysample);
  arm_mat_init_f32(&TRANS_M_XMatrix, 3, 1, (float *)TRANS_M_X);
  arm_mat_init_f32(&XTMatrix, 1, 3, (float *)XT);
  arm_mat_init_f32(&XT_M_TRANSMatrix, 1, 3, (float *)XT_M_TRANS);
  arm_mat_init_f32(&XT_M_TRANS_M_XMatrix, 1, 1, (float *)XT_M_TRANS_M_X);
  arm_mat_init_f32(&XT_M_PARAMMatrix, 1, 1, (float *)XT_M_PARAM);
  arm_mat_init_f32(&deta_PARAMMatrix, 3, 1, (float *)deta_PARAM);
  arm_mat_init_f32(&lastParamMatrix, 3, 1, (float *)lastParamVector);
  arm_mat_init_f32(&lastTransMatrix, 3, 3, (float *)lastTrans);
  arm_mat_init_f32(&KAIN_M_XTMatrix, 3, 3, (float *)KAIN_M_XT);
  arm_mat_init_f32(&KAIN_M_XT_M_TRANSMatrix, 3, 3, (float *)KAIN_M_XT_M_TRANS);
}
uint16_t measurePower = 0;
extern fp32 toque_coefficient; // (20/16384)*(0.3)*(187/3591)/9.55

void PowerControl_AutoUpdateParam() // 在功率控制后，发送电流前
{
  measurePower = power;
  Xsample[0][0] = motors[0].speed * motors[0].speed; // 电机速度平方和
  Xsample[1][0] = pid_V->output * pid_V->output;     // 电机电流平方和
  Xsample[2][0] = 1;                                 // 常数项
  Ysample[0][0] = power - setCurrent[0] * toque_coefficient * motors[0].speed;
  if (measurePower > 1)
  {
    arm_mat_trans_f32(&XsampleMatrix, &XTMatrix);
    arm_mat_mult_f32(&transMatrix, &XsampleMatrix, &TRANS_M_XMatrix);
    arm_mat_mult_f32(&XTMatrix, &transMatrix, &XT_M_TRANSMatrix);
    arm_mat_mult_f32(&XT_M_TRANSMatrix, &XsampleMatrix, &XT_M_TRANS_M_XMatrix);
    arm_mat_scale_f32(&TRANS_M_XMatrix,
                      1.0f / (1 + (XT_M_TRANS_M_X[0][0] / lamdaPowerAutoUpdate) / lamdaPowerAutoUpdate),
                      &kainMatrix);
		
    arm_mat_mult_f32(&XTMatrix, &paramMatrix, &XT_M_PARAMMatrix);
    arm_mat_scale_f32(&kainMatrix, Ysample[0][0] - XT_M_PARAM[0][0], &deta_PARAMMatrix);

    arm_mat_scale_f32(&paramMatrix, 1.0f, &lastParamMatrix);
    arm_mat_add_f32(&lastParamMatrix, &deta_PARAMMatrix, &paramMatrix);

    arm_mat_mult_f32(&kainMatrix, &XTMatrix, &KAIN_M_XTMatrix);
    arm_mat_mult_f32(&KAIN_M_XTMatrix, &transMatrix, &KAIN_M_XT_M_TRANSMatrix);
    arm_mat_scale_f32(&transMatrix, 1.0f, &lastTransMatrix);
    arm_mat_sub_f32(&lastTransMatrix, &KAIN_M_XT_M_TRANSMatrix, &transMatrix);
  }
  powerPredict = setCurrent[0] * toque_coefficient * motors[0].speed + paramVector[0][0] * Xsample[0][0] + paramVector[1][0] * Xsample[1][0] + paramVector[2][0] * Xsample[2][0];
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_CAN1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  delay_init();
  size_t inastatus;
  HAL_TIM_Base_Start_IT(&htim1);
  AutoFox_INA226_Constructor(&ina226); // Initialize the INA226
  inastatus = AutoFox_INA226_Init(&ina226, 0x40, 10e-3, 10.0f);
  CAN_Init();
  for (int i = 0; i < MOTOR_NUM; i++)
  {
    PID_Init(&pid_V[i], 8, 0, 1, 0, 16000);
    PID_Init(&pid_ANGLE[i].outer, 0.2, 0, 0, 0, 8000);
    PID_Init(&pid_ANGLE[i].inner, 6, 0, 4, 0, 16000);
  }
  PowerControl_AutoUpdateParamInit();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_Delay(1);
    lastTime = currentTime;
    currentTime = HAL_GetTick();
    detaTime = currentTime - lastTime;
    for (int i = 0; i < MOTOR_NUM; i++)
    {
      if (setV_SLope[i] - setV[i] > 5)
      {
        setV_SLope[i] -= 100;
      }
      else if (setV_SLope[i] - setV[i] < -5)
      {
        setV_SLope[i] += 100;
      }
      if (control_mode == CONTROL_ACCLERATE || control_mode == CONTROL_REAL_COMPETE)
      {
        PID_SingleCalc(&pid_V[i], setV_SLope[i], motors[i].speed);
        setCurrent[i] = pid_V[i].output;
      }
      else if (control_mode == CONTROL_POSITION)
      {
        PID_CascadeCalc_totalAngle(&pid_ANGLE[i], setAngle[i], motors[i].totalAngle, motors[i].speed);
        setCurrent[i] = pid_ANGLE[i].inner.output;
        setV[i] = pid_ANGLE[i].outer.output;
      }
    }

    //  voltage = AutoFox_INA226_GetBusVoltage_uV(&ina226) / 1000000.f;
    //  current = AutoFox_INA226_GetCurrent_uA(&ina226) / 1000000.f;
    power = AutoFox_INA226_GetPower_uW(&ina226) / 1000000.f;
    if (PowerControl)
    {
     // Chassis_PowerCtrl();
      PowerControl_AutoUpdateParam();
      // ????????? ????????????????????
    }
    if (Detect__Mode)
      Power_detect();
    if (MotorControl)
      USER_CAN_SetMotorCurrent(&hcan1, 0x200, setCurrent[0], setCurrent[1], 1600, 1600);
    if (robermaster == RobermasterA)
    {
      HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, GPIO_PIN_SET);
      if (RealBuffer > MAXBUFFER / 8.0 * 1)
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_RESET);
      if (RealBuffer > MAXBUFFER / 8.0 * 2)
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET);
      if (RealBuffer > MAXBUFFER / 8.0 * 3)
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_RESET);
      if (RealBuffer > MAXBUFFER / 8.0 * 4)
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_RESET);
      if (RealBuffer > MAXBUFFER / 8.0 * 5)
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_RESET);
      if (RealBuffer > MAXBUFFER / 8.0 * 6)
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);
      if (RealBuffer > MAXBUFFER / 8.0 * 7)
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_RESET);
      if (RealBuffer >= MAXBUFFER / 8.0 * 8)
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, GPIO_PIN_RESET);
    }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void control_real_compete(void);
void control_acclerate(void);
void control_position(void);
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

#ifdef USE_FULL_ASSERT
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
