/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : ZX-D30 motor test firmware.
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"

#include <stdint.h>
#include <string.h>

#define ZXD30_UART_BAUD             9600U
#define ZXD30_RUN_STARTUP_AT        0U
#define ZXD30_ENABLE_BLE_AT_BOOT    1U
#define ZXD30_DEVICE_NAME           "ZX_MOTOR_TEST"
#define UART_RX_QUEUE_SIZE          16U

#define MOTOR_GEAR_L_DUTY           18U
#define MOTOR_GEAR_1_DUTY           45U
#define MOTOR_GEAR_2_DUTY           75U
#define MOTOR_GEAR_3_DUTY           100U
#define MOTOR_GEAR_4_DUTY           20U
#define MOTOR_GEAR_U_START_DUTY     70U
#define MOTOR_GEAR_U_TARGET_DUTY    100U
#define MOTOR_RAMP_STEP_PERCENT     1U
#define MOTOR_RAMP_TOTAL_TIME_MS    800U

typedef enum
{
  MOTOR_ID_M1 = 0,
  MOTOR_ID_M2 = 1
} MotorId_t;

typedef enum
{
  MOTOR_DIR_FORWARD = 0,
  MOTOR_DIR_REVERSE = 1
} MotorDirection_t;

typedef struct
{
  TIM_HandleTypeDef *in1_timer;
  uint32_t in1_channel;
  TIM_HandleTypeDef *in2_timer;
  uint32_t in2_channel;
} MotorBridge_t;

static volatile uint8_t g_rx_byte = 0;
static volatile uint8_t g_accept_user_command = 0;
static volatile uint8_t g_rx_queue[UART_RX_QUEUE_SIZE];
static volatile uint8_t g_rx_queue_head = 0U;
static volatile uint8_t g_rx_queue_tail = 0U;
static volatile uint8_t g_rx_queue_overflow = 0U;
static uint8_t g_current_duty_percent = 0U;
static uint8_t g_target_duty_percent = 0U;
static uint8_t g_ramp_active = 0U;
static uint8_t g_ramp_step_percent = MOTOR_RAMP_STEP_PERCENT;
static uint32_t g_ramp_period_ms = 1U;
static uint32_t g_ramp_tick = 0U;
static uint8_t g_report_duty_when_target_reached = 0U;
static MotorDirection_t g_m1_direction = MOTOR_DIR_FORWARD;

static const MotorBridge_t g_motor_bridge[2] =
{
  {&htim2, TIM_CHANNEL_2, &htim14, TIM_CHANNEL_1},
  {&htim2, TIM_CHANNEL_3, &htim2, TIM_CHANNEL_4}
};

static void SystemClock_Config(void);
static void ZXD30_SendString(const char *text);
static void ZXD30_SendDutyPercent(uint8_t percent);
static void ZXD30_SendAT(const char *command, uint32_t delay_ms);
static void ZXD30_StartupConfig(void);
static uint16_t DutyFromPercent(uint8_t percent);
static uint32_t RampPeriodFromDelta(uint8_t current_percent, uint8_t target_percent);
static void Motor_SetSingleDirection(MotorId_t motor, MotorDirection_t direction, uint16_t duty);
static void Motor_SetPairDirection(MotorDirection_t m1_direction);
static void ApplyPairDutyPercent(uint8_t percent);
static void StopMotors(void);
static void StartDutyRamp(uint8_t target_percent, uint8_t step_percent, uint32_t period_ms);
static void StartSmoothDutyRamp(uint8_t target_percent);
static void StartRelativeDutyRampDown(uint8_t delta_percent);
static void StartSmoothGearU(void);
static void ServiceDutyRamp(void);
static void ProcessBluetoothByte(uint8_t data);
static void QueueBluetoothByte(uint8_t data);
static uint8_t TryDequeueBluetoothByte(uint8_t *data);
static void RestartBluetoothReceive(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM14_Init();
  MX_USART1_UART_Init();

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);

  StopMotors();
  ZXD30_StartupConfig();
  RestartBluetoothReceive();
  g_accept_user_command = 1;

  while (1)
  {
    uint8_t data;

    if (TryDequeueBluetoothByte(&data) != 0U)
    {
      ProcessBluetoothByte(data);
    }

    ServiceDutyRamp();
  }
}

static void ZXD30_SendString(const char *text)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)strlen(text), HAL_MAX_DELAY);
}

static void ZXD30_SendDutyPercent(uint8_t percent)
{
  char text[13];
  uint8_t index = 0U;

  text[index++] = 'd';
  text[index++] = 'u';
  text[index++] = 't';
  text[index++] = 'y';
  text[index++] = '=';

  if (percent >= 100U)
  {
    text[index++] = '1';
    text[index++] = '0';
    text[index++] = '0';
  }
  else if (percent >= 10U)
  {
    text[index++] = (char)('0' + (percent / 10U));
    text[index++] = (char)('0' + (percent % 10U));
  }
  else
  {
    text[index++] = (char)('0' + percent);
  }

  text[index++] = '%';
  text[index++] = '\r';
  text[index++] = '\n';
  text[index] = '\0';

  ZXD30_SendString(text);
}

static void ZXD30_SendAT(const char *command, uint32_t delay_ms)
{
  ZXD30_SendString(command);
  ZXD30_SendString("\r\n");
  HAL_Delay(delay_ms);
}

static void ZXD30_StartupConfig(void)
{
  HAL_Delay(800);

#if ZXD30_RUN_STARTUP_AT
  ZXD30_SendAT("AT", 100);
  ZXD30_SendAT("AT+NAME=" ZXD30_DEVICE_NAME, 200);
  ZXD30_SendAT("AT+BAUD=3", 250);
#if ZXD30_ENABLE_BLE_AT_BOOT
  ZXD30_SendAT("AT+ENBLE=1", 250);
#endif
  HAL_Delay(800);
#endif
}

static uint16_t DutyFromPercent(uint8_t percent)
{
  uint32_t period = (uint32_t)htim2.Init.Period + 1U;

  if (percent >= 100U)
  {
    return (uint16_t)period;
  }

  return (uint16_t)((period * percent) / 100U);
}

static void Motor_SetSingleDirection(MotorId_t motor, MotorDirection_t direction, uint16_t duty)
{
  const MotorBridge_t *bridge = &g_motor_bridge[(uint32_t)motor];

  if (duty == 0U)
  {
    __HAL_TIM_SET_COMPARE(bridge->in1_timer, bridge->in1_channel, 0);
    __HAL_TIM_SET_COMPARE(bridge->in2_timer, bridge->in2_channel, 0);
    return;
  }

  if (direction == MOTOR_DIR_FORWARD)
  {
    __HAL_TIM_SET_COMPARE(bridge->in1_timer, bridge->in1_channel, duty);
    __HAL_TIM_SET_COMPARE(bridge->in2_timer, bridge->in2_channel, 0);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(bridge->in1_timer, bridge->in1_channel, 0);
    __HAL_TIM_SET_COMPARE(bridge->in2_timer, bridge->in2_channel, duty);
  }
}

static void Motor_SetPairDirection(MotorDirection_t m1_direction)
{
  MotorDirection_t m2_direction;

  g_m1_direction = m1_direction;
  m2_direction = (m1_direction == MOTOR_DIR_FORWARD) ? MOTOR_DIR_REVERSE : MOTOR_DIR_FORWARD;

  Motor_SetSingleDirection(MOTOR_ID_M1, g_m1_direction, DutyFromPercent(g_current_duty_percent));
  Motor_SetSingleDirection(MOTOR_ID_M2, m2_direction, DutyFromPercent(g_current_duty_percent));
}

static void ApplyPairDutyPercent(uint8_t percent)
{
  g_current_duty_percent = percent;
  Motor_SetPairDirection(g_m1_direction);
}

static void StopMotors(void)
{
  g_ramp_active = 0U;
  g_target_duty_percent = 0U;
  g_current_duty_percent = 0U;
  g_report_duty_when_target_reached = 0U;
  Motor_SetSingleDirection(MOTOR_ID_M1, MOTOR_DIR_FORWARD, 0U);
  Motor_SetSingleDirection(MOTOR_ID_M2, MOTOR_DIR_FORWARD, 0U);
}

static void StartDutyRamp(uint8_t target_percent, uint8_t step_percent, uint32_t period_ms)
{
  g_target_duty_percent = target_percent;
  g_ramp_step_percent = (step_percent == 0U) ? 1U : step_percent;
  g_ramp_period_ms = (period_ms == 0U) ? 1U : period_ms;
  g_ramp_tick = HAL_GetTick();
  g_ramp_active = (g_current_duty_percent != g_target_duty_percent) ? 1U : 0U;

  if (g_ramp_active == 0U)
  {
    ApplyPairDutyPercent(g_target_duty_percent);
  }
}

static uint32_t RampPeriodFromDelta(uint8_t current_percent, uint8_t target_percent)
{
  uint8_t delta;

  if (current_percent >= target_percent)
  {
    delta = (uint8_t)(current_percent - target_percent);
  }
  else
  {
    delta = (uint8_t)(target_percent - current_percent);
  }

  if (delta == 0U)
  {
    return 1U;
  }

  return (MOTOR_RAMP_TOTAL_TIME_MS + delta - 1U) / delta;
}

static void StartSmoothDutyRamp(uint8_t target_percent)
{
  g_report_duty_when_target_reached = 0U;

  if (g_current_duty_percent != target_percent)
  {
    g_report_duty_when_target_reached = 1U;
  }

  StartDutyRamp(target_percent, MOTOR_RAMP_STEP_PERCENT, RampPeriodFromDelta(g_current_duty_percent, target_percent));
}

static void StartRelativeDutyRampDown(uint8_t delta_percent)
{
  uint8_t base_percent = (g_ramp_active != 0U) ? g_target_duty_percent : g_current_duty_percent;
  uint8_t target_percent;

  if (base_percent > delta_percent)
  {
    target_percent = (uint8_t)(base_percent - delta_percent);
  }
  else
  {
    target_percent = 0U;
  }

  StartSmoothDutyRamp(target_percent);
}

static void StartSmoothGearU(void)
{
  if (g_current_duty_percent < MOTOR_GEAR_U_START_DUTY)
  {
    ApplyPairDutyPercent(MOTOR_GEAR_U_START_DUTY);
  }

  g_report_duty_when_target_reached = 1U;
  StartDutyRamp(MOTOR_GEAR_U_TARGET_DUTY,
                MOTOR_RAMP_STEP_PERCENT,
                RampPeriodFromDelta(g_current_duty_percent, MOTOR_GEAR_U_TARGET_DUTY));
}

static void ServiceDutyRamp(void)
{
  uint32_t now;

  if (g_ramp_active == 0U)
  {
    return;
  }

  now = HAL_GetTick();

  if ((uint32_t)(now - g_ramp_tick) < g_ramp_period_ms)
  {
    return;
  }
  g_ramp_tick = now;

  if (g_current_duty_percent == g_target_duty_percent)
  {
    g_ramp_active = 0U;
  }
  else if (g_current_duty_percent < g_target_duty_percent)
  {
    uint8_t next = (uint8_t)(g_current_duty_percent + g_ramp_step_percent);

    if (next > g_target_duty_percent)
    {
      next = g_target_duty_percent;
    }
    ApplyPairDutyPercent(next);
  }
  else
  {
    uint8_t next;

    if (g_current_duty_percent > g_ramp_step_percent)
    {
      next = (uint8_t)(g_current_duty_percent - g_ramp_step_percent);
    }
    else
    {
      next = 0U;
    }

    if (next < g_target_duty_percent)
    {
      next = g_target_duty_percent;
    }

    ApplyPairDutyPercent(next);
  }

  if ((g_current_duty_percent == g_target_duty_percent) && (g_report_duty_when_target_reached != 0U))
  {
    g_ramp_active = 0U;
    g_report_duty_when_target_reached = 0U;
    ZXD30_SendDutyPercent(g_current_duty_percent);
  }
}

static void ProcessBluetoothByte(uint8_t data)
{
  switch (data)
  {
    case 'L':
    case 'l':
      StartSmoothDutyRamp(MOTOR_GEAR_L_DUTY);
      break;

    case '1':
      StartSmoothDutyRamp(MOTOR_GEAR_1_DUTY);
      break;

    case '2':
      StartSmoothDutyRamp(MOTOR_GEAR_2_DUTY);
      break;

    case '3':
      StartSmoothDutyRamp(MOTOR_GEAR_3_DUTY);
      break;

    case '4':
      StartSmoothDutyRamp(MOTOR_GEAR_4_DUTY);
      break;

    case 'U':
    case 'u':
      StartSmoothGearU();
      break;

    case '-':
      StartRelativeDutyRampDown(1U);
      break;

    case '0':
    case 's':
    case 'S':
      StartSmoothDutyRamp(0U);
      break;

    case 'F':
    case 'f':
      g_ramp_active = 0U;
      g_target_duty_percent = g_current_duty_percent;
      g_report_duty_when_target_reached = 0U;
      Motor_SetPairDirection(MOTOR_DIR_FORWARD);
      ZXD30_SendString("dir: M1 forward, M2 reverse\r\n");
      break;

    case 'R':
    case 'r':
      g_ramp_active = 0U;
      g_target_duty_percent = g_current_duty_percent;
      g_report_duty_when_target_reached = 0U;
      Motor_SetPairDirection(MOTOR_DIR_REVERSE);
      ZXD30_SendString("dir: M1 reverse, M2 forward\r\n");
      break;

    case '\r':
    case '\n':
    case ' ':
      break;

    default:
      ZXD30_SendString("cmd: L/1/2/3/4/U/-/0/s/f/r\r\n");
      break;
  }
}

static void QueueBluetoothByte(uint8_t data)
{
  uint8_t next_head = (uint8_t)((g_rx_queue_head + 1U) % UART_RX_QUEUE_SIZE);

  if (next_head == g_rx_queue_tail)
  {
    g_rx_queue_overflow = 1U;
    return;
  }

  g_rx_queue[g_rx_queue_head] = data;
  g_rx_queue_head = next_head;
}

static uint8_t TryDequeueBluetoothByte(uint8_t *data)
{
  if (g_rx_queue_tail == g_rx_queue_head)
  {
    return 0U;
  }

  *data = g_rx_queue[g_rx_queue_tail];
  g_rx_queue_tail = (uint8_t)((g_rx_queue_tail + 1U) % UART_RX_QUEUE_SIZE);
  return 1U;
}

static void RestartBluetoothReceive(void)
{
  if (HAL_UART_Receive_IT(&huart1, (uint8_t *)&g_rx_byte, 1) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1)
  {
    if (g_accept_user_command != 0U)
    {
      QueueBluetoothByte(g_rx_byte);
    }

    RestartBluetoothReceive();
  }
}

static void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
