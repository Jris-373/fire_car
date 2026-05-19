/* 电机模块实现：
 * - TIM4 输出左右轮 PWM；
 * - TIM3/TIM8 读取左右编码器；
 * - TIM10 中断执行速度环；
 * - FreeRTOS 航向任务根据 JY61P yaw 做左右轮差速修正。
 */
#include "motor.h"

#include <math.h>

#include "cmsis_os.h"
#include "gpio.h"
#include "JY61P.h"
#include "task.h"
#include "tim.h"

Motor_t g_left_motor = {
  .name = "left",
  .pwm_tim = &htim4,
  .pwm_channel = TIM_CHANNEL_1,
  .encoder_tim = &htim3,
  .in1_gpio_port = AIN1_GPIO_Port,
  .in1_pin = AIN1_Pin,
  .in2_gpio_port = AIN2_GPIO_Port,
  .in2_pin = AIN2_Pin,
  .driver_direction_sign = MOTOR_LEFT_DRIVER_DIRECTION_SIGN,
  .encoder_direction_sign = MOTOR_LEFT_ENCODER_DIRECTION_SIGN,
};

Motor_t g_right_motor = {
  .name = "right",
  .pwm_tim = &htim4,
  .pwm_channel = TIM_CHANNEL_2,
  .encoder_tim = &htim8,
  .in1_gpio_port = BIN1_GPIO_Port,
  .in1_pin = BIN1_Pin,
  .in2_gpio_port = BIN2_GPIO_Port,
  .in2_pin = BIN2_Pin,
  .driver_direction_sign = MOTOR_RIGHT_DRIVER_DIRECTION_SIGN,
  .encoder_direction_sign = MOTOR_RIGHT_ENCODER_DIRECTION_SIGN,
};

static Motor_t *const motor_table[MOTOR_ID_COUNT] = {
  &g_left_motor,
  &g_right_motor,
};

static volatile float motor_chassis_speed_mps = 0.0f;
static volatile float motor_heading_target_deg = 0.0f;
static volatile float motor_heading_correction_mps = 0.0f;
static osThreadId_t motorHeadingTaskHandle = NULL;
static const osThreadAttr_t motorHeadingTaskAttributes = {
  .name = "headingTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t)osPriorityHigh,
};
static PID_Controller_t motor_heading_pid;

static void Motor_HeadingTask(void *argument);
static float Motor_ClampTargetSpeed(float target_speed_mps);

/* 将角度误差约束到 -180~180 度，避免航向跨圈时误差突变。 */
static float Motor_NormalizeAngleDeg(float angle_deg)
{
  while (angle_deg > 180.0f)
  {
    angle_deg -= 360.0f;
  }

  while (angle_deg < -180.0f)
  {
    angle_deg += 360.0f;
  }

  return angle_deg;
}

/* 对车体速度命令做统一限幅，避免方向环叠加后轮速过大。 */
static float Motor_ClampChassisSpeed(float speed_mps)
{
  return Motor_ClampTargetSpeed(speed_mps);
}

/* 将目标轮速限制在当前配置允许的安全范围内。 */
static float Motor_ClampTargetSpeed(float target_speed_mps)
{
  if (target_speed_mps > MOTOR_MAX_TARGET_SPEED_MPS)
  {
    return MOTOR_MAX_TARGET_SPEED_MPS;
  }

  if (target_speed_mps < -MOTOR_MAX_TARGET_SPEED_MPS)
  {
    return -MOTOR_MAX_TARGET_SPEED_MPS;
  }

  return target_speed_mps;
}

/* 为指定电机对象设置 H 桥方向控制引脚电平。 */
static void Motor_WriteDirectionPins(const Motor_t *motor, GPIO_PinState in1_state, GPIO_PinState in2_state)
{
  if ((motor == NULL) || (motor->in1_gpio_port == NULL) || (motor->in2_gpio_port == NULL))
  {
    return;
  }

  HAL_GPIO_WritePin(motor->in1_gpio_port, motor->in1_pin, in1_state);
  HAL_GPIO_WritePin(motor->in2_gpio_port, motor->in2_pin, in2_state);
}

/* 清除方向脚并将 PWM 占空比置零，使电机停止输出。 */
static void Motor_StopDriverOutput(Motor_t *motor)
{
  if (motor == NULL)
  {
    return;
  }

  Motor_WriteDirectionPins(motor, GPIO_PIN_RESET, GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(motor->pwm_tim, motor->pwm_channel, 0U);
}

/* 对编码器测得的速度做低通滤波，减小控制环对噪声的敏感度。 */
static float Motor_ApplySpeedLowPass(Motor_t *motor, float raw_speed_mps)
{
  if (motor == NULL)
  {
    return 0.0f;
  }

  motor->current_speed_mps += MOTOR_SPEED_LPF_ALPHA * (raw_speed_mps - motor->current_speed_mps);
  return motor->current_speed_mps;
}

/* 将 PID 输出转换为电机转向和对应的 PWM 比较值。 */
static void Motor_ApplyDriverOutput(Motor_t *motor, float control_output)
{
  float signed_output;
  uint32_t compare_value;

  if (motor == NULL)
  {
    return;
  }

  signed_output = control_output * motor->driver_direction_sign;
  compare_value = (uint32_t)fabsf(signed_output);

  if (compare_value > (uint32_t)MOTOR_PWM_MAX_COMPARE)
  {
    compare_value = (uint32_t)MOTOR_PWM_MAX_COMPARE;
  }

  if (compare_value == 0U)
  {
    Motor_StopDriverOutput(motor);
    return;
  }

  if (signed_output >= 0.0f)
  {
    Motor_WriteDirectionPins(motor, GPIO_PIN_SET, GPIO_PIN_RESET);
  }
  else
  {
    Motor_WriteDirectionPins(motor, GPIO_PIN_RESET, GPIO_PIN_SET);
  }

  __HAL_TIM_SET_COMPARE(motor->pwm_tim, motor->pwm_channel, compare_value);
}

/* 初始化单个电机对象，包括 PID、PWM 输出和编码器计数器。 */
static void Motor_InitInstance(Motor_t *motor)
{
  if (motor == NULL)
  {
    return;
  }

  PID_Init(&motor->speed_pid,
           MOTOR_PID_DEFAULT_KP,
           MOTOR_PID_DEFAULT_KI,
           MOTOR_PID_DEFAULT_KD,
           1.0f / (float)MOTOR_SPEED_LOOP_HZ,
           -MOTOR_PWM_MAX_COMPARE,
           MOTOR_PWM_MAX_COMPARE);

  Motor_StopDriverOutput(motor);

  if (HAL_TIM_PWM_Start(motor->pwm_tim, motor->pwm_channel) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_TIM_SET_COMPARE(motor->pwm_tim, motor->pwm_channel, 0U);

  if (HAL_TIM_Encoder_Start(motor->encoder_tim, TIM_CHANNEL_ALL) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_TIM_SET_COUNTER(motor->encoder_tim, 0U);
  motor->previous_encoder_count = 0;
  motor->target_speed_mps = 0.0f;
  motor->raw_speed_mps = 0.0f;
  motor->current_speed_mps = 0.0f;
  motor->last_encoder_delta = 0;
  motor->initialized = 1U;
}

/* 基于该电机的编码器反馈执行一次速度环更新。 */
static void Motor_UpdateInstance(Motor_t *motor)
{
  int32_t current_encoder_count;
  int32_t delta_count;
  float measured_speed_mps;
  float filtered_speed_mps;
  float control_output;

  if ((motor == NULL) || (motor->initialized == 0U))
  {
    return;
  }

  current_encoder_count = (int32_t)__HAL_TIM_GET_COUNTER(motor->encoder_tim);
  delta_count = (int32_t)(int16_t)(current_encoder_count - motor->previous_encoder_count);
  motor->previous_encoder_count = current_encoder_count;
  motor->last_encoder_delta = delta_count;

  measured_speed_mps = ((float)delta_count * (float)MOTOR_SPEED_LOOP_HZ * MOTOR_WHEEL_CIRCUMFERENCE_M) /
                       MOTOR_ENCODER_COUNTS_PER_OUTPUT_REV;
  measured_speed_mps *= motor->encoder_direction_sign;
  motor->raw_speed_mps = measured_speed_mps;
  filtered_speed_mps = Motor_ApplySpeedLowPass(motor, measured_speed_mps);

  control_output = PID_Update(&motor->speed_pid, motor->target_speed_mps, filtered_speed_mps);
  Motor_ApplyDriverOutput(motor, control_output);
}

/* 初始化左右两个电机对象，并启动周期性的速度环定时器。 */
void Motor_Init(void)
{
  uint32_t motor_index;

  HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);

  PID_Init(&motor_heading_pid,
           MOTOR_HEADING_PID_DEFAULT_KP,
           MOTOR_HEADING_PID_DEFAULT_KI,
           MOTOR_HEADING_PID_DEFAULT_KD,
           1.0f / (float)MOTOR_HEADING_LOOP_HZ,
           -MOTOR_HEADING_CORRECTION_LIMIT_MPS,
           MOTOR_HEADING_CORRECTION_LIMIT_MPS);
  PID_SetIntegralLimits(&motor_heading_pid,
                        -MOTOR_HEADING_CORRECTION_LIMIT_MPS,
                        MOTOR_HEADING_CORRECTION_LIMIT_MPS);

  for (motor_index = 0U; motor_index < (uint32_t)MOTOR_ID_COUNT; ++motor_index)
  {
    Motor_InitInstance(motor_table[motor_index]);
  }

  if (HAL_TIM_Base_Start_IT(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
}

/* 在 TIM10 中断中调用，依次更新左右电机的速度环。 */
void Motor_SpeedLoopISR(void)
{
  uint32_t motor_index;

  for (motor_index = 0U; motor_index < (uint32_t)MOTOR_ID_COUNT; ++motor_index)
  {
    Motor_UpdateInstance(motor_table[motor_index]);
  }
}

/* 方向环任务以 50 Hz 运行，根据航向误差为左右轮叠加差速修正量。 */
void Motor_StartHeadingTask(void)
{
  if (motorHeadingTaskHandle == NULL)
  {
    motorHeadingTaskHandle = osThreadNew(Motor_HeadingTask, NULL, &motorHeadingTaskAttributes);
  }
}

/* 返回左电机对象，便于上层直接访问其状态。 */
Motor_t *Motor_GetLeft(void)
{
  return &g_left_motor;
}

/* 返回右电机对象，便于上层直接访问其状态。 */
Motor_t *Motor_GetRight(void)
{
  return &g_right_motor;
}

/* 根据电机编号返回对应的电机对象指针。 */
Motor_t *Motor_GetById(Motor_Id_t motor_id)
{
  if (motor_id >= MOTOR_ID_COUNT)
  {
    return NULL;
  }

  return motor_table[motor_id];
}

/* 设置整车的基础前进速度，方向环会在此基础上叠加左右差速修正。 */
void Motor_SetChassisSpeedMps(float speed_mps)
{
  motor_chassis_speed_mps = Motor_ClampChassisSpeed(speed_mps);
}

/* 返回当前整车基础速度命令。 */
float Motor_GetChassisSpeedMps(void)
{
  return motor_chassis_speed_mps;
}

/* 设置方向环的目标航向角，内部会自动归一化到 -180~180 度。 */
void Motor_SetHeadingTargetDeg(float heading_target_deg)
{
  motor_heading_target_deg = Motor_NormalizeAngleDeg(heading_target_deg);
}

/* 返回方向环当前使用的目标航向角。 */
float Motor_GetHeadingTargetDeg(void)
{
  return motor_heading_target_deg;
}

/* 返回方向环最近一次计算得到的左右轮差速修正量。 */
float Motor_GetHeadingCorrectionMps(void)
{
  return motor_heading_correction_mps;
}

/* 为指定电机设置目标速度，并先做统一的限幅处理。 */
void Motor_SetTargetSpeedMpsForMotor(Motor_t *motor, float target_speed_mps)
{
  if (motor == NULL)
  {
    return;
  }

  motor->target_speed_mps = Motor_ClampTargetSpeed(target_speed_mps);
}

/* 读取指定电机当前设定的目标速度。 */
float Motor_GetTargetSpeedMpsForMotor(const Motor_t *motor)
{
  if (motor == NULL)
  {
    return 0.0f;
  }

  return motor->target_speed_mps;
}

/* 读取指定电机当前用于控制环的滤波后速度。 */
float Motor_GetCurrentSpeedMpsForMotor(const Motor_t *motor)
{
  if (motor == NULL)
  {
    return 0.0f;
  }

  return motor->current_speed_mps;
}

/* 读取指定电机由编码器直接换算得到的原始速度。 */
float Motor_GetRawSpeedMpsForMotor(const Motor_t *motor)
{
  if (motor == NULL)
  {
    return 0.0f;
  }

  return motor->raw_speed_mps;
}

/* 获取速度环最近一次采样得到的编码器增量。 */
int32_t Motor_GetLastEncoderDeltaForMotor(const Motor_t *motor)
{
  if (motor == NULL)
  {
    return 0;
  }

  return motor->last_encoder_delta;
}

/* 返回指定电机的 PID 对象，便于调参或调试。 */
PID_Controller_t *Motor_GetSpeedPidForMotor(Motor_t *motor)
{
  if (motor == NULL)
  {
    return NULL;
  }

  return &motor->speed_pid;
}

/* 兼容旧接口，将单一速度命令解释为整车基础速度命令。 */
void Motor_SetTargetSpeedMps(float target_speed_mps)
{
  Motor_SetChassisSpeedMps(target_speed_mps);
}

/* 兼容旧接口，读取整车基础速度命令。 */
float Motor_GetTargetSpeedMps(void)
{
  return Motor_GetChassisSpeedMps();
}

/* 兼容旧接口，返回左右轮滤波后速度的平均值，表示整车线速度。 */
float Motor_GetCurrentSpeedMps(void)
{
  return (Motor_GetCurrentSpeedMpsForMotor(&g_left_motor) +
          Motor_GetCurrentSpeedMpsForMotor(&g_right_motor)) * 0.5f;
}

/* 兼容旧接口，返回左右轮原始速度的平均值。 */
float Motor_GetRawSpeedMps(void)
{
  return (Motor_GetRawSpeedMpsForMotor(&g_left_motor) +
          Motor_GetRawSpeedMpsForMotor(&g_right_motor)) * 0.5f;
}

/* 兼容旧接口，读取左电机编码器增量。 */
int32_t Motor_GetLastEncoderDelta(void)
{
  return Motor_GetLastEncoderDeltaForMotor(&g_left_motor);
}

/* 兼容旧接口，获取左电机 PID 对象。 */
PID_Controller_t *Motor_GetSpeedPid(void)
{
  return Motor_GetSpeedPidForMotor(&g_left_motor);
}

static void Motor_HeadingTask(void *argument)
{
  TickType_t last_wake_time = xTaskGetTickCount();
  float current_yaw_deg;
  float heading_error_deg;
  float heading_correction_mps;
  float chassis_speed_mps;
  const JY61P_Data_t *imu_data;

  (void)argument;

  for (;;)
  {
    imu_data = JY61P_GetLatestData();
    chassis_speed_mps = motor_chassis_speed_mps;

    if (imu_data->last_update_tick == 0U)
    {
      Motor_SetTargetSpeedMpsForMotor(&g_left_motor, 0.0f);
      Motor_SetTargetSpeedMpsForMotor(&g_right_motor, 0.0f);
      motor_heading_correction_mps = 0.0f;
      PID_Reset(&motor_heading_pid);
      vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(MOTOR_HEADING_TASK_PERIOD_MS));
      continue;
    }

    current_yaw_deg = Motor_NormalizeAngleDeg(imu_data->angle_deg.z);

    if (fabsf(chassis_speed_mps) < MOTOR_HEADING_STOP_THRESHOLD_MPS)
    {
      motor_heading_target_deg = current_yaw_deg;
      motor_heading_correction_mps = 0.0f;
      PID_Reset(&motor_heading_pid);
      Motor_SetTargetSpeedMpsForMotor(&g_left_motor, 0.0f);
      Motor_SetTargetSpeedMpsForMotor(&g_right_motor, 0.0f);
      vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(MOTOR_HEADING_TASK_PERIOD_MS));
      continue;
    }

    heading_error_deg = Motor_NormalizeAngleDeg(motor_heading_target_deg - current_yaw_deg);
    heading_correction_mps = PID_Update(&motor_heading_pid, 0.0f, -heading_error_deg);
    motor_heading_correction_mps = heading_correction_mps;

    Motor_SetTargetSpeedMpsForMotor(&g_left_motor, chassis_speed_mps - heading_correction_mps);
    Motor_SetTargetSpeedMpsForMotor(&g_right_motor, chassis_speed_mps + heading_correction_mps);

    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(MOTOR_HEADING_TASK_PERIOD_MS));
  }
}
