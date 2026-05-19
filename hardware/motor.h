/* 电机模块：封装左右轮 PWM、编码器速度环和基于 IMU 航向的差速修正。 */
#ifndef __MOTOR_H__
#define __MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "PID.h"

/* 速度环和机械参数。编码器计数换算以减速箱输出轴为基准。 */
#define MOTOR_PI                            3.1415926f
#define MOTOR_SPEED_LOOP_HZ                 500U
#define MOTOR_PWM_MAX_COMPARE               4199.0f

#define MOTOR_ENCODER_PPR                   13.0f
#define MOTOR_ENCODER_QUADRATURE_FACTOR     4.0f
#define MOTOR_GEAR_RATIO                    30.0f
#define MOTOR_WHEEL_DIAMETER_M              0.065f
#define MOTOR_WHEEL_CIRCUMFERENCE_M         (MOTOR_PI * MOTOR_WHEEL_DIAMETER_M)
#define MOTOR_ENCODER_COUNTS_PER_OUTPUT_REV (MOTOR_ENCODER_PPR * MOTOR_ENCODER_QUADRATURE_FACTOR * MOTOR_GEAR_RATIO)
#define MOTOR_RPM_TO_MPS                    (MOTOR_WHEEL_CIRCUMFERENCE_M / 60.0f)
#define MOTOR_MPS_TO_RPM                    (60.0f / MOTOR_WHEEL_CIRCUMFERENCE_M)

/* Motor parameter note:
 * - Provided no-load output speed: 366 rpm
 * - With 65 mm wheel, theoretical no-load linear speed is about:
 *   pi * 0.065 * 366 / 60 ~= 1.25 m/s
 * - Rated output speed: 293 rpm -> about 1.00 m/s
 * - The current software target clamp below is still set to 400 rpm
 *   (about 1.36 m/s), which is higher than the datasheet no-load speed.
 */
#define MOTOR_MAX_TARGET_SPEED_RPM          400.0f
#define MOTOR_MAX_TARGET_SPEED_MPS          (MOTOR_MAX_TARGET_SPEED_RPM * MOTOR_RPM_TO_MPS)

/* 左右轮安装方向不同，用符号统一驱动方向和编码器方向。 */
#define MOTOR_LEFT_DRIVER_DIRECTION_SIGN    -1.0f
#define MOTOR_LEFT_ENCODER_DIRECTION_SIGN   1.0f
#define MOTOR_RIGHT_DRIVER_DIRECTION_SIGN   1.0f
#define MOTOR_RIGHT_ENCODER_DIRECTION_SIGN  -1.0f

/* 速度环 PID 参数以 PWM 输出为控制量，输入误差单位为 m/s。 */
#define MOTOR_PID_DEFAULT_KP                (10.0f * MOTOR_MPS_TO_RPM)
#define MOTOR_PID_DEFAULT_KI                (35.0f * MOTOR_MPS_TO_RPM)
#define MOTOR_PID_DEFAULT_KD                (0.0f * MOTOR_MPS_TO_RPM)
#define MOTOR_SPEED_LPF_ALPHA               0.25f

/* 航向环在 FreeRTOS 任务中运行，根据 yaw 误差叠加左右轮差速。 */
#define MOTOR_HEADING_LOOP_HZ               50U
#define MOTOR_HEADING_TASK_PERIOD_MS        (1000U / MOTOR_HEADING_LOOP_HZ)
#define MOTOR_HEADING_STOP_THRESHOLD_MPS    0.02f
#define MOTOR_HEADING_CORRECTION_LIMIT_MPS  0.30f
#define MOTOR_HEADING_PID_DEFAULT_KP        0.010f
#define MOTOR_HEADING_PID_DEFAULT_KI        0.0015f
#define MOTOR_HEADING_PID_DEFAULT_KD        0.0000f

typedef enum
{
  /* 电机数组索引，保持和 motor_table[] 顺序一致。 */
  MOTOR_ID_LEFT = 0,
  MOTOR_ID_RIGHT,
  MOTOR_ID_COUNT
} Motor_Id_t;

typedef struct
{
  /* 一个 Motor_t 对应一侧电机的执行器、传感器和速度 PID 状态。 */
  const char *name;
  TIM_HandleTypeDef *pwm_tim;
  uint32_t pwm_channel;
  TIM_HandleTypeDef *encoder_tim;
  GPIO_TypeDef *in1_gpio_port;
  uint16_t in1_pin;
  GPIO_TypeDef *in2_gpio_port;
  uint16_t in2_pin;
  float driver_direction_sign;
  float encoder_direction_sign;
  volatile float target_speed_mps;
  volatile float raw_speed_mps;
  volatile float current_speed_mps;
  volatile int32_t last_encoder_delta;
  int32_t previous_encoder_count;
  uint8_t initialized;
  PID_Controller_t speed_pid;
} Motor_t;

extern Motor_t g_left_motor;
extern Motor_t g_right_motor;

/* 初始化 PWM、编码器、速度 PID，并启动速度环定时器。 */
void Motor_Init(void);

/* TIM10 中断调用的 500 Hz 速度环入口。 */
void Motor_SpeedLoopISR(void);

/* 创建 50 Hz 航向保持任务。 */
void Motor_StartHeadingTask(void);

/* 电机对象访问接口。 */
Motor_t *Motor_GetLeft(void);
Motor_t *Motor_GetRight(void);
Motor_t *Motor_GetById(Motor_Id_t motor_id);

/* 整车速度和航向命令接口。 */
void  Motor_SetChassisSpeedMps(float speed_mps);
float Motor_GetChassisSpeedMps(void);
void  Motor_SetHeadingTargetDeg(float heading_target_deg);
float Motor_GetHeadingTargetDeg(void);
float Motor_GetHeadingCorrectionMps(void);

/* 单侧电机调试和控制接口。 */
void  Motor_SetTargetSpeedMpsForMotor(Motor_t *motor, float target_speed_mps);
float Motor_GetTargetSpeedMpsForMotor(const Motor_t *motor);
float Motor_GetCurrentSpeedMpsForMotor(const Motor_t *motor);
float Motor_GetRawSpeedMpsForMotor(const Motor_t *motor);
int32_t Motor_GetLastEncoderDeltaForMotor(const Motor_t *motor);
PID_Controller_t *Motor_GetSpeedPidForMotor(Motor_t *motor);

/* 兼容旧代码的单电机/整车简化接口。 */
void Motor_SetTargetSpeedMps(float target_speed_mps);
float Motor_GetTargetSpeedMps(void);
float Motor_GetCurrentSpeedMps(void);
float Motor_GetRawSpeedMps(void);
int32_t Motor_GetLastEncoderDelta(void);
PID_Controller_t *Motor_GetSpeedPid(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H__ */
