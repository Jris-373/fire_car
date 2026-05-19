/* PID 模块实现：
 * 控制器内部保存积分项和上一拍误差，调用者需要按固定周期调用 PID_Update()。
 */
#include "PID.h"

/* 将数值限制在给定范围内，避免控制器输出或积分项失控。 */
static float PID_Clamp(float value, float min_value, float max_value)
{
  if (value > max_value)
  {
    return max_value;
  }

  if (value < min_value)
  {
    return min_value;
  }

  return value;
}

/* 在控制器投入运行前，初始化增益、采样周期和输出限幅。 */
void PID_Init(PID_Controller_t *pid,
              float kp,
              float ki,
              float kd,
              float sample_time_s,
              float output_min,
              float output_max)
{
  pid->sample_time_s = sample_time_s;
  pid->integral = 0.0f;
  pid->previous_error = 0.0f;

  PID_SetTunings(pid, kp, ki, kd);
  PID_SetOutputLimits(pid, output_min, output_max);
  PID_SetIntegralLimits(pid, output_min, output_max);
}

/* 在重新开始控制前清空积分项和上一拍误差。 */
void PID_Reset(PID_Controller_t *pid)
{
  pid->integral = 0.0f;
  pid->previous_error = 0.0f;
}

/* 运行过程中动态更新比例、积分、微分系数。 */
void PID_SetTunings(PID_Controller_t *pid, float kp, float ki, float kd)
{
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
}

/* 设置控制器最终输出的上下限，使其落在执行器允许范围内。 */
void PID_SetOutputLimits(PID_Controller_t *pid, float output_min, float output_max)
{
  pid->output_min = output_min;
  pid->output_max = output_max;
}

/* 单独限制积分项范围，减轻输出饱和时的积分饱和问题。 */
void PID_SetIntegralLimits(PID_Controller_t *pid, float integral_min, float integral_max)
{
  pid->integral_min = integral_min;
  pid->integral_max = integral_max;
}

/* 执行一次离散 PID 计算，并返回经过限幅后的控制输出。 */
float PID_Update(PID_Controller_t *pid, float setpoint, float measurement)
{
  float error = setpoint - measurement;
  float derivative = 0.0f;
  float output;

  if (pid->sample_time_s > 0.0f)
  {
    pid->integral += pid->ki * error * pid->sample_time_s;
    pid->integral = PID_Clamp(pid->integral, pid->integral_min, pid->integral_max);
    derivative = (error - pid->previous_error) / pid->sample_time_s;
  }

  output = (pid->kp * error) + pid->integral + (pid->kd * derivative);
  output = PID_Clamp(output, pid->output_min, pid->output_max);

  pid->previous_error = error;
  return output;
}
