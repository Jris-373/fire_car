/* PID 模块：提供离散 PID 控制器，用于速度环和航向环。 */
#ifndef __PID_H__
#define __PID_H__

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  /* kp/ki/kd 使用调用者传入的实际工程单位，不在模块内做单位换算。 */
  float kp;
  float ki;
  float kd;
  /* 固定采样周期，PID_Update() 按该周期计算积分和微分项。 */
  float sample_time_s;
  float integral;
  float previous_error;
  /* 输出限幅约束最终控制量，积分限幅单独用于减轻积分饱和。 */
  float output_min;
  float output_max;
  float integral_min;
  float integral_max;
} PID_Controller_t;

/* 初始化控制器参数、采样周期和限幅范围。 */
void PID_Init(PID_Controller_t *pid,
              float kp,
              float ki,
              float kd,
              float sample_time_s,
              float output_min,
              float output_max);
/* 清空积分项和上一拍误差，常用于控制模式切换或停车。 */
void PID_Reset(PID_Controller_t *pid);

/* 运行中更新 PID 参数。 */
void PID_SetTunings(PID_Controller_t *pid, float kp, float ki, float kd);

/* 设置最终输出限幅。 */
void PID_SetOutputLimits(PID_Controller_t *pid, float output_min, float output_max);

/* 设置积分项限幅。 */
void PID_SetIntegralLimits(PID_Controller_t *pid, float integral_min, float integral_max);

/* 执行一次 PID 计算，返回已限幅的控制输出。 */
float PID_Update(PID_Controller_t *pid, float setpoint, float measurement);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H__ */
