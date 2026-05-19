/* JY61P IMU 模块：通过 I2C 读取加速度和角度，并缓存最近一次解析结果。 */
#ifndef __JY61P_H__
#define __JY61P_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef struct
{
  /* 工程单位三轴向量，当前用于 g 和 degree。 */
  float x;
  float y;
  float z;
} JY61P_Vector3f_t;

typedef struct
{
  /* 传感器寄存器中的原始 int16 三轴数据。 */
  int16_t x;
  int16_t y;
  int16_t z;
} JY61P_Vector3i16_t;

typedef struct
{
  /* raw 字段保留寄存器原始值，便于调试；工程单位字段供控制逻辑直接使用。 */
  JY61P_Vector3i16_t accel_raw;
  JY61P_Vector3i16_t angle_raw;
  JY61P_Vector3f_t accel_g;
  JY61P_Vector3f_t angle_deg;
  uint32_t last_update_tick;
} JY61P_Data_t;

/* 检查 I2C 设备是否在线，并清空本地缓存。 */
HAL_StatusTypeDef JY61P_Init(void);

/* 按需读取单个数据块。成功后会同步刷新 latest cache。 */
HAL_StatusTypeDef JY61P_ReadAcceleration(JY61P_Vector3f_t *accel_g);
HAL_StatusTypeDef JY61P_ReadAngles(JY61P_Vector3f_t *angle_deg);

/* 连续读取加速度和角度，用于周期采样任务。 */
HAL_StatusTypeDef JY61P_ReadAll(JY61P_Data_t *data);

/* 创建 IMU 周期读取任务。 */
void JY61P_StartTask(void);

/* 返回最近一次成功读取的数据缓存。 */
const JY61P_Data_t *JY61P_GetLatestData(void);

#ifdef __cplusplus
}
#endif

#endif /* __JY61P_H__ */
