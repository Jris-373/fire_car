/* JY61P IMU 模块实现：
 * 周期任务通过 I2C 读取 WIT 寄存器块，并将原始值换算成工程单位。
 */
#include "JY61P.h"

#include <string.h>

#include "cmsis_os.h"
#include "i2c.h"
#include "task.h"

#define JY61P_I2C_ADDR_7BIT         0x50U
#define JY61P_I2C_ADDR              (JY61P_I2C_ADDR_7BIT << 1)
#define JY61P_I2C_READ_CMD          0x03U
#define JY61P_READ_ADDR_HIGH_BYTE   0x00U
#define JY61P_REG_ACCEL_START       0x34U
#define JY61P_REG_ANGLE_START       0x3DU
#define JY61P_VECTOR_BYTES          6U
#define JY61P_I2C_TIMEOUT_MS        50U
#define JY61P_TASK_PERIOD_MS        10U
#define JY61P_ACCEL_SCALE_G_PER_LSB (16.0f / 32768.0f)
#define JY61P_ANGLE_SCALE_DEG_PER_LSB (180.0f / 32768.0f)

static JY61P_Data_t jy61p_latest_data;
static osThreadId_t jy61pTaskHandle = NULL;
static const osThreadAttr_t jy61pTaskAttributes = {
  .name = "jy61pTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t)osPriorityRealtime7,
};

static void JY61P_Task(void *argument);

static int16_t JY61P_ParseInt16(const uint8_t *buffer, uint8_t value_index)
{
  /* WIT 协议返回的 16 位寄存器数据采用小端格式：先低字节，后高字节。 */
  uint8_t byte_index = (uint8_t)(value_index * 2U);
  uint16_t raw_value = ((uint16_t)buffer[byte_index + 1U] << 8) | buffer[byte_index];

  return (int16_t)raw_value;
}

static void JY61P_UpdateAccelerationCache(const uint8_t *buffer)
{
  jy61p_latest_data.accel_raw.x = JY61P_ParseInt16(buffer, 0U);
  jy61p_latest_data.accel_raw.y = JY61P_ParseInt16(buffer, 1U);
  jy61p_latest_data.accel_raw.z = JY61P_ParseInt16(buffer, 2U);

  /* 根据 WIT 官方寄存器表，将原始值换算为加速度：
   * accel = raw / 32768 * 16g
   */
  jy61p_latest_data.accel_g.x = (float)jy61p_latest_data.accel_raw.x * JY61P_ACCEL_SCALE_G_PER_LSB;
  jy61p_latest_data.accel_g.y = (float)jy61p_latest_data.accel_raw.y * JY61P_ACCEL_SCALE_G_PER_LSB;
  jy61p_latest_data.accel_g.z = (float)jy61p_latest_data.accel_raw.z * JY61P_ACCEL_SCALE_G_PER_LSB;
}

static void JY61P_UpdateAngleCache(const uint8_t *buffer)
{
  jy61p_latest_data.angle_raw.x = JY61P_ParseInt16(buffer, 0U);
  jy61p_latest_data.angle_raw.y = JY61P_ParseInt16(buffer, 1U);
  jy61p_latest_data.angle_raw.z = JY61P_ParseInt16(buffer, 2U);

  /* 根据 WIT 官方寄存器表，将原始值换算为角度：
   * angle = raw / 32768 * 180 deg
   */
  jy61p_latest_data.angle_deg.x = (float)jy61p_latest_data.angle_raw.x * JY61P_ANGLE_SCALE_DEG_PER_LSB;
  jy61p_latest_data.angle_deg.y = (float)jy61p_latest_data.angle_raw.y * JY61P_ANGLE_SCALE_DEG_PER_LSB;
  jy61p_latest_data.angle_deg.z = (float)jy61p_latest_data.angle_raw.z * JY61P_ANGLE_SCALE_DEG_PER_LSB;
}

static HAL_StatusTypeDef JY61P_ReadRegisterBlock(uint8_t start_register, uint8_t *buffer, uint16_t length)
{
  HAL_StatusTypeDef status;
  /* 模块在 I2C 下采用 WIT 读寄存器命令格式：
   * [0x03, 0x00, 起始寄存器地址]，随后返回对应寄存器数据。
   */
  uint8_t command_buffer[3] = {JY61P_I2C_READ_CMD, JY61P_READ_ADDR_HIGH_BYTE, start_register};

  status = HAL_I2C_Master_Transmit(&hi2c1, JY61P_I2C_ADDR, command_buffer, sizeof(command_buffer), JY61P_I2C_TIMEOUT_MS);
  if (status != HAL_OK)
  {
    return status;
  }

  return HAL_I2C_Master_Receive(&hi2c1, JY61P_I2C_ADDR, buffer, length, JY61P_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef JY61P_Init(void)
{
  /* 保存一份本地缓存，便于上层直接获取最近一次解析后的结果，
   * 不需要每次都重新解析原始字节流。
   */
  memset(&jy61p_latest_data, 0, sizeof(jy61p_latest_data));
  return HAL_I2C_IsDeviceReady(&hi2c1, JY61P_I2C_ADDR, 3U, JY61P_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef JY61P_ReadAcceleration(JY61P_Vector3f_t *accel_g)
{
  HAL_StatusTypeDef status;
  uint8_t rx_buffer[JY61P_VECTOR_BYTES];

  status = JY61P_ReadRegisterBlock(JY61P_REG_ACCEL_START, rx_buffer, sizeof(rx_buffer));
  if (status != HAL_OK)
  {
    return status;
  }

  JY61P_UpdateAccelerationCache(rx_buffer);
  jy61p_latest_data.last_update_tick = HAL_GetTick();

  if (accel_g != NULL)
  {
    *accel_g = jy61p_latest_data.accel_g;
  }

  return HAL_OK;
}

HAL_StatusTypeDef JY61P_ReadAngles(JY61P_Vector3f_t *angle_deg)
{
  HAL_StatusTypeDef status;
  uint8_t rx_buffer[JY61P_VECTOR_BYTES];

  status = JY61P_ReadRegisterBlock(JY61P_REG_ANGLE_START, rx_buffer, sizeof(rx_buffer));
  if (status != HAL_OK)
  {
    return status;
  }

  JY61P_UpdateAngleCache(rx_buffer);
  jy61p_latest_data.last_update_tick = HAL_GetTick();

  if (angle_deg != NULL)
  {
    *angle_deg = jy61p_latest_data.angle_deg;
  }

  return HAL_OK;
}

HAL_StatusTypeDef JY61P_ReadAll(JY61P_Data_t *data)
{
  HAL_StatusTypeDef status;
  uint8_t rx_buffer[JY61P_VECTOR_BYTES];

  /* 按两个寄存器块分别读取加速度和角度数据。
   * 这样实现更直接，也和官方寄存器表一一对应。
   */
  status = JY61P_ReadRegisterBlock(JY61P_REG_ACCEL_START, rx_buffer, sizeof(rx_buffer));
  if (status != HAL_OK)
  {
    return status;
  }
  JY61P_UpdateAccelerationCache(rx_buffer);

  status = JY61P_ReadRegisterBlock(JY61P_REG_ANGLE_START, rx_buffer, sizeof(rx_buffer));
  if (status != HAL_OK)
  {
    return status;
  }
  JY61P_UpdateAngleCache(rx_buffer);

  jy61p_latest_data.last_update_tick = HAL_GetTick();

  if (data != NULL)
  {
    *data = jy61p_latest_data;
  }

  return HAL_OK;
}

void JY61P_StartTask(void)
{
  if (jy61pTaskHandle == NULL)
  {
    jy61pTaskHandle = osThreadNew(JY61P_Task, NULL, &jy61pTaskAttributes);
  }
}

const JY61P_Data_t *JY61P_GetLatestData(void)
{
  return &jy61p_latest_data;
}

static void JY61P_Task(void *argument)
{
  TickType_t last_wake_time = xTaskGetTickCount();
  HAL_StatusTypeDef sensor_status = JY61P_Init();

  (void)argument;

  for (;;)
  {
    if (sensor_status == HAL_OK)
    {
      sensor_status = JY61P_ReadAll(NULL);
    }
    else
    {
      sensor_status = JY61P_Init();
    }

    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(JY61P_TASK_PERIOD_MS));
  }
}
