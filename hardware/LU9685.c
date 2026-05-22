/* LU9685-20CU servo controller implementation.
 * The board protocol is five bytes:
 *   FA ADDR CHANNEL ANGLE FE
 * Angle 0..180 sets servo output; angle 200 releases PWM.
 */
#include "LU9685.h"

#include <stddef.h>

#include "cmsis_os.h"
#include "usart.h"

#define LU9685_FRAME_HEAD              0xFAU
#define LU9685_FRAME_TAIL              0xFEU
#define LU9685_RESET_CODE              0xFBU
#define LU9685_FRAME_SIZE              5U
#define LU9685_UART_TX_TIMEOUT_MS      50U
#define LU9685_POSE_INTERPOLATION_INTERVAL_MS 20U

#define ARM_JOINT_MASK(joint)          ((uint8_t)(1U << (uint8_t)(joint)))
#define ARM_POSE_ENABLE_ALL            ((uint8_t)((1U << (uint8_t)ARM_JOINT_COUNT) - 1U))

static volatile uint8_t lu9685_tx_busy = 0U;
static uint8_t lu9685_cached_angles[ARM_JOINT_COUNT];
static uint8_t lu9685_cached_valid_mask = 0U;

/* Values are taken from docs/mechanical_arm_calibration.md, section 3. */
static const ArmJointConfig_t lu9685_joint_configs[ARM_JOINT_COUNT] = {
  {ARM_JOINT_GRIPPER,      0U, 41U,  75U,  41U, 1U, 20U},
  {ARM_JOINT_WRIST_PITCH,  1U, 37U, 160U, 145U, 2U, 20U},
  {ARM_JOINT_WRIST_ROTATE, 2U, 113U, 113U, 113U, 2U, 20U},
  {ARM_JOINT_ELBOW,        3U, 0U,   80U,  10U, 2U, 20U},
  {ARM_JOINT_SHOULDER,     4U, 14U, 115U, 110U, 1U, 20U},
  {ARM_JOINT_BASE,         5U, 0U,   95U,  30U, 1U, 20U},
};

static const ArmPose_t lu9685_pose_table[ARM_POSE_COUNT] = {
  /* angles[] order: gripper, wrist pitch, wrist rotate, elbow, shoulder, base. */
  {{41U, 149U, 113U, 10U, 110U, 30U}, ARM_POSE_ENABLE_ALL}, /* ARM_POSE_HOME */
  {{75U, 149U, 113U, 10U, 110U, 30U}, ARM_POSE_ENABLE_ALL}, /* ARM_POSE_CARRY */
  {{41U,  96U, 113U, 54U,  20U, 30U}, ARM_POSE_ENABLE_ALL}, /* ARM_POSE_ZONE1_PICK */
  {{75U,  96U, 113U, 54U,  20U, 30U}, ARM_POSE_ENABLE_ALL}, /* ARM_POSE_ZONE23_PLACE */
};

static const ArmJoint_t lu9685_pose_move_order[ARM_JOINT_COUNT] = {
  ARM_JOINT_BASE,
  ARM_JOINT_SHOULDER,
  ARM_JOINT_ELBOW,
  ARM_JOINT_WRIST_PITCH,
  ARM_JOINT_WRIST_ROTATE,
  ARM_JOINT_GRIPPER,
};

static void LU9685_DelayMs(uint32_t delay_ms)
{
  if (delay_ms == 0U)
  {
    return;
  }

  if (osKernelGetState() == osKernelRunning)
  {
    osDelay(delay_ms);
  }
  else
  {
    HAL_Delay(delay_ms);
  }
}

static void LU9685_RestoreIRQ(uint32_t primask)
{
  if (primask == 0U)
  {
    __enable_irq();
  }
}

static uint8_t LU9685_TryLockTx(void)
{
  uint8_t locked = 0U;
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  if (lu9685_tx_busy == 0U)
  {
    lu9685_tx_busy = 1U;
    locked = 1U;
  }
  LU9685_RestoreIRQ(primask);

  return locked;
}

static void LU9685_UnlockTx(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  lu9685_tx_busy = 0U;
  LU9685_RestoreIRQ(primask);
}

static uint8_t LU9685_IsJointValid(ArmJoint_t joint)
{
  return ((uint32_t)joint < (uint32_t)ARM_JOINT_COUNT) ? 1U : 0U;
}

static uint8_t LU9685_IsJointAngleSafe(ArmJoint_t joint, uint8_t angle)
{
  const ArmJointConfig_t *config;

  if (LU9685_IsJointValid(joint) == 0U)
  {
    return 0U;
  }

  config = &lu9685_joint_configs[joint];
  return ((angle >= config->min_angle) && (angle <= config->max_angle)) ? 1U : 0U;
}

static uint8_t LU9685_AbsDiffU8(uint8_t a, uint8_t b)
{
  return (a >= b) ? (uint8_t)(a - b) : (uint8_t)(b - a);
}

static uint32_t LU9685_CeilDivU32(uint32_t numerator, uint32_t denominator)
{
  if (denominator == 0U)
  {
    return 0U;
  }

  return (numerator + denominator - 1U) / denominator;
}

static uint8_t LU9685_InterpolateAngle(uint8_t start_angle,
                                       uint8_t target_angle,
                                       uint32_t step_index,
                                       uint32_t total_steps)
{
  uint32_t distance;
  uint32_t offset;

  if ((total_steps == 0U) || (step_index >= total_steps))
  {
    return target_angle;
  }

  if (target_angle >= start_angle)
  {
    distance = (uint32_t)(target_angle - start_angle);
    offset = (distance * step_index) / total_steps;
    return (uint8_t)(start_angle + offset);
  }

  distance = (uint32_t)(start_angle - target_angle);
  offset = (distance * step_index) / total_steps;
  return (uint8_t)(start_angle - offset);
}

static LU9685_Status_t LU9685_SendFrame(uint8_t command, uint8_t value)
{
  uint8_t frame[LU9685_FRAME_SIZE];
  HAL_StatusTypeDef tx_status;

  if (LU9685_TryLockTx() == 0U)
  {
    return LU9685_STATUS_BUSY;
  }

  frame[0] = LU9685_FRAME_HEAD;
  frame[1] = LU9685_DEFAULT_ADDRESS;
  frame[2] = command;
  frame[3] = value;
  frame[4] = LU9685_FRAME_TAIL;

  tx_status = HAL_UART_Transmit(&huart5, frame, LU9685_FRAME_SIZE, LU9685_UART_TX_TIMEOUT_MS);
  LU9685_UnlockTx();

  return (tx_status == HAL_OK) ? LU9685_STATUS_OK : LU9685_STATUS_ERROR;
}

static void LU9685_UpdateJointCache(ArmJoint_t joint, uint8_t angle)
{
  lu9685_cached_angles[joint] = angle;
  lu9685_cached_valid_mask |= ARM_JOINT_MASK(joint);
}

void LU9685_Init(void)
{
  uint8_t joint_index;

  lu9685_tx_busy = 0U;
  lu9685_cached_valid_mask = 0U;

  for (joint_index = 0U; joint_index < (uint8_t)ARM_JOINT_COUNT; joint_index++)
  {
    lu9685_cached_angles[joint_index] = lu9685_joint_configs[joint_index].home_angle;
  }
}

LU9685_Status_t LU9685_SetAngle(uint8_t channel, uint8_t angle)
{
  if ((channel > LU9685_MAX_CHANNEL) || (angle > LU9685_MAX_SERVO_ANGLE))
  {
    return LU9685_STATUS_INVALID_PARAM;
  }

  return LU9685_SendFrame(channel, angle);
}

LU9685_Status_t LU9685_ReleaseChannel(uint8_t channel)
{
  if (channel > LU9685_MAX_CHANNEL)
  {
    return LU9685_STATUS_INVALID_PARAM;
  }

  return LU9685_SendFrame(channel, LU9685_RELEASE_PWM_ANGLE);
}

LU9685_Status_t LU9685_ResetModule(void)
{
  return LU9685_SendFrame(LU9685_RESET_CODE, LU9685_RESET_CODE);
}

const ArmJointConfig_t *LU9685_GetJointConfig(ArmJoint_t joint)
{
  if (LU9685_IsJointValid(joint) == 0U)
  {
    return NULL;
  }

  return &lu9685_joint_configs[joint];
}

const ArmPose_t *LU9685_GetPose(ArmPoseId_t pose_id)
{
  if ((uint32_t)pose_id >= (uint32_t)ARM_POSE_COUNT)
  {
    return NULL;
  }

  return &lu9685_pose_table[pose_id];
}

LU9685_Status_t LU9685_SetJointAngle(ArmJoint_t joint, uint8_t angle)
{
  LU9685_Status_t status;

  if (LU9685_IsJointValid(joint) == 0U)
  {
    return LU9685_STATUS_INVALID_PARAM;
  }

  if (LU9685_IsJointAngleSafe(joint, angle) == 0U)
  {
    return LU9685_STATUS_OUT_OF_RANGE;
  }

  status = LU9685_SetAngle(lu9685_joint_configs[joint].channel, angle);
  if (status == LU9685_STATUS_OK)
  {
    LU9685_UpdateJointCache(joint, angle);
  }

  return status;
}

LU9685_Status_t LU9685_MoveJointSmooth(ArmJoint_t joint, uint8_t target_angle)
{
  const ArmJointConfig_t *config;
  uint8_t current_angle;
  uint8_t next_angle;
  uint8_t step_deg;
  LU9685_Status_t status;

  if (LU9685_IsJointValid(joint) == 0U)
  {
    return LU9685_STATUS_INVALID_PARAM;
  }

  if (LU9685_IsJointAngleSafe(joint, target_angle) == 0U)
  {
    return LU9685_STATUS_OUT_OF_RANGE;
  }

  config = &lu9685_joint_configs[joint];
  step_deg = (config->step_deg == 0U) ? 1U : config->step_deg;

  if ((lu9685_cached_valid_mask & ARM_JOINT_MASK(joint)) == 0U)
  {
    return LU9685_STATUS_CACHE_INVALID;
  }

  current_angle = lu9685_cached_angles[joint];
  while (current_angle != target_angle)
  {
    if (current_angle < target_angle)
    {
      next_angle = (uint8_t)(current_angle + step_deg);
      if (next_angle > target_angle)
      {
        next_angle = target_angle;
      }
    }
    else
    {
      if ((current_angle - target_angle) < step_deg)
      {
        next_angle = target_angle;
      }
      else
      {
        next_angle = (uint8_t)(current_angle - step_deg);
      }
    }

    status = LU9685_SetJointAngle(joint, next_angle);
    if (status != LU9685_STATUS_OK)
    {
      return status;
    }

    current_angle = next_angle;
    LU9685_DelayMs(config->step_delay_ms);
  }

  return LU9685_STATUS_OK;
}

LU9685_Status_t LU9685_MovePose(const ArmPose_t *pose)
{
  uint8_t order_index;
  ArmJoint_t joint;
  const ArmJointConfig_t *config;
  uint8_t start_angles[ARM_JOINT_COUNT];
  uint8_t last_angles[ARM_JOINT_COUNT];
  uint8_t distance;
  uint8_t step_deg;
  uint8_t cache_ready = 1U;
  uint32_t joint_steps;
  uint32_t max_steps = 0U;
  uint32_t total_steps;
  uint32_t step_index;
  uint8_t next_angle;
  LU9685_Status_t status;

  if (pose == NULL)
  {
    return LU9685_STATUS_INVALID_PARAM;
  }

  for (order_index = 0U; order_index < (uint8_t)ARM_JOINT_COUNT; order_index++)
  {
    joint = lu9685_pose_move_order[order_index];
    if ((pose->enabled_mask & ARM_JOINT_MASK(joint)) != 0U)
    {
      if (LU9685_IsJointAngleSafe(joint, pose->angles[joint]) == 0U)
      {
        return LU9685_STATUS_OUT_OF_RANGE;
      }

      if ((lu9685_cached_valid_mask & ARM_JOINT_MASK(joint)) == 0U)
      {
        cache_ready = 0U;
      }
    }
  }

  if (cache_ready == 0U)
  {
    return LU9685_STATUS_CACHE_INVALID;
  }

  for (order_index = 0U; order_index < (uint8_t)ARM_JOINT_COUNT; order_index++)
  {
    joint = lu9685_pose_move_order[order_index];
    if ((pose->enabled_mask & ARM_JOINT_MASK(joint)) != 0U)
    {
      config = &lu9685_joint_configs[joint];
      start_angles[joint] = lu9685_cached_angles[joint];
      last_angles[joint] = start_angles[joint];

      distance = LU9685_AbsDiffU8(start_angles[joint], pose->angles[joint]);
      if (distance != 0U)
      {
        step_deg = (config->step_deg == 0U) ? 1U : config->step_deg;
        joint_steps = LU9685_CeilDivU32((uint32_t)distance, (uint32_t)step_deg);
        if (joint_steps > max_steps)
        {
          max_steps = joint_steps;
        }
      }
    }
  }

  if (max_steps == 0U)
  {
    return LU9685_STATUS_OK;
  }

  total_steps = max_steps;
  if (total_steps == 0U)
  {
    total_steps = 1U;
  }

  for (step_index = 1U; step_index <= total_steps; step_index++)
  {
    for (order_index = 0U; order_index < (uint8_t)ARM_JOINT_COUNT; order_index++)
    {
      joint = lu9685_pose_move_order[order_index];
      if ((pose->enabled_mask & ARM_JOINT_MASK(joint)) != 0U)
      {
        next_angle = LU9685_InterpolateAngle(start_angles[joint],
                                             pose->angles[joint],
                                             step_index,
                                             total_steps);
        if ((next_angle != last_angles[joint]) || (step_index == total_steps))
        {
          status = LU9685_SetJointAngle(joint, next_angle);
          if (status != LU9685_STATUS_OK)
          {
            return status;
          }
          last_angles[joint] = next_angle;
        }
      }
    }

    if (step_index < total_steps)
    {
      LU9685_DelayMs(LU9685_POSE_INTERPOLATION_INTERVAL_MS);
    }
  }

  return LU9685_STATUS_OK;
}

LU9685_Status_t LU9685_MovePoseById(ArmPoseId_t pose_id)
{
  return LU9685_MovePose(LU9685_GetPose(pose_id));
}

LU9685_Status_t LU9685_ReleaseJoint(ArmJoint_t joint)
{
  if (LU9685_IsJointValid(joint) == 0U)
  {
    return LU9685_STATUS_INVALID_PARAM;
  }

  lu9685_cached_valid_mask &= (uint8_t)(~ARM_JOINT_MASK(joint));
  return LU9685_ReleaseChannel(lu9685_joint_configs[joint].channel);
}

LU9685_Status_t LU9685_ReleaseAllJoints(void)
{
  uint8_t joint_index;
  LU9685_Status_t status;

  for (joint_index = 0U; joint_index < (uint8_t)ARM_JOINT_COUNT; joint_index++)
  {
    status = LU9685_ReleaseJoint((ArmJoint_t)joint_index);
    if (status != LU9685_STATUS_OK)
    {
      return status;
    }
    LU9685_DelayMs(5U);
  }

  return LU9685_STATUS_OK;
}

LU9685_Status_t LU9685_AssumeJointAngle(ArmJoint_t joint, uint8_t angle)
{
  if (LU9685_IsJointValid(joint) == 0U)
  {
    return LU9685_STATUS_INVALID_PARAM;
  }

  if (LU9685_IsJointAngleSafe(joint, angle) == 0U)
  {
    return LU9685_STATUS_OUT_OF_RANGE;
  }

  LU9685_UpdateJointCache(joint, angle);
  return LU9685_STATUS_OK;
}

LU9685_Status_t LU9685_AssumePose(ArmPoseId_t pose_id)
{
  const ArmPose_t *pose = LU9685_GetPose(pose_id);
  uint8_t joint_index;

  if (pose == NULL)
  {
    return LU9685_STATUS_INVALID_PARAM;
  }

  for (joint_index = 0U; joint_index < (uint8_t)ARM_JOINT_COUNT; joint_index++)
  {
    if ((pose->enabled_mask & ARM_JOINT_MASK(joint_index)) != 0U)
    {
      if (LU9685_IsJointAngleSafe((ArmJoint_t)joint_index, pose->angles[joint_index]) == 0U)
      {
        return LU9685_STATUS_OUT_OF_RANGE;
      }
    }
  }

  for (joint_index = 0U; joint_index < (uint8_t)ARM_JOINT_COUNT; joint_index++)
  {
    if ((pose->enabled_mask & ARM_JOINT_MASK(joint_index)) != 0U)
    {
      LU9685_UpdateJointCache((ArmJoint_t)joint_index, pose->angles[joint_index]);
    }
  }

  return LU9685_STATUS_OK;
}

uint8_t LU9685_GetCachedJointAngle(ArmJoint_t joint, uint8_t *angle)
{
  if ((LU9685_IsJointValid(joint) == 0U) || (angle == NULL))
  {
    return 0U;
  }

  if ((lu9685_cached_valid_mask & ARM_JOINT_MASK(joint)) == 0U)
  {
    return 0U;
  }

  *angle = lu9685_cached_angles[joint];
  return 1U;
}
