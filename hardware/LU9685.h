/* LU9685-20CU servo controller driver: UART5 serial command interface. */
#ifndef __LU9685_H__
#define __LU9685_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define LU9685_DEFAULT_ADDRESS          0x00U
#define LU9685_MAX_CHANNEL              19U
#define LU9685_MAX_SERVO_ANGLE          180U
#define LU9685_RELEASE_PWM_ANGLE        200U

typedef enum
{
  LU9685_STATUS_OK = 0,
  LU9685_STATUS_ERROR,
  LU9685_STATUS_INVALID_PARAM,
  LU9685_STATUS_OUT_OF_RANGE,
  LU9685_STATUS_BUSY,
  LU9685_STATUS_CACHE_INVALID
} LU9685_Status_t;

typedef enum
{
  ARM_JOINT_GRIPPER = 0,
  ARM_JOINT_WRIST_PITCH,
  ARM_JOINT_WRIST_ROTATE,
  ARM_JOINT_ELBOW,
  ARM_JOINT_SHOULDER,
  ARM_JOINT_BASE,
  ARM_JOINT_COUNT
} ArmJoint_t;

typedef enum
{
  ARM_POSE_HOME = 0,
  ARM_POSE_CARRY,
  ARM_POSE_ZONE1_PICK,
  ARM_POSE_ZONE23_PLACE,
  ARM_POSE_COUNT
} ArmPoseId_t;

typedef struct
{
  ArmJoint_t joint;
  uint8_t channel;
  uint8_t min_angle;
  uint8_t max_angle;
  uint8_t home_angle;
  uint8_t step_deg;
  uint16_t step_delay_ms;
} ArmJointConfig_t;

typedef struct
{
  uint8_t angles[ARM_JOINT_COUNT];
  uint8_t enabled_mask;
} ArmPose_t;

/* UART5 is wired to LU9685: 9600 8N1, PC12 TX, PD2 RX. */
void LU9685_Init(void);

/* Low-level LU9685 serial commands. */
LU9685_Status_t LU9685_SetAngle(uint8_t channel, uint8_t angle);
LU9685_Status_t LU9685_ReleaseChannel(uint8_t channel);
LU9685_Status_t LU9685_ResetModule(void);

/* Arm-level helpers with per-joint safety limits from docs/mechanical_arm_calibration.md. */
const ArmJointConfig_t *LU9685_GetJointConfig(ArmJoint_t joint);
const ArmPose_t *LU9685_GetPose(ArmPoseId_t pose_id);
LU9685_Status_t LU9685_SetJointAngle(ArmJoint_t joint, uint8_t angle);
LU9685_Status_t LU9685_MoveJointSmooth(ArmJoint_t joint, uint8_t target_angle);
LU9685_Status_t LU9685_MovePose(const ArmPose_t *pose);
LU9685_Status_t LU9685_MovePoseById(ArmPoseId_t pose_id);
LU9685_Status_t LU9685_ReleaseJoint(ArmJoint_t joint);
LU9685_Status_t LU9685_ReleaseAllJoints(void);

/* Cache helpers for no-feedback servos. Use after manual positioning or known startup pose. */
LU9685_Status_t LU9685_AssumeJointAngle(ArmJoint_t joint, uint8_t angle);
LU9685_Status_t LU9685_AssumePose(ArmPoseId_t pose_id);
uint8_t LU9685_GetCachedJointAngle(ArmJoint_t joint, uint8_t *angle);

#ifdef __cplusplus
}
#endif

#endif /* __LU9685_H__ */
