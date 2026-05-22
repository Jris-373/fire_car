/* MQTT task command parser for the fire car mission flow. */
#ifndef __MQTT_COMMAND_H__
#define __MQTT_COMMAND_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "ESP01S.h"
#include "main.h"

#define MQTT_COMMAND_TOPIC             "fire_car/down"
#define MQTT_COMMAND_COMPAT_TOPIC      "fire_car/mission"
#define MQTT_COMMAND_STATUS_TOPIC      "fire_car/status"

#define MQTT_COMMAND_SOURCE_ZONE       1U
#define MQTT_COMMAND_MIN_TARGET_ZONE   2U
#define MQTT_COMMAND_MAX_TARGET_ZONE   3U

typedef struct
{
  uint32_t sequence;
  uint32_t id;
  uint8_t has_id;
  uint8_t from_zone;
  uint8_t to_zone;
  uint32_t received_tick;
  char raw_payload[128];
} MQTT_Command_t;

/* Create the MQTT command parser task. */
void MQTT_Command_StartTask(void);

/* Read and clear the latest accepted command for the mission state machine. */
uint8_t MQTT_Command_ReadLatest(MQTT_Command_t *command);

/* Publish a state update to MQTT_COMMAND_STATUS_TOPIC. */
ESP01S_Status_t MQTT_Command_ReportState(const MQTT_Command_t *command, const char *state, const char *reason);

#ifdef __cplusplus
}
#endif

#endif /* __MQTT_COMMAND_H__ */
