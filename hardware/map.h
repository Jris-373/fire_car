/* 地图模块：用 10cm 网格记录可走区域/障碍，并支持 DFS 探索。 */
#ifndef __MAP_H__
#define __MAP_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define MAP_WIDTH                 64U
#define MAP_HEIGHT                64U
#define MAP_CELL_SIZE_CM          10U
#define MAP_MAX_CELLS             (MAP_WIDTH * MAP_HEIGHT)

typedef enum
{
  MAP_CELL_BLOCK = 0x00,
  MAP_CELL_FREE = 0x01,
  MAP_CELL_VISITED = 0x02,
  MAP_CELL_UNKNOWN = 0xFF
} Map_CellState_t;

typedef enum
{
  MAP_DIR_UP = 0,
  MAP_DIR_RIGHT,
  MAP_DIR_DOWN,
  MAP_DIR_LEFT,
  MAP_DIR_COUNT
} Map_Direction_t;

typedef struct
{
  int16_t x;
  int16_t y;
} Map_Position_t;

void Map_Init(void);
void Map_Reset(uint16_t start_x, uint16_t start_y, Map_Direction_t start_direction);

HAL_StatusTypeDef Map_SetCell(uint16_t x, uint16_t y, Map_CellState_t state);
Map_CellState_t Map_GetCell(uint16_t x, uint16_t y);

Map_Position_t Map_GetCurrentPosition(void);
Map_Direction_t Map_GetCurrentDirection(void);
HAL_StatusTypeDef Map_SetCurrentPose(uint16_t x, uint16_t y, Map_Direction_t direction);

HAL_StatusTypeDef Map_GetNeighborPosition(Map_Position_t position,
                                          Map_Direction_t direction,
                                          Map_Position_t *neighbor);
HAL_StatusTypeDef Map_GetNextExploreDirection(Map_Direction_t *direction);
HAL_StatusTypeDef Map_UpdateAfterMoveAttempt(Map_Direction_t direction, uint8_t can_cross);
HAL_StatusTypeDef Map_BacktrackStep(Map_Direction_t *direction);

uint32_t Map_GetExploredCount(void);
uint8_t Map_IsExploreFinished(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAP_H__ */
