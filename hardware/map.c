/* 地图模块实现：
 * - RAM 中维护网格；
 * - DFS 只决定探索方向，实际运动和避障结果由上层控制后回填。
 */
#include "map.h"

#include <string.h>

/* 地图主体：下标顺序为 map_cells[y][x]，每个元素代表一个 10cm 网格。 */
static Map_CellState_t map_cells[MAP_HEIGHT][MAP_WIDTH];
/* 当前小车所在的地图格坐标和朝向。 */
static Map_Position_t map_current_position;
static Map_Direction_t map_current_direction;
/* DFS 回溯栈：成功跨到新格子时压入旧位置，走到死路时弹出用于回退。 */
static Map_Position_t map_dfs_stack[MAP_MAX_CELLS];
static uint32_t map_dfs_stack_top = 0U;
/* 已经成功到达过的格子数量，用于调试或判断探索进度。 */
static uint32_t map_explored_count = 0U;

static uint8_t Map_IsInside(int16_t x, int16_t y);
static Map_Direction_t Map_GetDirectionBetween(Map_Position_t from, Map_Position_t to);
static void Map_PushBacktrackPosition(Map_Position_t position);
static HAL_StatusTypeDef Map_PopBacktrackPosition(Map_Position_t *position);

/* 判断坐标是否在当前静态地图范围内，越界格子统一视为不可用。 */
static uint8_t Map_IsInside(int16_t x, int16_t y)
{
  return ((x >= 0) && (x < (int16_t)MAP_WIDTH) && (y >= 0) && (y < (int16_t)MAP_HEIGHT)) ? 1U : 0U;
}

/* 根据两个相邻格坐标反推出从 from 走到 to 的方向，供 DFS 回退使用。 */
static Map_Direction_t Map_GetDirectionBetween(Map_Position_t from, Map_Position_t to)
{
  if (to.x > from.x)
  {
    return MAP_DIR_RIGHT;
  }

  if (to.x < from.x)
  {
    return MAP_DIR_LEFT;
  }

  if (to.y > from.y)
  {
    return MAP_DIR_DOWN;
  }

  return MAP_DIR_UP;
}

/* 将当前位置压入回退栈；栈满时不再压入，避免数组越界。 */
static void Map_PushBacktrackPosition(Map_Position_t position)
{
  if (map_dfs_stack_top < MAP_MAX_CELLS)
  {
    map_dfs_stack[map_dfs_stack_top] = position;
    map_dfs_stack_top++;
  }
}

/* 弹出一个回退目标位置；没有可回退位置时返回 HAL_ERROR。 */
static HAL_StatusTypeDef Map_PopBacktrackPosition(Map_Position_t *position)
{
  if ((position == NULL) || (map_dfs_stack_top == 0U))
  {
    return HAL_ERROR;
  }

  map_dfs_stack_top--;
  *position = map_dfs_stack[map_dfs_stack_top];

  return HAL_OK;
}

/* 初始化地图模块：创建一张全未知的新地图。 */
void Map_Init(void)
{
  Map_Reset(MAP_WIDTH / 2U, MAP_HEIGHT / 2U, MAP_DIR_UP);
}

/* 清空 RAM 地图，并把起点设为已访问；默认起点可理解为小车开机位置。 */
void Map_Reset(uint16_t start_x, uint16_t start_y, Map_Direction_t start_direction)
{
  memset(map_cells, (int)MAP_CELL_UNKNOWN, sizeof(map_cells));

  if (Map_IsInside((int16_t)start_x, (int16_t)start_y) == 0U)
  {
    start_x = MAP_WIDTH / 2U;
    start_y = MAP_HEIGHT / 2U;
  }

  map_current_position.x = (int16_t)start_x;
  map_current_position.y = (int16_t)start_y;
  map_current_direction = (start_direction < MAP_DIR_COUNT) ? start_direction : MAP_DIR_UP;
  map_dfs_stack_top = 0U;
  map_explored_count = 1U;
  map_cells[start_y][start_x] = MAP_CELL_VISITED;
}

/* 手动设置某个格子的状态，适合上层根据传感器或调试命令修正地图。 */
HAL_StatusTypeDef Map_SetCell(uint16_t x, uint16_t y, Map_CellState_t state)
{
  if ((Map_IsInside((int16_t)x, (int16_t)y) == 0U) ||
      ((state != MAP_CELL_BLOCK) &&
       (state != MAP_CELL_FREE) &&
       (state != MAP_CELL_VISITED) &&
       (state != MAP_CELL_UNKNOWN)))
  {
    return HAL_ERROR;
  }

  map_cells[y][x] = state;
  return HAL_OK;
}

/* 读取某个格子的状态；越界时返回 BLOCK，让路径规划自然避开边界外。 */
Map_CellState_t Map_GetCell(uint16_t x, uint16_t y)
{
  if (Map_IsInside((int16_t)x, (int16_t)y) == 0U)
  {
    return MAP_CELL_BLOCK;
  }

  return map_cells[y][x];
}

/* 获取当前小车所在格坐标。 */
Map_Position_t Map_GetCurrentPosition(void)
{
  return map_current_position;
}

/* 获取当前小车朝向。 */
Map_Direction_t Map_GetCurrentDirection(void)
{
  return map_current_direction;
}

/* 外部定位修正入口：当上层确认当前位置时，可同步更新地图模块状态。 */
HAL_StatusTypeDef Map_SetCurrentPose(uint16_t x, uint16_t y, Map_Direction_t direction)
{
  if ((Map_IsInside((int16_t)x, (int16_t)y) == 0U) || (direction >= MAP_DIR_COUNT))
  {
    return HAL_ERROR;
  }

  map_current_position.x = (int16_t)x;
  map_current_position.y = (int16_t)y;
  map_current_direction = direction;
  map_cells[y][x] = MAP_CELL_VISITED;

  return HAL_OK;
}

/* 根据当前位置和方向计算相邻格坐标，只允许上下左右四个方向移动。 */
HAL_StatusTypeDef Map_GetNeighborPosition(Map_Position_t position,
                                          Map_Direction_t direction,
                                          Map_Position_t *neighbor)
{
  Map_Position_t next = position;

  if ((neighbor == NULL) || (direction >= MAP_DIR_COUNT))
  {
    return HAL_ERROR;
  }

  switch (direction)
  {
    case MAP_DIR_UP:
      next.y--;
      break;

    case MAP_DIR_RIGHT:
      next.x++;
      break;

    case MAP_DIR_DOWN:
      next.y++;
      break;

    case MAP_DIR_LEFT:
      next.x--;
      break;

    default:
      return HAL_ERROR;
  }

  if (Map_IsInside(next.x, next.y) == 0U)
  {
    return HAL_ERROR;
  }

  *neighbor = next;
  return HAL_OK;
}

/* DFS 选路：按 上、右、下、左 的顺序寻找当前格周围第一个未知格。 */
HAL_StatusTypeDef Map_GetNextExploreDirection(Map_Direction_t *direction)
{
  Map_Direction_t dir;
  Map_Position_t neighbor;

  if (direction == NULL)
  {
    return HAL_ERROR;
  }

  for (dir = MAP_DIR_UP; dir < MAP_DIR_COUNT; dir++)
  {
    if (Map_GetNeighborPosition(map_current_position, dir, &neighbor) == HAL_OK)
    {
      if (map_cells[neighbor.y][neighbor.x] == MAP_CELL_UNKNOWN)
      {
        *direction = dir;
        return HAL_OK;
      }
    }
  }

  return HAL_ERROR;
}

/* 上层尝试走一格后调用：
 * - can_cross 为 0，目标格标记为障碍；
 * - can_cross 非 0，更新当前位置并把旧位置压入 DFS 回退栈。
 */
HAL_StatusTypeDef Map_UpdateAfterMoveAttempt(Map_Direction_t direction, uint8_t can_cross)
{
  Map_Position_t target;
  Map_Position_t previous_position = map_current_position;

  if (Map_GetNeighborPosition(map_current_position, direction, &target) != HAL_OK)
  {
    return HAL_ERROR;
  }

  map_current_direction = direction;

  if (can_cross == 0U)
  {
    map_cells[target.y][target.x] = MAP_CELL_BLOCK;
    return HAL_OK;
  }

  Map_PushBacktrackPosition(previous_position);
  map_cells[previous_position.y][previous_position.x] = MAP_CELL_VISITED;
  map_current_position = target;
  map_cells[target.y][target.x] = MAP_CELL_VISITED;
  map_explored_count++;

  return HAL_OK;
}

/* 当前格没有未知邻居时调用，返回需要物理回退的一格方向。 */
HAL_StatusTypeDef Map_BacktrackStep(Map_Direction_t *direction)
{
  Map_Position_t back_position;

  if (direction == NULL)
  {
    return HAL_ERROR;
  }

  if (Map_PopBacktrackPosition(&back_position) != HAL_OK)
  {
    return HAL_ERROR;
  }

  *direction = Map_GetDirectionBetween(map_current_position, back_position);
  map_current_direction = *direction;
  map_current_position = back_position;
  map_cells[back_position.y][back_position.x] = MAP_CELL_VISITED;

  return HAL_OK;
}

/* 返回成功跨越过的格子数量，便于串口打印探索进度。 */
uint32_t Map_GetExploredCount(void)
{
  return map_explored_count;
}

/* 当前没有未知邻格且回退栈为空时，认为 DFS 探索结束。 */
uint8_t Map_IsExploreFinished(void)
{
  Map_Direction_t direction;

  if (Map_GetNextExploreDirection(&direction) == HAL_OK)
  {
    return 0U;
  }

  return (map_dfs_stack_top == 0U) ? 1U : 0U;
}
