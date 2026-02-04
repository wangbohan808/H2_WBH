#include "base_state.h"


/* 正常工作模式与产测模式，选择一套流程进行执行 */
BASE_WORK_MODE_E base_work_mode = BASE_WORK_MODE_NORMAL;
/* 机器人是否在位，影响正常工作模式下基站的工作流程 */
ROBOT_AT_DOCK_STATE_E robot_at_dock_state = ROBOT_STATE_NOT_AT_DOCK;
DUST_BAG_STATE_E dust_bag_state = DUST_BAG_STATE_UNSTALL;

