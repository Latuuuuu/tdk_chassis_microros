
#ifndef INC_CONTROL_LIB_PINPOINT_INC_PINPOINT_MONITOR_HPP_
#define INC_CONTROL_LIB_PINPOINT_INC_PINPOINT_MONITOR_HPP_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes */

// 宣告全域變數 (extern 表示在其他檔案中定義)
extern float pos_x, pos_y, pos_z, vel_x, vel_y, vel_z;

// 函數宣告
void pinpoint_init(void);
void pinpoint_monitor(void);
void update_pinpoint_pose(void);
void relocateRobot(float x, float y, float angle);
float normalizeAngle(float radian);

#ifdef __cplusplus
}
#endif

#endif /* INC_CONTROL_LIB_PINPOINT_INC_PINPOINT_MONITOR_HPP_ */
