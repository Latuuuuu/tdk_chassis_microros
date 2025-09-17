#include <Control_Lib/Pinpoint/Inc/Pinpoint.hpp>
#include <Control_Lib/Pinpoint/Inc/Pinpoint_monitor.hpp>
#include "main.h"
#include "cmsis_os.h"
#include <stdbool.h>
#include <cmath>
#include "chassis_config.h"
#include "chassis_config.h"

#define update_period 5

extern PinpointI2C pinpoint;
extern PinpointI2C::BulkData bd;
extern I2C_HandleTypeDef hi2c1;
extern int sec;
extern int temp;
int fail_reloc = 0;


//float pos_x, pos_y, pos_z;//, vel_x, vel_y, vel_z;
bool check = 0;

 void pinpoint_init(){
 	osDelay(500);
 	pinpoint.Pinpoint_Init();
 }
//void pinpoint_init(){
//    // 增加更長的延遲確保 I2C 裝置穩定
//    osDelay(500);
//
//    // 檢查 I2C 匯流排狀態
//    if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
//        HAL_I2C_DeInit(&hi2c1);
//        HAL_I2C_Init(&hi2c1);
//    }
//    check = 1;
//    // 嘗試 ping 裝置
//    if (pinpoint.ping()) {
//        pinpoint.Pinpoint_Init();
//    } else {
//        // I2C 裝置未回應，可能需要重試或報錯
//    }
//}

void pinpoint_monitor(){
	if (sec == update_period){
		// pinpoint.Pinpoint_TaskLoop();
		// sec = 0;
		  // 檢查連線狀態
        if (pinpoint.isConnected()) {
            pinpoint.Pinpoint_TaskLoop();
//        } else {
//            // I2C 連線異常，嘗試重新初始化
//            HAL_I2C_DeInit(&hi2c1);
//            osDelay(100);
//            HAL_I2C_Init(&hi2c1);
        }
        sec = 0;
	}
//	if(temp == 1000){
//		relocateRobot(100,1100,0.2);
//		temp = 0;
//	}
}

//void relocateRobot(float x, float y, float angle) {
//	PinpointI2C::Pose newPos;
//    newPos.x_mm = x;
//    newPos.y_mm = y;
//    newPos.heading = normalizeAngle(angle);
//
//    if (!pinpoint.writePosition(newPos)) {
//    	fail_reloc++;
//        // 處理錯誤
//    }
//}
float normalizeAngle(float radian) {
    // 如果角度为负数，将其调整到 [0, 2π] 范围内
    while (radian < 0) {
        radian += PI * 2;
    }
    // 将角度限制在 [0, 2π] 范围内
    return fmod(radian, PI*2);
}
void update_pinpoint_pose(){
	 pos_x = bd.pos_x_mm;
	 pos_y = bd.pos_y_mm;
	 pos_z = bd.heading_rad;
}


