/*stm32 include*/
#include "stm32f446xx.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
//#include "motor_ctrl.hpp"
#include "timers.h"
#include "motor_monitor.hpp"
#include "trace.hpp"
#include "uros_init.h"
#include "motor_config.h"
#include "chassis_monitor.hpp"
#include "Pinpoint_monitor.hpp"
#include "chassis_config.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;
extern I2C_HandleTypeDef hi2c1;

uint16_t adcRead[7] = {0};

double LastCNT = 0;
double CNT = 500;
int turn =0;
double currentsp = 0;
int sec = 0,tct = 0;
float temp=0;
int trace_mode1 = 0;
int ach_state = 1;
int a = 0;
extern int trace_mode;

//PinpointI2C pinpoint(&hi2c1);
//PinpointI2C::BulkData bd;

//TimerHandle_t xTimer;

//void motorTimerCallback(TimerHandle_t xTimer);

void StartDefaultTask(void *argument)
{
//    xTimer = xTimerCreate("MotorTimer", pdMS_TO_TICKS(1), pdTRUE, (void *)0, motorTimerCallback);
//    xTimerStart(xTimer, 0);
//    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	HAL_TIM_Base_Start_IT(&htim5);
	uros_init();
    motor_init();
//    pinpoint_init();
//    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
//    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    trace_init();
    relocateRobot(0.0, 546.0, 0.0);
    for(;;)
    {
        uros_agent_status_check();
        osDelay(1000/FREQUENCY);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	if (htim->Instance == TIM5)
	{
		sec++;
		tct++;

		if(trace_mode == 1){chassis_set_speed(0.0, vy, vz);}
		else {chassis_set_speed(vx, vy, vz);}
		update_chassis_pose();
		chassis_give_speed();
		trace();
		update_pose(pos_x, pos_y, pos_z, vel_x, vel_y, vel_z,(float)ach_state);
//		if (trace_mode){
//		}else{
//			chassis_monitor();
//		}

//		pinpoint_monitor();
//		if (sec == 1000){
//			relocateRobot(0,0,temp);
//			sec = 0;
//		}

	}
  /* USER CODE END Callback 0 */
	if (htim->Instance == TIM6)
	{
		HAL_IncTick();
	}
  /* USER CODE BEGIN Callback 1 */
//
  /* USER CODE END Callback 1 */
}

//void motorTimerCallback(TimerHandle_t xTimer)
//{
//
//	chassis_monitor();
//	pinpoint_monitor();
//	sec++;
//}
//TODO:motor_PID,chassis,odometry,
//TODO: check other PWM output


