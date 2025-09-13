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
int temp=0;
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
    pinpoint_init();
//    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
//    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    trace_init();

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
		temp++;
//		chassis_monitor();
		chassis_set_speed(vx, vy, vz);
		update_pinpoint_pose();
		chassis_give_speed();
		update_pose(pos_x, pos_y, pos_z, vel_x, vel_y, vel_z);
		pinpoint_monitor();
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


