
#ifndef SRC_MAIN_PROGRAM_TRACE_R_HPP_
#define SRC_MAIN_PROGRAM_TRACE_R_HPP_

#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "stm32f446xx.h"
#include <stdbool.h>


/*-----------Front------------*/
/*          0   Right         */
/*          1     |           */
/*          2     |           */
/*          3     V           */
/*          4   Left          */
/*----------------------------*/
/*-----------Middle-----------*/
/*          5   Right         */
/*          6   Left          */
/*----------------------------*/

// TODO: trace_check_point





void trace_init();            // Initialize infrared sensor
void trace_R();               // Main trace function for right direction  
static float trace_transfer();   // Output: the rotation speed. Compute the offset from straight line
static void trace_line();
static void trace_check_point();     // Check if the car reach the check point and refresh the current true location
static bool type_check(int type);
static void loc_inter();
static void dir_check();
static void T_inter(float inter_x, float inter_y, float rad_ori);
static void T_inter_5(float inter_x, float inter_y, float rad_ori);
static void T_inter_1(float inter_x, float inter_y, float rad_ori);
static int turn_check(float x_now,float y_now,float w_now);
static void inter_goal_select();
static void set_limit_dir_val();
static void trace_limit_val();

struct intersection1{
	float _x;
	float _y;
	float _b_xu;//x upper bound
	float _b_xl;//x lower bound
};

#ifdef __cplusplus
}
#endif



#endif /* SRC_MAIN_PROGRAM_TRACE_R_HPP_ */
