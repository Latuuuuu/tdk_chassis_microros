#ifndef TRACE_HPP
#define TRACE_HPP

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
float trace_transfer();   // Output: the rotation speed. Compute the offset from straight line
void trace_line();
void trace_check_point();     // Check if the car reach the check point and refresh the current true location
bool type_check(int type);
void loc_inter();
void dir_check();
void T_inter(float inter_x, float inter_y, float rad_ori);
void trace();
bool turn_check(float x_now,float y_now,float w_now);
void inter_goal_select();
void set_limit_dir_val();
void trace_limit_val();

struct intersection{
	float _x;
	float _y;
	float _b_xu;//x upper bound
	float _b_xl;//x lower bound
};

#ifdef __cplusplus
}
#endif

#endif // TRACE_HPP
