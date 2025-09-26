#include "trace.hpp"
#include "chassis_monitor.hpp"
#include "chassis.hpp"
#include "Pinpoint_monitor.hpp"
#include "Pinpoint.hpp"
#include "uros_init.h"
#include <cmath>
#include <algorithm>

#define trace_dis 20
#define Pi 3.1415926

extern ADC_HandleTypeDef hadc1;
extern uint16_t adcRead[7];
extern Chassis chassis;
extern PinpointI2C::BulkData bd;
extern int trace_mode;
extern float vx, vy, vz;
extern int inter_goal;
int ach_state = 1;
float vx1, vy1, vz1;
float w_kp = 0.2;
float w_kd = 0.0;
//float Pi = 3.1415;
/*-----Variable for computing the offset from black line-----*/
float weight_err = 0.0, weight_change = 0.0, weight_lastTime = 0.0;
float cmd_W = 0.0;
using intersection = struct intersection;
struct intersection inter_1 = { 83.0, 616.0, 120.0, 43.0 }; //cm
struct intersection inter_2 = { 0.0, 616.0, 40.0, -40.0 }; //cm
struct intersection inter_3 = { -81.0, 616.0, -41.0, -116.0 }; //cm
struct intersection inter_4 = { -151.0, 616.0, -116.0, -191.0 }; //cm
struct intersection inter_5 = { -267.0, 616.0, -227.0, -307.0 }; //cm
struct intersection inter_6 = { -487.0, 335.0, -447.0, -527.0 }; //cm

intersection* inter_set[6] = { &inter_1, &inter_2, &inter_3, &inter_4, &inter_5,&inter_6 };

int inter_now = 0; //表無路口
int dir_now = 0;
float w_trace = 0;
int done = 0;
float inter_goal_x = 0.0, inter_goal_y = 0.0, inter_goal_w = 0.0;
int last_trace_mode = 0; // for set original limit val
float limit_val = 0.0; //original val for limit before trace
int limit_dir = 0; //1:x 2:y select limit which dir
extern int inter_goal;
bool turn_finish = 0;
int last_dir = 0;
bool allow_turn = 1;
int test;

void trace() {
 	if (trace_mode == 1) {
		if (last_trace_mode == 2 || last_trace_mode == 0) {
			set_limit_dir_val();
			last_dir = dir_now;
			ach_state = 0;
			done = 0;
		}
		if (done == 2) {
			ach_state = 2.0;
			vx = 0.0;
			vy = 0.0;
			vz = 0.0;
		} else if (done == 0) {
			ach_state = 0;
			trace_line();
			trace_limit_val();
		}else if (done ==  1){
			vz = 0.0;
		}
	} else if (trace_mode == 2) {
//		if (last_trace_mode == 1) {
//			ach_state = 0;
//			last_dir = dir_now;
//		}
//		inter_goal_select();
//		test = turn_check(inter_goal_x, inter_goal_y, inter_goal_w);
//		if ((ach_state == 3|| dir_now != last_dir) && test == 1) {
//			ach_state = 3.0;
//			vx = 0.0;
//			vy = 0.0;
//			vz = 0.0;
//			turn_finish = 1;
//		} else if (test == 2) {
//			ach_state = 0;
//			vx = 0;
//			vy = 0;
//		}
	} else {
		if (last_trace_mode == 1){
			vz = 0;
		}
		ach_state = 1.0;
		last_dir = dir_now;
	}
	trace_check_point();
	last_trace_mode = trace_mode;
}

void trace_init() {
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adcRead, 7);
}

float trace_transfer() { //vz>0 ：逆時針
	weight_err = ((float) (-4.0 * adcRead[0] - adcRead[1] + adcRead[3]
			+ 6.0 * adcRead[4])
			/ (float) (adcRead[0] + adcRead[1] + adcRead[2] + adcRead[3]
					+ adcRead[4]));
	weight_change = weight_err - weight_lastTime;
	weight_lastTime = weight_err;
	cmd_W = (weight_err * w_kp + weight_change * w_kd); // Robtion correction//    temp = (weight_err * w_kp + weight_change * w_kd); // Rotation correction
	return cmd_W;
}

void trace_line() {
	int temp;
		if ((inter_now - 1) < 0){
			temp = 0;
		}else{
			temp = inter_now - 1;
		}
	if (chassis.x < (inter_set[temp]->_x - 4) || chassis.x > (inter_set[temp]->_x + 4) ||
			chassis.y < (inter_set[temp]->_y - 4) || chassis.y > (inter_set[temp]->_y + 4))
	{
		vz = trace_transfer();
		vx = 0.0;

	}

//	chassis.setSpeed(vx, vy, w_trace);
}

void inter_goal_select(){
	int temp;
	if ((inter_now - 1) < 0){
		temp = 0;
	}else{
		temp = inter_now - 1;
	}
	inter_goal_x = inter_set[temp]->_x;
	inter_goal_y = inter_set[temp]->_y;
	if (dir_now == 1){
		inter_goal_w = 0;
	}else if (dir_now == 2){
		inter_goal_w = PI /2;
	}else if (dir_now == 3){
		inter_goal_w = PI;
	}else if (dir_now == 4){
		inter_goal_w = (3 * PI) / 2;
	}
}

void trace_check_point() {
	loc_inter();
	dir_check();
	if (inter_now == 1) {
		if (dir_now == 1) { //向左走
			T_inter(inter_1._x, inter_1._y, 0);
		} else if (dir_now == 2) { //向下
			T_inter(inter_1._x, inter_1._y, Pi / 2);
		} else if (dir_now == 3) { //向右
			T_inter(inter_1._x, inter_1._y, Pi);
		} else if (dir_now == 4) { // 向上
			T_inter(inter_1._x, inter_1._y, 3 * Pi / 2);
		}
	} else if (inter_now == 2) {
		if (dir_now == 1) { //向左走
			T_inter(inter_2._x, inter_2._y, 0);
		} else if (dir_now == 2) { //向下
			T_inter(inter_2._x, inter_2._y, Pi / 2);
		} else if (dir_now == 3) { //向右
			T_inter(inter_2._x, inter_2._y, Pi);
		} else if (dir_now == 4) { // 向上
			T_inter(inter_2._x, inter_2._y, 3 * Pi / 2);
		}
	} else if (inter_now == 3) {
		if (dir_now == 1) { //向左走
			T_inter(inter_3._x, inter_3._y, 0);
		} else if (dir_now == 2) { //向下
			T_inter(inter_3._x, inter_3._y, Pi / 2);
		} else if (dir_now == 3) { //向右
			T_inter(inter_3._x, inter_3._y, Pi);
		} else if (dir_now == 4) { // 向上
			T_inter(inter_3._x, inter_3._y, 3 * Pi / 2);
		}
	} else if (inter_now == 4) {
		if (dir_now == 1) { //向左走
			T_inter(inter_4._x, inter_4._y, 0);
		} else if (dir_now == 2) { //向下
			T_inter(inter_4._x, inter_4._y, Pi / 2);
		} else if (dir_now == 3) { //向右
			T_inter(inter_4._x, inter_4._y, Pi);
		} else if (dir_now == 4) { // 向上
			T_inter(inter_4._x, inter_4._y, 3 * Pi / 2);
		}
	} else if (inter_now == 5) {
		if (dir_now == 1) { //向左走
			T_inter(inter_5._x, inter_5._y, 0);
		} else if (dir_now == 2) { //向下
			T_inter(inter_5._x, inter_5._y, Pi / 2);
		} else if (dir_now == 3) { //向右
			T_inter(inter_5._x, inter_5._y, Pi);
		} else if (dir_now == 4) { // 向上
			T_inter(inter_5._x, inter_5._y, 3 * Pi / 2);
		}
	} else if (inter_now == 6) {
		if (dir_now == 1) { //向左走
			T_inter(inter_6._x, inter_6._y, 0);
		} else if (dir_now == 2) { //向下
			T_inter(inter_6._x, inter_6._y, Pi / 2);
		} else if (dir_now == 3) { //向右
			T_inter(inter_6._x, inter_6._y, Pi);
		} else if (dir_now == 4) { // 向上
			T_inter(inter_6._x, inter_6._y, 3 * Pi / 2);
		}
	}
}

int turn_check(float x_now, float y_now, float w_now) {
	if (vz < 0) { //右轉
		if (type_check(6)) {
			relocateRobot(x_now, y_now, w_now );
			return 1;
		}
		return 2;
	} else if (vz > 0) { //左轉
		if (type_check(7)) {
			relocateRobot(x_now, y_now, w_now );
			return 1;
		}
		return 2;
	}
	return 0;
}
void T_inter(float inter_x, float inter_y, float rad_ori) {
	if (inter_now == inter_goal) {
		if (dir_now == 1) {
			if (done == 0 && type_check(1)) {
				relocateRobot(inter_x , inter_y - trace_dis, rad_ori);
				done = 1;
			}
			if (done == 1 && type_check(2)) {
				relocateRobot(inter_x, inter_y, rad_ori);
				done = 2;
			}
		} else if (dir_now == 2) {
			if (done == 0 && type_check(1)) {
				relocateRobot(inter_x + trace_dis, inter_y, rad_ori);
				done = 1;
			}
			if (done == 1 && type_check(2)) {
				relocateRobot(inter_x, inter_y, rad_ori);
				done = 2;
			}
		} else if (dir_now == 3) {
			if (done == 0 && type_check(1)) {
				relocateRobot(inter_x , inter_y + trace_dis, rad_ori);
				done = 1;
			}
			if (done == 1 && type_check(2)) {
				relocateRobot(inter_x, inter_y, rad_ori);
				done = 2;
			}
		} else if (dir_now == 4) {
			if (done == 0 && type_check(1)) {
				relocateRobot(inter_x - trace_dis, inter_y , rad_ori);
				done = 1;
			}
			if (done == 1 && type_check(2)) {
				relocateRobot(inter_x, inter_y, rad_ori);
				done = 2;
			}
		}
	}
}

//void ten_inter(float inter_x, float inter_y, float rad_ori) {
//	if (type_check(1)) {
//		relocateRobot(inter_x - trace_dis, inter_y, rad_ori);
//	} else if (type_check(2)) {
//		relocateRobot(inter_x, inter_y, rad_ori);
//	}
//	if (vz > 0) { //右轉
//		if (type_check(5)) {
//			relocateRobot(inter_x, inter_y, rad_ori - (float) Pi / 2);
//		} else {
//			relocateRobot(inter_x, inter_y, rad_ori - (float) Pi / 2 + 0.2);
//		}
//	} else if (vz < 0) { //左轉
//		if (type_check(5)) {
//			relocateRobot(inter_x, inter_y, rad_ori + (float) Pi / 2);
//		} else {
//			relocateRobot(inter_x, inter_y, rad_ori + (float) Pi / 2 - 0.2);
//		}
//	}
//}
void loc_inter() {
	float _x = chassis.x;
	float _y = chassis.y;
	if (_y > 576.0 && _y < 656.0) {
		if (_x < inter_1._b_xu && _x > inter_1._b_xl) {
			inter_now = 1;
		} else if (_x < inter_2._b_xu && _x > inter_2._b_xl) {
			inter_now = 2;
		} else if (_x < inter_3._b_xu && _x > inter_3._b_xl) {
			inter_now = 3;
		} else if (_x < inter_4._b_xu && _x > inter_4._b_xl) {
			inter_now = 4;
		} else if (_x < inter_5._b_xu && _x > inter_5._b_xl) {
			inter_now = 5;
		} else {
			inter_now = 0;
		}
	} else if (_y > 295.0 && _y < 375.0) {
		if (_x < inter_6._b_xu && _x > inter_6._b_xl) {
			inter_now = 6;
		} else {
			inter_now = 0;
		}
	} else {
		inter_now = 0;
	}
}

void dir_check() { //0:know 1:up 2:left 3:down 4:right
	float _dir = chassis.theta;
	if (_dir < 0.78539 || _dir > 5.4979) {
		dir_now = 1;
	} else if (_dir < 2.3562 && _dir > 0.78539) {
		dir_now = 2;
	} else if (_dir < 3.927 && _dir > 2.3562) {
		dir_now = 3;
	} else if (_dir < 5.4979 && _dir > 3.927) {
		dir_now = 4;
	}
}


void set_limit_dir_val(){
	if (dir_now == 1 || dir_now == 3){
		limit_dir = 1;
		limit_val = chassis.x;
	}else if (dir_now == 2 || dir_now == 4){
		limit_dir = 2;
		limit_val = chassis.y;
	}
}
void trace_limit_val() {
	if (limit_dir == 1) {
		if (chassis.x > limit_val + 2.0) {
			chassis.x = limit_val + 2.0;
		} else if (chassis.x < limit_val - 2.0) {
			chassis.x = limit_val - 2.0;
		}
	} else if (limit_dir == 2) {
		if (chassis.y > limit_val + 2.0) {
			chassis.y = limit_val + 2.0;
		} else if (chassis.y < limit_val - 2.0) {
			chassis.y = limit_val - 2.0;
		}
	}
}

bool type_check(int type) { //確認特徵點，更新座標
	int black_line_val = 2700; //大於是黑
	int black_center_val = 4000;

	switch (type) {
	//橫線在前面
	case 1:
		if (adcRead[0] >= black_line_val && adcRead[2] >= black_line_val
				&& adcRead[4] >= black_line_val)
			return 1;
		else
			return 0;
		break;
		//橫線在中間
	case 2:
		if (adcRead[5] >= black_center_val || adcRead[6] >= black_center_val)
			return 1;
		else
			return 0;
		break;
		//十字路口
	case 3:

		if (adcRead[2] >= black_line_val && adcRead[5] >= black_line_val
				&& adcRead[6] >= black_line_val)
			return 1;
		else
			return 0;
		break;
		//前面有白
	case 4:

		if (adcRead[0] >= black_line_val || adcRead[1] >= black_line_val
				|| adcRead[2] >= black_line_val || adcRead[3] >= black_line_val
				|| adcRead[4] >= black_line_val)
			return 1;
		else
			return 0;
		break;
		//轉彎確認—雙邊
	case 5:

		if ((adcRead[5] >= black_line_val || adcRead[6] >= black_line_val)
				&& adcRead[2] >= black_line_val)
			return 1;
		else
			return 0;
		break;
	case 6: //右轉確認—單邊

		if ((adcRead[5] >= black_line_val || adcRead[6] >= black_line_val) || (adcRead[3] >= black_line_val || adcRead[3] >= black_line_val))
			return 1;
		else
			return 0;
		break;
	case 7: //左轉確認-單邊

		if ((adcRead[5] >= black_line_val || adcRead[6] >= black_line_val) || (adcRead[0] >= black_line_val))
			return 1;
		else
			return 0;
		break;
	default:
		return 0;
	}
}
