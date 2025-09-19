#include "trace.hpp"
#include "chassis_monitor.hpp"
#include "chassis.hpp"
#include "Pinpoint_monitor.hpp"
#include "Pinpoint.hpp"
#include "uros_init.h"
#include <cmath>
#include <algorithm>

#define trace_dis 10
#define Pi 3.1415926

extern Chassis chassis;
extern PinpointI2C::BulkData bd;
extern bool trace_mode;
extern float vx, vy, vz;
float vx1, vy1, vz1;
extern bool trace_mode;

float w_kp = 0.3;
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
struct intersection inter_5 = { -267.0, 616.0, -307.0, -227.0 }; //cm
intersection inter_set[5] = {inter_1, inter_2, inter_3, inter_4, inter_5};

int inter_now = 0; //表無路口
int dir_now = 0;
int T_R[3];
float v__y = 0;
float w_trace = 0;
int done = 0;
int last_dir;
int sta = 0;

void trace() {
//	if (chassis.y == 616.0) {
//		chassis.setSpeed(0.0, 0.0, 0.3);
//	} else
//	if ((inter_now-1) >= 0){ //靠近路口
//		if (done == 0){ //車頭碰到路口前
//			trace_line(vy1);
//			sta = 0;
//		}else if (done == 1){ //線在車碰到車頭
//			chassis.setSpeed(0.0,vy1,0.0);
//			sta = 1;
//		}else if (done == 2){ //線在車身
//			chassis.setSpeed(0.0,0.0,vz1);
//			sta = 2;
//		}
//	}else{
	if(trace_mode){
		trace_line(vy);
//		sta = -1;
//	}
//	if (done[temp] == 1) {
//		chassis.setSpeed(0.0, v__y, 0.0);
//	} else {
//		trace_line(v__y);
//	}
		chassis.getLocation();

	}
//	trace_check_point();
}
void trace_init() {
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adcRead, 7);
}

float trace_transfer() { //vz>0 ：逆時針
	weight_err = ((float) (-3.0 * adcRead[0] - adcRead[1] + 2.0 * adcRead[3]
			+ 5.0 * adcRead[4])
			/ (float) (adcRead[0] + adcRead[1] + adcRead[2] + adcRead[3]
					+ adcRead[4]));
	weight_change = weight_err - weight_lastTime;
	weight_lastTime = weight_err;
	cmd_W = (weight_err * w_kp + weight_change * w_kd); // Robtion correction//    temp = (weight_err * w_kp + weight_change * w_kd); // Rotation correction
	return cmd_W;
}

void trace_line(float vy) {
	w_trace = trace_transfer();
	chassis.setSpeed(0, vy, w_trace);
}

void trace_check_point() {

	loc_inter();
	dir_check();
	if (inter_now == 1) {
		if (dir_now == 1) { //向上走
			T_inter(inter_1._x, inter_1._y, 0);
		} else if (dir_now == 2) { //向左
			T_inter(inter_1._x, inter_1._y, Pi / 2);
		} else if (dir_now == 4) {
			T_inter(inter_1._x, inter_1._y, 3 * Pi / 4);
		}
	} else if (inter_now == 2) {
		if (dir_now == 1) { //向上走
			T_inter(inter_2._x, inter_2._y, 0);
		} else if (dir_now == 2) {
			T_inter(inter_2._x, inter_2._y, Pi / 2);
		} else if (dir_now == 4) {
			T_inter(inter_2._x, inter_2._y, 3 * Pi / 4);
		}
	} else if (inter_now == 3) {
		if (dir_now == 1) { //向上走
			T_inter(inter_3._x, inter_3._y, 0);
		} else if (dir_now == 2) {
			T_inter(inter_3._x, inter_3._y, Pi / 2);
		} else if (dir_now == 4) {
			T_inter(inter_3._x, inter_3._y, 3 * Pi / 4);
		}
	} else if (inter_now == 4) {
		if (dir_now == 1) { //向上走
			T_inter(inter_4._x, inter_4._y, 0);
		} else if (dir_now == 2) {
			T_inter(inter_4._x, inter_4._y, Pi / 2);
		} else if (dir_now == 4) {
			T_inter(inter_4._x, inter_4._y, 3 * Pi / 4);
		}
	} else if (inter_now == 5) {
		if (dir_now == 1) { //向上走
			T_inter(inter_5._x, inter_5._y, 0);
		} else if (dir_now == 2) {
			T_inter(inter_5._x, inter_5._y, Pi / 2);
		} else if (dir_now == 4) {
			T_inter(inter_5._x, inter_5._y, 3 * Pi / 4);
		}
	}
//	if (last_dir != dir_now){
////		std::fill(done, done + 5, 0);
//		done = 0;
//	}
	last_dir = dir_now;
}
void T_inter(float inter_x, float inter_y, float rad_ori) {
	if (dir_now == 1) {
		if (done == 0 && type_check(1)) {
			relocateRobot(inter_x - trace_dis, inter_y, rad_ori);
			done = 1;
		} else if (done == 0){
//			relocateRobot(inter_x - trace_dis - 30, inter_y, rad_ori);
		}
		if (done == 1 && type_check(2)) {
			relocateRobot(inter_x, inter_y, rad_ori);
			done = 2;
		} else if (done == 1){
//			relocateRobot(inter_x - 5, inter_y, rad_ori);
		}
	} else if (dir_now == 2) {
		if (done == 0 && type_check(1)) {
			relocateRobot(inter_x, inter_y - trace_dis, rad_ori);
			done = 1;
		} else if (done == 0){
//			relocateRobot(inter_x, inter_y - trace_dis - 30, rad_ori);
		}
		if (done == 1 && type_check(2)) {
			relocateRobot(inter_x, inter_y, rad_ori);
			done = 2;
		} else if (done == 1) {
//			relocateRobot(inter_x, inter_y - 5, rad_ori);

		}
	} else if (dir_now == 3) {
		if (done == 0 && type_check(1)) {
			relocateRobot(inter_x + trace_dis, inter_y, rad_ori);
			done = 1;
		} else if (done == 0){
//			relocateRobot(inter_x + trace_dis + 30, inter_y, rad_ori);
		}
		if (done == 1 && type_check(2)) {
			relocateRobot(inter_x, inter_y, rad_ori);
			done = 2;
		} else if (done == 1){
//			relocateRobot(inter_x + 5, inter_y, rad_ori);
		}
	} else if (dir_now == 4) {
		if (done == 0 && type_check(1)) {
			relocateRobot(inter_x, inter_y + trace_dis, rad_ori);
			done = 1;
		} else if (done == 0){
//			relocateRobot(inter_x, inter_y + trace_dis + 30, rad_ori);
		}
		if (done == 1 && type_check(2)) {
			relocateRobot(inter_x, inter_y, rad_ori);
			done = 2;
		} else if (done == 1){
//			relocateRobot(inter_x, inter_y + 5, rad_ori);
		}
	}
	if (done == 2 && vz1 > 0) { //右轉
		if (type_check(6)) {
			relocateRobot(inter_x, inter_y, rad_ori - (float) Pi / 2);
			done = 0;
		} else {
//			relocateRobot(inter_x, inter_y, rad_ori - (float) Pi / 2 + 0.2);
		}
	} else if (done == 2 && vz1 < 0) { //左轉
		if (type_check(7)) {
			relocateRobot(inter_x, inter_y, rad_ori + (float) Pi / 2);
			done = 0;
		} else {
//			relocateRobot(inter_x, inter_y, rad_ori + (float) Pi / 2 - 0.2);
		}
	}
}

void ten_inter(float inter_x, float inter_y, float rad_ori) {
	if (type_check(1)) {
		relocateRobot(inter_x - trace_dis, inter_y, rad_ori);
	} else if (type_check(2)) {
		relocateRobot(inter_x, inter_y, rad_ori);
	}
	if (vz > 0) { //右轉
		if (type_check(5)) {
			relocateRobot(inter_x, inter_y, rad_ori - (float) Pi / 2);
		} else {
			relocateRobot(inter_x, inter_y, rad_ori - (float) Pi / 2 + 0.2);
		}
	} else if (vz < 0) { //左轉
		if (type_check(5)) {
			relocateRobot(inter_x, inter_y, rad_ori + (float) Pi / 2);
		} else {
			relocateRobot(inter_x, inter_y, rad_ori + (float) Pi / 2 - 0.2);
		}
	}
}
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
	} else {
		inter_now = 0;
	}
}

void dir_check() { //0:know 1:up 2:left 3:down 4:right
	float _dir = fmod(chassis.theta, (float) (2 * Pi));
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
bool type_check(int type) { //確認特徵點，更新座標

	int black_line_val = 3000; //大於是白

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
		if (adcRead[5] >= 4000 || adcRead[6] >= 4000)
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

		if (adcRead[5] >= black_line_val && adcRead[6] >= black_line_val
				&& adcRead[0] >= black_line_val && adcRead[2] >= black_line_val
				&& adcRead[4] >= black_line_val)
			return 1;
		else
			return 0;
		break;
	case 6: //右轉確認—單邊

		if (adcRead[5] >= black_line_val && adcRead[0] >= black_line_val
				&& adcRead[2] >= black_line_val && adcRead[4] >= black_line_val)
			return 1;
		else
			return 0;
		break;
	case 7: //左轉確認-單邊

		if (adcRead[6] >= black_line_val && adcRead[0] >= black_line_val
				&& adcRead[2] >= black_line_val && adcRead[4] >= black_line_val)
			return 1;
		else
			return 0;
		break;
	default:
		return 0;
	}
}
