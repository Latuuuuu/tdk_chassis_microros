#include "mis_ctrl.hpp"
#include "chassis_monitor.hpp"

extern int mission_control;
int mis_num;
int mis_dir;
int last_mc;

void change_mis_state(){
	if (last_mc == 0 && mission_control != 0){
		mis_dir = get_tens_digit(mission_control);
		mis_num = get_units_digit(mission_control);
	}
	last_mc = mission_control;
}

// 取得十位數
int get_tens_digit(int number) {
	if (number < 0) number = -number;  // 處理負數
	return (number / 10) % 10;
}

// 取得個位數
int get_units_digit(int number) {
	if (number < 0) number = -number;  // 處理負數
	return number % 10;
}

void set_default_position(){
	if (mis_dir == 0) {
		if (mis_num == 1) {
		    relocateRobot(0.0, 0.0, 0.0);
		}else if (mis_num == 2){
		    relocateRobot(83.0, 508.0, 0.0);
		}else if (mis_num == 3){
		    relocateRobot(-267.0, 680.0, 3.1415926);
		}else if (mis_num == 4){
		    relocateRobot(-289.0, 335.0, 1.5*3.1415926);
		}
	}else if (mis_dir == 1) {
		if (mis_num == 1) {
		    relocateRobot(0.0, 0.0, 0.0);
		}else if (mis_num == 2){
		    relocateRobot(-83.0, 508.0, 0.0);
		}else if (mis_num == 3){
		    relocateRobot(267.0, 680.0, 3.1415926);
		}else if (mis_num == 4){
		    relocateRobot(289.0, 335.0, 1.5*3.1415926);
		}
	}
}
