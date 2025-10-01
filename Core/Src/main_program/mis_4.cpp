#include "mis_4.hpp"
#include "chassis_monitor.hpp"
#include "chassis.hpp"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include <cstdint>
#include <stdbool.h>

extern Chassis chassis;
int pro = 0;
extern int emer;

void mis_4_L(){
	relocateRobot(-287,335,0.5*3.1415926);
	float _y = chassis.y;
	float _x = chassis.x;
	float _w = chassis.theta;
	if (pro == 0){
		chassis_set_speed(0.0,30.0,0.0);
		if (_x < (-487.0 + 17.6)){
			chassis_set_speed(0,0,0);
			pro = 1;
		}
	}else if (pro == 1){
		chassis_set_speed(-30.0,0.0,0.0);
		if (_y < (260.0 - 42.4)){
			chassis_set_speed(0,0,0);
			pro = 2;
		}
	}else if (pro == 2){
		chassis_set_speed(0.0,-30.0,0.0);
		if (_x > (-487+70)){
			chassis_set_speed(0,0,0);
			pro = 3;
		}
	}else if (pro == 3){
		chassis_set_speed(-10.0,0.0,0.0);
		if (_y < (260 - 74.6)){
			chassis_set_speed(0,0,0);
			pro = 4;
		}
	}else if (pro == 4){
		chassis_set_speed(0.0,-30.0,0.0);
		if (_x > (-487+196.4)){
			chassis_set_speed(0,0,0);
			pro = 5;
		}
	}else if (pro == 5){
		chassis_set_speed(-20.0,0.0,0.0);
		if (_y < (260 - 145.4)){
			chassis_set_speed(0,0,0);
			pro = 6;
		}
	}else if (pro == 6){
		chassis_set_speed(30.0,0.0,0.0);
		if (_x < (-487.0 - 50.0)){
			chassis_set_speed(0,0,0);
			pro = 7;
		}
	}else if (pro == 7){
		chassis_set_speed(0.0,-10.0,0.0);
		if (_y < (260 - 174.6)){
			chassis_set_speed(0,0,0);
			pro = 8;
		}
	}else if (pro == 8){
		chassis_set_speed(0.0,30.0,0.0);
		if (_x < (-487 + 119.4)){
			chassis_set_speed(0,0,0);
			pro = 9;
		}
	}else if (pro == 9){
		chassis_set_speed(-20.0,0.0,0.0);
		if (_y < (260 - 245.4)){
			chassis_set_speed(0,0,0);
			pro = 10;
		}
	}else if (pro == 10){
		chassis_set_speed(0.0,-30.0,0.0);
		if (_x > (-487.0 - 80.0)){
			chassis_set_speed(0,0,0);
			emer = 0;
			pro = 0;
		}
	}
}

void mis_4_R(){
	relocateRobot(287,335,1.5*3.1415926);
	float _y = chassis.y;
	float _x = chassis.x;
	float _w = chassis.theta;
	if (pro == 0){
		chassis_set_speed(0.0,30.0,0.0);
		if (_x > (487.0 - 17.6)){
			chassis_set_speed(0,0,0);
			pro = 1;
		}
	}else if (pro == 1){
		chassis_set_speed(30.0,0.0,0.0);
		if (_y < (260.0 - 42.4)){
			chassis_set_speed(0,0,0);
			pro = 2;
		}
	}else if (pro == 2){
		chassis_set_speed(0.0,-30.0,0.0);
		if (_x < (487-120)){
			chassis_set_speed(0,0,0);
			pro = 3;
		}
	}else if (pro == 3){
		chassis_set_speed(10.0,0.0,0.0);
		if (_y < (260 - 74.6)){
			chassis_set_speed(0,0,0);
			pro = 4;
		}
	}else if (pro == 4){
		chassis_set_speed(0.0,-30.0,0.0);
		if (_x < (487-196.4)){
			chassis_set_speed(0,0,0);
			pro = 5;
		}
	}else if (pro == 5){
		chassis_set_speed(20.0,0.0,0.0);
		if (_y < (260 - 145.4)){
			chassis_set_speed(0,0,0);
			pro = 6;
		}
	}else if (pro == 6){
		chassis_set_speed(30.0,0.0,0.0);
		if (_x > (487.0 - 50.0)){
			chassis_set_speed(0,0,0);
			pro = 7;
		}
	}else if (pro == 7){
		chassis_set_speed(0.0,10.0,0.0);
		if (_y < (260 - 174.6)){
			chassis_set_speed(0,0,0);
			pro = 8;
		}
	}else if (pro == 8){
		chassis_set_speed(0.0,30.0,0.0);
		if (_x < (487 + 119.4)){
			chassis_set_speed(0,0,0);
			pro = 9;
		}
	}else if (pro == 9){
		chassis_set_speed(20.0,0.0,0.0);
		if (_y < (260 - 245.4)){
			chassis_set_speed(0,0,0);
			pro = 10;
		}
	}else if (pro == 10){
		chassis_set_speed(0.0,-40.0,0.0);
		if (_x < (487.0 - 80.0)){
			chassis_set_speed(0,0,0);
			emer = 0;
			pro = 0;
		}
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_4) {
		emer = 1;
	}

}


