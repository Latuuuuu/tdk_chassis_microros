#include "trace.hpp"
#include "chassis_monitor.hpp"
#include "chassis.hpp"

extern Chassis chassis;

int w_kp = 0.21;
int w_kd = 0;

/*-----Variable for computing the offset from black line-----*/
float weight_err = 0, weight_change = 0, weight_lastTime = 0;
double cmd_Vx = 0, cmd_Vy = 0, cmd_W = 0;

void trace_init(){
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcRead, 7);
}

float trace_transfer(){
    weight_err = ((float)(-3*adcRead[0] - adcRead[1] + adcRead[3] + 3*adcRead[4])
    				/(adcRead[0] + adcRead[1] + adcRead[2] + adcRead[3] + adcRead[4]));
    weight_change = weight_err - weight_lastTime;
    weight_lastTime = weight_err;
    cmd_W = (weight_err * w_kp + weight_change * w_kd); // Robtion correction//    temp = (weight_err * w_kp + weight_change * w_kd); // Rotation correction
    return cmd_W;
}

void trace_line(float vy){
	float w_trace = trace_transfer();
	chassis.setSpeed(0, vy, w_trace);
}

void trace_ckeck_point(){

}

bool intersection_check(int type){//確認特徵點，更新座標

	int black_line_val = 2000;//大於是白

	switch(type){

	//線在前面
	case 1:

		if(adcRead[0] <= black_line_val && adcRead[2] <= black_line_val && adcRead[4] <= black_line_val)
			return 1;
		else
			return 0;
		break;
	//線在中間
	case 2:

		if(adcRead[0] <= black_line_val && adcRead[2] <= black_line_val && adcRead[4] <= black_line_val &&
				adcRead[5] <= black_line_val && adcRead[6] <= black_line_val)
			return 1;
		else
			return 0;
		break;
	//十字路口
	case 3:

		if(adcRead[2] >= black_line_val && adcRead[5] >= black_line_val && adcRead[6] >= black_line_val)
			return 1;
		else
			return 0;
		break;
	//前面有白
	case 4:

		if(adcRead[0] >= black_line_val || adcRead[1] >= black_line_val || adcRead[2] >= black_line_val || adcRead[3] >= black_line_val || adcRead[4] >= black_line_val)
			return 1;
		else
			return 0;
		break;
	////轉彎確認—雙邊
	case 5:

		if(adcRead[5] <= black_line_val && adcRead[6] <= black_line_val && adcRead[0] >= black_line_val && adcRead[2] <= black_line_val && adcRead[4] >= black_line_val)
			return 1;
		else
			return 0;
		break;
	case 6://右轉確認—單邊

		if(adcRead[5] <= black_line_val && adcRead[0] >= black_line_val && adcRead[2] <= black_line_val && adcRead[4] >= black_line_val)
			return 1;
		else
			return 0;
		break;
	case 7://左轉確認-單邊

			if(adcRead[6] <= black_line_val && adcRead[0] >= black_line_val && adcRead[2] <= black_line_val && adcRead[4] >= black_line_val)
				return 1;
			else
				return 0;
			break;
	}
}
