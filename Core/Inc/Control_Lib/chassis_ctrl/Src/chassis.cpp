#include "chassis.hpp"
#include "motor_config.h"
float Vx_global,Vy_global,dt;
void Chassis::setSpeed(float Vx_goal, float Vy_goal, float W_goal){
	float circle = WHEEL_DIA * PI;
    _Vx_goal = Vx_goal; _Vy_goal = Vy_goal; _W_goal = W_goal;
    Mecan_InverseKinematics();
    _motorFR->setSpeed(_V_FR_goal/circle);
    _motorFL->setSpeed(_V_FL_goal/circle);
    _motorBR->setSpeed(_V_BR_goal/circle);
    _motorBL->setSpeed(_V_BL_goal/circle);
}

void Chassis::getLocation(){
	Mecan_ForwardKinematics();
	dt = DT;
    theta += _W_now * DT/1000;                                              // rad
    Vy_global = _Vx_now * cos(theta) - _Vy_now * sin(theta);   // cm/s
    Vx_global = _Vx_now * sin(theta) + _Vy_now * cos(theta);   // cm/s
    x += (Vx_global * (dt/1000));                                        // cm
    y += (Vy_global * (dt/1000));										// cm
}

void Chassis::Mecan_ForwardKinematics(){
    _V_FR_now = _motorFR->getSpeed() * WHEEL_DIA * PI;                   // cm/s
    _V_FL_now = _motorFL->getSpeed() * WHEEL_DIA * PI;                   // cm/s
    _V_BR_now = _motorBR->getSpeed() * WHEEL_DIA * PI;                   // cm/s
    _V_BL_now = _motorBL->getSpeed() * WHEEL_DIA * PI;                   // cm/s

    _Vx_now = (-_V_FR_now + _V_FL_now + _V_BR_now - _V_BL_now) / 4.0f;
    _Vy_now = (_V_FR_now + _V_FL_now + _V_BR_now + _V_BL_now) / 4.0f;
    _W_now = -(-_V_FR_now +   _V_FL_now - _V_BR_now + _V_BL_now) / (CHASSIS_WIDTH + CHASSIS_LENGTH)/4;
}

void Chassis::Mecan_InverseKinematics(){
//    _V_FR_goal = -_Vx_goal + _Vy_goal + _W_goal * _chassis_factor; // cm/s
//    _V_FL_goal = _Vx_goal + _Vy_goal - _W_goal * _chassis_factor;
//    _V_BR_goal = _Vx_goal + _Vy_goal + _W_goal * _chassis_factor;
//    _V_BL_goal = -_Vx_goal + _Vy_goal - _W_goal * _chassis_factor;
    _V_FR_goal = (-_Vx_goal + _Vy_goal + _W_goal * (CHASSIS_LENGTH + CHASSIS_WIDTH)) ; // cm/s
    _V_FL_goal = (_Vx_goal + _Vy_goal - _W_goal * (CHASSIS_LENGTH + CHASSIS_WIDTH));
    _V_BR_goal = (_Vx_goal + _Vy_goal + _W_goal *(CHASSIS_LENGTH + CHASSIS_WIDTH));
    _V_BL_goal = (-_Vx_goal + _Vy_goal - _W_goal * (CHASSIS_LENGTH + CHASSIS_WIDTH));
}
