#ifndef PID_H
#define PID_H
#include "main.h"
#include "tim.h"
#include "gpio.h"

typedef struct{
  double KP;
	double KI;
  double KD;
  double fdb;
  double ref;
  double cur_error;
  double error[2];
  double output;
  double outputMax;
  double outputMin;
	double integral;
}PID_t;

void PID_Param_init_p(PID_t*pid);
void PID_Param_init_v_motor_right(PID_t*pid);
void PID_Param_init_v_motor_left(PID_t*pid);
void PID_Calc(__IO PID_t*pid);
//void PID_Pos_Calc(__IO PID_t*pid);


#endif
