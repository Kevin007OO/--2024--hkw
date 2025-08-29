#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "PID.h"
void PID_Param_init_p(PID_t*pid){//pid三个参数大小要适配，否则调节混乱
	pid->KP=10;//响应速度控制，值越大响应越快
  pid->KI=0.6;//积分项过大会有明显超调，过小则对比例项误差调节效果不明显
  pid->KD=1.4;
	pid->fdb=0;
  pid->ref=0;
  pid->cur_error=0;
  pid->error[0]=0;
  pid->error[1]=0;
  pid->output=0;
  pid->outputMax=1000;
	pid->outputMin=0;
}

void PID_Param_init_v_motor_right(PID_t*pid){//pid三个参数大小要适配，否则调节混乱
	pid->KP=1.8;//响应速度控制，值越大响应越快
  pid->KI=0.2;//积分项过大会有明显超调，过小则对比例项误差调节效果不明显
  pid->KD=0.01;
	pid->fdb=0;
  pid->ref=0;
  pid->cur_error=0;
  pid->error[0]=0;
  pid->error[1]=0;
  pid->output=0;
  pid->outputMax=1000;//输出幅值也与pid参数有关，现在调的应该是在这一幅值下比较理想的响应结果，在逆时针旋转100圈时误差也很小1
	pid->outputMin=0;
}

void PID_Param_init_v_motor_left(PID_t*pid){//pid三个参数大小要适配，否则调节混乱
	pid->KP=1.8;//响应速度控制，值越大响应越快
  pid->KI=0.2;//积分项过大会有明显超调，过小则对比例项误差调节效果不明显
  pid->KD=0.01;
	pid->fdb=0;
  pid->ref=0;
  pid->cur_error=0;
  pid->error[0]=0;
  pid->error[1]=0;
  pid->output=0;
  pid->outputMax=1000;//输出幅值也与pid参数有关，现在调的应该是在这一幅值下比较理想的响应结果，在逆时针旋转100圈时误差也很小1
	pid->outputMin=0;
}
void PID_Calc(__IO PID_t*pid){
	 pid->cur_error=pid->ref-pid->fdb;
   pid->output+=(pid->KP*(pid->cur_error-pid->error[1])+pid->KI*pid->cur_error+pid->KD*(pid->cur_error-2*pid->error[1]+pid->error[0]));
   pid->error[0]=pid->error[1];
   pid->error[1]=pid->ref-pid->fdb;
   if(pid->output>pid->outputMax)pid->output=pid->outputMax;
	if(pid->output<-pid->outputMax)pid->output=-pid->outputMax;
}

/*
void PID_Pos_Calc(__IO PID_t*pid){
	pid->cur_error=pid->ref-pid->fdb;
  pid->output=pid->KP*pid->cur_error+pid->KI*pid->integral+pid->KD*(pid->cur_error-pid->error[1]);
	pid->integral+=pid->cur_error;
	if(abs(pid->integral)>100) pid->integral=100;
	pid->error[0]=pid->error[1];
  pid->error[1]=pid->cur_error;
	if(pid->output>pid->outputMax)pid->output=pid->outputMax;
  if(pid->output<-pid->outputMax)pid->output=-pid->outputMax;
}
*/