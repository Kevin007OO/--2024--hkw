#include "DJI.h"
#include "Caculate.h"
#include "stdlib.h"
//直径按1.9cm算，单圈 5.966cm/360° => 0.01657cm/°=> 60.350°/cm 该值需随实际情况调整
#define DEG_PER_CM 60.350
#define M2006_POSITION_TOLERANCE 50

void M2006_init(void)
{
	hDJI[0].motorType = M2006;
	hDJI[1].motorType = M2006;
	hDJI[2].motorType = M2006;
	hDJI[3].motorType = M2006;
	DJI_Init();
}

void set_M2006_position(float position)//单位：度°
{
	positionServo(position, &hDJI[0]);//位置伺服函数
	positionServo(position, &hDJI[1]);//位置伺服函数
	positionServo(position, &hDJI[2]);//位置伺服函数
	positionServo(position, &hDJI[3]);//位置伺服函数
}

void set_M2006_position_cm(float position_cm)
{
	float position;
	position = (float)DEG_PER_CM * position_cm;
	set_M2006_position(position);
}

char M2006_done(void)
{
	if(abs(hDJI[0].posPID.cur_error) <= M2006_POSITION_TOLERANCE)//到位
		return 1;
	else
		return 0;
}

