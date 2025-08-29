#ifndef MOTOR_H
#define MOTOR_H
#include "gpio.h"
#include "tim.h"
#include "main.h"
#include "PID.h"
#include "string.h"
#include "usart.h"
#define encoder_data int


typedef struct
{
uint8_t MOTOR_DIR; // 电机转动方向
uint8_t is_motor_enable; // 电机是否使能
uint16_t dutyfactor; // 占空比
TIM_HandleTypeDef TIM_EncoderHandle; // 配置进行编码器encoder模式的定时器
TIM_HandleTypeDef TIM_PWMHandle; // 配置进行PWM输出的定时器
uint16_t TIM_CHANNELHanle; // 配置进行PWM输出的定时器对应的通道
PID_t motor_pid_v; // 电机的PID参数
PID_t motor_pid_p;
encoder_data motor_Capture_Count; // 编码器计数
float actual_speed; // 实际转速
	double errorPercent;
	uint8_t hold_mode; // 保持模式开关 (0:关闭, 1:开启)
}motor_data;

void motor_init(TIM_HandleTypeDef htim,TIM_HandleTypeDef htim_PWM,uint16_t TIM_CHANNELHanle ,motor_data *motor_);
/**
*@note：电机使能函数
*/void set_motor_enable(motor_data *motor_);
/**
*@note：电机失能函数
*/
void set_motor_disable(motor_data *motor_);


/**
1.电机使能
2.进行编码器与PWM通道的开启
*/
void motor_start(motor_data *motor_);

void motor_stop(motor_data *motor_);
/**
*@brief：设置电机正反转
*/
void set_motor_direction(motor_data *motor_,GPIO_TypeDef* GPIOx1,uint16_t GPIO_Pin1,GPIO_TypeDef* GPIOx2,uint16_t GPIO_Pin2);
/**
*@brief：设置电机速度
*/
void set_motor_speed(motor_data *motor_,float v);

void set_pidp_ref(motor_data* motor,double ref);
//设置旋转圈数
void set_motor_posi(motor_data *motor_,double rotations);
/**
*@note：注意电机使能才能进行后续处理
*@note：在这里进行定时中断置标志位之后的一系列数据处理，包括读取编码器cnt，计算电机实际转
速，通过PID算法计算该给电机的PWM输出值（注意要将该输出值与pid->outputMax进行比较）等等
*/
//void motor_pid_control_short(motor_data *motor_);
//void motor_pid_control_long(motor_data *motor_);
void motor_pid_control();
/**
*@brief: 设置电机保持/刹车模式
*@param motor: 电机结构体指针
*@param state: 1为开启保持模式，0为关闭
*@note: 开启后，电机将主动维持速度为0，抵抗外力
*/
void set_motor_hold(motor_data *motor, uint8_t state);

#endif
