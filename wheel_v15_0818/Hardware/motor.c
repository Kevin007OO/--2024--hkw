#include "motor.h"
#include "math.h"
#define WHEEL_DIAMETER 5.5
#define MAX_SPEED 500 //普通移动时的最大速度
#define PI 3.14159265358

__IO uint16_t counter_now1=0,counter_now2=0; //临时存放从编码器定时器读出的瞬时计数值
__IO float rpm1=0,rpm2=0; //存放根据编码器值计算出的电机实时转速
// double counter_posi=0.0; /旧版控制函数使用
extern uint16_t	distance1,distance2; //从stp_23l.c解析出的、经过main.c传递过来的两个激光雷达的距离值
extern motor_data motor2,motor1; //电机的数据结构实例，包含了PID参数、定时器句柄等所有与电机相关的信息
int wflag1=0,wflag2=0,endflag=0,startflag=0,once_transmit_flag=0,correct_enable=0; //wflag1, wflag2: w代表working或waiting。它们用于标记两个电机是否已经完成了当前的移动任务。0=空闲，1=正在运行，2=已完成。endflag: 用于区分机器人是在执行普通移动（0）还是任务的最后一段减速移动（1）。startflag:用于判断机器是否处于初始加速状态，便于增加加速状态
double distanceRef1,distanceRef2; //用于存放经过单位换算后的雷达距离值

double start_position1,distance_traveled1=0;
double start_position2,distance_traveled2=0;
/* ------------用于超时机制的变量------------ */
extern uint8_t timeout_enabled;
extern uint32_t timeout_ticks;
extern uint32_t movement_timer;
/* ---------------------------------------- */
double filter_para1=0.2,filter_para2=0.6;//增加一阶低通滤波系数

static const uint8_t ack = 0xAA;       // 向主控的反馈信息

// ====== 自适应 FOH：按上一帧到下一帧的实际间隔来设置斜线长度 ======
#define CTRL_DT_SEC        (0.0025)     // TIM1 周期 2.5ms
#define FOH_GAIN           (1.10)       // 斜线长度 = 上一次实际采样间隔 * 1.10
#define FOH_MIN_STEPS      (2)          // 保底不少于 2 个 tick
#define FOH_MAX_STEPS      (200)        // 上限保护，避免异常拖得太久
#define FOH_PERIOD_SMOOTH  (0.2)        // 对采样周期的 EMA 平滑系数 beta

typedef struct {
    double y_cm;               // 当前插值输出（cm）
    double slope_per_tick;     // 每 tick 的增量（cm/2.5ms）
    uint16_t ticks_left;       // 当前斜线还剩多少 tick
    double last_raw_cm;        // 上一次原始值（cm）
    uint16_t ticks_since_raw;  // 距离“上一次原始值更新”已过多少 tick
    double period_est_ticks;   // 采样周期（tick）的指数平均估计
    uint8_t inited;
} FOH_t;

static FOH_t foh1 = {0};  // distance1
static FOH_t foh2 = {0};  // distance2

static inline double mm_to_cm_uint(uint16_t mm) { return ((double)mm) / 10.0; }

static inline uint16_t clamp_u16(int v, int lo, int hi) {
    if (v < lo) return (uint16_t)lo;
    if (v > hi) return (uint16_t)hi;
    return (uint16_t)v;
}

// 每个控制 tick 都要调用，raw_cm 是“本 tick 读到的最新原始测距（cm）”
static inline double foh_step(FOH_t *f, double raw_cm)
{
    if (!f->inited) {
        f->y_cm = raw_cm;
        f->last_raw_cm = raw_cm;
        f->slope_per_tick = 0.0;
        f->ticks_left = 0;
        f->ticks_since_raw = 0;
        f->period_est_ticks = (40e-3 / CTRL_DT_SEC); // 初值按 40ms 估计
        f->inited = 1;
        return f->y_cm;
    }

    // tick 计数 +1（用于估计真实采样间隔）
    f->ticks_since_raw++;

    // 原始值有更新：用“刚刚过去的间隔”更新周期估计，并重新规划斜线
    if (raw_cm != f->last_raw_cm) {
        // 指数平均估计采样周期（tick）
        double Tm = (double)f->ticks_since_raw;
        f->period_est_ticks = (1.0 - FOH_PERIOD_SMOOTH) * f->period_est_ticks
                            + (FOH_PERIOD_SMOOTH)       * Tm;

        // 斜线长度 N = 估计周期 * 略大系数（避免提前走完）
        int N = (int)(FOH_GAIN * f->period_est_ticks + 0.5);
        N = clamp_u16(N, FOH_MIN_STEPS, FOH_MAX_STEPS);

        // 以当前输出为起点，N 个 tick 内走到新原始值
        f->slope_per_tick = (raw_cm - f->y_cm) / (double)N;
        f->ticks_left     = (uint16_t)N;

        // 更新原始值基准与计时
        f->last_raw_cm    = raw_cm;
        f->ticks_since_raw = 0;
    }

    // 沿斜线推进；若下一帧提前到达，会在上面被“重新规划”打断
    if (f->ticks_left > 0) {
        f->y_cm += f->slope_per_tick;
        f->ticks_left--;
        if (f->ticks_left == 0) {
            // 斜线刚好走完：对齐到当前目标，避免微小累计误差
            f->y_cm = f->last_raw_cm;
        }
    } else {
        // 没有新目标 & 斜线已走完：维持当前值（通常下一帧很快到）
        f->y_cm = f->last_raw_cm;
    }

    return f->y_cm;
}


/**
*@note：电机初始化函数
*/
void motor_init(TIM_HandleTypeDef htim,TIM_HandleTypeDef htim_PWM,uint16_t TIM_CHANNELHanle ,motor_data *motor_)
{
motor_->MOTOR_DIR=0;
motor_->dutyfactor=0;
motor_->is_motor_enable=0;
motor_->actual_speed=0;
motor_->hold_mode = 0; // 默认关闭保持模式
motor_->TIM_EncoderHandle=htim;
motor_->TIM_PWMHandle=htim_PWM;
motor_->TIM_CHANNELHanle=TIM_CHANNELHanle;
motor_->motor_Capture_Count=65536.0/2.0;
PID_Param_init_p(&(motor_->motor_pid_p));
	if(motor_ == &motor1){
		PID_Param_init_v_motor_right(&(motor_->motor_pid_v));
	}else if(motor_ == &motor2){
		PID_Param_init_v_motor_left(&(motor_->motor_pid_v));
	}

}
/**
*@note：电机使能函数
void set_motor_enable(motor_data *motor_)
{
  motor_->is_motor_enable=1;
	HAL_GPIO_WritePin(STBY_GPIO_Port,STBY_Pin,1);
}
*/
/**
*@note：电机失能函数
*/
/**void set_motor_disable(motor_data *motor_)
{
  motor_->is_motor_enable=0;
	HAL_GPIO_WritePin(STBY_GPIO_Port,STBY_Pin,0);
}
*/

//电机开启
void motor_start(motor_data *motor_){
	HAL_TIM_PWM_Start(&htim1,motor_->TIM_CHANNELHanle);
	__HAL_TIM_SetCounter(&motor_->TIM_EncoderHandle,32768);
	HAL_TIM_Encoder_Start(&motor_->TIM_EncoderHandle,TIM_CHANNEL_ALL);
}
//电机关闭（直接关闭PWM波输出）
void motor_stop(motor_data *motor_){
	HAL_TIM_PWM_Stop(&htim1,motor_->TIM_CHANNELHanle);
	//HAL_TIM_Base_Stop_IT(&htim1);
	//HAL_TIM_Encoder_Stop(&htim2,TIM_CHANNEL_ALL);
}

/**
*@brief: 设置电机主动保持速度为0的模式（刹车）
*/
void set_motor_hold(motor_data *motor, uint8_t state)
{
    motor->hold_mode = state;
    
    // 如果是开启保持模式，为了让电机能快速响应，
    // 立即将速度环的目标值设置为0。
    if (state == 1)
    {
        motor->motor_pid_v.ref = 0;
    }
}

//电机转向
void set_motor_direction(motor_data *motor_,GPIO_TypeDef* GPIOx1,uint16_t GPIO_Pin1,GPIO_TypeDef* GPIOx2,uint16_t GPIO_Pin2){
	if(motor_->motor_pid_v.ref == 0){
		HAL_GPIO_WritePin(GPIOx1, GPIO_Pin1, 0); // 刹车模式
    HAL_GPIO_WritePin(GPIOx2, GPIO_Pin2, 0);
		if(endflag==0 && (fabs(motor1.motor_pid_p.cur_error)>=0.8 || fabs(motor2.motor_pid_p.cur_error)>=0.8) &&correct_enable == 1){
			wflag1 = 1;
			wflag2 = 1;
			set_motor_hold(&motor1, 0);
			set_motor_hold(&motor2, 0);
		}
//		wflag1 = 2;
//		wflag2 = 2;
		return;
	}
	if(motor_->motor_pid_v.output>0){
      HAL_GPIO_WritePin(GPIOx1, GPIO_Pin1, 0); // 和设置的初始转向相同
      HAL_GPIO_WritePin(GPIOx2, GPIO_Pin2, 1);
		  __HAL_TIM_SET_COMPARE(&htim1, motor_->TIM_CHANNELHanle, motor_->motor_pid_v.output);//这里必须分开写，否则不能正确设定值。HAL库不支持多通道或运算调用
		}
		else if(motor_->motor_pid_v.output<0){
      HAL_GPIO_WritePin(GPIOx1, GPIO_Pin1, 1); // 和设置的初始转向相反
      HAL_GPIO_WritePin(GPIOx2, GPIO_Pin2, 0);
      __HAL_TIM_SET_COMPARE(&htim1,motor_->TIM_CHANNELHanle, -(motor_->motor_pid_v.output));
		}
}
//转速
void set_motor_speed(motor_data *motor_,float rpm){
	motor_->motor_pid_v.ref=(motor_->motor_pid_p.ref>0?1:-1)*rpm;
}
//圈数
void set_motor_posi(motor_data *motor_,double rotations){
	motor_->motor_pid_p.ref=rotations;
}
void set_pidp_ref(motor_data* motor,double ref){
	motor_init(motor->TIM_EncoderHandle,htim1,motor->TIM_CHANNELHanle, motor);
	set_motor_posi(motor,ref);
	motor_start(motor);
}

//新的控制函数，停车时保持速度为0而不是关闭pwm波
// in motor.c

void motor_pid_control(){//依据位置环提供当前误差分析，调节速度环的目标值，速度环负责输出
	/* ======================================================= */
    /* 1. 超时判断逻辑                        */
    /* ======================================================= */
	// 仅当电机在运行时才增加计时器
	if (wflag1 == 1 || wflag2 == 1)
	{
		movement_timer++;
	}
	// 检查是否超时
	if (timeout_enabled && (movement_timer > timeout_ticks))
	{
		// 如果电机1仍在运行，则强制进入主动刹车模式
		if (wflag1 == 1) { 
			set_motor_hold(&motor1, 1); // <--- 修改点：替换 motor_stop
			wflag1 = 2;
		}
		// 如果电机2仍在运行，则强制进入主动刹车模式
		if (wflag2 == 1) {
			set_motor_hold(&motor2, 1); // <--- 修改点：替换 motor_stop
			wflag2 = 2;
		}
		movement_timer = 0; // 复位计时器
	}
	
	

	/* ======================================================= */
    /* 2. 获取传感器数据与计算位置误差                       */
    /* ======================================================= */
	counter_now1=__HAL_TIM_GetCounter(&motor1.TIM_EncoderHandle);
	counter_now2=__HAL_TIM_GetCounter(&motor2.TIM_EncoderHandle);

	// --- 用 FOH 将台阶样本连接成折线（单位先转 cm）---
	double raw1_cm = mm_to_cm_uint(distance1);
	double raw2_cm = mm_to_cm_uint(distance2);
	double smooth1_cm = foh_step(&foh1, raw1_cm);
	double smooth2_cm = foh_step(&foh2, raw2_cm);

	// --- 再把 cm → 圈数（与原逻辑一致）---
	distanceRef1 = smooth1_cm / WHEEL_DIAMETER / 3.14159265358;
	distanceRef2 = smooth2_cm / WHEEL_DIAMETER / 3.14159265358;

	//distanceRef1=distance1/10/WHEEL_DIAMETER/PI;
	//distanceRef2=distance2/10/WHEEL_DIAMETER/PI;
	double ref=80/WHEEL_DIAMETER/PI;
	double flagref=40/WHEEL_DIAMETER/PI;
	double startref=30/WHEEL_DIAMETER/PI;//启动位置
	double endref=20/WHEEL_DIAMETER/PI;
	double current_percent1=fabs(motor1.motor_pid_p.cur_error/ref);
	double current_percent2=fabs(motor2.motor_pid_p.cur_error/ref);
	motor1.motor_pid_p.fdb=distanceRef2;
	motor2.motor_pid_p.fdb=distanceRef1;
	motor1.motor_pid_p.cur_error=motor1.motor_pid_p.ref-motor1.motor_pid_p.fdb;
	motor2.motor_pid_p.cur_error=motor1.motor_pid_p.ref-motor2.motor_pid_p.fdb;

	if(startflag == 1){
		distance_traveled1 = fabs(start_position1-distanceRef1);
		distance_traveled2 = fabs(start_position2-distanceRef2);
	}
	/* ======================================================= */
    /* 3. 主控制逻辑                                       */
    /* ======================================================= */
	// --- 对电机2进行逻辑判断 --- (与电机1逻辑完全相同)
	if (motor2.hold_mode == 1)
	{
		motor2.motor_pid_v.ref = 0;
		startflag = 0;
	}else if(startflag==1&&fabs(motor2.motor_pid_p.cur_error)>flagref&&distance_traveled2<=startref){
		motor2.motor_pid_v.ref=distance_traveled2/startref*(motor2.motor_pid_p.cur_error>0?1:-1)*MAX_SPEED+(motor2.motor_pid_p.cur_error>0?1:-1)*0.15*100;
	}
	else
	{
		if(endflag==0){ // 普通移动模式
			startflag = 0;
			if((motor2.motor_pid_p.cur_error>=ref)||(motor2.motor_pid_p.cur_error<=-ref)){
				motor2.motor_pid_v.ref=(motor2.motor_pid_p.cur_error>0?1:-1)*MAX_SPEED;
			}
			else if(motor2.motor_pid_p.cur_error<=ref&&motor2.motor_pid_p.cur_error>=-ref){
				motor2.motor_pid_v.ref=(motor2.motor_pid_p.cur_error>0?1:-1)*pow(current_percent2,1.5)*MAX_SPEED+(motor2.motor_pid_p.cur_error>0?1:-1)*0.15*100;
			}
			// 到达目标，进入主动刹车
			if((motor2.motor_pid_p.cur_error)<=(0.8/WHEEL_DIAMETER/PI)&&motor2.motor_pid_p.cur_error>=(-0.8/WHEEL_DIAMETER/PI)&&wflag2 == 1){
				set_motor_hold(&motor2, 1); // <--- 修改点：替换 motor_stop
    		wflag2 = 2;
			}
		} else { // 末段移动模式
			if((motor2.motor_pid_p.cur_error>=endref)||(motor2.motor_pid_p.cur_error<=-endref)){
				motor2.motor_pid_v.ref=(motor2.motor_pid_p.cur_error>0?1:-1)*MAX_SPEED;
			}
			else if(motor2.motor_pid_p.cur_error<=endref&&motor2.motor_pid_p.cur_error>=-endref){
				motor2.motor_pid_v.ref=motor2.motor_pid_p.cur_error/endref*120+(motor2.motor_pid_p.cur_error>0?1:-1)*0.2*120;
			}
			// 到达目标，进入主动刹车
			if((motor2.motor_pid_p.cur_error)<=(2/WHEEL_DIAMETER/PI)&&motor2.motor_pid_p.cur_error>=(-2/WHEEL_DIAMETER/PI)&&wflag2 == 1){
				set_motor_hold(&motor2, 1); // <--- 修改点：替换 motor_stop
				motor_stop(&motor2);
				wflag2=2;
			}
		}
	}
	// --- 对电机1进行逻辑判断 ---
	if (motor1.hold_mode == 1)
	{
		motor1.motor_pid_v.ref = 0;
	}else if(startflag==1&&fabs(motor1.motor_pid_p.cur_error)>flagref&&distance_traveled1<=startref){
		motor1.motor_pid_v.ref=distance_traveled1/startref*(motor1.motor_pid_p.cur_error>0?1:-1)*MAX_SPEED+(motor1.motor_pid_p.cur_error>0?1:-1)*0.15*100;
	}
	else
	{
		if(endflag==0){ // 普通移动模式
			if((motor1.motor_pid_p.cur_error>=ref)||(motor1.motor_pid_p.cur_error<=-ref)){
				motor1.motor_pid_v.ref=(motor1.motor_pid_p.cur_error>0?1:-1)*MAX_SPEED;
			}
			else if(motor1.motor_pid_p.cur_error<=ref&&motor1.motor_pid_p.cur_error>=-ref){
				motor1.motor_pid_v.ref=(motor1.motor_pid_p.cur_error>0?1:-1)*pow(current_percent1,1.5)*MAX_SPEED+(motor1.motor_pid_p.cur_error>0?1:-1)*0.15*100;
			}
			// 到达目标，进入主动刹车
			if((motor1.motor_pid_p.cur_error)<=(0.8/WHEEL_DIAMETER/PI)&&motor1.motor_pid_p.cur_error>=(-0.8/WHEEL_DIAMETER/PI)&&wflag1 == 1){
				set_motor_hold(&motor1, 1); // <--- 修改点：替换 motor_stop
				wflag1 = 2;
			}
		} else { // 末段移动模式
			if((motor1.motor_pid_p.cur_error>=endref)||(motor1.motor_pid_p.cur_error<=-endref)){
				motor1.motor_pid_v.ref=(motor1.motor_pid_p.cur_error>0?1:-1)*MAX_SPEED;
			}
			else if(motor1.motor_pid_p.cur_error<=endref&&motor1.motor_pid_p.cur_error>=-endref){
				motor1.motor_pid_v.ref=motor1.motor_pid_p.cur_error/endref*120+(motor1.motor_pid_p.cur_error>0?1:-1)*0.2*120;
			}
			// 到达目标，进入主动刹车
			if((motor1.motor_pid_p.cur_error)<=(2/WHEEL_DIAMETER/PI)&&motor1.motor_pid_p.cur_error>=(-2/WHEEL_DIAMETER/PI)&&wflag1 == 1){
				set_motor_hold(&motor1, 1); // <--- 修改点：替换 motor_stop
				motor_stop(&motor1);
				wflag1=2;
			}
		}
	}

	// --- 任务完成判断与串口回报 ---
	if(wflag1==2&&wflag2==2&&once_transmit_flag==1){
		HAL_UART_Transmit(&huart3,(uint8_t*)&ack,1,HAL_MAX_DELAY);
		once_transmit_flag = 0;
		wflag1=0;
		wflag2=0;
		correct_enable = 1;
		if(endflag == 1) endflag=0;
	}

		// --- 速度环PID计算与执行 --- 
	rpm1=((counter_now1-32768)*1000*60/2.5/4/238);//修改
	// 增加低通滤波
  motor1.motor_pid_v.fdb = motor1.motor_pid_v.fdb * (1-filter_para1) + rpm1 * filter_para1; // 0.8为滤波系数，可根据需要调整
	PID_Calc(&motor1.motor_pid_v);
	__HAL_TIM_SET_COUNTER(&motor1.TIM_EncoderHandle,32768);

	rpm2=((counter_now2-32768)*1000*60/2.5/4/238);//修改
	// 增加低通滤波
  motor2.motor_pid_v.fdb = motor2.motor_pid_v.fdb * (1-filter_para2) + rpm2 * filter_para2; // 0.8为滤波系数，可根据需要调整
	PID_Calc(&motor2.motor_pid_v);
	__HAL_TIM_SET_COUNTER(&motor2.TIM_EncoderHandle,32768);
	
}

//pid控制
//void motor_pid_control_long(motor_data *motor_){//依据位置环提供当前误差分析，调节速度环的目标值，速度环负责输出
//	counter_now=__HAL_TIM_GetCounter(&motor_->TIM_EncoderHandle);
//	counter_posi=(counter_now-32768.0)/(459*4);
//	motor_->motor_pid_p.fdb+=counter_posi;
//	PID_Pos_Calc(&motor_->motor_pid_p);
//	motor_->errorPercent=motor_->motor_pid_p.cur_error/motor_->motor_pid_p.ref;
//	if(motor_->errorPercent>=0.9){//给予电机一个启动速度，避免启动时动力不够堵转
//		motor_->motor_pid_v.ref=(motor_->motor_pid_p.ref>0?1:-1)*40;//考虑了设定位置目标值的正负，这就可以保证速度环输出方向的正确性
//	}
//	if(motor_->errorPercent<=0.9&&motor_->errorPercent>=0.3){//100等参数是使几个条件句联系起来，避免输出震荡而设置的
//    motor_->motor_pid_v.ref=(motor_->motor_pid_p.ref>0?1:-1)*(100*(1-motor_->errorPercent)+30);
//	}
//	if(0<motor_->errorPercent<=0.3){
//		motor_->motor_pid_v.ref=(motor_->motor_pid_p.ref>0?1:-1)*250*(motor_->errorPercent+0.1);//0.1是考虑了末端阻力，还需要根据实际情况调整
//	}
//	if(motor_->errorPercent<=0){//使电机停转，可能还会用到驱动的刹车模式，后续启动时还要调用电机启动函数
//		motor_stop(motor_);
//		HAL_UART_Transmit_DMA(&huart3,"1",10);
//	}
//	rpm=((counter_now-32768)*1000*60/10/4/459);
//	motor_->motor_pid_v.fdb=rpm;
//	PID_Calc(&motor_->motor_pid_v);
//	__HAL_TIM_SET_COUNTER(&motor_->TIM_EncoderHandle,32768);
//}
//void motor_pid_control_short(motor_data *motor_){//依据位置环提供当前误差分析，调节速度环的目标值，速度环负责输出
//	counter_now=__HAL_TIM_GetCounter(&motor_->TIM_EncoderHandle);
//	counter_posi=(counter_now-32768.0)/(459*4);
//	motor_->motor_pid_p.fdb+=counter_posi;
//	PID_Pos_Calc(&motor_->motor_pid_p);
//	motor_->errorPercent=motor_->motor_pid_p.cur_error/motor_->motor_pid_p.ref;
//	if(motor_->errorPercent>=0.9){//给予电机一个启动速度，避免启动时动力不够堵转
//		motor_->motor_pid_v.ref=(motor_->motor_pid_p.ref>0?1:-1)*15;//考虑了设定位置目标值的正负，这就可以保证速度环输出方向的正确性
//	}
//	if(motor_->errorPercent<=0.9&&motor_->errorPercent>=0.3){//100等参数是使几个条件句联系起来，避免输出震荡而设置的
//    motor_->motor_pid_v.ref=(motor_->motor_pid_p.ref>0?1:-1)*(50*(1-motor_->errorPercent)+10);
//	}
//	if(0<motor_->errorPercent<=0.3){
//		motor_->motor_pid_v.ref=(motor_->motor_pid_p.ref>0?1:-1)*45/0.4*(motor_->errorPercent+0.1);//0.1是考虑了末端阻力，还需要根据实际情况调整
//	}
//	if(motor_->errorPercent<=0){//使电机停转，可能还会用到驱动的刹车模式，后续启动时还要调用电机启动函数
//		motor_stop(motor_);
//		HAL_UART_Transmit_DMA(&huart3,"1",10);
//	}
//	rpm=((counter_now-32768)*1000*60/10/4/459);
//	motor_->motor_pid_v.fdb=rpm;
//	PID_Calc(&motor_->motor_pid_v);
//	__HAL_TIM_SET_COUNTER(&motor_->TIM_EncoderHandle,32768);
//}

/*自由停止
void motor_pid_control(){//依据位置环提供当前误差分析，调节速度环的目标值，速度环负责输出
		//
	// 仅当电机在运行时才增加计时器
	if (wflag1 == 1 || wflag2 == 1)
	{
		movement_timer++;
	}

	// 检查是否超时
	if (timeout_enabled && (movement_timer > timeout_ticks))
	{
		// 如果电机1仍在运行，则强制停止并标记为完成
		if (wflag1 == 1) { 
			motor1.motor_pid_v.ref = 0;
			motor_stop(&motor1);
			wflag1 = 2;
		}
		// 如果电机2仍在运行，则强制停止并标记为完成
		if (wflag2 == 1) {
			motor2.motor_pid_v.ref = 0;
			motor_stop(&motor2);
			wflag2 = 2;
		}
		movement_timer = 0; // 复位计时器，避免重复触发
	}
	counter_now1=__HAL_TIM_GetCounter(&motor1.TIM_EncoderHandle);
	counter_now2=__HAL_TIM_GetCounter(&motor2.TIM_EncoderHandle);
//	counter_posi=(counter_now-32768.0)/(459*4);
	distanceRef1=distance1/10/WHEEL_DIAMETER/PI;
	distanceRef2=distance2/10/WHEEL_DIAMETER/PI;
	double ref=80/WHEEL_DIAMETER/PI;
	//double reff=200/WHEEL_DIAMETER/PI; //实际上没有用到了
	double endref=40/WHEEL_DIAMETER/PI;
	motor1.motor_pid_p.fdb=distanceRef2;
	motor2.motor_pid_p.fdb=distanceRef1;
	motor1.motor_pid_p.cur_error=motor1.motor_pid_p.ref-motor1.motor_pid_p.fdb;
	motor2.motor_pid_p.cur_error=motor1.motor_pid_p.ref-motor2.motor_pid_p.fdb;
//	int is_posi=(&motor1.motor_pid_p.cur_error>0?1:-1);

//	if((motor_->motor_pid_p.cur_error>=reff)||(motor_->motor_pid_p.cur_error<=-reff)){
//		motor_->motor_pid_v.ref=-(is_posi)*((is_posi)*motor_->motor_pid_p.cur_error-reff)/ref*450+(motor_->motor_pid_p.cur_error>0?1:-1)*500;
//	}
	if(endflag==0){
		if((motor1.motor_pid_p.cur_error>=ref)||(motor1.motor_pid_p.cur_error<=-ref)){
		motor1.motor_pid_v.ref=(motor1.motor_pid_p.cur_error>0?1:-1)*MAX_SPEED;
	}
	else if(motor1.motor_pid_p.cur_error<=ref&&motor1.motor_pid_p.cur_error>=-ref){
		motor1.motor_pid_v.ref=motor1.motor_pid_p.cur_error/ref*370+(motor1.motor_pid_p.cur_error>0?1:-1)*0.2*100;
	}
	if((motor1.motor_pid_p.cur_error)<=(0.8/WHEEL_DIAMETER/PI)&&motor1.motor_pid_p.cur_error>=(-0.8/WHEEL_DIAMETER/PI)&&wflag1 == 1){
		motor1.motor_pid_v.ref=0;
		motor_stop(&motor1);
		wflag1=2;
	}
	if((motor2.motor_pid_p.cur_error>=ref)||(motor2.motor_pid_p.cur_error<=-ref)){
		motor2.motor_pid_v.ref=(motor2.motor_pid_p.cur_error>0?1:-1)*MAX_SPEED;
	}
	else if(motor2.motor_pid_p.cur_error<=ref&&motor2.motor_pid_p.cur_error>=-ref){
		motor2.motor_pid_v.ref=motor2.motor_pid_p.cur_error/ref*370+(motor2.motor_pid_p.cur_error>0?1:-1)*0.2*100;
	}
	if((motor2.motor_pid_p.cur_error)<=(0.8/WHEEL_DIAMETER/PI)&&motor2.motor_pid_p.cur_error>=(-0.8/WHEEL_DIAMETER/PI)&&wflag2 == 1){
		motor2.motor_pid_v.ref=0;
		motor_stop(&motor2);
		wflag2=2;
	}
	if(wflag1==2&&wflag2==2){
		HAL_UART_Transmit(&huart3,"1\r\n",3,0xFF);
		wflag1=0;
		wflag2=0;
	}
}else if(endflag == 1){
		if((motor1.motor_pid_p.cur_error>=endref)||(motor1.motor_pid_p.cur_error<=-endref)){
		motor1.motor_pid_v.ref=(motor1.motor_pid_p.cur_error>0?1:-1)*MAX_SPEED;
	}
	else if(motor1.motor_pid_p.cur_error<=endref&&motor1.motor_pid_p.cur_error>=-endref){
		motor1.motor_pid_v.ref=motor1.motor_pid_p.cur_error/endref*120+(motor1.motor_pid_p.cur_error>0?1:-1)*0.2*120;
	}
	if((motor1.motor_pid_p.cur_error)<=(2/WHEEL_DIAMETER/PI)&&motor1.motor_pid_p.cur_error>=(-2/WHEEL_DIAMETER/PI)&&wflag1 == 1){
		motor1.motor_pid_v.ref=0;
		motor_stop(&motor1);
		wflag1=2;
	}
	if((motor2.motor_pid_p.cur_error>=endref)||(motor2.motor_pid_p.cur_error<=-endref)){
		motor2.motor_pid_v.ref=(motor2.motor_pid_p.cur_error>0?1:-1)*MAX_SPEED;
	}
	else if(motor2.motor_pid_p.cur_error<=endref&&motor2.motor_pid_p.cur_error>=-endref){
		motor2.motor_pid_v.ref=motor2.motor_pid_p.cur_error/endref*120+(motor2.motor_pid_p.cur_error>0?1:-1)*0.2*120;
	}
	if((motor2.motor_pid_p.cur_error)<=(2/WHEEL_DIAMETER/PI)&&motor2.motor_pid_p.cur_error>=(-2/WHEEL_DIAMETER/PI)&&wflag2 == 1){
		motor2.motor_pid_v.ref=0;
		motor_stop(&motor2);
		wflag2=2;
	}
	if(wflag1==2&&wflag2==2){
		HAL_UART_Transmit(&huart3,"1\r\n",3,0xFF);
		wflag1=0;
		wflag2=0;
		endflag=0;
	}
}
		
	rpm1=((counter_now1-32768)*1000*60/5/4/459);
	motor1.motor_pid_v.fdb=rpm1;
	PID_Calc(&motor1.motor_pid_v);
	__HAL_TIM_SET_COUNTER(&motor1.TIM_EncoderHandle,32768);
	rpm2=((counter_now2-32768)*1000*60/5/4/459);
	motor2.motor_pid_v.fdb=rpm2;
	PID_Calc(&motor2.motor_pid_v);
	__HAL_TIM_SET_COUNTER(&motor2.TIM_EncoderHandle,32768);
}
*/