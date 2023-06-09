#ifndef PID_H
#define PID_H
#include "struct_typedef.h"
enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;
    //PID 三参数
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //最大输出
    fp32 max_iout; //最大积分输出

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
    fp32 error[3]; //误差项 0最新 1上一次 2上上次

} pid_type_def;

extern void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);


extern fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set);


extern void PID_clear(pid_type_def *pid);

typedef enum{

	PID_Position,
	PID_Speed

} PID_ID;

typedef struct _PID_TypeDef
{
	PID_ID id;
	fp32 target;
	fp32 lastNoneZeroTarget;
	fp32 kp;
	fp32 ki;
	fp32 kd;
	
	fp32 measure;
	fp32 err;
	fp32 last_err;
	
	fp32 pout;
	fp32 iout;
	fp32 dout;
	
	fp32 output;
	fp32 last_output;
	fp32 MaxOutput;
	fp32 IntegralLimit;
	fp32 DeadBand;
	fp32 ControlPeriod;
	fp32 Max_Err;
	
	uint32_t thistime;
	uint32_t lasttime;
	uint8_t dtime;
	
	void(*f_param_init)(struct _PID_TypeDef *pid,  //PID参数初始化
				   PID_ID id,
				   uint16_t maxOutput,
				   uint16_t integralLimit,
				   fp32 deadband,
				   uint16_t controlPeriod,
					 int16_t max_err,     
					 int16_t  target,
				   fp32 kp,
				   fp32 ki,
				   fp32 kd);
  void (*f_pid_reset)(struct _PID_TypeDef *pid, fp32 kp,fp32 ki, fp32 kd);	 //修改三个参数
	fp32 (*f_cal_pid)(struct _PID_TypeDef *pid, fp32 measure); //计算pid
	
	
} PID_TypeDef_M1502;

void pid_init(PID_TypeDef_M1502* pid);

#endif
