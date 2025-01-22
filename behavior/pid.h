#ifndef PID_H
#define PID_H


enum PID_MODE
{
	PID_POSITION = 0,
	PID_DELTA,
	PID_DIFF
};  //λ��ʽ������ʽ����

typedef struct
{
	unsigned char mode;
	float Kp;
	float Ki;
	float Kd;
	//΢������޷�
	float diff_max;
	float diff_min;
	//��������޷�
	float max_out;  
	float max_iout; 
	
	float set;
	float out;
	float Pout;
	float Iout;
	float Dout;
	float Dbuf[3];  
	float error[3]; 
} PidTypeDef;


typedef struct
{
PidTypeDef inner; //�ڻ��Ƕ�
PidTypeDef outer; //�⻷�ٶ�

}PID_DOUBLE;//����PID
#ifndef NULL
#define NULL 0
#endif



extern void PID_Init(PidTypeDef *pid, unsigned char mode, float kp, float ki, float kd, float max_out, float max_iout, float diff_max, float diff_min);

extern float PID_Calc(PidTypeDef *pid, float ref, float set);
extern float PID_Calc_add_limit(PidTypeDef *pid, float ref, float set, float T);


extern void PID_clear(PidTypeDef *pid);


extern float PID_CALCAL( PID_DOUBLE*PID, float angle_target, float angle_feedback, float speed_feedback, float inner_limit);

#endif


