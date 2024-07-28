#ifndef __CONTROL_H_
#define __CONTROL_H_

#include "main.h"

typedef struct
{
	float Kp;
	float Ki;
	float Kd;
	int16_t Error;		//ŒÛ≤Ó
	int16_t Error_sum;	//ŒÛ≤Ó¿€º∆
	int16_t Error_last;	//…œ¥ŒŒÛ≤Ó
	int16_t OUT;
}PID_TypeDef;

#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))


int16_t GFP_abs(int16_t p);
void Limit(int16_t  *motoA);
void Load(int16_t moto1,int16_t moto2);
void Set_PWM1(uint16_t PWM1,uint8_t direction1);
void Set_PWM2(uint16_t PWM2,uint8_t direction2);
void Get_encoder(int16_t *g_encoder_left,int16_t *g_encoder_tight);

int16_t Set_PID_UP(PID_TypeDef *PID_UP,float expect_angle,float angle,short gyroy_Y);
int16_t Set_PID_Speed(PID_TypeDef *PID_Speed,int16_t Target,int16_t encoder_left,int16_t encoder_tight);

#endif
