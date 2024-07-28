#include "control.h"

/********************************* 
   ֱ����
ֱ����PD��������Kp*Ek+Kd*Ek_D
���룺 �����Ƕ�  ��ʵ�Ƕ�   ���ٶ�
�����pwm
*********************************/
int16_t Set_PID_UP(PID_TypeDef *PID_UP,float expect_angle,float angle,short gyroy_Y)
{
//	int16_t output;
	
	PID_UP->OUT = PID_UP->Kp * (angle - expect_angle) + PID_UP->Kd * (gyroy_Y - 0);
	
	return PID_UP->OUT;
}
/*******************************
	�ٶȻ�
�ٶȻ�PI��Kp*Ek+Ki*Ek_S
���룺��е��ֵ  �����ٶ�  �����ٶ�
*******************************/
int16_t Set_PID_Speed(PID_TypeDef *PID_Speed,int16_t Target,int16_t encoder_left,int16_t encoder_tight)
{
//	static int16_t output,encoder_Err,enc_Err_Low,encoder_S;
//	static int16_t enc_Err_last = 0;
//	int16_t output;
	float a = 0.5;
	
	//�����ٶ�ƫ��
//	encoder_Err = (encoder_left + encoder_tight) - Target;
	
	//�������˲�
	PID_Speed->Error = (1 - a) * ((encoder_left + encoder_tight) - Target) + a * PID_Speed->Error_last;
	PID_Speed->Error_last = PID_Speed->Error;
	
	//���ٶ�ƫ�����
	PID_Speed->Error_sum += PID_Speed->Error;
	
	//�����޷�
//	encoder_S = encoder_S > 10000 ? 10000: (encoder_S < (-10000) ? (-10000): encoder_S);
	
	if(PID_Speed->Error_sum > 10000)
		PID_Speed->Error_sum = 10000;
	if(PID_Speed->Error_sum < -10000)
		PID_Speed->Error_sum = -10000;
	
//	if(pitch > 40 || pitch < -40) encoder_S = 0;
	
	//�ٶȻ��������
	PID_Speed->OUT = PID_Speed->Kp * PID_Speed->Error + PID_Speed->Ki * PID_Speed->Error_sum;
	
	return PID_Speed->OUT;
}



void Get_encoder(int16_t *g_encoder_left,int16_t *g_encoder_tight)		//��ȡ������������
{

	*g_encoder_left = (int16_t)(__HAL_TIM_GET_COUNTER(&htim3) );         //��ȡCNT��ֵ
	*g_encoder_tight = (int16_t)(__HAL_TIM_GET_COUNTER(&htim4) );
//	__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3);					//�����ж���ת��ת//��ת1��ת0

//			distance = (uint16_t)(__HAL_TIM_GET_COUNTER(&htim2) );         //��ȡCNT��ֵ
//			__HAL_TIM_SET_COUNTER(&htim2,0);

	__HAL_TIM_SET_COUNTER(&htim3,0);								//���CNT��ֵ
	__HAL_TIM_SET_COUNTER(&htim4,0);
	
}

void Set_PWM2(uint16_t PWM2,uint8_t direction2)		//direction = 0ǰ��
{
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,(PWM2));
	if(direction2)
	{
		HAL_GPIO_WritePin(left1_GPIO_Port,left1_Pin,GPIO_PIN_SET);//
		HAL_GPIO_WritePin(left2_GPIO_Port,left2_Pin,GPIO_PIN_RESET);//
	}
	else
	{
		HAL_GPIO_WritePin(left1_GPIO_Port,left1_Pin,GPIO_PIN_RESET);//
		HAL_GPIO_WritePin(left2_GPIO_Port,left2_Pin,GPIO_PIN_SET);//
	}
}

void Set_PWM1(uint16_t PWM1,uint8_t direction1)
{
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,(PWM1));
	if(direction1)
	{
		HAL_GPIO_WritePin(tight2_GPIO_Port,tight1_Pin,GPIO_PIN_SET);//
		HAL_GPIO_WritePin(tight1_GPIO_Port,tight2_Pin,GPIO_PIN_RESET);//
	}
	else 
	{
		HAL_GPIO_WritePin(tight2_GPIO_Port,tight1_Pin,GPIO_PIN_RESET);//
		HAL_GPIO_WritePin(tight1_GPIO_Port,tight2_Pin,GPIO_PIN_SET);//
	}
}

void Load(int16_t moto1,int16_t moto2)
{
	uint8_t num1,num2;
	if(moto1 > 0)
	{	
		num1 = 0;
		moto1 = moto1+0;//+400
	}
	else
	{	
		num1 = 1;
		moto1 = moto1-0;//-350
	}
	if(moto2 > 0)
	{	
		num2 = 0;
		moto2 = moto2+0;//+400
	}
	else
	{	
		num2 = 1;
		moto2 = moto2-0;//-450
	}
	Set_PWM1(GFP_abs(moto1),num1);
	Set_PWM2(GFP_abs(moto2),num2);
}

///*�޷�����*/
void Limit(int16_t  *motoA)
{
	if(*motoA>7000)*motoA=7000;
	if(*motoA<-7000)*motoA=-7000;
	
//	if(*motoB>PWM_MAX)*motoB=PWM_MAX;
//	if(*motoB<PWM_MIN)*motoB=PWM_MIN;
}

/*����ֵ����*/
int16_t GFP_abs(int16_t p)
{
	int16_t q;
	q=p>0?p:(-p);
	return q;
}

