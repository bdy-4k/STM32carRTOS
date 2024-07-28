#ifndef __MPUIIC_H
#define __MPUIIC_H

//IO��������
#define MPU_SDA_IN()  {GPIOA->CRL&=0XFF0FFFFF;GPIOA->CRL|=8<<20;}		//���� MPU_IIC_SDA(1);
#define MPU_SDA_OUT() {GPIOA->CRL&=0XFF0FFFFF;GPIOA->CRL|=3<<20;}       //���� MPU_IIC_SDA(0);

//IO��������	 
//#define MPU_IIC_SCL    PBout(10) 		//SCL
//#define MPU_IIC_SDA    PBout(11) 		//SDA	 
//#define MPU_READ_SDA   PBin(11) 		//����SDA 


#define MPU_IIC_SCL(n)    HAL_GPIO_WritePin(GPIOA,mpu6050_SCL_Pin,(GPIO_PinState)n) 		//SCL
//#define MPU_IIC_SDA(n)    HAL_GPIO_WritePin(GPIOA,mpu6050_SDA_Pin,(GPIO_PinState)n) 		//SDA	 
#define MPU_READ_SDA   HAL_GPIO_ReadPin(GPIOA,mpu6050_SDA_Pin) 		//����SDA 

void MPU_IIC_SDA(uint8_t BitValue);		//ʹ�ú궨����У�������ö�����ͻ�Ͼ��棩Ϊ��ȥ�������װ�ɺ�������

//IIC���в�������
void MPU_IIC_Delay(void);				//MPU IIC��ʱ����
void MPU_IIC_Init(void);                //��ʼ��IIC��IO��				 
void MPU_IIC_Start(void);				//����IIC��ʼ�ź�
void MPU_IIC_Stop(void);	  			//����IICֹͣ�ź�
void MPU_IIC_Send_Byte(uint8_t txd);			//IIC����һ���ֽ�
uint8_t MPU_IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
uint8_t MPU_IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void MPU_IIC_Ack(void);					//IIC����ACK�ź�
void MPU_IIC_NAck(void);				//IIC������ACK�ź�

void IMPU_IC_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data);
uint8_t MPU_IIC_Read_One_Byte(uint8_t daddr,uint8_t addr);	  
void MY_Delay_us(uint32_t us);

void delay_ms(uint16_t  nms);

#endif
















