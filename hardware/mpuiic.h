#ifndef __MPUIIC_H
#define __MPUIIC_H

//IO方向设置
#define MPU_SDA_IN()  {GPIOA->CRL&=0XFF0FFFFF;GPIOA->CRL|=8<<20;}		//等于 MPU_IIC_SDA(1);
#define MPU_SDA_OUT() {GPIOA->CRL&=0XFF0FFFFF;GPIOA->CRL|=3<<20;}       //等于 MPU_IIC_SDA(0);

//IO操作函数	 
//#define MPU_IIC_SCL    PBout(10) 		//SCL
//#define MPU_IIC_SDA    PBout(11) 		//SDA	 
//#define MPU_READ_SDA   PBin(11) 		//输入SDA 


#define MPU_IIC_SCL(n)    HAL_GPIO_WritePin(GPIOA,mpu6050_SCL_Pin,(GPIO_PinState)n) 		//SCL
//#define MPU_IIC_SDA(n)    HAL_GPIO_WritePin(GPIOA,mpu6050_SDA_Pin,(GPIO_PinState)n) 		//SDA	 
#define MPU_READ_SDA   HAL_GPIO_ReadPin(GPIOA,mpu6050_SDA_Pin) 		//输入SDA 

void MPU_IIC_SDA(uint8_t BitValue);		//使用宏定义会有（变量和枚举类型混合警告）为了去除警告封装成函数操作

//IIC所有操作函数
void MPU_IIC_Delay(void);				//MPU IIC延时函数
void MPU_IIC_Init(void);                //初始化IIC的IO口				 
void MPU_IIC_Start(void);				//发送IIC开始信号
void MPU_IIC_Stop(void);	  			//发送IIC停止信号
void MPU_IIC_Send_Byte(uint8_t txd);			//IIC发送一个字节
uint8_t MPU_IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
uint8_t MPU_IIC_Wait_Ack(void); 				//IIC等待ACK信号
void MPU_IIC_Ack(void);					//IIC发送ACK信号
void MPU_IIC_NAck(void);				//IIC不发送ACK信号

void IMPU_IC_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data);
uint8_t MPU_IIC_Read_One_Byte(uint8_t daddr,uint8_t addr);	  
void MY_Delay_us(uint32_t us);

void delay_ms(uint16_t  nms);

#endif
















