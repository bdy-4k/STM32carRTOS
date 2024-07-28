#include "main.h"
#include "mpuiic.h"

 //MPU IIC ��ʱ����
void MPU_IIC_Delay(void)
{
	MY_Delay_us(2);
}

//��ʼ��IIC
void MPU_IIC_Init(void)
{					     
//  GPIO_InitTypeDef  GPIO_InitStructure;
//	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//��ʹ������IO PORTBʱ�� 
//		
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;	 // �˿�����
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; 		 //�������
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
//  GPIO_Init(GPIOB, &GPIO_InitStructure);					 //�����趨������ʼ��GPIO 
//	
//  GPIO_SetBits(GPIOB,GPIO_Pin_10|GPIO_Pin_11);						 //PB10,PB11 �����	
	MPU_IIC_SDA(1);	  	    
	MPU_IIC_SCL(1); 
}
//����IIC��ʼ�ź�
void MPU_IIC_Start(void)
{
	MPU_SDA_OUT();     //sda�����
	MPU_IIC_SDA(1);	  	    
	MPU_IIC_SCL(1); 
	MPU_IIC_Delay();
 	MPU_IIC_SDA(0);//START:when CLK is high,DATA change form high to low 
	MPU_IIC_Delay();
	MPU_IIC_SCL(0);//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void MPU_IIC_Stop(void)
{
	MPU_SDA_OUT();//sda�����
	MPU_IIC_SCL(0);
	MPU_IIC_SDA(0);//STOP:when CLK is high DATA change form low to high
 	MPU_IIC_Delay();
	MPU_IIC_SCL(1); 
	MPU_IIC_SDA(1);//����I2C���߽����ź�
	MPU_IIC_Delay();							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
uint8_t MPU_IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	MPU_SDA_IN();      //SDA����Ϊ����  
	MPU_IIC_SDA(1);MPU_IIC_Delay();	   
	MPU_IIC_SCL(1);MPU_IIC_Delay();	 
	while(MPU_READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			MPU_IIC_Stop();
			return 1;
		}
	}
	MPU_IIC_SCL(0);//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
void MPU_IIC_Ack(void)
{
	MPU_IIC_SCL(0);
	MPU_SDA_OUT();
	MPU_IIC_SDA(0);
	MPU_IIC_Delay();
	MPU_IIC_SCL(1);
	MPU_IIC_Delay();
	MPU_IIC_SCL(0);
}
//������ACKӦ��		    
void MPU_IIC_NAck(void)
{
	MPU_IIC_SCL(0);
	MPU_SDA_OUT();
	MPU_IIC_SDA(1);
	MPU_IIC_Delay();
	MPU_IIC_SCL(1);
	MPU_IIC_Delay();
	MPU_IIC_SCL(0);
}					 				     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void MPU_IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t;   
	MPU_SDA_OUT(); 	    
    MPU_IIC_SCL(0);//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        MPU_IIC_SDA((txd & 0x80)>> 7);
        txd<<=1; 	  
		    MPU_IIC_SCL(1);
		    MPU_IIC_Delay(); 
		    MPU_IIC_SCL(0);	
		    MPU_IIC_Delay();
//		HAL_I2C_Master_Transmit(hi2c2,);
    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
uint8_t MPU_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	MPU_SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        MPU_IIC_SCL(0); 
        MPU_IIC_Delay();
		MPU_IIC_SCL(1);
        receive<<=1;
        if(MPU_READ_SDA)receive++;   
		MPU_IIC_Delay(); 
    }					 
    if (!ack)
        MPU_IIC_NAck();//����nACK
    else
        MPU_IIC_Ack(); //����ACK   
    return receive;
}

//ʹ��ϵͳ�δ�ʱ����׼��ʱ΢��
void MY_Delay_us(uint32_t us)
{       
//    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000000);
//    HAL_Delay(us);									//����������Ҫ1usʱ��
//    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

   uint16_t i=0;  
   while(us--)
   {
      i=72;  //�Լ�����
      while(i--) ;    
   }
}
//
void MPU_IIC_SDA(uint8_t BitValue)
{
	HAL_GPIO_WritePin(GPIOA,mpu6050_SDA_Pin,(GPIO_PinState)BitValue);
}



//void delay_ms(uint16_t  nms)
//{
// uint32_t  temp;
// SysTick->LOAD = 9000*nms;
// SysTick->VAL=0X00;//��ռ�����
// SysTick->CTRL=0X01;//ʹ�ܣ����������޶����������ⲿʱ��Դ
// do
// {
//  temp=SysTick->CTRL;//��ȡ��ǰ������ֵ
// }while((temp&0x01)&&(!(temp&(1<<16))));//�ȴ�ʱ�䵽��
//    SysTick->CTRL=0x00; //�رռ�����
//    SysTick->VAL =0X00; //��ռ�����
//}



















