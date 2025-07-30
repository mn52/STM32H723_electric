#include "stm32h7xx.h"
#include "My_I2C.h"
#include "PCA9685.h"
void PCA_Write(uint8_t Addr,uint8_t Data){

    IIC_Start_PCA();
	
    IIC_SendByte_PCA(PCA_Addr);
    IIC_NAck_PCA();
	
	IIC_SendByte_PCA(Addr);
	IIC_NAck_PCA();
	
	IIC_SendByte_PCA(Data);
	IIC_NAck_PCA();
	
	IIC_Stop_PCA();
	
}

uint8_t PCA_Read(uint8_t Addr){

    uint8_t Data;
	
    IIC_Start_PCA();
	
    IIC_SendByte_PCA(PCA_Addr);
    IIC_NAck_PCA();
	
	IIC_SendByte_PCA(Addr);
	IIC_NAck_PCA();

	IIC_Stop_PCA();
	
	iic_delay_PCA();
	
	IIC_Start_PCA();
	
	IIC_SendByte_PCA(PCA_Addr |0x01);
	IIC_NAck_PCA();
	
	Data=(uint8_t)IIC_ReadByte_PCA(0);
	
	IIC_Stop_PCA();
	
	return Data;
	
}

void PCA_SetPWM(uint8_t Channel,uint32_t ON,uint32_t OFF){

    int ACK;
    IIC_Start_PCA();
	
    IIC_SendByte_PCA(PCA_Addr);
	ACK=IIC_WaitAck_PCA();
	
	IIC_SendByte_PCA(LED0_ON_L+4*Channel);
	ACK=IIC_WaitAck_PCA();
	
	IIC_SendByte_PCA(ON&0xFF);
	ACK=IIC_WaitAck_PCA();
	
	IIC_SendByte_PCA(ON>>8);
	ACK=IIC_WaitAck_PCA();
	
	IIC_SendByte_PCA(OFF&0xFF);
	ACK=IIC_WaitAck_PCA();
	
	IIC_SendByte_PCA(OFF>>8);
	ACK=IIC_WaitAck_PCA();
	
	IIC_Stop_PCA();
}

void PCA_SetFreq(float freq){

    uint8_t prescale,oldmode,newmode;
	double prescaleValue;
	freq*=0.99;//333hz�£�ϵ��Ϊ0.9164�����������������ܴﵽ20.000955ms
	prescaleValue=25000000;//pca9685�ڲ�ʱ��Ϊ25MHz
	prescaleValue/=4096;//�ֱ���
	prescaleValue/=freq;
	prescaleValue-=1;
	prescale=prescaleValue+0.5f;//��������
	oldmode=PCA_Read(PCA_Mode1);
	
	newmode=(oldmode&0x7F) | 0x10;
	PCA_Write(PCA_Mode1,newmode);
	PCA_Write(PCA_Pre,prescale);
	PCA_Write(PCA_Mode1,oldmode);
	
	HAL_Delay(6);
	
	PCA_Write(PCA_Mode1,oldmode |0xA1);
	
}

void SetAngle(uint8_t Channel, uint16_t angel){

    int off=0;
	off=(int)((225-angel)*2.2777777);
	PCA_SetPWM(Channel,0,off);
	
}

void PCA_Init(float hz){

	//uint32_t off=0;
    IIC_Init_PCA();
	PCA_Write(PCA_Mode1,0x00);
	PCA_SetFreq(hz);
//	off=(uint32_t)(angel*17.5);
//	
//	PCA_SetPWM(0,0,off);
	
	HAL_Delay(100);

}



