#include "MPU6050.h"

uint8_t mpu6050_write(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf)//返回值 0：读成功  -1：读失败
{ 
		// unsigned char i;
     //addr=addr<<1;                     //注意dma库地址不包含最后一位，需要移位
		 uint8_t i;
	     MPU6050_IIC_Start();              //启动总线
		 MPU6050_IIC_Send_Byte(addr << 1);      //发送器件地址           
		 MPU6050_IIC_Send_Byte(reg);       //发送器件子地址

		 for(i=0;i<len;i++)            
			 MPU6050_IIC_Send_Byte(*buf++);  //发送数据
		 MPU6050_IIC_Stop();               //结束总线

		 return 0;
}

uint8_t mpu6050_read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)//返回值 0：读成功  -1：读失败
{
		 unsigned char i;
     //addr=addr<<1;                     //注意dma库地址不包含最后一位，需要移位
		 MPU6050_IIC_Start();              //启动总线           
		 MPU6050_IIC_Send_Byte(addr << 1);      //发送器件地址   
         	
		 MPU6050_IIC_Send_Byte(reg);       //发送器件子地址

		 MPU6050_IIC_Start();              //重新启动总线
		 MPU6050_IIC_Send_Byte((addr << 1) | 0x01);
		 for(i=0;i<len-1;i++)  
			 *buf++=MPU6050_IIC_Read_Byte(0);//发送数据
		 *buf=MPU6050_IIC_Read_Byte(1);
		 MPU6050_IIC_Stop();               //结束总线
	
		 return 0;
}


void mpu6050_write_reg(uint8_t reg, uint8_t dat)
{
   mpu6050_write(MPU_ADDR,reg,1,&dat);
}

uint8_t   mpu6050_read_reg (uint8_t reg)
{
	 uint8_t dat;
   mpu6050_read(MPU_ADDR,reg,1,&dat);
	 return dat;
}

//设置MPU6050陀螺仪传感器满量程范围
//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
	mpu6050_write_reg(GYRO_CONFIG,fsr<<3);//设置陀螺仪满量程范围  
	return 0;
}
//设置MPU6050加速度传感器满量程范围
//fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
	mpu6050_write_reg(ACCEL_CONFIG,fsr<<3);//设置加速度传感器满量程范围  
	return 0;
}
//设置MPU6050的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU_Set_LPF(uint16_t lpf)
{
	uint8_t data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	mpu6050_write_reg(MPU_CFG_REG,data);//设置数字低通滤波器  
	return 0;
}
//设置MPU6050的采样率(假定Fs=1KHz)
//rate:4~1000(Hz)
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU_Set_Rate(uint16_t rate)
{
	uint8_t data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	mpu6050_write_reg(MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
 	return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}

void MPU6050_Init(void)
{ 
	uint8_t res; 
	MPU6050_IIC_IO_Init(); //初始化IIC总线
	mpu6050_write_reg(PWR_MGMT_1,0X80);	//复位MPU6050
    HAL_Delay(100);
	mpu6050_write_reg(PWR_MGMT_1,0X00);	//唤醒MPU6050 
	MPU_Set_Gyro_Fsr(3);					//陀螺仪传感器,±2000dps
	MPU_Set_Accel_Fsr(0);					//加速度传感器,±2g
	MPU_Set_Rate(200);						//设置采样率50Hz
	mpu6050_write_reg(MPU_INT_EN_REG,0X00);	//关闭所有中断
	mpu6050_write_reg(MPU_USER_CTRL_REG,0X00);	//I2C主模式关闭
	mpu6050_write_reg(MPU_FIFO_EN_REG,0X00);	//关闭FIFO
	mpu6050_write_reg(MPU_INTBP_CFG_REG,0X80);	//INT引脚低电平有效
	res=mpu6050_read_reg(MPU_DEVICE_ID_REG); 
	if(res==MPU_ADDR)//器件ID正确
	{
		mpu6050_write_reg(PWR_MGMT_1,0X01);	//设置CLKSEL,PLL X轴为参考
		mpu6050_write_reg(PWR_MGMT_2,0X00);	//加速度与陀螺仪都工作
		MPU_Set_Rate(200);		//原来为100				//设置采样率为50Hz
 	}
}
 
//得到温度值
//返回值:温度值(扩大了100倍)
short MPU_Get_Temperature(void)
{
    uint8_t buf[2]; 
    short raw;
	float temp;
	mpu6050_read(MPU_ADDR,TEMP_OUT_H,2,buf); 
    raw=((uint16_t)buf[0]<<8)|buf[1];  
    temp=36.53+((double)raw)/340;  
    return temp*100;
}
//得到陀螺仪值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
uint8_t MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    uint8_t buf[6],res;  
	res=mpu6050_read(MPU_ADDR,GYRO_XOUT_H,6,buf);
	if(res==0)
	{
		*gx=(short)(((uint16_t)buf[0]<<8)|buf[1]);  
		*gy=(short)(((uint16_t)buf[2]<<8)|buf[3]);  
		*gz=(short)(((uint16_t)buf[4]<<8)|buf[5]);
	} 	
    return res;
}
//得到加速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
uint8_t MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    uint8_t buf[6],res;  
	res=mpu6050_read(MPU_ADDR,ACCEL_XOUT_H,6,buf);
	if(res==0)
	{
		*ax=((uint16_t)buf[0]<<8)|buf[1];  
		*ay=((uint16_t)buf[2]<<8)|buf[3];  
		*az=((uint16_t)buf[4]<<8)|buf[5];
	} 	
    return res;
}
