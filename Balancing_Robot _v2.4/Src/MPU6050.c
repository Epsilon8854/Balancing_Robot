/**
  ******************************************************************************
  * @file    MPU6050.c
  * @author  Waveshare Team
  * @version V1.0
  * @date    29-August-2014
  * @brief   This file includes the MPU6050 driver functions
  
  ******************************************************************************
  * @attention
  *
  *Waveshare�� �ҽ��� HAL���̺귯������ ���۵ǵ��� ������
  ******************************************************************************
  */

#include "MPU6050.h"
#include "I2C.h"

MPU6050_TypeDef MPU6050_GyroOffset, MPU6050_AccOffset;

//������ ���� 1����Ʈ �б� 
// �Ķ����1 : ���� ��巹��, �Ķ���� 2 : ������ �������� ��巹��
// ���ϰ� : �������䰪

uint8_t MPU6050_ReadOneByte(uint8_t RegAddr)
{
	uint8_t Data = 0;
	HAL_I2C_Mem_Read(&MPU6050_I2C_PORT,MPU6050_DEVICE_ADDR,RegAddr,1,&Data,1,1000);
	return Data;
}

//������  1����Ʈ ���� 
// �Ķ����1 : ���� ��巹��, �Ķ���� 2 : ������ �������� ��巹��
// ���ϰ� : �������䰪
void MPU6050_WriteOneByte(uint8_t RegAddr, uint8_t Data)
{
	HAL_I2C_Mem_Write(&MPU6050_I2C_PORT,MPU6050_DEVICE_ADDR,RegAddr,I2C_MEMADD_SIZE_8BIT,&Data,1,1000);
}

bool MPU6050_WriteBits(uint8_t RegAddr, uint8_t BitStart, uint8_t Length, uint8_t Data)
{

   uint8_t Dat, Mask;
    
		Dat = MPU6050_ReadOneByte(RegAddr);
    Mask = (0xFF << (BitStart + 1)) | 0xFF >> ((8 - BitStart) + Length - 1);
    Data <<= (8 - Length);
    Data >>= (7 - BitStart);
    Dat &= Mask;
    Dat |= Data;
    MPU6050_WriteOneByte(RegAddr, Dat);
    
    return true;
}

bool MPU6050_WriteOneBit(uint8_t RegAddr, uint8_t BitNum, uint8_t Data)
{
    uint8_t Dat;
    
    Dat = MPU6050_ReadOneByte(RegAddr);
    Dat = (Data != 0) ? (Dat | (1 << BitNum)) : (Dat & ~(1 << BitNum));
    MPU6050_WriteOneByte(RegAddr, Dat);
    
    return true;
}

//�����б� (����̽� ��巹��, �������� ��巹��, ������ ũ��, ���� ������)
bool MPU6050_ReadBuff(uint8_t RegAddr, uint8_t Num, uint8_t *pBuff)
{
	// �޸� �б�(����̽� ��巹��, 8��Ʈ ��巹�� �޸� ũ��, ���� ������, ���ۼ���, �õ�Ƚ��)
	return HAL_I2C_Mem_Read(&MPU6050_I2C_PORT,MPU6050_DEVICE_ADDR,RegAddr,I2C_MEMADD_SIZE_8BIT,pBuff,Num,1000);
}

bool MPU6050_Check(void) 
{
   	if(MPU6050_ADDRESS_AD0_LOW == MPU6050_ReadOneByte(MPU6050_RA_WHO_AM_I))  
   	{
   		return true;
   	}
   	else 
   	{
   		return false;
   	}
}

void MPU6050_CalAvgValue(uint8_t *pIndex, int16_t *pAvgBuffer, int16_t InVal, int32_t *pOutVal)
{	
	uint8_t i;
	
  	*(pAvgBuffer + ((*pIndex) ++)) = InVal;
  	*pIndex &= 0x07;
  	
  	*pOutVal = 0;
	for(i = 0; i < 8; i ++) 
  	{
    	*pOutVal += *(pAvgBuffer + i);
  	}
  	*pOutVal >>= 3;
}

void MPU6050_SetClockSource(uint8_t source)
{
    MPU6050_WriteBits(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

void MPU6050_SetFullScaleGyroRange(uint8_t range)
{
    MPU6050_WriteBits(MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

void MPU6050_SetLPF(uint8_t	LowPassFilter)
{
   MPU6050_WriteOneByte(MPU6050_RA_CONFIG, LowPassFilter);
}

void MPU6050_SetFullScaleAccelRange(uint8_t range)
{
    MPU6050_WriteBits(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

void MPU6050_SetSleepEnabled(uint8_t enabled) 
{
    MPU6050_WriteOneBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

void MPU6050_SetI2CMasterModeEnabled(uint8_t enabled) 
{
    MPU6050_WriteOneBit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

void MPU6050_SetI2CBypassEnabled(uint8_t enabled) 
{
    MPU6050_WriteOneBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}

void MPU6050_GetData(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* tmpr) 
{
	uint8_t Buffer[14] = {0};
	int16_t InBuffer[7] = {0};
	int16_t temp;

	MPU6050_ReadBuff(MPU6050_RA_ACCEL_XOUT_H, 14, Buffer);
	InBuffer[0] = (((int16_t)Buffer[0]) << 8) | Buffer[1];
	InBuffer[1] = (((int16_t)Buffer[2]) << 8) | Buffer[3];
	InBuffer[2] = (((int16_t)Buffer[4]) << 8) | Buffer[5];

	InBuffer[3] = (((int16_t)Buffer[8]) << 8) | Buffer[9];
	InBuffer[4] = (((int16_t)Buffer[10]) << 8) | Buffer[11];
	InBuffer[5] = (((int16_t)Buffer[12]) << 8) | Buffer[13];
	
	temp = (((int16_t)Buffer[6]) << 8) | Buffer[7];
	InBuffer[6] = (int16_t)(temp* 10L / 34) + 3653;
	
	*ax = *(InBuffer + 0);
	*ay = *(InBuffer + 1);
	*az = *(InBuffer + 2);
	*gx = *(InBuffer + 3);
	*gy = *(InBuffer + 4);
	*gz = *(InBuffer + 5);
	*tmpr = *(InBuffer + 6);
}

void MPU6050_InitOffset(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* tmpr)
{
	uint8_t i;
	int32_t	 TempAx = 0, TempAy = 0, TempAz = 0, TempGx = 0, TempGy = 0, TempGz = 0;
	
 	for(i = 0; i < 32; i ++)
 	{
		MPU6050_GetData(ax,ay,az,gx,gy,gz,tmpr);

		TempAx += *ax;
		TempAy += *ay;
		TempAz += *az;		
		TempGx += *gx;
		TempGy += *gy;
		TempGz += *gz;

		HAL_Delay(50);
	}

	MPU6050_AccOffset.X = TempAx >> 5;
	MPU6050_AccOffset.Y = TempAy >> 5;
	MPU6050_AccOffset.Z = TempAz >> 5;
	
	MPU6050_GyroOffset.X = TempGx >> 5;
	MPU6050_GyroOffset.Y = TempGy >> 5;
	MPU6050_GyroOffset.Z = TempGz >> 5;

}

void MPU6050_Init(uint8_t	lpf)
{
  MPU6050_SetClockSource(MPU6050_CLOCK_PLL_XGYRO); 
  MPU6050_SetFullScaleGyroRange(MPU6050_GYRO_FS_250);
  MPU6050_SetFullScaleAccelRange(MPU6050_ACCEL_FS_2);
	
	/*MPU6050_DLPF_BW_256         0x00
		MPU6050_DLPF_BW_188         0x01
		MPU6050_DLPF_BW_98          0x02
		MPU6050_DLPF_BW_42          0x03
		MPU6050_DLPF_BW_20          0x04
		MPU6050_DLPF_BW_10          0x05
		MPU6050_DLPF_BW_5           0x06
	 */
  MPU6050_SetLPF(lpf);

  MPU6050_SetSleepEnabled(0); 
  MPU6050_SetI2CMasterModeEnabled(0);
  MPU6050_SetI2CBypassEnabled(1);

  MPU6050_WriteOneBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_LEVEL_BIT, 0);
  MPU6050_WriteOneBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_OPEN_BIT, 0);
  MPU6050_WriteOneBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_LATCH_INT_EN_BIT, 1);
  MPU6050_WriteOneBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_RD_CLEAR_BIT, 1);
  MPU6050_WriteOneBit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DATA_RDY_BIT, 1);
  HAL_Delay(100);  // ���̷� ����ȭ ���
}
