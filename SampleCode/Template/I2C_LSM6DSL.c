
/* Includes ------------------------------------------------------------------*/
/* Includes ------------------------------------------------------------------*/
#include "I2C_LSM6DSL.h"

static int16_t LSM6DSL_ACCx,LSM6DSL_ACCy,LSM6DSL_ACCz;
static int16_t LSM6DSL_GYROx,LSM6DSL_GYROy,LSM6DSL_GYROz;

static int16_t cLSM6DSL_ACCx =0;
static int16_t cLSM6DSL_ACCy =0;
static int16_t cLSM6DSL_ACCz =0;

static int16_t cLSM6DSL_GYROx =0;
static int16_t cLSM6DSL_GYROy =0;
static int16_t cLSM6DSL_GYROz =0;

uint8_t FlagSwitch = 0;

#define PI (float)3.14159265f
float RollAng = 0.0f, PitchAng = 0.0f;
#define FILTER_COUNT  	(16)
#define FILTER_FACTOR  	(4)

#define	GYRO_SCALE 	(1)	//(70)

int16_t ax_buf[FILTER_COUNT], ay_buf[FILTER_COUNT],az_buf[FILTER_COUNT];
int16_t gx_buf[FILTER_COUNT], gy_buf[FILTER_COUNT],gz_buf[FILTER_COUNT];
int16_t gx, gy, gz, ax ,ay, az;
float angle, angle_dot, f_angle, f_angle_dot;

/*MEMS calibration*/
uint8_t Flag_Calibrate = 0;

/*misc marco*/
#define MEMSABS(X)						((X) >= 0 ? (X) : -(X))
#define MEMSCONVERTA(x,y)				(y = (x>=0)?(1):(0))
#define MEMSCONVERTB(x,y)				(x = (y == 1)?(-x):(x))

#define MEMSMAKEWORD(v1,v2)         ((((uint16_t)(v1))<<8)+(uint16_t)(v2))      //v1,v2 is UINT8


void LSM6DSL_Delay(uint16_t nCount)
{
    /* Decrement nCount value */
    while (nCount != 0)
    {
        nCount--;
    }
}

void appLSM6DSL_Read(uint8_t DeviceAddr, uint8_t RegisterAddr,
                              uint16_t NumByteToRead,
                              uint8_t* pBuffer)
{
	#if defined (ENABLE_I2C_POLLING_DISCRETE)
	uint8_t i, tmp;
	I2C_T *i2c = I2C_SENSOR_PORT;
	
	I2C_START(i2c);                         			//Start
	I2C_WAIT_READY(i2c);

	I2C_SET_DATA(i2c, DeviceAddr | I2C_WR );             		//send slave address+W
	I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
	I2C_WAIT_READY(i2c);

	I2C_SET_DATA(i2c, RegisterAddr);             		//send index
	I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
	I2C_WAIT_READY(i2c);

	I2C_SET_CONTROL_REG(i2c, I2C_CTL_STA_SI);		//Start
	I2C_WAIT_READY(i2c);

	I2C_SET_DATA(i2c, DeviceAddr | I2C_RD );    			//send slave address+R
	I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
	I2C_WAIT_READY(i2c);

	for (i=0; i<NumByteToRead; i++)
	{
		I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
		I2C_WAIT_READY(i2c);
		tmp = I2C_GET_DATA(i2c);           			//read data
		pBuffer[i]=tmp;
	}
	I2C_STOP(i2c);									//Stop

	#elif defined (ENABLE_I2C_POLLING_API)
	
	/* u8SlaveAddr     Access Slave address(7-bit) */
	uint8_t u8SlaveAddr = DeviceAddr >>1;
//	uint8_t i = 0;
	
	I2C_ReadMultiBytesOneReg(I2C_SENSOR_PORT, u8SlaveAddr, RegisterAddr, pBuffer, NumByteToRead);	
//	i = I2C_ReadByteOneReg(I2C_SENSOR_PORT, u8SlaveAddr, RegisterAddr);
//	*pBuffer =  i;

	#elif defined (ENABLE_I2C_IRQ)

	/* u8SlaveAddr     Access Slave address(7-bit) */
	uint8_t u8SlaveAddr = DeviceAddr >>1;
	
	I2Cx_ReadMultiFromSlaveIRQ(u8SlaveAddr , RegisterAddr, pBuffer, NumByteToRead);
	
	#endif
}

void appLSM6DSL_Write(uint8_t DeviceAddr, uint8_t RegisterAddr,
                               uint16_t NumByteToWrite,
                               uint8_t* pBuffer)
{
	#if defined (ENABLE_I2C_POLLING_DISCRETE)
	uint8_t i;
	uint32_t tmp;
	
	I2C_T *i2c = I2C_SENSOR_PORT;	
	I2C_START(i2c);                    			//Start
	I2C_WAIT_READY(i2c);

	I2C_SET_DATA(i2c, DeviceAddr | I2C_WR );        			//send slave address
	I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
	I2C_WAIT_READY(i2c);

	I2C_SET_DATA(i2c, RegisterAddr);        			//send index
	I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
	I2C_WAIT_READY(i2c);

	for (i=0; i<NumByteToWrite; i++)
	{
		tmp = pBuffer[i];
		I2C_SET_DATA(i2c, tmp);            		//send Data
		I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
		I2C_WAIT_READY(i2c);
	}

	I2C_STOP(i2c);								//Stop

	#elif defined (ENABLE_I2C_POLLING_API)

	/* u8SlaveAddr     Access Slave address(7-bit) */
	uint8_t u8SlaveAddr = DeviceAddr >>1;

	I2C_WriteMultiBytesOneReg(I2C_SENSOR_PORT, u8SlaveAddr, RegisterAddr, pBuffer, NumByteToWrite);
//	I2C_WriteByteOneReg(I2C_SENSOR_PORT, u8SlaveAddr , RegisterAddr, *pBuffer);	

	#elif defined (ENABLE_I2C_IRQ)

	/* u8SlaveAddr     Access Slave address(7-bit) */
	uint8_t u8SlaveAddr = DeviceAddr >>1;
	
	I2Cx_WriteMultiToSlaveIRQ(u8SlaveAddr , RegisterAddr, pBuffer, NumByteToWrite);

	#endif
}

void appLSM6DSL_SetAccCalData(AXIS_TypeDef axis,int16_t data)
{
	switch(axis)
	{
		case AXIS_X:
			cLSM6DSL_ACCx = data; 
			break;

		case AXIS_Y:
			cLSM6DSL_ACCy = data; 
			break;

		case AXIS_Z:
			cLSM6DSL_ACCz = data; 
			break;				
	}
}

void appLSM6DSL_SetGyroCalData(AXIS_TypeDef axis,int16_t data)
{
	switch(axis)
	{
		case AXIS_X:
			cLSM6DSL_GYROx = data; 
			break;

		case AXIS_Y:
			cLSM6DSL_GYROy = data; 
			break;

		case AXIS_Z:
			cLSM6DSL_GYROz = data; 
			break;				
	}
}

void appLSM6DSL_SetAccData(AXIS_TypeDef axis,int16_t data)
{
	switch(axis)
	{
		case AXIS_X:
			LSM6DSL_ACCx = data; 
			break;

		case AXIS_Y:
			LSM6DSL_ACCy = data; 
			break;

		case AXIS_Z:
			LSM6DSL_ACCz = data; 
			break;				
	}
}

void appLSM6DSL_SetGyroData(AXIS_TypeDef axis,int16_t data)
{
	switch(axis)
	{
		case AXIS_X:
			LSM6DSL_GYROx = data; 
			break;

		case AXIS_Y:
			LSM6DSL_GYROy = data; 
			break;

		case AXIS_Z:
			LSM6DSL_GYROz = data; 
			break;				
	}
}


int16_t appLSM6DSL_GetAccData(AXIS_TypeDef axis)
{
	int16_t data = 0;
	
	switch(axis)
	{
		case AXIS_X:
			data = LSM6DSL_ACCx + cLSM6DSL_ACCx; 
			break;

		case AXIS_Y:
			data = LSM6DSL_ACCy + cLSM6DSL_ACCy; 
			break;

		case AXIS_Z:
			data = LSM6DSL_ACCz + cLSM6DSL_ACCz; 
			break;				
	}
	
	return data ;
}

int16_t appLSM6DSL_GetGyroData(AXIS_TypeDef axis)
{
	int16_t data = 0;
	
	switch(axis)
	{
		case AXIS_X:
			data = LSM6DSL_GYROx + cLSM6DSL_GYROx; 
			break;

		case AXIS_Y:
			data = LSM6DSL_GYROy + cLSM6DSL_GYROy; 
			break;

		case AXIS_Z:
			data = LSM6DSL_GYROz + cLSM6DSL_GYROz; 
			break;				
	}
	
	return data ;
}


void appLSM6DSL_GetAcc(void)
{
	uint8_t tmpxl, tmpxh, tmpyl, tmpyh, tmpzl, tmpzh, tmp;
	int16_t ax_s,ay_s,az_s;
	uint8_t u8WaitCnt=0;
    float sensitivity = LSM6DSL_XL_FS_2G_SENSITIVITY;	//default
	
	do{
		appLSM6DSL_Read(ACC_ADDRESS, LSM6DSL_XG_STATUS_REG, 1,&tmp);
		if (u8WaitCnt++>30)
			break;
	}while(!(tmp&BIT(0)));

	#if 1	//calculate linear acceleration in mg
	appLSM6DSL_Read(ACC_ADDRESS, LSM6DSL_XG_CTRL1_XL, 1,&tmp);
    tmp &= LSM6DSL_XL_FS_MASK;
//	printf("tmp(A) = 0x%2X\r\n",tmp);//debug
    switch(tmp)
    {
      case LSM6DSL_XL_FS_2G:
        sensitivity = LSM6DSL_XL_FS_2G_SENSITIVITY;
        break;
      case LSM6DSL_XL_FS_4G:
        sensitivity = LSM6DSL_XL_FS_4G_SENSITIVITY;
        break;
      case LSM6DSL_XL_FS_8G:
        sensitivity = LSM6DSL_XL_FS_8G_SENSITIVITY;
        break;
      case LSM6DSL_XL_FS_16G:
        sensitivity = LSM6DSL_XL_FS_16G_SENSITIVITY;
        break;
    }
	#endif	

	appLSM6DSL_Read(ACC_ADDRESS, LSM6DSL_XG_OUT_X_H_XL, 1, &tmpxh);
	appLSM6DSL_Read(ACC_ADDRESS, LSM6DSL_XG_OUT_X_L_XL, 1, &tmpxl);
	ax_s = ((int16_t) ((tmpxh << 8) | tmpxl));
//	printf("ax_s:%2d,%2d,%2d |",ax_s,tmpxh,tmpxl);	//debug

	appLSM6DSL_Read(ACC_ADDRESS, LSM6DSL_XG_OUT_Y_H_XL, 1, &tmpyh);
	appLSM6DSL_Read(ACC_ADDRESS, LSM6DSL_XG_OUT_Y_L_XL, 1, &tmpyl);
	ay_s = ((int16_t) ((tmpyh << 8) | tmpyl));
//	printf("ay_s:%2d,%2d,%2d |",ay_s,tmpyh,tmpyl);	//debug	

	appLSM6DSL_Read(ACC_ADDRESS, LSM6DSL_XG_OUT_Z_H_XL, 1, &tmpzh);
	appLSM6DSL_Read(ACC_ADDRESS, LSM6DSL_XG_OUT_Z_L_XL, 1, &tmpzl);
	az_s = ((int16_t) ((tmpzh << 8) | tmpzl));
//	printf("az_s:%2d,%2d,%2d\r\n",az_s,tmpzh,tmpzl);	//debug	

	appLSM6DSL_SetAccData(AXIS_X,(int16_t)(ax_s*sensitivity));	
	appLSM6DSL_SetAccData(AXIS_Y,(int16_t)(ay_s*sensitivity));	
	appLSM6DSL_SetAccData(AXIS_Z,(int16_t)(az_s*sensitivity));	

	LSM6DSL_Delay(5);
}

void appLSM6DSL_GetGyro(void)
{
	uint8_t tmpxl, tmpxh, tmpyl, tmpyh, tmpzl, tmpzh, tmp;
	int16_t gx_s,gy_s,gz_s;
	uint8_t u8WaitCnt=0;
    float sensitivity = LSM6DSL_G_FS_125_SENSITIVITY;	//default
    
	do{
		appLSM6DSL_Read(GYRO_ADDRESS, LSM6DSL_XG_STATUS_REG, 1,&tmp);
		u8WaitCnt++;
		if (u8WaitCnt>30)
			break;
	}while(!(tmp&BIT(1)));

	#if 1	//calculate angular rate in mdps
	appLSM6DSL_Read(GYRO_ADDRESS, LSM6DSL_XG_CTRL2_G, 1,&tmp);
    tmp &= LSM6DSL_G_FS_MASK;
//	printf("tmp(G) = 0x%2X\r\n",tmp);//debug
    switch(tmp)
    {
      case LSM6DSL_G_FS_125:
        sensitivity = LSM6DSL_G_FS_125_SENSITIVITY;
        break;
      case LSM6DSL_G_FS_245:
        sensitivity = LSM6DSL_G_FS_245_SENSITIVITY;
        break;
      case LSM6DSL_G_FS_500:
        sensitivity = LSM6DSL_G_FS_500_SENSITIVITY;
        break;
      case LSM6DSL_G_FS_1000:
        sensitivity = LSM6DSL_G_FS_1000_SENSITIVITY;
        break;
      case LSM6DSL_G_FS_2000:
        sensitivity = LSM6DSL_G_FS_2000_SENSITIVITY;
        break;
    }
	#endif	

	appLSM6DSL_Read(GYRO_ADDRESS, LSM6DSL_XG_OUT_X_H_G, 1, &tmpxh);
	appLSM6DSL_Read(GYRO_ADDRESS, LSM6DSL_XG_OUT_X_L_G, 1, &tmpxl);
	gx_s = (((int16_t)(tmpxh << 8)) | ((int16_t)tmpxl));
//	printf("gx_s:%4d,%4d,%4d |",gx_s,tmpxh,tmpxl);	//debug

	appLSM6DSL_Read(GYRO_ADDRESS, LSM6DSL_XG_OUT_Y_H_G, 1, &tmpyh);
	appLSM6DSL_Read(GYRO_ADDRESS, LSM6DSL_XG_OUT_Y_L_G, 1, &tmpyl);
	gy_s = (((int16_t)(tmpyh << 8)) | ((int16_t)tmpyl));
//	printf("gy_s:%4d,%4d,%4d |",gy_s,tmpyh,tmpyl);	//debug

	appLSM6DSL_Read(GYRO_ADDRESS, LSM6DSL_XG_OUT_Z_H_G, 1, &tmpzh);
	appLSM6DSL_Read(GYRO_ADDRESS, LSM6DSL_XG_OUT_Z_L_G, 1, &tmpzl);
	gz_s = (((int16_t)(tmpzh << 8)) | ((int16_t)tmpzl));
//	printf("gz_s:%4d,%4d,%4d \r\n",gz_s,tmpzh,tmpzl);	//debug
	
	appLSM6DSL_SetGyroData(AXIS_X,(int16_t)(gx_s*sensitivity)/1000);	
	appLSM6DSL_SetGyroData(AXIS_Y,(int16_t)(gy_s*sensitivity)/1000);	
	appLSM6DSL_SetGyroData(AXIS_Z,(int16_t)(gz_s*sensitivity)/1000);

//	printf("Gyro : %4d,%4d,%4d\r\n",gx_s,gy_s,gz_s);

	LSM6DSL_Delay(5);
}

void appLSM6DSL_Set_IF_Addr_Incr(void)
{
	uint8_t value = 0;
	
	appLSM6DSL_Read(ACC_ADDRESS, LSM6DSL_XG_CTRL3_C, 1,&value);
	value &= ~LSM6DSL_XG_IF_INC_MASK;
	value |= LSM6DSL_XG_IF_INC;

	appLSM6DSL_Write(ACC_ADDRESS, LSM6DSL_XG_CTRL3_C, 1,&value);
}

void appLSM6DSL_Set_BDU(void)
{
	uint8_t value = 0;
	
	appLSM6DSL_Read(ACC_ADDRESS, LSM6DSL_XG_CTRL3_C, 1,&value);
	value &= ~LSM6DSL_ACC_GYRO_BDU_MASK;
	value |= LSM6DSL_ACC_GYRO_BDU_BLOCK_UPDATE;

	appLSM6DSL_Write(ACC_ADDRESS, LSM6DSL_XG_CTRL3_C, 1,&value);
}

void appLSM6DSL_Set_FIFO_MODE(void)
{
	uint8_t value = 0;
	
	appLSM6DSL_Read(ACC_ADDRESS, LSM6DSL_XG_FIFO_CTRL5, 1,&value);
	value &= ~LSM6DSL_XG_FIFO_MODE_MASK;
	value |= LSM6DSL_XG_FIFO_MODE_BYPASS;

	appLSM6DSL_Write(ACC_ADDRESS, LSM6DSL_XG_FIFO_CTRL5, 1,&value);
}

void appLSM6DSL_Set_ODR_G(void)
{
	uint8_t value = 0;
	
	appLSM6DSL_Read(GYRO_ADDRESS, LSM6DSL_XG_CTRL2_G, 1,&value);
	value &= ~LSM6DSL_G_ODR_MASK;
	value |= LSM6DSL_G_ODR_1K66HZ;

	appLSM6DSL_Write(GYRO_ADDRESS, LSM6DSL_XG_CTRL2_G, 1,&value);
}

void appLSM6DSL_Set_ODR_XL(void)
{
	uint8_t value = 0;
	
	appLSM6DSL_Read(ACC_ADDRESS, LSM6DSL_XG_CTRL1_XL, 1,&value);
	value &= ~LSM6DSL_XL_ODR_MASK;
	value |= LSM6DSL_XL_ODR_1K66HZ;

	appLSM6DSL_Write(ACC_ADDRESS, LSM6DSL_XG_CTRL1_XL, 1,&value);
}

void appLSM6DSL_Set_FS_G(void)
{
	uint8_t value = 0;
	
	appLSM6DSL_Read(ACC_ADDRESS, LSM6DSL_XG_CTRL2_G, 1,&value);
	value &= ~LSM6DSL_G_FS_MASK;
	value |= LSM6DSL_G_FS_245;	//LSM6DSL_G_FS_500 , LSM6DSL_G_FS_1000

	appLSM6DSL_Write(ACC_ADDRESS, LSM6DSL_XG_CTRL2_G, 1,&value);
}

void appLSM6DSL_Set_FS_XL(void)
{
	uint8_t value = 0;
	
	appLSM6DSL_Read(ACC_ADDRESS, LSM6DSL_XG_CTRL1_XL, 1,&value);
	value &= ~LSM6DSL_XL_FS_MASK;
	value |= LSM6DSL_XL_FS_2G;	//LSM6DSL_XL_FS_4G , LSM6DSL_XL_FS_8G

	appLSM6DSL_Write(ACC_ADDRESS, LSM6DSL_XG_CTRL1_XL, 1,&value);
}

void appLSM6DSL_Set_SW_RESET(void)
{
	uint8_t value = 0;
	
	appLSM6DSL_Read(ACC_ADDRESS, LSM6DSL_XG_CTRL3_C, 1,&value);
	value &= ~LSM6DSL_ACC_GYRO_SW_RESET_MASK;
	value |= LSM6DSL_ACC_GYRO_SW_RESET_RESET_DEVICE;

	appLSM6DSL_Write(ACC_ADDRESS, LSM6DSL_XG_CTRL3_C, 1,&value);
}


void appLSM6DSL_SetACC(void)
{
	uint8_t data;

	appLSM6DSL_Read(ACC_ADDRESS, LSM6DSL_XG_WHO_AM_I_ADDR, 1, &data);	// value : 0x6A	

	printf("WHO_AM_I : 0x%2X\r\n" , data);
	
	appLSM6DSL_Set_SW_RESET();
	appLSM6DSL_Set_IF_Addr_Incr();
	appLSM6DSL_Set_BDU();
	appLSM6DSL_Set_FIFO_MODE();
	appLSM6DSL_Set_ODR_XL();
	appLSM6DSL_Set_FS_XL();

}

void appLSM6DSL_SetGyro(void)
{
	uint8_t data;

	appLSM6DSL_Read(GYRO_ADDRESS, LSM6DSL_XG_WHO_AM_I_ADDR, 1, &data);	// value : 0x6A

	printf("WHO_AM_I : 0x%2X\r\n" , data);
	
	appLSM6DSL_Set_IF_Addr_Incr();
	appLSM6DSL_Set_BDU();
	appLSM6DSL_Set_FIFO_MODE();
	appLSM6DSL_Set_ODR_G();
	appLSM6DSL_Set_FS_G();	
	
}

void appLSM6DSL_Setup(void)
{
	appLSM6DSL_SetACC();	
	appLSM6DSL_SetGyro();

}	

void appLSM6DSL_GetData(void)
{
	appLSM6DSL_GetAcc();
	appLSM6DSL_GetGyro();

	#if 1	//debug
	printf("ACC:%5d,%5d,%5d, GYRO:%5d,%5d,%5d\r\n",
				appLSM6DSL_GetAccData(AXIS_X),appLSM6DSL_GetAccData(AXIS_Y),appLSM6DSL_GetAccData(AXIS_Z),
				appLSM6DSL_GetGyroData(AXIS_X),appLSM6DSL_GetGyroData(AXIS_Y),appLSM6DSL_GetGyroData(AXIS_Z));
	#endif
}	


void appLSM6DSL_GetWhoAmI(void)
{
	uint8_t data;
	
	appLSM6DSL_Read(ACC_ADDRESS, LSM6DSL_XG_WHO_AM_I_ADDR, 1, &data);	// value : 0x6A	
	printf("WHO_AM_I : 0x%2X\r\n" , data);
}	


void Gyroscope_Calibration(void)
{
	int32_t gyroX = 0;
	int32_t gyroY = 0;
	int32_t gyroZ = 0;		

//	uint16_t integerX = 0;
//	uint16_t integerY = 0;	
//	uint16_t integerZ = 0;	
	
	if (Flag_Calibrate)
	{
		gyroX = appLSM6DSL_GetGyroData(AXIS_X);
		if (MEMSABS(gyroX)>0)
		{
			appLSM6DSL_SetGyroCalData(AXIS_X,-gyroX);

		}
		gyroY = appLSM6DSL_GetGyroData(AXIS_Y);
		if (MEMSABS(gyroY)>0)
		{
			appLSM6DSL_SetGyroCalData(AXIS_Y,-gyroY);

		}
		gyroZ = appLSM6DSL_GetGyroData(AXIS_Z);
		if (MEMSABS(gyroZ)>0)
		{
			appLSM6DSL_SetGyroCalData(AXIS_Z,-gyroZ);

		}
		
//		printf("%s : %4d,%4d,%4d\r\n",__FUNCTION__,gyroX,gyroY,gyroZ);
	}
}

void Accelerator_Calibration(void)
{
	int32_t accX = 0;
	int32_t accY = 0;
	int32_t accZ = 0;		

//	uint16_t integerX = 0;
//	uint16_t integerY = 0;	
//	uint16_t integerZ = 0;		
	
	if (Flag_Calibrate)
	{
		appLSM6DSL_SetAccCalData(AXIS_X,0);	//reset calibration data
		accX = appLSM6DSL_GetAccData(AXIS_X);
		if (MEMSABS(accX)>0)
		{
			appLSM6DSL_SetAccCalData(AXIS_X,-accX);

		}

		appLSM6DSL_SetAccCalData(AXIS_Y,0);		//reset calibration data
		accY = appLSM6DSL_GetAccData(AXIS_Y);		
		if (MEMSABS(accY)>0)
		{
			appLSM6DSL_SetAccCalData(AXIS_Y,-accY);

		}

		appLSM6DSL_SetAccCalData(AXIS_Z,0);		//reset calibration data
		accZ = appLSM6DSL_GetAccData(AXIS_Z);		
		if ((MEMSABS(accZ)>1000)||(MEMSABS(accZ)<=999))
		{
			appLSM6DSL_SetAccCalData(AXIS_Z,-accZ+1000);

		}
		
//		printf("%s : %4d,%4d,%4d\r\n",__FUNCTION__,accX,accY,accZ);
	}
}

void Accelerator_filter(void)
{
	#if defined (ENABLE_AVERAGE_FILTER)

	uint8_t i;
	int32_t ax_sum = 0, ay_sum = 0, az_sum = 0; 

	for(i = 1 ; i < FILTER_COUNT; i++)
	{
		ax_buf[i - 1] = ax_buf[i];
		ay_buf[i - 1] = ay_buf[i];
		az_buf[i - 1] = az_buf[i];
	}

	ax_buf[FILTER_COUNT - 1] = appLSM6DSL_GetAccData(AXIS_X);
	ay_buf[FILTER_COUNT - 1] = appLSM6DSL_GetAccData(AXIS_Y);
	az_buf[FILTER_COUNT - 1] = appLSM6DSL_GetAccData(AXIS_Z);

	for(i = 0 ; i < FILTER_COUNT; i++)
	{
		ax_sum += ax_buf[i];
		ay_sum += ay_buf[i];
		az_sum += az_buf[i];
	}

	ax = (int16_t)(ax_sum>>FILTER_FACTOR); //	/ FILTER_COUNT);
	ay = (int16_t)(ay_sum>>FILTER_FACTOR); //	/ FILTER_COUNT);
	az = (int16_t)(az_sum>>FILTER_FACTOR); //	/ FILTER_COUNT);

	#else
	ax = appLSM6DSL_GetAccData(AXIS_X);
	ay = appLSM6DSL_GetAccData(AXIS_Y);
	az = appLSM6DSL_GetAccData(AXIS_Z);

	#endif
	
}

void Gyroscope_filter(void)
{
	#if defined (ENABLE_AVERAGE_FILTER)	
	uint8_t i;
	int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0; 

	for(i = 1 ; i < FILTER_COUNT; i++)
	{
		gx_buf[i - 1] = gx_buf[i];
		gy_buf[i - 1] = gy_buf[i];
		gz_buf[i - 1] = gz_buf[i];
	}

	gx_buf[FILTER_COUNT - 1] = appLSM6DSL_GetGyroData(AXIS_X);
	gy_buf[FILTER_COUNT - 1] = appLSM6DSL_GetGyroData(AXIS_Y);
	gz_buf[FILTER_COUNT - 1] = appLSM6DSL_GetGyroData(AXIS_Z);

	for(i = 0 ; i < FILTER_COUNT; i++)
	{
		gx_sum += gx_buf[i];
		gy_sum += gy_buf[i];
		gz_sum += gz_buf[i];
	}

	gx = (int16_t)(gx_sum>>FILTER_FACTOR);// / FILTER_COUNT);
	gy = (int16_t)(gy_sum>>FILTER_FACTOR);// / FILTER_COUNT);
	gz = (int16_t)(gz_sum>>FILTER_FACTOR);// / FILTER_COUNT);

	#else
	gx = appLSM6DSL_GetGyroData(AXIS_X);
	gy = appLSM6DSL_GetGyroData(AXIS_Y);
	gz = appLSM6DSL_GetGyroData(AXIS_Z);	

	#endif
	
}

void Angle_Calculate(void)
{  
	float s1 = 0;
	float s2 = 0;	

	Accelerator_filter();
	Gyroscope_filter();

	s1 = sqrt((float)((ay *ay )+(az *az )));
	s2 = sqrt((float)((ax *ax )+(az *az )));

	PitchAng = atan(ax /s1)*180/PI;
	RollAng = atan(ay /s2)*180/PI;

//	PitchAng = atan(ax /s1)*57.295779;
//	RollAng = atan(ay /s2)*57.295779;

	#if defined (ENABLE_KALMAN_FILTER)
	angle_dot = gx*GYRO_SCALE;	
	kalman_filter(RollAng, angle_dot, &f_angle, &f_angle_dot);
	#endif

	#if 1	//debug

	if (FlagSwitch)
	{
		printf("Acc:%5d,%5d,%5d,",ax ,ay ,az );
		printf("Gyro:%5d,%5d,%5d,",gx ,gy ,gz );	
		printf("\r\n");
	}
	else
	{		
		printf("Pitch:%8.3lf,",PitchAng);
		printf("Roll:%8.3lf,",RollAng);

		#if defined (ENABLE_KALMAN_FILTER)
		printf("Angle:%8.3lf,",f_angle);
		#endif
		
		printf("\r\n");
	}
	
	#endif
}


