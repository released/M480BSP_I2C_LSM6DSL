# M480BSP_I2C_LSM6DSL
 M480BSP_I2C_LSM6DSL


update @ 2020/06/19

1. use I2C0 initial LSM6DSL (PA5 : SCL , PA4 : SDA)

	- PIN#2 : SCL	(use external pull up res.)
	
	- PIN#3 : SDA	(use external pull up res.)
	
	- PIN#4 (SA0/SDO) : HIGH
	
	- PIN#5 (CS) : HIGH

2. 3 define in i2c_master.h , default use IRQ(ENABLE_I2C_POLLING_DISCRETE / ENABLE_I2C_POLLING_API / ENABLE_I2C_IRQ)

3. appLSM6DSL_Write , appLSM6DSL_Read in I2C_LSM6DSL.c , is low level I2C driver interface

4. below is terminal output log

![image](https://github.com/released/M480BSP_I2C_LSM6DSL/blob/master/LOG.jpg)
	
5. below is I2C communication with LA capture	
	
![image](https://github.com/released/M480BSP_I2C_LSM6DSL/blob/master/LA_capture.jpg)
