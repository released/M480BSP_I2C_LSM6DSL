/*************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    A project template for M480 MCU.
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "I2C_LSM6DSL.h"
#include "i2c_master.h"


#define LED_R									(PH0)
#define LED_Y									(PH1)
#define LED_G									(PH2)

extern uint8_t FlagSwitch;


typedef enum{

	flag_uart_rx = 0 ,
	flag_error ,	
	flag_ccc ,
	
	flag_DEFAULT	
}Flag_Index;

uint32_t BitFlag = 0;
#define BitFlag_ON(flag)							(BitFlag|=flag)
#define BitFlag_OFF(flag)							(BitFlag&=~flag)
#define BitFlag_READ(flag)							((BitFlag&flag)?1:0)
#define ReadBit(bit)								(uint32_t)(1<<bit)

#define is_flag_set(idx)							(BitFlag_READ(ReadBit(idx)))
#define set_flag(idx,en)							( (en == 1) ? (BitFlag_ON(ReadBit(idx))) : (BitFlag_OFF(ReadBit(idx))))


/*
	If the SDO/SA0 pin is connected to the supply voltage, 
		LSb is ．1・ (address 1101011b); 
	else if the SDO/SA0 pin is 
		connected to ground, the LSb value is ．0・ (address 1101010b)

	PIN#2 : SCL	(use external pull up res.)
	PIN#3 : SDA	(use external pull up res.)
	
	PIN#4 (SA0/SDO) : HIGH
	PIN#5 (CS) : HIGH

*/

void I2C0_Init(void)	//PA5 : SCL , PA4 : SDA
{
    SYS_ResetModule(I2C0_RST);

    /* Open I2C module and set bus clock */
    I2C_Open(MASTER_I2C, LSM6DSL_I2C_SPEED);

    I2C_SetSlaveAddr(MASTER_I2C, 0, LSM6DSL_ADDRESS, I2C_GCMODE_DISABLE);   /* Slave Address : 1101011b */

    /* Get I2C0 Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(MASTER_I2C));

	#if defined (ENABLE_I2C_IRQ)
    I2C_EnableInt(MASTER_I2C);
    NVIC_EnableIRQ(MASTER_I2C_IRQn);
	#endif
	
}


void UARTx_Process(void)
{
	uint8_t res = 0;
	
	res = UART_READ(UART0);

	if (res > 0x7F)
	{
		printf("invalid command\r\n");
	}
	else
	{
		switch(res)
		{
			case '1':
				printf("...\r\n\r\n");
				break;	

			case 'X':
			case 'x':
			case 'Z':
			case 'z':
				NVIC_SystemReset();
			
				break;		
			
		}
	}
}

void UART0_IRQHandler(void)
{
    if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk))     /* UART receive data available flag */
    {
        while(UART_GET_RX_EMPTY(UART0) == 0)
        {
			UARTx_Process();
        }
    }

    if(UART0->FIFOSTS & (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk))
    {
        UART_ClearIntFlag(UART0, (UART_INTSTS_RLSINT_Msk| UART_INTSTS_BUFERRINT_Msk));
    }
}

void UART0_Init(void)
{
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);

	/* Set UART receive time-out */
	UART_SetTimeoutCnt(UART0, 20);

	UART0->FIFO &= ~UART_FIFO_RFITL_4BYTES;
	UART0->FIFO |= UART_FIFO_RFITL_8BYTES;

	/* Enable UART Interrupt - */
	UART_ENABLE_INT(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_TOCNTEN_Msk | UART_INTEN_RXTOIEN_Msk);
	
	NVIC_EnableIRQ(UART0_IRQn);

	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());	
}

void TMR1_IRQHandler(void)
{
	static uint16_t CNT = 0;	
//	static uint32_t log = 0;	

	static uint16_t CNT_SWITCH = 0;
	
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        TIMER_ClearIntFlag(TIMER1);
	
		if (CNT++ > 1000)
		{		
			CNT = 0;
//			printf("%s : %2d\r\n" , __FUNCTION__ , log++);

			LED_G ^= 1;
		}

		if (CNT_SWITCH++ >= 10000)
		{		
			CNT_SWITCH = 0;
			FlagSwitch ^= 1;

//        	printf("addr : 0x%2X\r\n",LSM6DS3_ADDRESS);			
//			convertDecToBin(LSM6DS3_ADDRESS);
//			printf("\r\n");			
		}	
		
    }
}

void TIMER1_HW_Init(void)
{
    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0);
}

void TIMER1_Init(void)
{
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);	
    TIMER_Start(TIMER1);
}

void TIMER0_HW_Init(void)
{
	CLK_EnableModuleClock(TMR0_MODULE);
	CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);
}

void TIMER0_Polling(uint32_t u32Usec)
{
	TIMER_Delay(TIMER0, u32Usec);
}

void LED_Init(void)
{
	GPIO_SetMode(PH,BIT0,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT1,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT2,GPIO_MODE_OUTPUT);
	
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable External XTAL (4~24 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(FREQ_192MHZ);
    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART clock source from HXT */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

    CLK_EnableModuleClock(I2C0_MODULE);

    CLK_EnableModuleClock(PDMA_MODULE);

	TIMER0_HW_Init();
	TIMER1_HW_Init();
	
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Set I2C0 multi-function pins */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~(SYS_GPA_MFPL_PA5MFP_Msk | SYS_GPA_MFPL_PA4MFP_Msk)) |
                    (SYS_GPA_MFPL_PA4MFP_I2C0_SDA | SYS_GPA_MFPL_PA5MFP_I2C0_SCL);
	
    /* Lock protected registers */
    SYS_LockReg();
}

/*
 * This is a template project for M480 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART0 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */

int main()
{	

    SYS_Init();
	UART0_Init();

	I2C0_Init();
	
	LED_Init();
	TIMER1_Init();

	appLSM6DSL_Setup();

    /* Got no where to go, just loop forever */
    while(1)
    {
//		TIMER0_Polling(1000);

		appLSM6DSL_GetData();
//		appLSM6DSL_GetWhoAmI();

		LED_Y ^= 1;
		
	
    }

}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
