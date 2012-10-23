/*
 ****************************************************************************
 *
 * Startup code for Cortex-M3
 *
 * (c)Copyright Atollic AB.
 *
 ****************************************************************************
*/

extern unsigned long _sdata, _edata, _sidata, _sbss, _ebss;
extern unsigned long _estack;
extern void __libc_init_array();
extern void SystemInit();
extern void main();

void Default_Handler()
{
	/* Hang here */
	while(1)
	{
	}
}

__attribute__((naked)) void Reset_Handler()
{
	/* Data and BSS variables */
	unsigned long *srcdata, *dstdata, *sbss;

	/* Set up the stack pointer */
	asm("ldr sp,=_estack\n\t");

	srcdata = &_sidata;
	dstdata = &_sdata;
	sbss = &_sbss;

	/* Copy data */
	while(dstdata != &_edata)
	{
		*(dstdata++) = *(srcdata++);
	}

	/* Clear BSS */
	while(sbss != &_ebss)
	{
		*(sbss++) = '\0';
	}

	/* Initialize System */
	SystemInit();

	/* Run static constructors */
	__libc_init_array();

	/* Jump to main */
	main();

	/* In case main returns, use default handler */
	Default_Handler();
}

/* Weak definitions of handlers point to Default_Handler if not implemented */
void NMI_Handler() __attribute__ ((weak, alias("Default_Handler")));
void HardFault_Handler() __attribute__ ((weak, alias("Default_Handler")));
void MemManage_Handler() __attribute__ ((weak, alias("Default_Handler")));
void BusFault_Handler() __attribute__ ((weak, alias("Default_Handler")));
void UsageFault_Handler() __attribute__ ((weak, alias("Default_Handler")));
void SVC_Handler() __attribute__ ((weak, alias("Default_Handler")));
void DebugMonitor_Handler() __attribute__ ((weak, alias("Default_Handler")));
void PendSV_Handler() __attribute__ ((weak, alias("Default_Handler")));
void SysTick_Handler() __attribute__ ((weak, alias("Default_Handler")));
void INT5_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTRX0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTTX0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTRX1_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTTX1_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTVCNB_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTEMG1_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTOVV1_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTADBPDB_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTTB00_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTTB01_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTTB10_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTTB11_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTTB40_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTTB41_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTTB50_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTTB51_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTPMD1_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTCAP00_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTCAP10_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTCAP40_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTCAP50_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INT6_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INT7_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTADBCPA_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTADBCPB_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTTB20_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTTB21_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTTB30_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTTB31_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTCAP20_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTCAP21_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTCAP30_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTCAP31_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTADBSFT_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTADBTMR_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTENC1_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTRX3_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTTX3_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTTB60_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTTB61_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTTB70_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTTB71_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTCAP60_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTCAP61_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTCAP70_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTCAP71_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTC_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTD_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTE_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTF_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));


/* The Interrupt Vector Table */
void (* const InterruptVector[])() __attribute__ ((section(".isr_vector"))) = {
		/* Processor exceptions */
		(void (*)(void))&_estack,
		Reset_Handler,
		NMI_Handler,
		HardFault_Handler,
		MemManage_Handler,
		BusFault_Handler,
		UsageFault_Handler,
		0,
		0,
		0,
		0,
		SVC_Handler,
		DebugMonitor_Handler,
		0,
		PendSV_Handler,
		SysTick_Handler,
		/* External Interrupts */
        0,                         /* 0:  Reserved */
        0,                         /* 1:  Reserved */
        0,                         /* 2:  Reserved */
        0,                         /* 3:  Reserved */
        0,                         /* 4:  Reserved */
        INT5_IRQHandler,           /* 5:  Interrupt Pin (PE4/TB2IN//15pin or 17pin) */
        INTRX0_IRQHandler,         /* 6:  Serial reception (channel.0) */
        INTTX0_IRQHandler,         /* 7:  Serial transmit (channel.0) */
        INTRX1_IRQHandler,         /* 8:  Serial reception (channel.1) */
        INTTX1_IRQHandler,         /* 9:  Serial transmit (channel.1) */
        0,                         /* 10: Reserved */
        INTVCNB_IRQHandler,        /* 11: Vector Engine interrupt B */
        0,                         /* 12: Reserved */
        INTEMG1_IRQHandler,        /* 13: PMD1 EMG interrupt */
        0,                         /* 14: Reserved */
        INTOVV1_IRQHandler,        /* 15: PMD1 OVV interrupt */
        0,                         /* 16: Reserved */
        0,                         /* 17: Reserved */
        0,                         /* 18: Reserved */
        INTADBPDB_IRQHandler,      /* 19: ADCB conversion triggered by PMD1 is finished */
        INTTB00_IRQHandler,        /* 20: 16bit TMRB0 compare match detection 0/ Overflow */
        INTTB01_IRQHandler,        /* 21: 16bit TMRB0 compare match detection 1 */
        INTTB10_IRQHandler,        /* 22: 16bit TMRB1 compare match detection 0/ Overflow */
        INTTB11_IRQHandler,        /* 23: 16bit TMRB1 compare match detection 1 */
        INTTB40_IRQHandler,        /* 24: 16bit TMRB4 compare match detection 0/ Overflow */
        INTTB41_IRQHandler,        /* 25: 16bit TMRB4 compare match detection 1 */
        INTTB50_IRQHandler,        /* 26: 16bit TMRB5 compare match detection 0/ Overflow */
        INTTB51_IRQHandler,        /* 27: 16bit TMRB5 compare match detection 1 */
        0,                         /* 28: Reserved */
        INTPMD1_IRQHandler,        /* 29: PMD1 PWM interrupt */
        INTCAP00_IRQHandler,       /* 30: 16bit TMRB0 input capture 0 */
        0,                         /* 31: Reserved */
        INTCAP10_IRQHandler,       /* 32: 16bit TMRB1 input capture 0 */
        0,                         /* 33: Reserved */
        INTCAP40_IRQHandler,       /* 34: 16bit TMRB4 input capture 0 */
        0,                         /* 35: Reserved */
        INTCAP50_IRQHandler,       /* 36: 16bit TMRB5 input capture 0 */
        0,                         /* 37: Reserved */
        INT6_IRQHandler,           /* 38: Interrupt Pin (PE6/TB3IN//17pin or 19pin) */
        INT7_IRQHandler,           /* 39: Interrupt Pin (PE7/TB3OUT/18pin or 20pin) */
        0,                         /* 40: Reserved */
        0,                         /* 41: Reserved */
        0,                         /* 42: Reserved */
        INTADBCPA_IRQHandler,      /* 43: ADCB conversion monitoring function interrupt A */
        0,                         /* 44: Reserved */
        INTADBCPB_IRQHandler,      /* 45: ADCB conversion monitoring function interrupt B */
        INTTB20_IRQHandler,        /* 46: 16bit TMRB2 compare match detection 0/ Overflow */
        INTTB21_IRQHandler,        /* 47: 16bit TMRB2 compare match detection 1 */
        INTTB30_IRQHandler,        /* 48: 16bit TMRB3 compare match detection 0/ Overflow */
        INTTB31_IRQHandler,        /* 49: 16bit TMRB3 compare match detection 1 */
        INTCAP20_IRQHandler,       /* 50: 16bit TMRB2 input capture 0 */
        INTCAP21_IRQHandler,       /* 51: 16bit TMRB2 input capture 1 */
        INTCAP30_IRQHandler,       /* 52: 16bit TMRB3 input capture 0 */
        INTCAP31_IRQHandler,       /* 53: 16bit TMRB3 input capture 1 */
        0,                         /* 54: Reserved */
        INTADBSFT_IRQHandler,      /* 55: ADCB conversion started by software is finished */
        0,                         /* 56: Reserved */
        INTADBTMR_IRQHandler,      /* 57: ADCB conversion triggered by timer is finished */
        0,                         /* 58: Reserved */
        0,                         /* 59: Reserved */
        0,                         /* 60: Reserved */
        0,                         /* 61: Reserved */
        0,                         /* 62: Reserved */
        INTENC1_IRQHandler,        /* 63: Ender input1 interrupt */
        INTRX3_IRQHandler,         /* 64: Serial reception (channel.3) */
        INTTX3_IRQHandler,         /* 65: Serial transmit (channel.3) */
        INTTB60_IRQHandler,        /* 66: 16bit TMRB6 compare match detection 0 / Overflow */
        INTTB61_IRQHandler,        /* 67: 16bit TMRB6 compare match detection 1 */
        INTTB70_IRQHandler,        /* 68: 16bit TMRB7 compare match detection 0 / Overflow */
        INTTB71_IRQHandler,        /* 69: 16bit TMRB7 compare match detection 1 */
        INTCAP60_IRQHandler,       /* 70: 16bit TMRB6 input capture 0 */
        INTCAP61_IRQHandler,       /* 71: 16bit TMRB6 input capture 1 */
        INTCAP70_IRQHandler,       /* 72: 16bit TMRB7 input capture 0 */
        INTCAP71_IRQHandler,       /* 73: 16bit TMRB7 input capture 1 */
        INTC_IRQHandler,           /* 74: Interrupt Pin (PJ6/AINB9/74pin or 76 pin) */
        INTD_IRQHandler,           /* 75: Interrupt Pin (PJ7/AINB10/73pin or 75pin) */
        INTE_IRQHandler,           /* 76: Interrupt Pin (PK0/AINB11/72pin or 74pin) */
        INTF_IRQHandler            /* 77: Interrupt Pin (PK1/AINB12/71pin or 73pin) */
};
