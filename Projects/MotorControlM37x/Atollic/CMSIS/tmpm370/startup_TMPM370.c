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
void INT0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INT1_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INT2_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INT3_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INT4_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INT5_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTRX0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTTX0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTRX1_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTTX1_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTVCNA_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTVCNB_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTEMG0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTEMG1_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTOVV0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTOVV1_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTAD0PDA_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTAD1PDA_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTAD0PDB_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTAD1PDB_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTTB00_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTTB01_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTTB10_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTTB11_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTTB40_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTTB41_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTTB50_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTTB51_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTPMD0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTPMD1_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTCAP00_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTCAP01_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTCAP10_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTCAP11_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTCAP40_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTCAP41_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTCAP50_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTCAP51_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INT6_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INT7_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTRX2_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTTX2_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTAD0CPA_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTAD1CPA_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTAD0CPB_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTAD1CPB_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTTB20_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTTB21_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTTB30_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTTB31_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTCAP20_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTCAP21_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTCAP30_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTCAP31_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTAD0SFT_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTAD1SFT_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTAD0TMR_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTAD1TMR_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INT8_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INT9_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTA_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTB_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTENC0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
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
		INT0_IRQHandler,                        /* 16: Interrupt Pin (PH0/AINA0/96pin) */
        INT1_IRQHandler,                        /* 17: Interrupt Pin (PH1/AINA1/95pin) */
        INT2_IRQHandler,                        /* 18: Interrupt Pin (PH2/AINA2/94pin) */
        INT3_IRQHandler,                        /* 19: Interrupt Pin (PA0/TB0IN/2pin) */
        INT4_IRQHandler,                        /* 20: Interrupt Pin (PA2/TB1IN/4pin) */
        INT5_IRQHandler,                        /* 21: Interrupt Pin (PE4/TB2IN//15pin) */
        INTRX0_IRQHandler,                      /* 22: Serial reception (channel.0) */
        INTTX0_IRQHandler,                      /* 23: Serial transmit (channel.0) */
        INTRX1_IRQHandler,                      /* 24: Serial reception (channel.1) */
        INTTX1_IRQHandler,                      /* 25: Serial transmit (channel.1) */
        INTVCNA_IRQHandler,                     /* 26: Vector Engine interrupt A */
        INTVCNB_IRQHandler,                     /* 27: Vector Engine interrupt B */
        INTEMG0_IRQHandler,                     /* 28: PMD0 EMG interrupt */
        INTEMG1_IRQHandler,                     /* 29: PMD1 EMG interrupt */
        INTOVV0_IRQHandler,                     /* 30: PMD0 OVV interrupt */
        INTOVV1_IRQHandler,                     /* 31: PMD1 OVV interrupt */
        INTAD0PDA_IRQHandler,                   /* 32: ADC0 conversion triggered by PMD0 is finished */
        INTAD1PDA_IRQHandler,                   /* 33: ADC1 conversion triggered by PMD0 is finished */
        INTAD0PDB_IRQHandler,                   /* 34: ADC0 conversion triggered by PMD1 is finished */
        INTAD1PDB_IRQHandler,                   /* 35: ADC1 conversion triggered by PMD1 is finished */
        INTTB00_IRQHandler,                     /* 36: 16bit TMRB0 compare match detection 0 */
        INTTB01_IRQHandler,                     /* 37: 16bit TMRB0 compare match detection 1 */
        INTTB10_IRQHandler,                     /* 38: 16bit TMRB1 compare match detection 0 */
        INTTB11_IRQHandler,                     /* 39: 16bit TMRB1 compare match detection 1 */
        INTTB40_IRQHandler,                     /* 40: 16bit TMRB4 compare match detection 0 */
        INTTB41_IRQHandler,                     /* 41: 16bit TMRB4 compare match detection 1 */
        INTTB50_IRQHandler,                     /* 42: 16bit TMRB5 compare match detection 0 */
        INTTB51_IRQHandler,                     /* 43: 16bit TMRB5 compare match detection 1 */
        INTPMD0_IRQHandler,                     /* 44: PMD0 PWM interrupt */
        INTPMD1_IRQHandler,                     /* 45: PMD1 PWM interrupt */
        INTCAP00_IRQHandler,                    /* 46: 16bit TMRB0 input capture 0 */
        INTCAP01_IRQHandler,                    /* 47: 16bit TMRB0 input capture 1 */
        INTCAP10_IRQHandler,                    /* 48: 16bit TMRB1 input capture 0 */
        INTCAP11_IRQHandler,                    /* 49: 16bit TMRB1 input capture 1 */
        INTCAP40_IRQHandler,                    /* 50: 16bit TMRB4 input capture 0 */
        INTCAP41_IRQHandler,                    /* 51: 16bit TMRB4 input capture 1 */
        INTCAP50_IRQHandler,                    /* 52: 16bit TMRB5 input capture 0 */
        INTCAP51_IRQHandler,                    /* 53: 16bit TMRB5 input capture 1 */
        INT6_IRQHandler,                        /* 54: Interrupt Pin (PE6/TB3IN//17pin) */
        INT7_IRQHandler,                        /* 55: Interrupt Pin (PE7/TB3OUT/18pin) */
        INTRX2_IRQHandler,                      /* 56: Serial reception (channel.2) */
        INTTX2_IRQHandler,                      /* 57: Serial transmit (channel.2) */
        INTAD0CPA_IRQHandler,                   /* 58: AD0 conversion monitoring function interrupt A */
        INTAD1CPA_IRQHandler,                   /* 59: AD1 conversion monitoring function interrupt A */
        INTAD0CPB_IRQHandler,                   /* 60: AD0 conversion monitoring function interrupt B */
        INTAD1CPB_IRQHandler,                   /* 61: AD1 conversion monitoring function interrupt B */
        INTTB20_IRQHandler,                     /* 62: 16bit TMRB2 compare match detection 0 */
        INTTB21_IRQHandler,                     /* 63: 16bit TMRB2 compare match detection 1 */
        INTTB30_IRQHandler,                     /* 64: 16bit TMRB3 compare match detection 0 */
        INTTB31_IRQHandler,                     /* 65: 16bit TMRB3 compare match detection 1 */
        INTCAP20_IRQHandler,                    /* 66: 16bit TMRB2 input capture 0 */
        INTCAP21_IRQHandler,                    /* 67: 16bit TMRB2 input capture 1 */
        INTCAP30_IRQHandler,                    /* 68: 16bit TMRB3 input capture 0 */
        INTCAP31_IRQHandler,                    /* 69: 16bit TMRB3 input capture 1 */
        INTAD0SFT_IRQHandler,                   /* 70: ADC0 conversion started by software is finished */
        INTAD1SFT_IRQHandler,                   /* 71: ADC1 conversion started by software is finished */
        INTAD0TMR_IRQHandler,                   /* 72: ADC0 conversion triggered by timer is finished */
        INTAD1TMR_IRQHandler,                   /* 73: ADC1 conversion triggered by timer is finished */
        INT8_IRQHandler,                        /* 74: Interrupt Pin (PA7/TB4IN/9pin) */
        INT9_IRQHandler,                        /* 75: Interrupt Pin (PD3/33pin) */
        INTA_IRQHandler,                        /* 76: Interrupt Pin (FTEST2/PL1/21pin) */
        INTB_IRQHandler,                        /* 77: Interrupt Pin (FTEST3/PL0/20pin) */
        INTENC0_IRQHandler,                     /* 78: Ender input0 interrupt */
        INTENC1_IRQHandler,                     /* 79: Ender input1 interrupt */
        INTRX3_IRQHandler,                      /* 80: Serial reception (channel.3) */
        INTTX3_IRQHandler,                      /* 81: Serial transmit (channel.3) */
        INTTB60_IRQHandler,                     /* 82: 16bit TMRB6 compare match detection 0 */
        INTTB61_IRQHandler,                     /* 83: 16bit TMRB6 compare match detection 1 */
        INTTB70_IRQHandler,                     /* 84: 16bit TMRB7 compare match detection 0 */
        INTTB71_IRQHandler,                     /* 85: 16bit TMRB7 compare match detection 1 */
        INTCAP60_IRQHandler,                    /* 86: 16bit TMRB6 input capture 0 */
        INTCAP61_IRQHandler,                    /* 87: 16bit TMRB6 input capture 1 */
        INTCAP70_IRQHandler,                    /* 88: 16bit TMRB7 input capture 0 */
        INTCAP71_IRQHandler,                    /* 89: 16bit TMRB7 input capture 1 */
        INTC_IRQHandler,                        /* 90: Interrupt Pin (PJ6/AINB9/74pin) */
        INTD_IRQHandler,                        /* 91: Interrupt Pin (PJ7/AINB10/73pin) */
        INTE_IRQHandler,                        /* 92: Interrupt Pin (PK0/AINB11/72pin) */
        INTF_IRQHandler                         /* 93: Interrupt Pin (PK1/AINB12/71pin) */
};
