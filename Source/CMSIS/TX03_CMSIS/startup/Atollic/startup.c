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
void INT6_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INT7_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INT8_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INT9_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTA_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTB_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTC_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTD_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTE_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTF_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTRX0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTTX0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTRX1_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTTX1_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTRX2_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTTX2_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTRX3_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTTX3_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTUART4_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTUART5_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTSBI0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTSBI1_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTSBI2_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTSSP0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTSSP1_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTSSP2_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTUSBH_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTUSBD_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTUSBWKUP_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTCANRX_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTCANTX_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTCANGB_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTETH_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTETHWK_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTADAHP_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTADAM0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTADAM1_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTADA_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTADBHP_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTADBM0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTADBM1_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTADB_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTEMG0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTPMD0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTENC0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTEMG1_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTPMD1_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTENC1_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTMTEMG0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTMTPTB00_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTMTTTB01_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTMTCAP00_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTMTCAP01_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTMTEMG1_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTMTPTB10_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTMTTTB11_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTMTCAP10_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTMTCAP11_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTMTEMG2_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTMTPTB20_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTMTTTB21_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTMTCAP20_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTMTCAP21_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTMTEMG3_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTMTPTB30_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTMTTTB31_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTMTCAP30_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTMTCAP31_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTRMCRX_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTTB0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTCAP00_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTCAP01_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTTB1_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTCAP10_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTCAP11_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTTB2_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTCAP20_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTCAP21_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTTB3_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTCAP30_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTCAP31_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTTB4_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTCAP40_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTCAP41_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTTB5_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTCAP50_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTCAP51_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTTB6_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTCAP60_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTCAP61_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTTB7_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTCAP70_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTCAP71_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTRTC_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTDMAADA_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTDMAADB_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTDMADAA_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTDMADAB_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTDMASPR0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTDMASPT0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTDMASPR1_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTDMASPT1_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTDMASPR2_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTDMASPT2_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTDMAUTR4_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTDMAUTT4_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTDMAUTR5_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTDMAUTT5_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTDMARX0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTDMATX0_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTDMARX1_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTDMATX1_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTDMARX2_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTDMATX2_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTDMARX3_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTDMATX3_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTDMASBI1_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTDMASBI2_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTDMATB_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTDMARQ_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTDMAAERR_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void INTDMABERR_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));


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
        INT0_IRQHandler,            /* 0:   Interrupt pin 0 */
        INT1_IRQHandler,            /* 1:   Interrupt pin 1 */
        INT2_IRQHandler,            /* 2:   Interrupt pin 2 */
        INT3_IRQHandler,            /* 3:   Interrupt pin 3 */
        INT4_IRQHandler,            /* 4:   Interrupt pin 4 */
        INT5_IRQHandler,            /* 5:   Interrupt pin 5 */
        INT6_IRQHandler,            /* 6:   Interrupt pin 6 */
        INT7_IRQHandler,            /* 7:   Interrupt pin 7 */
        INT8_IRQHandler,            /* 8:   Interrupt pin 8 */
        INT9_IRQHandler,            /* 9:   Interrupt pin 9 */
        INTA_IRQHandler,            /* 10:  Interrupt pin A */
        INTB_IRQHandler,            /* 11:  Interrupt pin B */
        INTC_IRQHandler,            /* 12:  Interrupt pin C */
        INTD_IRQHandler,            /* 13:  Interrupt pin D */
        INTE_IRQHandler,            /* 14:  Interrupt pin E */
        INTF_IRQHandler,            /* 15:  Interrupt pin F */
        INTRX0_IRQHandler,          /* 16:  Serial reception (channel.0) */
        INTTX0_IRQHandler,          /* 17:  Serial transmission (channel.0) */
        INTRX1_IRQHandler,          /* 18:  Serial reception (channel.1) */
        INTTX1_IRQHandler,          /* 19:  Serial transmission (channel.1) */
        INTRX2_IRQHandler,          /* 20:  Serial reception (channel.2) */
        INTTX2_IRQHandler,          /* 21:  Serial transmission (channel.2) */
        INTRX3_IRQHandler,          /* 22:  Serial reception (channel.3) */
        INTTX3_IRQHandler,          /* 23:  Serial transmission (channel.3) */
        INTUART4_IRQHandler,        /* 24:  FULL UART(channel.4) */
        INTUART5_IRQHandler,        /* 25:  FULL UART(channel.5) */
        INTSBI0_IRQHandler,         /* 26:  Serial bus interface 0 */
        INTSBI1_IRQHandler,         /* 27:  Serial bus interface 1 */
        INTSBI2_IRQHandler,         /* 28:  Serial bus interface 2 */
        INTSSP0_IRQHandler,         /* 29:  SPI serial interface 0 */
        INTSSP1_IRQHandler,         /* 30:  SPI serial interface 1 */
        INTSSP2_IRQHandler,         /* 31:  SPI serial interface 2 */
        INTUSBH_IRQHandler,         /* 32:  USB Host Interrupt */
        INTUSBD_IRQHandler,         /* 33:  USB Device Interrupt */
        INTUSBWKUP_IRQHandler,      /* 34:  USB WakeUp */
        INTCANRX_IRQHandler,        /* 35:  CAN RX */
        INTCANTX_IRQHandler,        /* 36:  CAN TX */
        INTCANGB_IRQHandler,        /* 37:  CAN STAUTS */
        INTETH_IRQHandler,          /* 38:  EtherNET Interrupt */
        INTETHWK_IRQHandler,        /* 39:  EtherNET(magic packet detection) interrupt */
        INTADAHP_IRQHandler,        /* 40:  Highest priority AD conversion complete interrupt (channel.A) */
        INTADAM0_IRQHandler,        /* 41:  AD conversion monitoring function interrupt 0(channel.A) */
        INTADAM1_IRQHandler,        /* 42:  AD conversion monitoring function interrupt 1(channel.A) */
        INTADA_IRQHandler,          /* 43:  AD conversion interrupt(channel.A) */
        INTADBHP_IRQHandler,        /* 44:  Highest priority AD conversion complete interrupt (channel.B) */
        INTADBM0_IRQHandler,        /* 45:  AD conversion monitoring function interrupt 0(channel.B) */
        INTADBM1_IRQHandler,        /* 46:  AD conversion monitoring function interrupt 1(channel.B) */
        INTADB_IRQHandler,          /* 47:  AD conversion interrupt(channel.B) */
        INTEMG0_IRQHandler,         /* 48:  PMD0 EMG interrupt (MPT0) */
        INTPMD0_IRQHandler,         /* 49:  PMD0 PWM interrupt (MPT0) */
        INTENC0_IRQHandler,         /* 50:  PMD0 Encoder input interrupt (MPT0) */
        INTEMG1_IRQHandler,         /* 51:  PMD1 EMG interrupt (MPT1) */
        INTPMD1_IRQHandler,         /* 52:  PMD1 PWM interrupt (MPT1) */
        INTENC1_IRQHandler,         /* 53:  PMD1 Encoder input interrupt (MPT1) */
        INTMTEMG0_IRQHandler,       /* 54:  16-bit MPT0 IGBT EMG interrupt */
        INTMTPTB00_IRQHandler,      /* 55:  16-bit MPT0 IGBT period/ TMRB compare match detection 0 */
        INTMTTTB01_IRQHandler,      /* 56:  16-bit MPT0 IGBT trigger/ TMRB compare match detection 1 */
        INTMTCAP00_IRQHandler,      /* 57:  16-bit MPT0 input capture 0 */
        INTMTCAP01_IRQHandler,      /* 58:  16-bit MPT0 input capture 1 */
        INTMTEMG1_IRQHandler,       /* 59:  16-bit MPT1 IGBT EMG interrupt */
        INTMTPTB10_IRQHandler,      /* 60:  16-bit MPT1 IGBT period/ TMRB compare match detection 0 */
        INTMTTTB11_IRQHandler,      /* 61:  16-bit MPT1 IGBT trigger/ TMRB compare match detection 1 */
        INTMTCAP10_IRQHandler,      /* 62:  16-bit MPT1 input capture 0 */
        INTMTCAP11_IRQHandler,      /* 63:  16-bit MPT1 input capture 1 */
        INTMTEMG2_IRQHandler,       /* 64:  16-bit MPT2 IGBT EMG interrupt */
        INTMTPTB20_IRQHandler,      /* 65:  16-bit MPT2 IGBT period/ TMRB compare match detection 0 */
        INTMTTTB21_IRQHandler,      /* 66:  16-bit MPT2 IGBT trigger/ TMRB compare match detection 1 */
        INTMTCAP20_IRQHandler,      /* 67:  16-bit MPT2 input capture 0 */
        INTMTCAP21_IRQHandler,      /* 68:  16-bit MPT2 input capture 1 */
        INTMTEMG3_IRQHandler,       /* 69:  16-bit MPT3 IGBT EMG interrupt */
        INTMTPTB30_IRQHandler,      /* 70:  16-bit MPT3 IGBT period/ TMRB compare match detection 0 */
        INTMTTTB31_IRQHandler,      /* 71:  16-bit MPT3 IGBT trigger/ TMRB compare match detection 1 */
        INTMTCAP30_IRQHandler,      /* 72:  16-bit MPT3 input capture 0 */
        INTMTCAP31_IRQHandler,      /* 73:  16-bit MPT3 input capture 1 */
        INTRMCRX_IRQHandler,        /* 74:  Remote Controller reception interrupt */
        INTTB0_IRQHandler,          /* 75:  16-bit TMRB_0 match detection 0 */
        INTCAP00_IRQHandler,        /* 76:  16-bit TMRB_0 input capture 00 */
        INTCAP01_IRQHandler,        /* 77:  16-bit TMRB_0 input capture 01 */
        INTTB1_IRQHandler,          /* 78:  16-bit TMRB_1 match detection 1 */
        INTCAP10_IRQHandler,        /* 79:  16-bit TMRB_1 input capture 10 */
        INTCAP11_IRQHandler,        /* 80:  16-bit TMRB_1 input capture 11 */
        INTTB2_IRQHandler,          /* 81:  16-bit TMRB_2 match detection 2 */
        INTCAP20_IRQHandler,        /* 82:  16-bit TMRB_2 input capture 20 */
        INTCAP21_IRQHandler,        /* 83:  16-bit TMRB_2 input capture 21 */
        INTTB3_IRQHandler,          /* 84:  16-bit TMRB_3 match detection 3 */
        INTCAP30_IRQHandler,        /* 85:  16-bit TMRB_3 input capture 30 */
        INTCAP31_IRQHandler,        /* 86:  16-bit TMRB_3 input capture 31 */
        INTTB4_IRQHandler,          /* 87:  16-bit TMRB_4 match detection 4 */
        INTCAP40_IRQHandler,        /* 88:  16-bit TMRB_4 input capture 40 */
        INTCAP41_IRQHandler,        /* 89:  16-bit TMRB_4 input capture 41 */
        INTTB5_IRQHandler,          /* 90:  16-bit TMRB_5 match detection 5 */
        INTCAP50_IRQHandler,        /* 91:  16-bit TMRB_5 input capture 50 */
        INTCAP51_IRQHandler,        /* 92:  16-bit TMRB_5 input capture 51 */
        INTTB6_IRQHandler,          /* 93:  16-bit TMRB_6 match detection 6 */
        INTCAP60_IRQHandler,        /* 94:  16-bit TMRB_6 input capture 60 */
        INTCAP61_IRQHandler,        /* 95:  16-bit TMRB_6 input capture 61 */
        INTTB7_IRQHandler,          /* 96:  16-bit TMRB_7 match detection 7 */
        INTCAP70_IRQHandler,        /* 97:  16-bit TMRB_7 input capture 70 */
        INTCAP71_IRQHandler,        /* 98:  16-bit TMRB_7 input capture 71 */
        INTRTC_IRQHandler,          /* 99:  RTC(Real time clock) interrupt */
        INTDMAADA_IRQHandler,       /* 100: DMA_ADC_A conversion end */
        INTDMAADB_IRQHandler,       /* 101: DMA_ADC_B conversion end */
        INTDMADAA_IRQHandler,       /* 102: DMA_DAC_A conversion trigger */
        INTDMADAB_IRQHandler,       /* 103: DMA_DAC_B conversion trigger */
        INTDMASPR0_IRQHandler,      /* 104: DMA_SSP_0 reception / DMA_I2C SIO_0 */
        INTDMASPT0_IRQHandler,      /* 105: DMA_SSP_0 transmission */
        INTDMASPR1_IRQHandler,      /* 106: DMA_SSP_1 reception */
        INTDMASPT1_IRQHandler,      /* 107: DMA_SSP_1 transmission */
        INTDMASPR2_IRQHandler,      /* 108: DMA_SSP_2 reception */
        INTDMASPT2_IRQHandler,      /* 109: DMA_SSP_2 transmission */
        INTDMAUTR4_IRQHandler,      /* 110: DMA_FUART_4 reception */
        INTDMAUTT4_IRQHandler,      /* 111: DMA_FUART_4 transmission */
        INTDMAUTR5_IRQHandler,      /* 112: DMA_FUART_5 reception */
        INTDMAUTT5_IRQHandler,      /* 113: DMA_FUART_5 transmission */
        INTDMARX0_IRQHandler,       /* 114: DMA_SIO/ UART_0 reception */
        INTDMATX0_IRQHandler,       /* 115: DMA_SIO/ UART_0 transmission */
        INTDMARX1_IRQHandler,       /* 116: DMA_SIO/ UART_1 reception */
        INTDMATX1_IRQHandler,       /* 117: DMA_SIO/ UART_1 transmission */
        INTDMARX2_IRQHandler,       /* 118: DMA_SIO/ UART_2 reception */
        INTDMATX2_IRQHandler,       /* 119: DMA_SIO/ UART_2 transmission */
        INTDMARX3_IRQHandler,       /* 120: DMA_SIO/ UART_3 reception */
        INTDMATX3_IRQHandler,       /* 121: DMA_SIO/ UART_3 transmission */
        INTDMASBI1_IRQHandler,      /* 122: DMA_I2C/ SIO_1 */
        INTDMASBI2_IRQHandler,      /* 123: DMA_I2C/ SIO_2 */
        INTDMATB_IRQHandler,        /* 124: 16-bit TMRB_0-4 match detection */
        INTDMARQ_IRQHandler,        /* 125: DMA request pin */
        INTDMAAERR_IRQHandler,      /* 126: DMA_A error transfer interrupt */
        INTDMABERR_IRQHandler       /* 127: DMA_B error transfer interrupt */

};
