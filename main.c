/*
 * 		- name: main.c
 *		- write data: 19 Nov. 2012
 *		- modification data: 31 Nov. 2012
 *		- author: Wayne Xu (a0219294)
 *		- version: v1.2
 *
 *		September 2014 -- Heavily modified and rewritten to work with the MSP430FR5969 Launchpad
 *			-- by Dr. C. L. Fullmer
 *				Most of the original operation and flavor have been retained.
 *				However, changes have been made:
 *					SW1 on the Launchpad now toggles from reading CH0 to CH1
 *					The internal RTC is used for the time
 *					various accomodations of pinouts to the FR5969 launchpad configuration
 *					A feed of Time and Temp reading every .5 secs is output through the UART to the PC at 9600/8/1
 *
 * ***description
 *
 * 1. Program Device Configuration;
 * 		- SMCLK, DCO at 1MHz
 * 		- ACLK, external oscillator 32.768kHz
 * 		- USCIA0 as UART, 9600, 8 bits Data, No parity, 1 Stop (Hardware Mode)
 * 		- USCIB0 as 3 line SPI, Master Mode, 100kHz SCLK
 * 		- P2.0 as output, is used as CS for ADS1118 ADC device
 * 		- P2.3 as output,is used as RST of LCD
 * 		- P2.4 as output,is used as RS of LCD
 * 		- P2.5 ,is used as CS of LCD
 * 		- P2.1, P2.2 as input: SW1, SW2.
 * 		- P1.0 as output, is used as switch of buzzer
 * 2. this project is used to measure the temperature by type-K thermocouple.
 * 		- far-end temperature is measured by thermocouple, local temperature is measured by the internal sensor of ADS1118
 * 		- ADC mode: inputs are AIN0/AIN1, FS=+/-0.256, SS, DR=128sps, PULLUP on DOUT
 * 		- temperature mode: DR=128sps, PULLUP on DOUT
 * 		- Reference Tables are used to transform ADC code to temperature.
 *
 */

/*
 * ======== Standard MSP430 includes ========
 */
#include <msp430.h>
#include <driverlib.h>
/*
 * ======== Grace related includes ========
 */
//#include <ti/mcu/msp430/csl/CSL.h>
//#include "UART_TxRx.h"

#include "ADS1118.h"
#include "LCD_driver.h"

void System_Initial();	//Initialize system
void set_Thrtemp();		//set threshold temperature
void set_Time();
void half_second();
void time_display();
void ADC_display();
void delay(void);		//delay function
void xmitTemp(void);	// transmit data on UART to PC

/** new system functions for the FR5969 ***/
void SysInit_FR5969();
void GPIO_init(void);
void Init_Clock();
void Init_RTC();
void Init_UART();
void InitSPI();
void InitTimers();

// redefined for the FR5969 BOOST board
#define BUZZ_ON 	P4OUT &= ~BIT2;	//set P4.2 low
#define BUZZ_OFF 	P4OUT |= BIT2;	//set P4.2 high


/*
 * ======== global variable ========
 */

/* flag is used to transmit parameter between interrupt service function and main function.
 * the flag will be changed by ISR in InterruptVectors_init.c     ...\grace\InterruptVectors_init.c
 *
 * Bit0, Launchpad S2 is pushed
 * Bit1, SW1 on BoosterPack is pushed
 * Bit2, SW2 on BoosterPack is pushed
 * Bit3, 1 second timer interrupt
 * Bit4, timer for ADC interrupts
 * Bit5, ADC input flag, 0 is internal temperature sensor, 1 is AIN0-AIN1
 * Bit6, make an inversion every half a second
 * Bit7, half a second interrupt
 * Bit8, for Fahrenheit display
 * Bit9, ADC channel flag, 0 for channel 0, 1 for channel 1.
 * BitA, Launchpad S1 is pushed -- added for FR5969 kit
 */
volatile unsigned int  flag;		//global flag.
volatile unsigned char Thr_state;	// state for threshold temperature setting state machine.
volatile unsigned char time_state;	// state for time setting state machine.

Calendar calendar;      // Global Calendar used for RTC
unsigned int set_time;	// temporary for setting time
unsigned int Thr_temp;	// Threshold temperature in degrees Centigrade
unsigned int set_temp;	// temporary for setting Threshold temperature
unsigned int num=0;		// temporary for setting Threshold temperature
		 int Act_temp;	// Actual temperature


//-----------------------------------------------------------------------------
int _system_pre_init(void)
{
	// Stop Watchdog timer
	WDT_A_hold(__MSP430_BASEADDRESS_WDT_A__);     // Stop WDT

	/*==================================*/
	/* Choose if segment initialization */
	/* should be done or not. */
	/* Return: 0 to omit initialization */
	/* 1 to run initialization */
	/*==================================*/
	return 1;
}

/*
 *  ======== main ========
 */
int main(int argc, char *argv[])
{
	/*** main system initialization
	 *     UART, GPIO, WDT, CLOCK, System Registers
	 */
	SysInit_FR5969();
	delay();


	/*** Initiaize local peripherals
	 * 		LCD, ADS1118, FRAM
	 */
   	_enable_interrupt();		// enable interrupt
    System_Initial();			// initialize system.
   	delay();					// delay and wait the first conversion.

   	while(1)
   	{
   		// flicker lites
   		if (flag & BITA)	P4OUT |= BIT6;	// sets red LED on if CH1 is currently selected..
   		else				P4OUT &= ~BIT6;

   		// half a second interrupt.
   		if(flag & BIT7)
   		{
   			half_second();
   		}

   		// read ADC result
   		if(flag & BIT4)			//Read ADC result
   		{
   			ADC_display();
   		}

   		// one second interrupt to display time
   		if (flag & BIT3)
   		{
   			if(!time_state) time_display();
   		}

   		if(flag & BIT0)				// SW2 service, set the Threshold temperature or Time.
   		{
   			flag &= ~ BIT0;			// flag is reset
   			if(Thr_state != 0)
   			{
   				set_Thrtemp();		// set threshold temperature.
   			}
   			else if(time_state != 0)
   			{
   				set_Time();			// set timer.
   			}
   			else
   				flag ^= BIT8;		// display temperature in Fahrenheit
   		}

   		if(flag & BIT1)				// if SW1 is pushed, threshold temperature state machine will be changed
   		{
   			flag &= ~BIT1;			// flag is reset
   			time_state = 0;			//when threshold temperature is setting, setting time is disable.
   			if(Thr_state >= 3)		// if in state 3, change to state 0;
   			{
   				Thr_state = 0;
   				Thr_temp = set_temp;				// assign threshold temperature
   				LCD_display_number(0,3,Thr_temp);	// display threshold temperature
   			}
   			else					//else, Thr_state is changed to next state
   			{
   				Thr_state ++;
   			}
   		}

   		if((flag & BIT2) && (!Thr_state))		// if SW2 is pushed, and Thr_state = 0, time setting state machine will be changed
   		{
   			flag &= ~ BIT2;						// flag is reset

			if(time_state >= 2)					// if in state 2, change to state 0;
			{
				time_state = 0;
				calendar = RTC_B_getCalendarTime (RTC_B_BASE); // load global calendar values
				calendar.Seconds = 0x00;		// reset seconds for accurate set
				RTC_B_calendarInit(RTC_B_BASE, calendar, RTC_B_FORMAT_BCD);
				time_display();					// display setting time
			}
			else
			{
			time_state ++;
			}

   		}

   		else
   		__no_operation();
   	}
    
// never gets here..

}


void delay(void)
{
	unsigned int k;

	for (k = 1000; k > 0; k--)
		__no_operation();
}

void ADC_display()
{
	//float sensor;

	static signed int local_data = 0, far_data = 0;
	signed int temp;

	flag &= ~ BIT4;					// flag is reset
	if (!(flag & BIT5))
	{
		local_data = ADS_Read(1);	//read local temperature data,and start a new convertion for far-end temperature sensor.
	}
	else
	{

		far_data = ADS_Read(0);		//read far-end temperature,and start a new convertion for local temperature sensor.
		temp = far_data + local_compensation(local_data);	// transform the local_data to compensation codes of far-end.

		temp = ADC_code2temp(temp);	// transform the far-end thermocouple codes to temperature.

		if(flag & BIT8)				// display temperature in Fahrenheit
		{
			Act_temp = (signed int)(((temp * 9) / 5) + 320);
			LCD_display_temp(1,5,Act_temp);
			LCD_display_char(1,11,'F');
		}
		else
		{
			Act_temp = temp;
			LCD_display_temp(1,5,Act_temp);
			LCD_display_char(1,11,'C');
		}

	}

	if(flag & BIT9)
		LCD_display_char(1,15,'1');
	else
		LCD_display_char(1,15,'0');

}



void time_display()
{
	char data[10] = "";
	unsigned int i;

	calendar = RTC_B_getCalendarTime (RTC_B_BASE);
	data[0] = ((calendar.Hours & 0xF0)>>4) + 0x30;
	data[1]  = (calendar.Hours & 0x0F) + 0x30;
	data[2] = ':';
	data[3] = ((calendar.Minutes & 0xF0)>>4) + 0x30;
	data[4] = (calendar.Minutes & 0x0F) + 0x30;
	data[5] = ':';
	data[6] = ((calendar.Seconds & 0xF0)>>4) + 0x30;
	data[7] = (calendar.Seconds & 0x0F) + 0x30;

	for(i=0; i<8; i++)
	{
		LCD_display_char(0,8+i,data[i]);
	}

	//================
	flag &= ~BIT3;						// flag is reset

}

void xmitTemp(void)
{
	char data[20] = "";
	// temp info
	int i,j,k,l;
	signed int numx = Act_temp;

	// first a time stamp
	calendar = RTC_B_getCalendarTime (RTC_B_BASE);
	data[0] = ((calendar.Hours & 0xF0)>>4) + 0x30;
	data[1]  = (calendar.Hours & 0x0F) + 0x30;
	data[2] = ':';
	data[3] = ((calendar.Minutes & 0xF0)>>4) + 0x30;
	data[4] = (calendar.Minutes & 0x0F) + 0x30;
	data[5] = ':';
	data[6] = ((calendar.Seconds & 0xF0)>>4) + 0x30;
	data[7] = (calendar.Seconds & 0x0F) + 0x30;
	data[8] = 0x20;
	data[9] = '*';
	data[10] = 0x20;
	// transmit time to PC by UART;
	for(i=0;i<11;i++)
	{
		while(!(UCA0IFG&UCTXIFG));
		UCA0TXBUF = data[i];
		__no_operation();
	}

	// calculate temp digits
	i = numx/1000%10;
	j = numx%1000/100;
	k = numx%100/10;
	l = numx%10;
	if (i == 0) data[0] = 0x20;
	else		data[0] = i + 0x30;
	data[1] = j + 0x30;
	data[2] = k + 0x30;
	data[3] = '.';
	data[4] = l + 0x30;
	if(flag & BIT8) data[5] = 'F';
	else			data[5] = 'C';
	data[6] = 0x0D; // carraige return
	data[7] = 0x0A; // line feed

	// transmit temperature to PC by UART;
	for(i=0;i<8;i++)
	{
		while(!(UCA0IFG&UCTXIFG));
		UCA0TXBUF = data[i];
		__no_operation();
	}
}



/*
 * function name: half_second()
 * description: it is executed every half a second. it has three functions,
 * the first one is to compare the Actual temperature and threshold temperature, if Actual temperature is higher than threshold
 * temperature, buzzer will work
 * the second one is to flicker the threshold temperature bit which is being set.
 * the third one is to flicker the time bit which is being set.
 */

void half_second()
{
	int threshold_temp;
	flag &= ~ BIT7;

	// judge actual temperature is higher than threshold temperature. if higher, buzzer will work
	if(flag & BIT8) // check for Farenheit conversion
	{
		threshold_temp = 10*(((Thr_temp * 9)/5)+32);
	}
	else threshold_temp = (10*Thr_temp);

	if((Act_temp >= threshold_temp) && (flag & BIT6))
	{
	   	BUZZ_ON;
	}
	else
	{
		BUZZ_OFF;
	}

	// output data to PC thru UART
	xmitTemp();

	//display threshold temperature setting
	if(Thr_state == 0x01)						//threshold temperature state machine output.
	{
		if (flag & BIT6)
			LCD_display_char(0,4,' ');			//display blank space for half a second
		else
			LCD_display_number(0,3,set_temp);	//display hundred place for half a second
	}
	else if(Thr_state == 0x02)
	{
		if (flag & BIT6)
			LCD_display_char(0,5,' ');			//display blank space for half a second
		else
			LCD_display_number(0,3,set_temp);	//display decade for half a second
	}
	else if(Thr_state == 0x03)
	{
		if (flag & BIT6)
			LCD_display_char(0,6,' ');			//display blank space for half a second
		else
			LCD_display_number(0,3,set_temp); 	//display unit's digit for half a second
	}

	// display time setting
	if(time_state == 0x01)
	{
		if (flag & BIT6)
		{
			LCD_display_char(0,8,' ');
			LCD_display_char(0,9,' ');			//display blank space for half a second in hours
		}
		else
			time_display();
	}
	else if(time_state == 0x02)
	{
		if (flag & BIT6)
		{
			LCD_display_char(0,11,' ');
			LCD_display_char(0,12,' ');			//display blank space for half a second in minutes
		}
		else
			time_display();
	}

}


/*
 * function name: set_Time()
 * description: set the time of day
 * 	NOTE: this routine is heavily modified to make use of the in board RTC in the FR5969
 * 		Only hours and minutes can be set
 * 		seconds are automatically reset to 00 after the last modification
 */
void set_Time()
{
	uint16_t number;

	calendar = RTC_B_getCalendarTime (RTC_B_BASE); // load global calendar values

	if (time_state == 0x01)
	{
		// set hours
		number = RTC_B_convertBCDToBinary (RTC_B_BASE,calendar.Hours);
		number++;
		if(number >= 24) number = 0;
	    calendar.Hours = RTC_B_convertBinaryToBCD(RTC_B_BASE,number);
	}
	else if (time_state  == 0x02)
	{
		// set minutes
		number = RTC_B_convertBCDToBinary (RTC_B_BASE,calendar.Minutes);
		number++;
		if(number >= 60) number = 0;
		calendar.Minutes = RTC_B_convertBinaryToBCD(RTC_B_BASE,number);
	}

	else
	__no_operation();

	calendar.Seconds    = 0x00; // sec reset to 0 after mods

	// Initialize RTC with the specified Calendar above
	RTC_B_calendarInit(RTC_B_BASE,
					   calendar,
					   RTC_B_FORMAT_BCD);
}



/*
 * function name:set_Thrtemp()
 * description: set the threshold temperature. the temporary is saved in variable set_temp.
 */
void set_Thrtemp()
{
	set_temp = Thr_temp;

	if (Thr_state == 0x01)
	{
		if (set_temp/100 == 9)
		{
			set_temp = set_temp-900;
	   	}
	   	else
	   	{
	   		set_temp += 100;
	   	}
	}
	else if (Thr_state == 0x02)
	{
		if (set_temp%100/10 == 9)
	   	{
	   		set_temp = set_temp-90;
	   	}
	   	else
	   	{
	   		set_temp += 10;
	   	}
	}
	else if (Thr_state == 0x03)
	{
	   	if (set_temp%10 == 9)
	   	{
	   		set_temp = set_temp-9;
	   	}
	   	else
	   	{
	   		set_temp ++;
	   	}
	}
	else
	__no_operation();

	Thr_temp = set_temp;
}


/*
 * function name:System_Initial()
 * description: Initialize the system. include I/O, LCD and ADS1118.
 */
void System_Initial()
{
	flag  = 0;		//reset flag

	Thr_state = 0;  //threshold temperature setting state machine counter
	time_state = 0;	//time setting state machine counter
	Thr_temp = 100; //configure threshold temperature to 100;
	Act_temp = 0;
	set_temp = Thr_temp; // set for future use in changing values

	// IO initial
	P1OUT |= BIT5; // CS off for LCD
	P2OUT = 0x00;
	P3OUT |= BIT4; // 3.4 CS set off for ADS1118
	P4OUT |= BIT2; // set buzzer off

	flag |= BIT8;	// Set temp to display Farenheit
	//flag |= BIT9; // set bit9 to use CH1 in the ads1118

	LCD_init();						// LCD initial
	LCD_clear();					// LCD clear
	LCD_display_string(0,"Th:\0");	// display "ADS1118"
	time_display();					// display current time
	LCD_display_string(1,"Temp:        CH0\0");	// display threshold temp and actual temp;
	LCD_display_char(1,10,0xDF);
	LCD_display_char(1,11,'F');
	LCD_display_number(0,3,Thr_temp);// display threshold temp number

	ADS_Config(0); 					// set ADS1118 to convert local temperature, and start conversion.

}


/****
 * 		SysInit_FR5969
 * 		initialization of the chip - a MSP430FR5969
 * 			UART, GPIO, WDT, CLOCK, System Registers
 */

void SysInit_FR5969()
{
	_disable_interrupt();

	// Set WatchDog Timer off, so doesn't fire during init
	WDTCTL = WDTPW + WDTHOLD;

	/* initialize Config for the MSP430 GPIO */
    GPIO_init();

    /* initialize Config for the MSP430 clock */
    Init_Clock();
    Init_RTC();

    /* initialize Config for the MSP430 USCI_A0 */
    Init_UART();
    /* initialize Config for the MSP430 USCI_B0 */
    InitSPI();

    /*
     * SR, Status Register
     *
     * ~SCG1 -- Disable System clock generator 1
     * ~SCG0 -- Disable System clock generator 0
     * ~OSCOFF -- Oscillator On
     * ~CPUOFF -- CPU On
     * GIE -- General interrupt enable
     *
     * Note: ~<BIT> indicates that <BIT> has value zero
     */
    //__bis_SR_register(GIE);

    InitTimers();

    // Enable globale interrupt
    __enable_interrupt();

	// Set WatchDog Timer off
	WDTCTL = WDTPW + WDTHOLD;

}

/*
 *  ======== GPIO_init ========
 *  Initialize MSP430 General Purpose Input Output Ports
 *
 *  Port pins must match funcionality of the ADS118 BOOST kit headers
 *
 *  Header P1
 *  	Pin 1 = Vcc
 *  	Pin 2 = Output to drive the Buzzer
 *  	Pin 3,4,5 = no connection
 *  	Pin 6 = not clear yet.. pulled high in circuit.. Vref ? (No.. error in board by TI - has no purpose)
 *  	Pin 7 = SPI SCLK
 *  	Pin 8 = SPI CS1
 *  	Pin 9 = SW1, normal HI
 *  	Pin 10 = SW2, normal HI
 *
 *  Header P2
 *  	Pin 1 = GND
 *  	Pin 2,3,4,5 = no connection
 *  	Pin 6 = SPI SMO
 *  	Pin 7 = SPI SMI
 *  	Pin 8 = LCD CS
 *  	Pin 9 = LCD RS
 *  	Pin 10 = LCD RST
 *
 *  S2 on 430G2 launchpad --> P1.3
 *  S1                        RST
 *
 * --------------------------------------------
 * 	FR5969 BOOST kit headers
 * 	Header P1
 * 		Pin 1 = Vcc
 * 		Pin 2 = P4.2 => out to buzzer
 * 		Pin 3 = P2.6
 * 		Pin 4 = P2.5
 * 		Pin 5 = P4.3
 * 		Pin 6 = P2.4 => unused - but held high by circuit -- set as Input
 * 		Pin 7 = P2.2 => SPI CLK
 * 		Pin 8 = P3.4 => SPI CS
 * 		Pin 9 = P3.5 => SW1 detect - normal HI
 * 		Pin 10 = P3.6 => SW2 detect - normal HI
 *
 * 	Header P2
 * 		Pin 1 = GND
 * 		Pin 2 = P1.2
 * 		Pin 3 = P3.H
 * 		Pin 4 = no connection
 * 		Pin 5 = RST
 * 		Pin 6 = P1.6 => SPI SMO
 * 		Pin 7 = P1.7 => SPI SMI
 * 		Pin 8 = P1.5 => LCD CS
 * 		Pin 9 = P1.4 => LCD RS
 * 		Pin 10 = P1.3 => LCD RST
 *
 *  S1 --> P4.5
 *  S2 --> P1.1
 *
 */
void GPIO_init()
{
    // Set all GPIO pins to output low for low power
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_PJ, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7|GPIO_PIN8|GPIO_PIN9|GPIO_PIN10|GPIO_PIN11|GPIO_PIN12|GPIO_PIN13|GPIO_PIN14|GPIO_PIN15);

    //  P1.6 & P1.7 SPI pins
    //  P1.1 SW2 - inupt
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5);
    // P2.4 PIN4 is left as an input pin
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN7);
    // P4.5 SW1 - input
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_PJ, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7|GPIO_PIN8|GPIO_PIN9|GPIO_PIN10|GPIO_PIN11|GPIO_PIN12|GPIO_PIN13|GPIO_PIN14|GPIO_PIN15);

    P4REN = BIT2;
    P4OUT = BIT2; // buzzer off

    // clear all interrupt flags
    P1IFG = 0;
    P2IFG = 0;
    P3IFG = 0;
    P4IFG = 0;


    // set up interrupts on pins
    //----------------------------------

    // S2 on MCU board -- P1.1
    /* Port 1 Resistor Enable Register */
    P1REN |= BIT1;	// set pullup on
    // input pin
    P1DIR &= ~BIT1;
    //GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN1);
    /* Set output to 1 for pullup resestor */
    P1OUT |= BIT1;
    /* Port 1 Interrupt Edge Select Register */
    P1IES |= BIT1;	// set for HI -> LOW transition
    /* Port 1 clear Interrupt Flag Register */
    P1IFG = 0;
    /* Port 1 Interrupt Enable Register */
    P1IE |= BIT1;

    // S1 on Launchpad board -- P4.5
    /* Port Resistor Enable Register */
    P4REN |= BIT5;	// set pullup on
    // input pin
    P4DIR &= ~BIT5;
    /* Set output to 1 for pullup resestor */
    P4OUT |= BIT5;
    /* Port Interrupt Edge Select Register */
    P4IES |= BIT5;	// set for HI -> LOW transition
    /* Port clear Interrupt Flag Register */
    P4IFG = 0;
    /* Port Interrupt Enable Register */
    P4IE |= BIT5;

    //-----------------------------------
    // SW1 & SW2 on ADS board
    GPIO_setAsInputPin(GPIO_PORT_P3, GPIO_PIN5 | GPIO_PIN6);
    //P3REN = (BIT5 | BIT6);	// resistor enable
    P3IES = (BIT5 | BIT6);	// set interrupt on edge select
    P3IFG = 0;				// clear interrupt flags
    P3IE  = (BIT5 | BIT6);	// set interupt enable on pins
    //-----------------------------------

	// Configure P2.0 - UCA0TXD and P2.1 - UCA0RXD
	//GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);
	//GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0);
    GPIO_setAsPeripheralModuleFunctionInputPin(
    		GPIO_PORT_P2,
    		GPIO_PIN0 + GPIO_PIN1,
    		GPIO_SECONDARY_MODULE_FUNCTION);

    // Configure SPI pins
	// Configure Pins for UCB0CLK
	/*
	 * Select Port 2.2
	 * Set Pin 2 to input Secondary Module Function, (UCB0CLK).
	 */
	GPIO_setAsPeripheralModuleFunctionInputPin(
			GPIO_PORT_P2,
			GPIO_PIN2,
			GPIO_SECONDARY_MODULE_FUNCTION
			);

	// Configure Pins for UCB0TXD/UCB0SIMO, UCB0RXD/UCB0SOMI
	//Set P1.6, P1.7 as Secondary Module Function Input.
	/*
	 * Select Port 1
	 * Set Pin 6, 7 to input Secondary Module Function, (UCB0TXD/UCB0SIMO, UCB0RXD/UCB0SOMI).
	 */
	GPIO_setAsPeripheralModuleFunctionInputPin(
			GPIO_PORT_P1,
			GPIO_PIN6 + GPIO_PIN7,
			GPIO_SECONDARY_MODULE_FUNCTION
			);

    // Set PJ.4 and PJ.5 as Primary Module Function Input, LFXT.
    GPIO_setAsPeripheralModuleFunctionInputPin(
           GPIO_PORT_PJ,
           GPIO_PIN4 + GPIO_PIN5,
           GPIO_PRIMARY_MODULE_FUNCTION
           );

    // Disable the GPIO power-on default high-impedance mode
    // to activate previously configured port settings
    PMM_unlockLPM5();

}

/*
 * Clock System Initialization
 */
void Init_Clock()
{
    // Set DCO frequency to 8 MHz
    CS_setDCOFreq(CS_DCORSEL_0, CS_DCOFSEL_6);
    //Set external clock frequency to 32.768 KHz
    CS_setExternalClockSource(32768, 0);
    //Set ACLK=LFXT
    CS_clockSignalInit(CS_ACLK, CS_LFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);
    // Set SMCLK = DCO with frequency divider of 1
    CS_clockSignalInit(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    // Set MCLK = DCO with frequency divider of 1
    CS_clockSignalInit(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    //Start XT1 with no time out
    CS_LFXTStart(CS_LFXTDRIVE_0);
}

/*
 * UART Communication Initialization
 */
void Init_UART()
{
	// Configure UART -- 9600 baud, NoParity, 1 Stop Bit -- LSB first
	// based on SMCLK = 8MHz
    if ( STATUS_FAIL == EUSCI_A_UART_initAdvance(EUSCI_A0_BASE,
                                                 EUSCI_A_UART_CLOCKSOURCE_SMCLK,
                                                 52,
                                                 1,
                                                 0x49,
                                                 EUSCI_A_UART_NO_PARITY,
                                                 EUSCI_A_UART_LSB_FIRST,
                                                 EUSCI_A_UART_ONE_STOP_BIT,
                                                 EUSCI_A_UART_MODE,
                                                 EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION ))
        return;

    EUSCI_A_UART_enable(EUSCI_A0_BASE);

    EUSCI_A_UART_clearInterruptFlag(EUSCI_A0_BASE,
                                    EUSCI_A_UART_RECEIVE_INTERRUPT);

    // Enable USCI_A0 RX interrupt
    EUSCI_A_UART_enableInterrupt(EUSCI_A0_BASE,
                                 EUSCI_A_UART_RECEIVE_INTERRUPT); // Enable interrupt


}

void InitSPI()
{
    //Initialize Master
    EUSCI_B_SPI_masterInit(EUSCI_B0_BASE,
                           EUSCI_B_SPI_CLOCKSOURCE_ACLK,
                           CS_getACLK(),
                           30000,
                           EUSCI_B_SPI_MSB_FIRST,
                           EUSCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT,
                           EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_HIGH,
                           EUSCI_B_SPI_3PIN
                           );

    //Enable SPI module
    EUSCI_B_SPI_enable(EUSCI_B0_BASE);

    EUSCI_B_SPI_clearInterruptFlag(EUSCI_B0_BASE,
                                   EUSCI_B_SPI_RECEIVE_INTERRUPT
                                   );

    // Enable USCI_B0 RX interrupt
    //EUSCI_B_SPI_enableInterrupt(EUSCI_B0_BASE, EUSCI_B_SPI_RECEIVE_INTERRUPT);

    //Wait for slave to initialize
    __delay_cycles(1000);

}

/*
 * Real Time Clock Initialization
 */
void Init_RTC()
{
    //Setup Current Time for Calendar
    calendar.Seconds    = 0x00;
    calendar.Minutes    = 0x20;
    calendar.Hours      = 0x04;
    calendar.DayOfWeek  = 0x01;
    calendar.DayOfMonth = 0x30;
    calendar.Month      = 0x04;
    calendar.Year       = 0x2014;

    // Initialize RTC with the specified Calendar above
    RTC_B_calendarInit(RTC_B_BASE,
                       calendar,
                       RTC_B_FORMAT_BCD);

    RTC_B_setCalendarEvent(RTC_B_BASE,
    		               RTC_B_CALENDAREVENT_MINUTECHANGE
    		               );

    RTC_B_clearInterrupt(RTC_B_BASE,
                         RTC_B_TIME_EVENT_INTERRUPT
                         );

    //RTC_B_enableInterrupt(RTC_B_BASE,
    //                      RTC_B_TIME_EVENT_INTERRUPT
    //                      );

    //Start RTC Clock
    RTC_B_startClock(RTC_B_BASE);
}

/***
 * Initialize Timer0 and Timer1
 * 	both set to A3 mode
 */
void InitTimers(void)
{
	// Timer 0
	// needs to be running at 0.5 sec
    /*
     * TA0CCTL0, Capture/Compare Control Register 0
     *
     * CM_0 -- No Capture
     * CCIS_0 -- CCIxA
     * ~SCS -- Asynchronous Capture
     * ~SCCI -- Latched capture signal (read)
     * ~CAP -- Compare mode
     * OUTMOD_0 -- PWM output mode: 0 - OUT bit value
     *
     * Note: ~<BIT> indicates that <BIT> has value zero
     */
    TA0CCTL0 = CM_0 + CCIS_0 + OUTMOD_0 + CCIE;

    /* TA0CCR0, Timer_A Capture/Compare Register 0 */
    TA0CCR0 = 16384;  // 0.5 sec -- 32768 clock

    /*
     * TA0CTL, Timer_A0 Control Register
     *
     * TASSEL_1 -- ACLK
     * ID_0 -- Divider - /1
     * MC_1 -- Up Mode
     */
    TA0CTL = TASSEL_1 + ID_0 + MC_1;
    //---------------------------------------------

    //---------------------------------------------
    // Timer 1
    //  set for 0.1 sec to sample temp
    //
    /*
     * TA1CCTL0, Capture/Compare Control Register 0
     *
     * CM_0 -- No Capture
     * CCIS_0 -- CCIxA
     * ~SCS -- Asynchronous Capture
     * ~SCCI -- Latched capture signal (read)
     * ~CAP -- Compare mode
     * OUTMOD_0 -- PWM output mode: 0 - OUT bit value
     *
     * Note: ~<BIT> indicates that <BIT> has value zero
     */
    TA1CCTL0 = CM_0 + CCIS_0 + OUTMOD_0 + CCIE;

    /* TA1CCR0, Timer_A Capture/Compare Register 0 */
    TA1CCR0 = 3280;  // 0.1 sec -- 32768 clk

    /*
     * TA1CTL, Timer_A3 Control Register
     *
     * TASSEL_1 -- ACLK
     * ID_0 -- Divider - /1
     * MC_1 -- Up Mode
     */
    TA1CTL = TASSEL_1 + ID_0 + MC_1;
	//----------------------------------------------
}


//***************************************************************************
//***************************************************************************
//***************************************************************************
//
//               ISR routines
//
//***************************************************************************8
//***************************************************************************8
//***************************************************************************8

/*
 * Port 3 interrupt service routine to handle board SW1 & SW2 button press
 *		SW1 & SW2 reside on the ADS1118 board
 *
 */
#if defined (__TI_COMPILER_VERSION__) || defined (__IAR_SYSTEMS_ICC__)
#pragma vector = PORT3_VECTOR
__interrupt
#elif defined(__GNUC__)
void Port_3(void) __attribute__((interrupt(PORT3_VECTOR)));
#else
#error Compiler not supported!
#endif
void Port_3(void)
{
	P3IE &= ~(BIT5 + BIT6);	// close P3 interrupt

	if (P3IFG & BIT5)
    {
    	flag |= BIT1;		// flag bit 1 is set
		P3IFG &= ~BIT5; 	// P3.5 IFG cleared
    }
    else if(P3IFG & BIT6)
    {
    	flag |= BIT2;		// flag bit 2 is set
    	P3IFG &= ~BIT6; 	// P3.6 IFG cleared
    }

	P3IE |= (BIT5 + BIT6);		// open P3 interrupt
}

/*
 * Port 1 interrupt service routine to handle S2 button press
 *
 */
#if defined (__TI_COMPILER_VERSION__) || defined (__IAR_SYSTEMS_ICC__)
#pragma vector = PORT1_VECTOR
__interrupt
#elif defined(__GNUC__)
void Port_1_ISR(void) __attribute__((interrupt(PORT1_VECTOR)));
#else
#error Compiler not supported!
#endif
void Port_1_ISR(void)
{
	P1IE &= ~BIT1;
	if(P1IFG & BIT1) // if its our flag
	{
		flag |= BIT0;		// Set S2 bit flag
		P1IFG &= ~BIT1; 	// Clear P1.1 IFG
	}
	P1IE |= BIT1;
}

/*
 * Port 4 interrupt service routine to handle S1 button press
 *
 */
#if defined (__TI_COMPILER_VERSION__) || defined (__IAR_SYSTEMS_ICC__)
#pragma vector = PORT4_VECTOR
__interrupt
#elif defined(__GNUC__)
void Port_4_ISR(void) __attribute__((interrupt(PORT4_VECTOR)));
#else
#error Compiler not supported!
#endif
void Port_4_ISR(void)
{
	P4IE &= ~BIT5;
	if(P4IFG & BIT5) // if its our flag
	{
		flag ^= BIT9;		// flip CH0/CH1
		flag ^= BITA;		// flip S1 bit flag
		P4IFG &= ~BIT5; 	// Clear P4.5 IFG
	}
	P4IE |= BIT5;
}


/*
 * USCI_A0 Interrupt Service Routine that receives PC I/O
 */
#if defined (__TI_COMPILER_VERSION__) || defined (__IAR_SYSTEMS_ICC__)
#pragma vector = USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
#elif defined(__GNUC__)
void USCI_A0_ISR(void) __attribute__((interrupt(USCI_A0_VECTOR)));
void USCI_A0_ISR(void)
#else
#error Compiler not supported!
#endif
{
    switch (__even_in_range(UCA0IV, USCI_UART_UCTXCPTIFG)) {
        case USCI_NONE: break;
        case USCI_UART_UCRXIFG:
			while(!(UCA0IFG&UCTXIFG));
			UCA0TXBUF = UCA0RXBUF;
			__no_operation();
			break;
        case USCI_UART_UCTXIFG: break;
        case USCI_UART_UCSTTIFG: break;
        case USCI_UART_UCTXCPTIFG: break;
    }
}

/*
 *  ======== Timer0_A0 Interrupt Service Routine ========
 *    set for 0.5 sec interval
 */
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A0_VECTOR
__interrupt
#elif defined(__GNUC__)
void TIMER0_A0_ISR(void) __attribute__((interrupt(TIMER0_A0_VECTOR)));
#endif
void TIMER0_A0_ISR(void)
{
	flag |= BIT7;
	flag ^= BIT6;
	if (!(flag & BIT6))
		flag |= BIT3;
}

/*
 *  ======== Timer1_A0 Interrupt Service Routine ========
 *   set for 0.10 sec interval
 */
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER1_A0_VECTOR
__interrupt
#elif defined(__GNUC__)
void TIMER1_A0_ISR(void) __attribute__((interrupt(TIMER1_A0_VECTOR)));
#endif
void TIMER1_A0_ISR(void)
{
	flag |= BIT4;
	flag ^= BIT5;
}
