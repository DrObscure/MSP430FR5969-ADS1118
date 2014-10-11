/******************************************************************************
 * ADS1118.c
 * ADC mode: Set the configuration to AIN0/AIN1, FS=+/-0.256, SS, DR=250sps, PULLUP on DOUT
 * Temperature mode: DR=250sps, PULLUP on DOUT
 *
 *  - Created on: Nov 15, 2012
 *	- modification data: 8 Jan. 2012
 *	- author: Wayne Xu (a0219294)
 *	- version: v1.3
 *
 *	September 2014 - heavily modified to work with the MSP430FR5969 chip and Lanuchpad
 *		-- by Dr. C. L. Fullmer
 *
 ******************************************************************************/
//#include <msp430g2553.h>
//#include <ti/mcu/msp430/csl/CSL.h>

#include <msp430fr5969.h>
#include <driverlib.h>
#include "ADS1118.h"


/******************************************************************************
 * function: WriteSPI(unsigned int config, int mode)
 * introduction: write SPI to transmit the configuration parameter for ADS11118, and receive the convertion result.
 * parameters: config: configuration parameter of ADS11118's register, mode (0/1): internal temperature sensor, far-end temperature
 * return value: ADC result
 *
 * Modifed for FR5969 and DriverLib
 * 		Dr. C.L. Fullmer -- September 2014
 *
*******************************************************************************/

static int msb;
static unsigned int temp;
static uint8_t dummy1, dummy2, dummy3, dummy0;

int WriteSPI(unsigned int config, int mode)
{

	temp = config;
// we always set BIT15 for the data SS conversion in config for now
//	if (mode==1)
//		temp = (temp | 0x8000);		// mode == 1, means to read the data and start a new conversion.

    // Set CS low to start access to device
	P3OUT &= ~BIT4;

	//change the polarity of UCI0B_CLK to driver ADS1118.
	// OK for FR5969 driverlib as is
    UCB0CTL1 |= UCSWRST;
    UCB0CTL0 = (UCMSB + UCMST + UCMODE_0 + UCSYNC)>>8;
    UCB0CTL1 &= ~UCSWRST;

// Xmit MSB
	//USCI_B0 TX buffer ready?
	while (!EUSCI_B_SPI_getInterruptStatus(EUSCI_B0_BASE,
										EUSCI_B_SPI_TRANSMIT_INTERRUPT)) ;
	//Transmit MSB to slave
	EUSCI_B_SPI_transmitData(EUSCI_B0_BASE, (uint8_t)((temp & 0xFF00) >> 8 ));
	// wait until done
	while (!EUSCI_B_SPI_getInterruptStatus(EUSCI_B0_BASE,
											EUSCI_B_SPI_RECEIVE_INTERRUPT)) ;
	dummy0 =  EUSCI_B_SPI_receiveData(EUSCI_B0_BASE);

// Xmit LSB
	//USCI_B0 TX buffer ready?
	while (!EUSCI_B_SPI_getInterruptStatus(EUSCI_B0_BASE,
										EUSCI_B_SPI_TRANSMIT_INTERRUPT));
	//Transmit LSB to slave
	EUSCI_B_SPI_transmitData(EUSCI_B0_BASE, (uint8_t)(temp & 0x00FF ));
	// wait until done
	while (!EUSCI_B_SPI_getInterruptStatus(EUSCI_B0_BASE,
											EUSCI_B_SPI_RECEIVE_INTERRUPT));
	dummy1 = EUSCI_B_SPI_receiveData(EUSCI_B0_BASE);

	// Note to self..
	// We repeat the process here again, and throw the result away...
	// it is the return of the original command, so we can use it for a sanity check
	//

	//USCI_B0 TX buffer ready?
	while (!EUSCI_B_SPI_getInterruptStatus(EUSCI_B0_BASE,
										EUSCI_B_SPI_TRANSMIT_INTERRUPT)) ;
	//Transmit MSB to slave
	EUSCI_B_SPI_transmitData(EUSCI_B0_BASE, (uint8_t)((temp & 0xFF00) >> 8 ));
	// wait until done
	while (!EUSCI_B_SPI_getInterruptStatus(EUSCI_B0_BASE,
											EUSCI_B_SPI_RECEIVE_INTERRUPT)) ;
	dummy2 =  EUSCI_B_SPI_receiveData(EUSCI_B0_BASE);

	//USCI_B0 TX buffer ready?
	while (!EUSCI_B_SPI_getInterruptStatus(EUSCI_B0_BASE,
										EUSCI_B_SPI_TRANSMIT_INTERRUPT));
	//Transmit LSB to slave
	EUSCI_B_SPI_transmitData(EUSCI_B0_BASE, (uint8_t)(temp & 0xFF ));
	// wait until done
	while (!EUSCI_B_SPI_getInterruptStatus(EUSCI_B0_BASE,
											EUSCI_B_SPI_RECEIVE_INTERRUPT));
	dummy3 = EUSCI_B_SPI_receiveData(EUSCI_B0_BASE);

	//change back the polarity of UCI0B_CLK for deriving LCD
    UCB0CTL1 |= UCSWRST;
    UCB0CTL0 = (UCCKPL + UCMSB + UCMST + UCMODE_0 + UCSYNC)>>8;
    UCB0CTL1 &= ~UCSWRST;

    // Set CS high to end transaction
    P3OUT |= BIT4;

    // sanity check... we should get our original config back
    dummy2 |= 0x80; // reset high bit.. 	ASD returns 0 for this bit
    dummy3 &= 0xFE; // clear lowest bit..	ASD returns 1 for this bit
    temp = ((dummy2 << 8) | dummy3);
    if(config == temp)
    	msb = (unsigned int)((dummy0<<8) | dummy1);
    else
    	msb = 0x0; // error on the SPI Tx/Rx
    return msb;
}

/******************************************************************************
 * function: ADS_Config (unsigned int mode)
 * introduction: configure and start conversion.
 * parameters:
 * mode = 0, ADS1118 is set to convert the voltage of integrated temperature sensor.
 * mode = 1, ADS1118 is set to convert the voltage of thermocouple.
 * return value:
*******************************************************************************/
void ADS_Config(unsigned int mode)
{

	unsigned int tmp;
	if(flag & BIT9)		// Set the configuration to AIN2/AIN3, FS=+/-0.256, SS, DR=128sps, PULLUP on DOUT
	{
		if (mode==1)
			tmp = ADSCON_CH1;
		else
			tmp = ADSCON_CH1 + ADS1118_TS;	// temperature sensor mode.
	}
	else				// Set the configuration to AIN0/AIN1, FS=+/-0.256, SS, DR=128sps, PULLUP on DOUT
	{
		if (mode==1)
			tmp = ADSCON_CH0;
		else
			tmp = ADSCON_CH0 + ADS1118_TS;	// temperature sensor mode.
	}

	// Write the configuration...
	WriteSPI(tmp,1);

}

/******************************************************************************
 * function: ADS_Read(unsigned int mode)
 * introduction: read the ADC result and tart a new conversion.
 * parameters:
 * mode = 0, ADS1118 is set to convert the voltage of integrated temperature sensor.
 * mode = 1, ADS1118 is set to convert the voltage of thermocouple.
 * return value:result of last conversion
 */
int ADS_Read(unsigned int mode)
{
	unsigned int tmp;
	int result;

	if(flag & BIT9) // Set the configuration to AIN2/AIN3, FS=+/-0.256, SS, DR=128sps, PULLUP on DOUT
	{
		if (mode==1)
			tmp = ADSCON_CH1;
		else
			tmp = ADSCON_CH1 + ADS1118_TS;// temperature sensor mode.
	}
	else			// Set the configuration to AIN0/AIN1, FS=+/-0.256, SS, DR=128sps, PULLUP on DOUT
	{
		if (mode==1)
			tmp = ADSCON_CH0;
		else
			tmp = ADSCON_CH0 + ADS1118_TS;// temperature sensor mode.
	}

	// Write Config & start new conversion
	result = WriteSPI(tmp,1);

	return result;
}

/******************************************************************************
 * function: local_compensation(int local_code)
 * introduction:
 * this function transform internal temperature sensor code to compensation code, which is added to thermocouple code.
 * local_data is at the first 14bits of the 16bits data register.
 * So we let the result data to be divided by 4 to replace right shifting 2 bits
 * for internal temperature sensor, 32 LSBs is equal to 1 Celsius Degree.
 * We use local_code/4 to transform local data to n* 1/32 degree.
 * the local temperature is transformed to compensation code for thermocouple directly.
 *                                                   (Tin -T[n-1])
 * comp codes = Code[n-1] + (Code[n] - Code[n-1])* {---------------}
 *													(T[n] - T[n-1])
 * for example: 5-10 degree the equation is as below
 *
 * tmp = (0x001A*(local_temp - 5))/5 + 0x0019;
 *
 * 0x0019 is the 'Code[n-1]' for 5 Degrees; 	0x001A = (Code[n] - Code[n-1])
 * (local_temp - 5) is (Tin -T[n-1]);			denominator '5' is (T[n] - T[n-1])
 *
 * the compensation range of local temperature is 0-125.
 * parameters: local_code, internal sensor result
 * return value: compensation codes
 ******************************************************************************/
int	local_compensation(int local_code)
{
	float tmp,local_temp;
	int comp;
	local_code = (float)(local_code / 4);
	local_temp = (float)(local_code / 32);	//

	if (local_temp >=0 && local_temp <=5)		//0~5
	{
		tmp  = (float)((0x0019*local_temp)/5);
		comp = (int)tmp;
	}
	else if (local_temp> 5 && local_temp <=10)	//5~10
	{
		tmp  = (float)((0x001A*(local_temp - 5))/5 + 0x0019);
		comp = (int)tmp;
	}
	else if (local_temp> 10 && local_temp <=20)	//10~20
	{
		tmp  = (float)((0x0033*(local_temp - 10))/10 + 0x0033);
		comp = (int)tmp;
	}
	else if (local_temp> 20 && local_temp <=30)	//20~30
	{
		tmp  = (float)((0x0034*(local_temp - 20))/10 + 0x0066);
		comp = (int)tmp;
	}
	else if (local_temp> 30 && local_temp <=40)	//30~40
	{
		tmp  = (float)((0x0034*(local_temp - 30))/10 + 0x009A);
		comp = (int)tmp;
	}
	else if (local_temp> 40 && local_temp <=50)	//40~50
	{
		tmp  = (float)((0x0035*(local_temp - 40))/10 + 0x00CE);
		comp = (int)tmp;
	}

	else if (local_temp> 50 && local_temp <=60)	//50~60
	{
		tmp  = (float)((0x0035*(local_temp - 50))/10 + 0x0103);
		comp = (int)tmp;
	}
	else if (local_temp> 60 && local_temp <=80)	//60~80
	{
		tmp  = (float)((0x006A*(local_temp - 60))/20 + 0x0138);
		comp = (int)tmp;
	}
	else if (local_temp> 80 && local_temp <=125)//80~125
	{
		tmp  = (float)((0x00EE*(local_temp - 80))/45 + 0x01A2);
		comp = (int)tmp;
	}
	else
	{
		comp = 0;
	}
	return comp;
}

/******************************************************************************
 * function: ADC_code2temp(int code)
 * introduction:
 * this function is used to convert ADC result codes to temperature.
 * converted temperature range is 0 to 500 Celsius degree
 * Omega Engineering Inc. Type K thermocouple is used, seebeck coefficient is about 40uV/Degree from 0 to 1000 degree.
 * ADC input range is +/-256mV. 16bits. so 1 LSB = 7.8125uV. the coefficient of code to temperature is 1 degree = 40/7.8125 LSBs.
 * Because of nonlinearity of thermocouple. Different coefficients are used in different temperature ranges.
 * the voltage codes is transformed to temperature as below equation
 * 							      (Codes - Code[n-1])
 * T = T[n-1] + (T[n]-T[n-1]) * {---------------------}
 * 							     (Code[n] - Code[n-1])
 *
 * parameters: code
 * return value: far-end temperature
*******************************************************************************/
int ADC_code2temp(int code)	// transform ADC code for far-end to temperature.
{
	float temp;
	int t;

	temp = (float)code;

	if (code > 0xFF6C && code <=0xFFB5)			//-30~-15
	{
		temp = (float)((15 * (temp - 0xFF6C)) / 0x0049) - 30.0f;
	}
	else if (code > 0xFFB5 && code <=0xFFFF)	//-15~0
	{
		temp = (float)((15 * (temp - 0xFFB5)) / 0x004B) - 15.0f;
	}
	else if (code >=0 && code <=0x0019)			//0~5
	{
		temp = (float)((5 * (temp - 0)) / 0x0019);
	}
	else if (code >0x0019 && code <=0x0033)		//5~10
	{
		temp = (float)((5 * (temp - 0x0019)) / 0x001A) + 5.0f;
	}
	else if (code >0x0033 && code <=0x0066)		//10~20
	{
		temp = (float)((10 * (temp - 0x0033)) / 0x0033) + 10.0f;
	}
	else if (code > 0x0066 && code <= 0x009A)	//20~30
	{
		temp = (float)((10 * (temp - 0x0066)) / 0x0034) + 20.0f;
	}
	else if (code > 0x009A && code <= 0x00CE)	//30~40
	{
		temp = (float)((10 * (temp - 0x009A)) / 0x0034) + 30.0f;
	}
	else if ( code > 0x00CE && code <= 0x0103)	//40~50
	{
		temp = (float)((10 * (temp - 0x00CE)) / 0x0035) + 40.0f;
	}
	else if ( code > 0x0103 && code <= 0x0138)	//50~60
	{
		temp = (float)((10 * (temp - 0x0103)) / 0x0035) + 50.0f;
	}
	else if (code > 0x0138 && code <=0x01A2)	//60~80
	{
		temp = (float)((20 * (temp - 0x0138)) / 0x006A) + 60.0f;
	}
	else if (code > 0x01A2 && code <= 0x020C)	//80~100
	{
		temp = (float)(((temp - 0x01A2) * 20)/ 0x06A) + 80.0f;
	}
	else if (code > 0x020C && code <= 0x02DE)	//100~140
	{
		temp = (float)(((temp - 0x020C) * 40)/ 0x0D2) + 100.0f;
	}
	else if (code > 0x02DE && code <= 0x03AC)	//140~180
	{
		temp = (float)(((temp - 0x02DE) * 40)/ 0x00CE) + 140.0f;
	}
	else if (code > 0x03AC && code <= 0x0478)	//180~220
	{
		temp = (float)(((temp - 0x03AB) * 40) / 0x00CD) + 180.0f;
	}
	else if (code > 0x0478 && code <= 0x0548)	//220~260
	{
		temp = (float)(((temp - 0x0478) * 40) / 0x00D0) + 220.0f;
	}
	else if (code > 0x0548 && code <= 0x061B)	//260~300
	{
		temp = (float)(((temp - 0x0548) * 40) / 0x00D3) + 260.0f;
	}
	else if (code > 0x061B && code <= 0x06F2)	//300~340
	{
		temp = (float)(((temp - 0x061B) * 40) /  0x00D7) + 300.0f;
	}
	else if (code > 0x06F2 && code <= 0x07C7)	//340~400
	{
		temp =(float) (((temp - 0x06F2) *  40)  / 0x00D5) + 340.0f;
	}
	else if (code > 0x07C7 && code <= 0x089F)	//380~420
	{
		temp =(float) (((temp - 0x07C7) * 40)  / 0x00D8) + 380.0f;
	}
	else if (code > 0x089F && code <= 0x0978)	//420~460
	{
		temp = (float)(((temp - 0x089F) * 40) / 0x00D9) + 420.0f;
	}
	else if (code > 0x0978 && code <=0x0A52)	//460~500
	{
		temp =(float)(((temp - 0x0978) * 40) / 0x00DA) + 460.0f;
	}
	else
	{
		temp = (float)0xA5A5;
	}

	t = (int)(10.0 * temp);

	return t;
}
