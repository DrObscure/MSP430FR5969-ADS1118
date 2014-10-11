/*-
 * Name: LCD_driver.h
 * Introduction: the driver codes for LCD NHD-C0216CZ-NSW-BBW-3V3
 * Created on: Dec 11, 2012
 * Author: a0219294
 */

#ifndef LCD_DRIVER_H_
#define LCD_DRIVER_H_

void LCD_init(void);			//initialize LCD
void LCD_clear(void);			//Clean LCD
void LCD_display_string(unsigned char L, char *ptr);
void LCD_display_number(unsigned char H,unsigned char L,unsigned int num);
void LCD_display_temp(unsigned char H,unsigned char L,unsigned int num);
void LCD_display_HEX(unsigned char H,unsigned char L,unsigned int num);
void LCD_display_char(unsigned char H,unsigned char L,char ch);
void LCD_display_time(unsigned char H,unsigned char L,unsigned long seconds);

void LCD_writecom(unsigned char c);		//write command
void LCD_writedata(unsigned char d);	//write data
void LCD_delay_Nms(unsigned char i);	//delay i ms

// NOTE: Changed to FR5969 BOOST confiuration using Port 1
#define LCD_CS_LOW 		P1OUT &= ~BIT5;	//set CS low
#define LCD_CS_HIGH 	P1OUT |= BIT5;	//set CS high
#define LCD_RS_LOW 		P1OUT &= ~BIT4;	//set RS low
#define LCD_RS_HIGH 	P1OUT |= BIT4;	//set RS high
#define LCD_RST_LOW 	P1OUT &= ~BIT3;	//set RST low
#define LCD_RST_HIGH 	P1OUT |= BIT3;	//set RST high

// Set CS high to end transaction


#endif /* LCD_DRIVER_H_ */
