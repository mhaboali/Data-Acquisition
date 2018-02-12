/*
 * config.h
 *
 * Created: 11/02/2018 09:10:07 م
 *  Author: user
 */ 


#ifndef CONFIG_H_
#define CONFIG_H_

#define F_CPU 16000000 												// Clock Speed
#define BAUDRATE 9600
#define BAUD_PRESCALLER (((F_CPU / (BAUDRATE * 16UL))) - 1)
#include <avr/sleep.h>
#include <inttypes.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <math.h>
#define sbi(reg,pin) 		reg|=_BV(pin)
#define cbi(reg,pin) 		reg&=~_BV(pin)
#define tbi(reg,pin) 		reg^=_BV(pin)
#define is_high(reg,pin) 	((reg&_BV(pin))==1)

// ====LCD:

#define data_pinsmode		DDRD
#define databus				PORTD
#define control_pinsmode	DDRC
#define control_bus			PORTC

#define rs			PC0
#define rw			PC1
#define en			PC2
#define maxlines	2
#define maxchars	16
#define firstline	0x80
#define secondline 0xc0
#define blankspace ' '


void lcd_init_4bits();
void send_cmd_4bits( char cmd);
void send_char_4bits( char dat);

void send_str_4bits(char *string);
void goto_XY_4bits(uint8_t x,uint8_t y);
void send_str_4bits_withXY(uint8_t x,uint8_t y,char *string);
void send_int_withXY(uint8_t x,uint8_t y,int value,uint8_t no_digits);
void clear_screen_4bits();
void check_busy();

void Display_constants();
//=======LCD

//====Speed Sensor:
#define perimeter_cm		157								//Perimeter of our cycle
#define lap_distance_m		1200							//length of track
#define avg_distance_m		31.4							//length of 20 revolution of cycle
#define min_period			142								//minimum expected period of the maximum velocity
#define max_period			333								//maximum expected period of the minimum velocity
volatile uint8_t      laps=0;								//number of laps
volatile unsigned int count_t1=0,count_t2=0;				//for handling 2 values from timer counter for calculating frequency
volatile unsigned int count_avg_t1=0,count_avg_t2=0;		//for handling 2 values from timer counter for calculating Average speed
volatile float        time_diff=0,freq=0,period=0;			//for handling difference in time between 2 pulses
volatile unsigned int avg_time=0,total_spokes_counter=0,inst_spokes_counter=0;			//count instantaneous pulses coming from ir sensor
volatile unsigned int inst_16ms_counter =0,total_16ms_counter=0;			//count time for capture pulses time
volatile int      elapsed_distance_cm=0,elapsed_distance_m=0,inst_speed=0,avg_speed=0;

void Speeds_Calculation();
void Calculate_speeds_distance();
void Display_speeds_distance();
//for speed sensor signal and calculating the Ins speed
void init_ext_interrupts();					//for counting speed sensor pulses
void init_timer2();						//initializing timer2 with out compare mode
//=====================================================================================

//=======Servo By Throttle Thumb:
void			init_ThrottleThump();
uint16_t		Read_Throttle(uint8_t adcx) ;
void			Display_throttle();
volatile float	lcd_throttle_value=0;
volatile uint16_t digital_throt_sensor_read=0;
void		init_servo();
void		move_servo();
float		min_deg=2000;		//at zero digital_sensor_readue from ADC
float		max_deg=3130;		//at 1023 digital_sensor_readue from ADC
float		duty_cycle;

//==================

//======StopWatch:
void			init_timer0();
void			Display_time();
volatile uint16_t seconds=0;
volatile uint16_t minutes=0;
volatile uint16_t couter_prescaling=0;
//======================
ISR(TIMER2_COMPA_vect);
ISR(TIMER0_OVF_vect);
ISR(INT1_vect);							//on place 2 in interrupt vector table
ISR(ADC_vect);


#endif /* CONFIG_H_ */