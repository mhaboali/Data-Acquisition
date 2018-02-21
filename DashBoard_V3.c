/*
 * DashBoard_V2.c
 *
 * Created: 30/01/2018 01:22:12 م
 *  Author: AboAli
 */ 

#include "config.h"
char int_buff[7]={0};   	 	// using it in communication for converting integer to string function

int main(void)
{ 
   _delay_ms(500);
   //Indication LED:
   sbi(DDRB,PB0);				
   sbi(PORTB,PB0);
   //=======================
   sei(); 						//turn on global interrupts
   //LCD Initialization
   lcd_init_4bits();
   //=================
   //===Stop Watch Initialization:
   init_timer0();
   //====================
   //Speed Sensor Initialization:
   init_timer2();		//initialize timer0 with ctc mode and overflow every 16ms
   init_ext_interrupts();
   //=============================
   //=====Throttle Sensor:
   init_ThrottleThump();
   //=========================
   //=======servo:
   init_servo();
   duty_cycle=(max_deg-min_deg)/(1024);			//by cross multiplying 
   //============================
   
while (1)
{
	Display_constants();
	Calculate_speeds_distance();
	Display_speeds_distance();
	Display_time();
	Display_throttle();
	//UART_send_online();
}  
   return 0;
}
ISR(ADC_vect)							//on place 22 in interrupt vector table
{
	move_servo();
	sbi(ADCSRA,ADSC);
}					
ISR(TIMER0_OVF_vect)
{
	couter_prescaling++;
	if(couter_prescaling==243)		//240 for actual timing
	{
		seconds++;
		couter_prescaling=0;
	}
	if(seconds==60)
	{
		minutes++;
		seconds=0;
	}
}
ISR(TIMER2_COMPA_vect)
{
	TCNT2=0;
	inst_16ms_counter++;					//for counting how many 16ms through one cycle
	total_16ms_counter++;				//counting total time through a number of cycles
}


//================Speed Sensor functions implementation 
 void init_timer2()						//CALCUTE TIME FOR SPEED CALCULATIONS
{  
  //ctc mode
   sbi(TCCR2A,WGM21);
   //Prescaler 1024
   sbi(TCCR2B,CS20);
   sbi(TCCR2B,CS21);
   sbi(TCCR2B,CS22);
   //ENALBE outcompare interrupt:
   sbi(TIMSK2,OCIE2A);
   TCNT2=0x00;
   OCR2A=255;				//will overflow every 16ms
} 
void init_ext_interrupts()
{
   cbi(DDRD,PD3);			//sensor input
   sbi(PORTD,PD3);			//enable pull up resistor
   //EICRA|=(1<<ISC01)|(1<<ISC00);	//set triger INT0 for rising edge mode
   EICRA|=(1<<ISC11);			//set trigger INT1 for falling edge mode
   //EIMSK|=(1<<INT0);			//turn on INT0
   EIMSK|=(1<<INT1);			//turn on INT1
}
ISR(INT1_vect)							//on place 2 in interrupt vector table
{
	total_spokes_counter++;
	inst_spokes_counter++;
	if(total_spokes_counter==1)
	{
		count_avg_t1=total_16ms_counter;
		
	}
	else if(total_spokes_counter==200)
	{
		count_avg_t2=total_16ms_counter;
		avg_time=ceil((count_avg_t2-count_avg_t1)*0.016);	//time counters difference * 16ms
		total_16ms_counter=0;				//reset time counter
		total_spokes_counter=0;				//reset spokes counter
	}
	//for instantaneous speed
	if(inst_spokes_counter==1)				//here completing one cycle
	{
		count_t1=inst_16ms_counter;
		inst_spokes_counter++;
	}
	else if(inst_spokes_counter==10)
	{
		count_t2=inst_16ms_counter;
		inst_16ms_counter=0;				//reset instantaneous time counter
		inst_spokes_counter=0;				//reset instantaneous spokes counter
	}
}

void Calculate_speeds_distance()
{
	if(inst_spokes_counter==10)
	{
		elapsed_distance_cm+=perimeter_cm;			//every 10 pulses"one cycle" increase elasped distance by 157cm
		if((elapsed_distance_cm>=120000) && (elapsed_distance_cm<125000))
		{
			laps++;
			elapsed_distance_cm=0;
		}
	}
	if(inst_16ms_counter<35)
	{
		period=(count_t2-count_t1)*16;					//to calculate difference in time between two pulses.....16 standing for 16ms
		if(period>0)
			freq=(1000.00/period);						//freq=1/period...but 1000/period in HZ
		else
			freq=0;
		inst_speed=ceil((perimeter_cm*freq/100)*3.6);			//instantaneous speed in km/hour
		avg_speed=ceil((avg_distance_m/avg_time)*3.6); 			//average speed in km/hour
		elapsed_distance_m=(int)elapsed_distance_cm/100;
	}
	else
	{
		inst_speed=0;
	}
}
void Display_speeds_distance()
{
	//Calculate_speeds_distance();
	if(inst_speed>9)
	{
		send_int_withXY(0,1,inst_speed,2);
	}
	else
	{
		send_str_4bits_withXY(0,1,"0");
		send_int_withXY(1,1,inst_speed,1);
	}
	if(avg_speed>9)
	{
		send_int_withXY(9,1,avg_speed,2);
	}
	else
	{
		send_str_4bits_withXY(9,1,"0");
		send_int_withXY(10,1,avg_speed,1);
	}
	if(laps>9)
	{
		send_int_withXY(6,2,laps,2);
	}
	else
	{
		send_str_4bits_withXY(6,2,"0");
		send_int_withXY(7,2,laps,1);
	}
	
	//send_int_withXY(11,2,laps,2);
}
void Speeds_Calculation()
{
	//For Average Speed
	if(total_spokes_counter==1)
	{
		count_avg_t1=total_16ms_counter;
		
	}
	else if(total_spokes_counter==200)
	{
		count_avg_t2=total_16ms_counter;
		avg_time=ceil((count_avg_t2-count_avg_t1)*0.016);	//time counters difference * 16ms
		total_16ms_counter=0;				//reset time counter
		total_spokes_counter=0;				//reset spokes counter
	}
	//for instantaneous speed
	if(inst_spokes_counter==1)				//here completing one cycle
	{
		count_t1=inst_16ms_counter;
		inst_spokes_counter++;
	}
	else if(inst_spokes_counter==10)
	{
		count_t2=inst_16ms_counter;
		inst_16ms_counter=0;				//reset instantaneous time counter
		inst_spokes_counter=0;				//reset instantaneous spokes counter
		elapsed_distance_cm+=perimeter_cm;			//every 10 pulses"one cycle" increase elasped distance by 157cm
		if((elapsed_distance_cm>=120000) && (elapsed_distance_cm<125000))
		{
			laps++;
			elapsed_distance_cm=0;
		}
	}
}

//===============================================================

//==================LCD Functions Implementation
void lcd_init_4bits()
{
	_delay_ms(300);
	data_pinsmode|=0xf0;			// Configure both databus and controlbus as output
	control_pinsmode|=0x07;
	databus=0x00;  					//initialize data bus port
	_delay_ms(20);
	send_cmd_4bits(0x30);
	_delay_ms(5);
	send_cmd_4bits(0x30);
	_delay_ms(1);
	send_cmd_4bits(0x30);
	_delay_ms(1);
	send_cmd_4bits(0x02);			 //Initialize the LCD in 4bit Mode
	_delay_ms(1);
	send_cmd_4bits(0x28);
	_delay_ms(1);
	send_cmd_4bits(0x06);			//entry mode set: increment cursor & without shifting entire display
	_delay_ms(1);
	send_cmd_4bits(0x14);			//cursor or display shift: only cursor shifted right
	_delay_ms(1);
	send_cmd_4bits(0x0E);			// Display ON cursor ON
	_delay_ms(1);
	send_cmd_4bits(0x40);			//enable CGRAM
	_delay_ms(1);
	send_cmd_4bits(0x80);			// Move the Cursor to First line First Position
	//send_cmd_4bits(0x01);
}
void send_cmd_4bits( char cmd)
{
	databus=(cmd & 0xf0)|(databus&0x0f);				// Send the Higher Nibble of the command to LCD
	control_bus &=~(1<<rs);								// Select the Command Register by pulling RS LOW
	control_bus &=~(1<<rw);								// Select the Write Operation  by pulling RW LOW
	control_bus |=1<<en;								// Send a High-to-Low Pulse at Enable Pin
	_delay_us(1);
	control_bus &=~(1<<en);
	
	_delay_us(10);										// wait for some time
	
	databus=((cmd<<4) & 0xf0)|(databus&0x0f);			// Send the Lower Nibble of the command to LCD
	control_bus &=~(1<<rs);								// Select the Command Register by pulling RS LOW
	control_bus &=~(1<<rw);								// Select the Write Operation  by pulling RW LOW
	control_bus |=1<<en;								// Send a High-to-Low Pulse at Enable Pin
	_delay_us(1);
	control_bus &=~(1<<en);	
	_delay_ms(1);
}
void send_char_4bits( char dat)
{
	//check_busy();
	databus=(dat & 0xf0)|(databus&0x0f);			// Send the Higher Nibble of the Data to LCD
	control_bus |=1<<rs;							// Select the Data Register by pulling RS HIGH
	control_bus &=~(1<<rw);							// Select the Write Operation  by pulling RW LOW
	control_bus |=1<<en;							// Send a High-to-Low Pulse at Enable Pin
	_delay_us(1);
	control_bus &=~(1<<en);
	_delay_us(10);
	
	databus=((dat <<4) & 0xf0)|(databus&0x0f);		// Send the Lower Nibble of the Data to LCD
	control_bus |=1<<rs;							// Select the Data Register by pulling RS HIGH
	control_bus &=~(1<<rw);							// Select the Write Operation  by pulling RW LOW
	control_bus |=1<<en;							// Send a High-to-Low Pulse at Enable Pin
	_delay_us(1);
	control_bus &=~(1<<en);
	_delay_ms(1);
}
void send_str_4bits(char *string)
{
	while(*string!='\0')
	{
		send_char_4bits(*string++);
	}
}
void goto_XY_4bits(uint8_t x,uint8_t y)
{
	if (y==1)
	{
		send_cmd_4bits(firstline+x);
	}
	else if(y==2)
	{
		send_cmd_4bits(secondline+x);
	}
}
void send_str_4bits_withXY(uint8_t x,uint8_t y,char *string)
{
	goto_XY_4bits(x,y);
	send_str_4bits(string);
}
void send_int_withXY(uint8_t x,uint8_t y,int value,uint8_t no_digits)
{
	char stringToDisplay[no_digits];
	itoa(value,stringToDisplay,10);
	send_str_4bits_withXY(x,y,stringToDisplay);
	//send_str_4bits(" ");
}
void check_busy()
{
	data_pinsmode=0;
	sbi(control_bus,rw);	//read mode
	cbi(control_bus,rs);	//command mode
	while(databus >= 0x80)			//check for busy (when it's <0x80 it's not busy
	{
		//flashing on and off
	}
	
	data_pinsmode=0xff;
}
void clear_screen_4bits()
{
	send_cmd_4bits(0x01);
	_delay_ms(2);
}
void Display_constants()
{
	 send_str_4bits_withXY(2,1," Km/h");		//for instantaneous speed
	 send_str_4bits_withXY(7,1,"  ");
	 send_str_4bits_withXY(8,2,"LAPs");
	 send_str_4bits_withXY(11,1," Km/h");
	 send_str_4bits_withXY(2,2,":");		//for stopwatch
	 send_str_4bits_withXY(15,2,"%");		//for Throttle
}
//===================================================

//================Thumb Throttle Functions implementation
void init_ThrottleThump()
{  
   //voltage reference:(with AVCC reference):
    sbi(ADMUX,REFS0);   
    //cbi(ADMUX,REFS1);

    //select ADC3 Channel:
    sbi(ADMUX,MUX0);
    sbi(ADMUX,MUX1);
    cbi(ADMUX,MUX2);
    cbi(ADMUX,MUX3);

    cbi(ADMUX,ADLAR);    //right adjustment

    //prescaler selector: (128 division factor):
    sbi(ADCSRA,ADPS0);
    sbi(ADCSRA,ADPS1);
    sbi(ADCSRA,ADPS2);
    sbi(ADCSRA,ADEN);	   //enable ADC
	sbi(ADCSRA,ADIE);	   //enable ADC interrupt
	sbi(ADCSRA,ADSC);	   //start first conversion

    //consumption reduction:
        //disable digital inputs:
        DIDR0=0xff;
        //POWER reduction:
        sbi(SMCR,SE);   //ENABLE SLEEP MODE
        sbi(SMCR,SM0);  //ADC noise cancellation
	
}
uint16_t Read_Throttle(uint8_t adcx) 
{
	/* adcx is the analog pin we want to use.  ADMUX's first few bits are
	 * the binary representations of the numbers of the pins so we can
	 * just 'OR' the pin's number with ADMUX to select that pin.
	 * We first zero the four bits by setting ADMUX equal to its higher
	 * four bits. */
	ADMUX	&=	0xf0;
	ADMUX	|=	adcx;

	/* This starts the conversion. */
	ADCSRA |= _BV(ADSC);

	/* This is an idle loop that just wait around until the conversion
	 * is finished.  It constantly checks ADCSRA's ADSC bit, which we just
	 * set above, to see if it is still set.  This bit is automatically
	 * reset (zeroed) when the conversion is ready so if we do this in
	 * a loop the loop will just go until the conversion is ready. */
	while ( (ADCSRA & _BV(ADSC)) );

	/* Finally, we return the converted value to the calling function. */
	return ADC;
}
void Display_throttle()
{
   digital_throt_sensor_read=Read_Throttle(3);
   lcd_throttle_value=floorf(((digital_throt_sensor_read-161)/(1023.0))*99*1.45);
   
   if(lcd_throttle_value>9)
   {
	   send_int_withXY(13,2,lcd_throttle_value,2);
   }
   else
   {
	   send_str_4bits_withXY(13,2,"0");
	   send_int_withXY(14,2,lcd_throttle_value,1);
   }
}
//==============================

//===============Servo Functions implementation
void init_servo()
{
	//set OC1A FOR generating PWM
	sbi(DDRB,PB1);
	cbi(PORTB,PB1);

	//SELECT MODE:
	//fast PWM: (with top = ICR1 & update its value @ BOTTOM , TOV flag set on TOP)
	sbi(TCCR1A,WGM11);
	cbi(TCCR1A,WGM10);
	sbi(TCCR1B,WGM13);
	sbi(TCCR1B,WGM12);
	// non inverting mode : to control on servo motion @ last 2ms in period
	sbi(TCCR1A,COM1A0);
	sbi(TCCR1A,COM1A1);

	//PRECALING (64):(to have 40000 cycle per second which means that every ms has 250 cycles)
	sbi(TCCR1B,CS11);
	//sbi(TCCR1B,CS10);
	//setting top value equal 39999 @which starting a new clock
	ICR1=39999;
	
}
void move_servo()
{
	
	OCR1A=(ICR1-max_deg)+(duty_cycle*(digital_throt_sensor_read-150));

}
//=============================================

//=============Stop Watch Functions implementation
void init_timer0()
{
	sbi(TCCR0B,CS02); 		//PREscaler 256
	sbi(TIMSK0,TOIE0);		//enable overflow interrupt
	
}
void Display_time()
{
	if(minutes>9)
		send_int_withXY(0,2,minutes,2);
	else
	{
		send_str_4bits_withXY(0,2,"0");
		send_int_withXY(1,2,minutes,1);
	}
	if(seconds>9)
		send_int_withXY(3,2,seconds,2);
	else
	{
		send_str_4bits_withXY(3,2,"0");
		send_int_withXY(4,2,seconds,1);
	}
}
//===================================================

void UART_init(void)
{
//	UBRR0H = (uint8_t)(BAUD_PRESCALLER>>8);
//	UBRR0L = (uint8_t)(BAUD_PRESCALLER);
	UBRR0 = 103;
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	UCSR0C = (3<<UCSZ00);
}
unsigned char UART_receive(void)
{
	while(!(UCSR0A & (1<<RXC0)));
	return UDR0;
	
}

void UART_send( unsigned char data)
{
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0 = data;
}

void WriteUART(uint16_t data)
{
	while(!(UCSR0A & (1<<0x05)));	//Wait for empty transmit buffer
	UDR0 = data;		//Write data into the buffer and transmit
}

 void UART_putstring(char* StringPtr)
 {
	 while(*StringPtr != 0x00)
	 {
		 UART_send(*StringPtr);
		 StringPtr++;
	 }
 }
void UART_send_online()
{
	UART_send('s');
	//UART_putstring("inst_speed:   ");
	itoa(inst_speed,int_buff,10);
	UART_putstring(int_buff);
	UART_send('a');
	//UART_putstring("avg_speed:   ");
	itoa(avg_speed,int_buff,10);
	UART_putstring(int_buff);
	UART_send('b');
	//UART_putstring("laps:   ");
	itoa(laps,int_buff,10);
	UART_putstring(int_buff);
	UART_send('c');
	//UART_putstring("throttle_value:   ");
	itoa(lcd_throttle_value,int_buff,10);
	UART_putstring(int_buff);
	UART_send('d');
}
 

/*
void UART_putstring(char* StringPtr)
{
	uint8_t counter=0;
	char * data;
	while(*StringPtr!=0x00)
	{
		counter++;
	}
	if (counter ==1)
	{
		*data = '0';
		//data++;
		*(data+1) = *StringPtr;
		*(data+2)='\0';
	}
	else
	{
		data = StringPtr;
		
	}
	while(*data != 0x00)
	{
		UART_send(*data);
		data++;
	}
}
*/