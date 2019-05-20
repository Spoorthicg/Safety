#include<avr/interrupt.h>
#include <avr/io.h>
#include<util/delay.h>

#define SET_BIT(PORT,PIN)  PORT|=(1<<PIN)
#define CLR_BIT(PORT,PIN)  PORT&=~(1<<PIN)
float distance=0; //integer for access obstacle distance
int pulse = 0;	 //interger to access all though the program
int edge = 0;	//interger for finding rising/falling edge
void speed_adjust(int x);
void trigger();
void find_object();

	
struct
{
 volatile unsigned int Flag_ISR1:1;
 volatile unsigned int FLAG:1;
}
FLAG_BIT;

void speed_adjust(int x)
	{  
	   if (x==0)
	     PORTB|=(1<<PB4);
	   else
	     PORTB&=~(1<<PB4);
	}
void trigger()
	 {
	    DDRD|=(1<<PD2);     
	    PORTD|=(1<<PIND2);
		_delay_us(15);    //triggering the sensor for 15usec
		PORTD &=~(1<<PIND2);
		DDRD&=~(1<<PD2);  // Make Echo pin as a input 
	 }
void find_object()
	{  
	  int speed;
	  if((distance<15) ||(distance>130))
		{
	     	PORTB&=~(1<<PB5);
	    	speed=0;
	    	speed_adjust(speed);
	 	}
	  else if((distance>15)&&(distance<130))
		{
	   		PORTB|=(1<<PB5);
	  	 	_delay_ms(100);
	    	PORTB&=~(1<<PB5);
	   			_delay_ms(100);	   
	   		speed=1;
	    	speed_adjust(speed);
	    } 
	}
void adc_init()
{
  ADMUX=0x00;
  ADMUX |=(1<<REFS0);
  ADCSRA|=(1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADIE);
  ADCSRA|=(1<<ADSC);
  while(ADCSRA & (1<<ADSC));
}
void extint1()
{
 EICRA=(1<<ISC10);
 EIMSK=(1<<INT1);
 EIMSK|=(1<<INT0);	//enabling ext interrupt
 EICRA|=(1<<ISC00);	//setting interrupt triggering logic change
 PCICR |= (1 << PCIE0);   // set PCIE0 to enable PCMSK0 scan
 PCMSK0 |= (1 << PCINT0); // set PCINT8 to trigger an interrupt on state change
 sei();
}
void colission()
{
  trigger();
  _delay_ms(50);
             //Serial.begin(9600);
	         //Serial.println("Distance:");
  _delay_ms(100);
	    		
	         //Serial.println(distance);
  find_object();

}
void park_brake()
{
  CLR_BIT(DDRD,PD1);
  SET_BIT(DDRB,PB3);
  SET_BIT(DDRB,PB0);
  
    if(PIND&(1<<PD1))
    {
      SET_BIT(PORTB,PB0);
      CLR_BIT(PORTB,PB3);
    }
    else
    {
      CLR_BIT(PORTB,PB0);
      SET_BIT(PORTB,PB3);
    }
}


int adc1()
{
  ADMUX&=0x00;
  ADMUX|=(1<<REFS0);
  ADCSRA|=(1<<ADEN);
  ADCSRA|=(1<<ADSC);
  while(ADCSRA & (1<<ADSC));
  return ADC;
}
int adc2()
{
  ADMUX&=0x00;
  ADMUX|=(1<<MUX0);
  ADMUX|=(1<<REFS0);
  ADCSRA|=(1<<ADEN);
  ADCSRA|=(1<<ADSC);
  ADC=0;
  while(ADCSRA & (1<<ADSC));
  return ADC;
}
int adc3()
{
  ADMUX&=0x00;
  ADMUX|=(1<<MUX1);
  ADMUX|=(1<<REFS0);
  ADCSRA|=(1<<ADEN);
  ADCSRA|=(1<<ADSC);
  ADC=0;
  while(ADCSRA & (1<<ADSC));
  return ADC;
}

void power_steering()
{
  CLR_BIT(DDRC,PC1);
  CLR_BIT(DDRC,PC2);
  CLR_BIT(DDRC,PC3);
  SET_BIT(DDRD,PD7);//motor
  SET_BIT(DDRB,PB1);//led
  
  uint16_t speed=0;
  uint16_t angle=0;
  uint16_t torque=0;
  { 
    speed= adc1();
    angle= adc2();
    torque= adc3();
    
    if((speed<=200) && (angle<=300)&& (torque<=100))
    {
      SET_BIT(PORTD,PD7);
      SET_BIT(PORTB,PB1);
    }
    else if((speed>200) && (angle>300) && (torque<50))
    {
      SET_BIT(PORTD,PD7);
      CLR_BIT(PORTB,PB1);
    }
    else
    {
      SET_BIT(PORTD,PD7);
      CLR_BIT(PORTB,PB1);
    }
  }
}

int main(void)
{
  CLR_BIT(DDRB,PB2);//connected cmos switch
  SET_BIT(PORTB,PB2);
  SET_BIT(DDRD,PD4);//right camera
  SET_BIT(DDRD,PD5);//left camera
  SET_BIT(DDRD,PD6);//back camera
  CLR_BIT(DDRC,PC0);//Angular sensor values
  uint8_t read_switch;
   
  DDRB |= (1<<PB4);     	//putting portB output pins
  DDRD &= ~(1<<PD2); 
  DDRB |= (1<<PB5);
  extint1();
    while(1)
    {
      if(FLAG_BIT.Flag_ISR1)
      {
        colission();
        park_brake();
        power_steering();
             if(FLAG_BIT.FLAG==1)
        {
          adc_init();
        }
        else
        {
          CLR_BIT(PORTD,PD5);
        CLR_BIT(PORTD,PD6);
        CLR_BIT(PORTD,PD4);
      }
    }
 }
   return 0;
}

ISR(INT1_vect)
{
  if((PIND&(1<<PD3)))
  FLAG_BIT.Flag_ISR1=1; 
  else
  FLAG_BIT.Flag_ISR1=0; 
}
ISR(PCINT0_vect)
{
  FLAG_BIT.FLAG=1;
}

ISR(INT0_vect)//interrupt service routine when there is a change in logic level
{
  if (edge==1)//when logic from HIGH to LOW
  {
    TCCR1B=0;//disabling counter
	pulse=TCNT1;//count memory is updated to integer
	distance=pulse/2.18543;
	PORTD &=~(1<<PIND2);
	TCNT1=0;//resetting the counter memory
	edge=0;
  }
  if (edge==0)//when logic change from LOW to HIGH
  {
    TCCR1B|=((1<<CS12)|(1<<CS10));//enabling counter PRESCALAR 1024
	edge=1;
  }
}

ISR(ADC_vect)
{
 if((ADC>=0)&&(ADC<80))
 {
   SET_BIT(PORTD,PD4);
   CLR_BIT(PORTD,PD5);
   CLR_BIT(PORTD,PD6);
 }
  else if((ADC>=80)&&(ADC<=160))
  {
    SET_BIT(PORTD,PD6);
    CLR_BIT(PORTD,PD4);
    CLR_BIT(PORTD,PD5);
  }
  else if((ADC>160))
  {
    SET_BIT(PORTD,PD5);
    CLR_BIT(PORTD,PD6);
    CLR_BIT(PORTD,PD4);
  }
}
