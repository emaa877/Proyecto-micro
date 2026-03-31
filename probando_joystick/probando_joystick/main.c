#define F_CPU 16000000
#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <inttypes.h>

#define TRUE  1
#define FALSE 0

#define BIT(x) (1<<x)
#define SETBITS(x,y) (x |= y)
#define CLEARBITS(x,y) (x &=~ y)
#define SETBIT(x,y) SETBITS(x, BIT(y))
#define CLEARBIT(x,y) CLEARBITS(x, BIT(y))
#define TESTBIT(x,y)  (x & BIT(y))

#define SS_OUTPUT (SETBIT(DDRB, DDB2)) //pin10 - SS

//Botones
#define B1_INPUT (CLEARBIT(DDRB, DDB0))
#define B1_PULL_UP (SETBIT(PORTB, PORTB0))
#define B1_TEST  (TESTBIT(PINB, PINB0))

#define B2_INPUT (CLEARBIT(DDRB, DDB1))
#define B2_PULL_UP (SETBIT(PORTB, PORTB1))
#define B2_TEST  (TESTBIT(PINB, PINB1))

#define B3_INPUT (CLEARBIT(DDRD, DDD3))
#define B3_PULL_UP (SETBIT(PORTD, PORTD3))
#define B3_TEST  (TESTBIT(PIND, PIND3))

#define B4_INPUT (CLEARBIT(DDRD, DDD2))
#define B4_PULL_UP (SETBIT(PORTD, PORTD2))
#define B4_TEST  (TESTBIT(PIND, PIND2))

//Switches
#define S1_INPUT (CLEARBIT(DDRD, DDD7))
#define S1_PULL_UP (SETBIT(PORTD, PORTD7))
#define S1_TEST  (TESTBIT(PIND, PIND7))

#define S2_INPUT (CLEARBIT(DDRD, DDD4))
#define S2_PULL_UP (SETBIT(PORTD, PORTD4))
#define S2_TEST  (TESTBIT(PIND, PIND4))

#define PUD_OFF  (CLEARBIT(MCUCR, PUD))

//#################UART#########################UART########
#include <stdlib.h> //libreria para atoi
#include <stdbool.h>
int mi_putc(char c,FILE *stream){
	while(!(UCSR0A&(1<<UDRE0))); //transmisión ocupada
	UDR0=c; //Cuando se desocupa, UDR0 puede recibir el nuevo dato c a trasmitir
	return 0;
}
int mi_getc(FILE *stream){
	while(!(UCSR0A&(1<<RXC0)));//recepción incompleta
	return UDR0;//Cuando se completa, se lee UDR0
}
#define fgetc()  mi_getc(&uart_io)
#define fputc(x) mi_putc(x,&uart_io)
FILE uart_io=FDEV_SETUP_STREAM(mi_putc,mi_getc,_FDEV_SETUP_RW);
void configuraUART(int baud_rate,bool Rx,bool Tx){
	UBRR0 =F_CPU/16/baud_rate-1;    //Configura baudrate
	UCSR0A &= ~(1<<U2X0);		    //Velocidad simple
	UCSR0B |=(1<<RXEN0)|(1<<TXEN0); //Habilita recepcion y transmision
	UCSR0C |=(1<<USBS0)|(3<<UCSZ00);//2 bits stop, 8 bits de dato
	stdout = stdin = &uart_io;
	if(Rx){
		UCSR0A &= ~(1<<RXC0); //apaga flag RX completa
		UCSR0B |= (1<<RXCIE0);
	}
	if(Tx){
		UCSR0A &= ~(1<<TXC0); //apaga flag TX completa
		UCSR0B |= (1<<TXCIE0);
	}
}
//##########################################################

void to_binary(char *text, uint8_t n);
uint8_t ADC_Read(uint8_t adctouse);

int main(void){
	configuraUART(9600,1,0); //Baudrate, interr RX, interr TX
	printf("...Inicializando...\n");
	PUD_OFF; //bit para activar las resistencias pull-up
	
	B1_INPUT; B2_INPUT; B3_INPUT; B4_INPUT; S1_INPUT; S1_INPUT;
	B1_PULL_UP; B2_PULL_UP; B3_PULL_UP; B4_PULL_UP; S1_PULL_UP; S2_PULL_UP;
	
	uint8_t digital_inputs[6];
	uint8_t analogic_inputs[6];
		
	while (1){
		_delay_ms(100);
		analogic_inputs[0] = ADC_Read(1);
		analogic_inputs[1] = ADC_Read(0);
		analogic_inputs[2] = ADC_Read(2);
		analogic_inputs[3] = ADC_Read(3);
		analogic_inputs[4] = ADC_Read(7);
		analogic_inputs[5] = ADC_Read(6);		
		printf("%d %d %d %d %d %d ", analogic_inputs[0], analogic_inputs[1], analogic_inputs[2], analogic_inputs[3], analogic_inputs[4], analogic_inputs[5]);
		
		digital_inputs[0] = B1_TEST;
		digital_inputs[1] = (uint8_t)(B2_TEST /2);
		digital_inputs[2] = (uint8_t)(B3_TEST /8);
		digital_inputs[3] = (uint8_t)(B4_TEST /4);
		digital_inputs[4] = (uint8_t)(S1_TEST /128);
		digital_inputs[5] = (uint8_t)(S2_TEST /16);
		printf("%d %d %d %d %d %d\n", digital_inputs[0], digital_inputs[1], digital_inputs[2], digital_inputs[3], digital_inputs[4], digital_inputs[5]);
    }
}

void to_binary(char *text, uint8_t n){
	printf("%s: ", text);
	uint8_t i;
	for (i= 1<<7; i>0; i=i/2) //8 bits
	(n&i) ? printf("1") : printf("0");
	printf("\n");
}

uint8_t ADC_Read(uint8_t adc_to_read){
	//uint16_t ADC_value;
	uint8_t ADC_value;
	ADMUX = adc_to_read;
	SETBIT(ADMUX, REFS0);   //AREF
	SETBIT(ADMUX, ADLAR); //resolucion 10 bits alineado a izq
	
	SETBITS(ADCSRA, 3);     //prescaler en 8
	SETBIT(ADCSRA, ADEN);
	SETBIT(ADCSRA, ADSC);   // inicia la conversion

	while(ADCSRA & (1 << ADSC)); //espera a que finalice la conversion

	//ADC_value = ADCL;
	//ADC_value = (ADCH << 8) + ADC_value;
	ADC_value = ADCH; //tomo los bits mas significativos
	return ADC_value;
}
