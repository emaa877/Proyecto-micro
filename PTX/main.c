#define ARDUINO 2 //1: Arduino UNO, 2: ARDUINO PRO-MINI

#include "main.h"
#include "nrf24l01.h"

//Utilizo comunicacion UART para verificar los registros. No es parte del proyecto la comunicacion UART
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

uint8_t ADC_Read(uint8_t adc_to_read);

struct NRF24{
	uint8_t PRX_PTX;
	uint8_t addr;
	uint8_t channel;
	uint8_t state;
};
struct NRF24 nrf24;

int main(void){
	nrf24.PRX_PTX = PTX; //PTX!!
	nrf24.addr = 0x01; //recordar que es de 5bytes. Sera address x5, cada byte igual
	nrf24.channel = 0x6E; //110 -> 2510 MHz
	
	configuraUART(9600,1,0); //Baudrate, interr RX, interr TX
	printf("...Inicializando...\n");
	
	PUD_OFF; //bit para activar las resistencias pull-up
	B1_INPUT; B2_INPUT; B3_INPUT; B4_INPUT; S1_INPUT; S1_INPUT;
	B1_PULL_UP; B2_PULL_UP; B3_PULL_UP; B4_PULL_UP; S1_PULL_UP; S2_PULL_UP;
			
	SS_OUTPUT;
	SPI_pin_setup();
	config_SPI_Master(0, 16); //modo=0 (polaridad 0, fase 0) y prescaler= 16 
	
	cli();
	NRF24L01_config(nrf24.PRX_PTX, nrf24.addr, nrf24.channel); 
	sei();
	
	printf("Register CONFIG: %d\n", Get_Reg(CONFIG));
	
	uint8_t payload[BUFFER_SIZE];
	
	while (1){
		payload[0] = ADC_Read(1); //Se leen los 8 MSB
		payload[1] = ADC_Read(0);
		payload[2] = ADC_Read(2);
		payload[3] = ADC_Read(3);
		payload[4] = ADC_Read(7);
		payload[5] = ADC_Read(6);
		
		payload[6]  = B1_TEST ? 1 : 0; //entradas digitales con pull-up, 0: presionadas, 1:sin presionar
		payload[7]  = B2_TEST ? 1 : 0;
		payload[8]  = B3_TEST ? 1 : 0;
		payload[9]  = B4_TEST ? 1 : 0;
		payload[10] = S1_TEST ? 1 : 0;
		payload[11] = S2_TEST ? 1 : 0;
			
		transmit_payload(payload, BUFFER_SIZE);	
		if(Get_Reg(STATUS) & BIT(MAX_RT)){ //encender LED VERDE
			for(uint8_t i=0; i<BUFFER_SIZE; i++){
				printf("%d ", payload[i]);
			}
			printf("\n");
		}
		reset_IRQ();
		_delay_ms(100); //hacerlo por interrupcion de timer el envio de datos
	}
}

uint8_t ADC_Read(uint8_t adc_to_read){
	uint8_t ADC_value;
	ADMUX = adc_to_read;
	SETBIT(ADMUX, REFS0);   //AREF
	SETBIT(ADMUX, ADLAR);   //resolucion 10 bits alineado a izq
	
	SETBITS(ADCSRA, 3);     //prescaler en 8
	SETBIT(ADCSRA, ADEN);
	SETBIT(ADCSRA, ADSC);   // inicia la conversion

	while(ADCSRA & (1 << ADSC)); //espera a que finalice la conversion

	ADC_value = ADCH; //tomo los bits mas significativos
	return ADC_value;
}
