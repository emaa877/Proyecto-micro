#define ARDUINO 1 //1: Arduino UNO, 2: ARDUINO PRO-MINI

#include "main.h"
#include "timer0.h"
#include "timer1.h"
#include "nrf24l01.h"

volatile float T; //periodo del pulso TC1

volatile uint8_t motor1_moving = FALSE; //FALSE parado, TRUE andando
volatile uint8_t motor2_moving = FALSE; //FALSE parado, TRUE andando
volatile uint8_t limit_switch_m1 = 0;   //0 ninguno, 1 izq, 2 der. Permite tener un conocimiento de que fin de carrera se alcanzo
volatile uint8_t limit_switch_m2 = 0;   //0 ninguno, 1 llego al optoacoplador
volatile uint8_t timer_connection = 0; //permite conocer el estado de la conexion: se resetea a 0 si
//recibio un nuevo dato y se incrementa en 1 por cada interrupcion del timer 0. Si llega a valer 100
//(en tiempo: 100*T_TC0) se resetea a 0 y se cambia el estado a DISCONNECTED xq quiere decir que nunca
//se reseteo a 0 por haber recibido un nuevo dato
volatile uint8_t motor1_reach_limit_switch = 0;
enum State{init, initial_homing, homing_M1, homing_M2, receive_p, manual_mode, automatic_mode};
volatile uint8_t last_state = 0;
volatile enum State state;

void configure_INT0();
void configure_INT1();
void configure_PCINT0();
void configure_PCINT11();
uint8_t reach_limit_switch_M1(uint8_t);
void homing(uint8_t motor, uint8_t l_switch);
void master_config();

struct NRF24{
	uint8_t PRX_PTX;
	uint8_t addr;
	uint8_t channel;
	uint8_t state;
	uint8_t payload[BUFFER_SIZE];
};
struct NRF24 nrf24;

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

int main(void){	
	state = init;
	
	while (1){	 
		switch(state){
			case init:
				configuraUART(9600,1,0); //Baudrate, interr RX, interr TX
				printf("...Inicializando...\n");
				
				/*float Tmax = 0.80;   //0.8 para M1 step 1/8 - M2 1/16 //0.4 para 1/16 en M1 y M2
				float Tmin = 0.40;   //0.4 para M1 step 1/8 - M2 1/16 //0.2 para 1/16 en M1 y M2
				float a = (Tmax + 50 * (Tmax-Tmin)/(200-50)); //0.9333333
				float b = (Tmax-Tmin)/(200-50); //0.0026667
				printf("%f %f\n", a, b);
				T = Tmin;*/
				T = 0.40;
				
				master_config();
				//printf("Register CONFIG: %d\n", Get_Reg(CONFIG)); //Vale 31 para PRX, 30 para PTX para esta configuracion
				reset_IRQ();
				state = initial_homing;
				last_state = init;
				break;
			
			case initial_homing:
				if(last_state == init){ 
					state = homing_M1;
					motor1_reach_limit_switch = 1;
				}
				else if(last_state == homing_M1) state = homing_M2;
				else state = receive_p;
				last_state = initial_homing;
				break;
				
			case homing_M1: 
				if(motor1_reach_limit_switch == 1) motor1_DIR_CCW;
				if(motor1_reach_limit_switch == 2) motor1_DIR_CW;
				
				motor1_moving = TRUE;
				if(reach_limit_switch_M1(motor1_reach_limit_switch)){ 
					motor1_moving = FALSE; 
					if(last_state == initial_homing) state = initial_homing;
					if(last_state == automatic_mode) state = automatic_mode;
					last_state = homing_M1;
				}
				break;
				
			case homing_M2: 
				if(limit_switch_m2 == 0){
					motor2_DIR_CCW;
					motor2_moving = TRUE;
				}
				else{
					limit_switch_m2 = 0;
					motor2_moving = FALSE;
					if(last_state == initial_homing) state = initial_homing;
					last_state = homing_M2;
				}
				break;
			
			case receive_p: 
				receive_payload();
				if(nrf24.state == CONNECTED && nrf24.payload[11] == 1 && limit_switch_m1 == 0){
					state = manual_mode;
				}
				if(nrf24.state == CONNECTED && nrf24.payload[11] == 0){
					state = automatic_mode;
				}
				last_state = receive_p;
				break;
			
			case manual_mode:
				//Control PWM
				T = (float)(0.9333333 - 0.0026667*nrf24.payload[4]);
				//funcion lineal para cambiar el periodo del pulso entre 50 y 200 del potenciometro
				//T es maximo para una lectura de potenciometro de 50 para abajo
				//T es minimo para una lectura de potenciometro de 200 para arriba
				if(T < 0.4) T = 0.4;
				if(T > 0.8) T = 0.8;
				configure_PWM_TC1(T, 50, 50);
				
				//Motor1
				if(nrf24.payload[0]<=87){
					motor1_moving = TRUE;
					motor1_DIR_CW;
				}
				if(nrf24.payload[0]>=167){
					motor1_moving = TRUE;
					motor1_DIR_CCW;
				}
				if(nrf24.payload[0]>87 && nrf24.payload[0]<167) motor1_moving = FALSE;
				
				//Motor2
				if(nrf24.payload[2]>=167){
					motor2_moving = TRUE;
					motor2_DIR_CCW;
				}
				if(nrf24.payload[2]<=87){
					motor2_moving = TRUE;
					motor2_DIR_CW;
				}
				if(nrf24.payload[2]>87 && nrf24.payload[2]<167) motor2_moving = FALSE;
				state = receive_p;
				last_state = manual_mode;
				break;
			
			case automatic_mode:
				if(last_state == receive_p){	
					state = homing_M1;
					motor1_reach_limit_switch = 1;
				} //cualquier trayectoria deseada
				//if(last_state == homing_M1) state = homing_M1; //hay q ir al limit switch2
				//if(last_state == homing_M1) state = homing_M2;
				//if(last_state == homing_M2) state = receive_p;
				if(last_state == homing_M1) state = receive_p;
				last_state = automatic_mode; //esta roto por ahora, se queda en un bucle. Si hizo homing 1 y despues homing 2 pero en homing 2 no paraba y ademas se perdio la conexion con el joystick
				break;
		}
		reach_limit_switch_M1(1);
		reach_limit_switch_M1(2);	
		/*if(nrf24.state == CONNECTED && nrf24.payload[11] == 1 && limit_switch_m1 == 0){ //modo manual: motores se van a mover si hay conexion con el joystick, el switch2 esta en 1 y motor1 no este en ningun fin de carrera
			//Control PWM 
			T = (float)(a - b*nrf24.payload[4]);
			//funcion lineal para cambiar el periodo del pulso entre 50 y 200 del potenciometro
			//T es maximo para una lectura de potenciometro de 50 para abajo
			//T es minimo para una lectura de potenciometro de 200 para arriba
			if(T < Tmin) T = Tmin;
			if(T > Tmax) T = Tmax;
			configure_PWM_TC1(T, 50, 50);
			
			//Motor1
			if(nrf24.payload[0]<=87){
				motor1_moving = TRUE;
				motor1_DIR_CW;
			}
			if(nrf24.payload[0]>=167){
				motor1_moving = TRUE;
				motor1_DIR_CCW;
			}
			if(nrf24.payload[0]>87 && nrf24.payload[0]<167) motor1_moving = FALSE;
			
			//Motor2
			if(nrf24.payload[2]>=167){
				motor2_moving = TRUE;
				motor2_DIR_CCW;
			}
			if(nrf24.payload[2]<=87){
				motor2_moving = TRUE;
				motor2_DIR_CW;
			}
			if(nrf24.payload[2]>87 && nrf24.payload[2]<167) motor2_moving = FALSE;
		}
		if(nrf24.state == CONNECTED && nrf24.payload[11]==0){ //Modo automático
			state_homing = 1;
			homing(MOTOR1, 1); //cualquier trayectoria deseada
			homing(MOTOR1, 2);
			homing(MOTOR2, 1);
			state_homing = 0;
		}
		
		reach_limit_switch_M1(1);
		reach_limit_switch_M1(2);	*/
	}
}

void master_config(){
	nrf24.PRX_PTX = PRX;  //PRX!!
	nrf24.addr = 0x01;    //es de 5bytes. Sera address x5, cada byte igual
	nrf24.channel = 0x6E; //110 -> 2510 MHz
	nrf24.state = DISCONNECTED;
	
	//Configuracion de los pines E/S
	PUD_OFF; 
	opto_isolator_INPUT; opto_isolator_PULL_UP;
	switchA_INPUT; switchB_INPUT; PCINT0_INPUT;
	motor1_DIR_OUTPUT; motor1_STEP_OUTPUT;
	motor2_DIR_OUTPUT; motor2_STEP_OUTPUT;
	GREEN_LED_OUTPUT; RED_LED_OUTPUT;	
	SPI_pin_setup();
	config_SPI_Master(0, 16); //modo=0 (polaridad 0, fase 0) y prescaler= 16
		
	RED_LED_ON;
		
	cli();
	//Timer 0: estado de la conexion
	configure_PWM_TC0(10); //T en ms	
	
	//Timer 1: motores
	configure_PWM_TC1(T, 50, 50);   //T en ms, dutyA%, dutyB% //para 1/4 step: 1.8min 0.8max//para full step 1.4ms min - 5.0ms max
	configura_ModoSalidas_TC1(0,0); //modo desconectado para A y B
	interrupciones_TC1(0,1,1,1);    //uso compare-A para pulsos de motor1 y compare-B para motor2
	
	//NRF24L01
	NRF24L01_config(nrf24.PRX_PTX, nrf24.addr, nrf24.channel); 
	
	//Switches
	configure_INT0();
	configure_INT1();
	configure_PCINT0();
	configure_PCINT11();
	sei();        
}

void configure_INT0(){ //fin de carrera 1
	INT0_FlancoSubida;
	INT0_FlagOFF;
	INT0_ON;
}

void configure_INT1(){ //fin de carrera 2
	INT1_FlancoSubida;
	INT1_FlagOFF;
	INT1_ON;
}

void configure_PCINT0(){ //pin IRQ del modulo, utilizado para generar una interrupcion en caso de recepcion de datos
	SETBIT(PCICR, PCIE0);
	SETBIT(PCMSK0, PCINT0);
}

void configure_PCINT11(){ //optoacoplador
	SETBIT(PCICR, PCIE1);
	SETBIT(PCMSK1, PCINT11);
}

uint8_t reach_limit_switch_M1(uint8_t f){
	if(limit_switch_m1 == f){
		_delay_ms(500);                     //Para no cambiar instantaneamente de direccion
		if(motor1_DIR_TEST) motor1_DIR_CCW; //return de DIR_TEST: CW!=0  CCW=0
		else motor1_DIR_CW;
		limit_switch_m1 = 0;
		for(int i=0; i<100 ; i++){          //Corre un poco el carro en el sentido opuesto al que venia
			motor1_STEP_HIGH;               //genero los pulsos de forma manual
			_delay_us(500);
			motor1_STEP_LOW;
			_delay_us(500);
		}
		return TRUE;
	}
	else return FALSE;	
}
/*
void homing(uint8_t motor, uint8_t f){
	if(motor == MOTOR1){
		if(f==1) motor1_DIR_CCW;
		if(f==2) motor1_DIR_CW;
		
		motor1_moving = TRUE;
		while(!reach_limit_switch_M1(f)){
			_delay_us(1);
		};
	}
	
	if(motor == MOTOR2){
		motor2_DIR_CCW;
		motor2_moving = TRUE;
		
		while(limit_switch_m2 == 0){
			_delay_us(1);
		};
		limit_switch_m2 = 0; //Vuelve al estado anterior
		motor2_moving = FALSE;
	}
}*/

//Interrupcion IRQ de recepcion de datos
ISR (PCINT0_vect){
	cli();
	CE_OFF; //Dejar de escuchar
	if(! PCINT0_TEST){                    //negado porque se activa por flanco de bajada
		if(Get_Reg(STATUS) & BIT(RX_DR)){ //pregunta si se recibieron datos
			if(nrf24.state == DISCONNECTED){  
				nrf24.state = CONNECTED; //Solo se escribe state cuando se establece conexion
				RED_LED_OFF;
				GREEN_LED_ON;
			} 
			Read_NRF(R_RX_PAYLOAD, nrf24.payload, BUFFER_SIZE);
			reset_IRQ();
			timer_connection = 0;
		}
	}
	SETBIT(PCIFR, PCIF0);
	sei();
}

ISR (PCINT1_vect){
	if(opto_isolator_TEST) limit_switch_m2 = 1;
	SETBIT(PCIFR, PCIF1);
}

//Interrupciones de switches
ISR (INT0_vect){
	_delay_ms(20); //Hay que verificar si el boton continua presionado luego de un intervalo de tiempo, al ser un pulsador mecanico tiene pequeños rebotes
	if(switchA_TEST){
		motor1_moving = FALSE;	//Detiene el motor en cada final de carrera
		limit_switch_m1 = 1;
	};
	INT0_FlagOFF;
}

ISR (INT1_vect){
	_delay_ms(20);
	if(switchB_TEST){
		motor1_moving = FALSE;	//Detiene el motor en cada final de carrera
		limit_switch_m1 = 2;
	};
	INT1_FlagOFF;
}

//Interrupcion timer 0
ISR (TIMER0_OVF_vect){
	timer_connection++;
	
	if(timer_connection == 100){ //t= 100*T_TC0
		timer_connection = 0;
		if(nrf24.state == CONNECTED){
			nrf24.state = DISCONNECTED;
			GREEN_LED_OFF;
			RED_LED_ON;
		}
	}
	SETBIT(TIFR0, TOV0); //apaga flag
}

//Interrupciones de timer1
ISR (TIMER1_COMPA_vect){ 
	if(motor1_moving && (nrf24.state == CONNECTED || state==homing_M1)) motor1_STEP_LOW;
	SETBIT(TIFR1, OCF1A); //apaga flag
}

ISR (TIMER1_COMPB_vect){
	if(motor2_moving && (nrf24.state == CONNECTED || state==homing_M2)) motor2_STEP_LOW;
	SETBIT(TIFR1, OCF1B); //apaga flag
}

ISR (TIMER1_OVF_vect){
	if(motor1_moving && (nrf24.state == CONNECTED || state==homing_M1)) motor1_STEP_HIGH;
	else motor1_STEP_LOW;
	
	if(motor2_moving && (nrf24.state == CONNECTED || state==homing_M2)) motor2_STEP_HIGH;
	else motor2_STEP_LOW;
	SETBIT(TIFR1, TOV1); //apaga flag
}