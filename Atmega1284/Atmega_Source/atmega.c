#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/portpins.h>
#include <avr/pgmspace.h>

//FreeRTOS include files
#include "FreeRTOS.h"
#include "task.h"
#include "croutine.h"
#include "usart_atmega1284.h"

//start tracker
unsigned char start_track = 0;

//controller variables
const unsigned char UP = 0x08;
const unsigned char DOWN = 0x04;
const unsigned char LEFT = 0x02;
const unsigned char RIGHT = 0x01;
const unsigned char SELECT = 0x20;
const unsigned char START = 0x10;
const unsigned char Y = 0x40;
const unsigned char B = 0x80;

unsigned char temp_data;
unsigned char controller_data;
unsigned char i;
unsigned char j;

//motor variables
unsigned char stepper1[8] = {0x01, 0x03, 0x02, 0x06, 0x04, 0x0C, 0x08, 0x09};
unsigned char count1;
unsigned char stepper2[8] = {0x01, 0x03, 0x02, 0x06, 0x04, 0x0C, 0x08, 0x09};
unsigned char count2;

//general variables
unsigned char recieve = 0;
unsigned char yaw = 0x0F;
unsigned char roll_rec;
int roll_degree = 90;
int roll_degree_rec;
double degree_count;

unsigned char data_to_send = 0x00;


double convert (double degree) {
	return ((degree / 5.625) * 64);
}


//start sm -----------------------------------------------------------------------------------------------------------
enum startState {INIT_s, check_start} start_state;

void start_Init(){
	start_state = INIT_s;
}

void start_Tick(){
	switch(start_state){
		case INIT_s:
		break;
		case check_start:
		break;
		default:
		break;
	}
	
	switch(start_state){
		case INIT_s:
		if (controller_data == 0x06) {
			start_state = check_start;
			if (start_track == 0) {
				start_track = 1;
				roll_degree = 90;
				roll_degree_rec = 90;
			}
			else {
				start_track = 0;
			}
		}
		break;
		case check_start:
		if (controller_data == 0x00) {
			start_state = INIT_s;
		}
		break;
		default:
		break;
	}
}

void startTask()
{
	start_Init();
	for(;;)
	{
		start_Tick();
		vTaskDelay(10);
	}
}

void startPulse(unsigned portBASE_TYPE Priority)
{
	xTaskCreate(startTask, (signed portCHAR *)"startTask", configMINIMAL_STACK_SIZE, NULL, Priority, NULL );
}
//----------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------


//recieve sm -----------------------------------------------------------------------------------------------------------
enum recState {INIT_r, recieve_data} rec_state;

void rec_Init(){
	rec_state = INIT_r;
}

void rec_Tick(){
	switch(rec_state) {
		case INIT_r:
		break;
		case recieve_data:
		if (USART_HasReceived(0)){
			recieve = USART_Receive(0);
			USART_Flush(0);
			if (recieve == 0xF1) {
				yaw = 0x01;
				//PORTA = yaw;
			}
			else if (recieve == 0xF2) {
				yaw = 0x02;
				//PORTA = yaw;`
			}
			else if (recieve == 0xF3) {
				yaw = 0x0F;
				//PORTA = yaw;
			}
			else if (recieve == 0xFF) {
				while (!USART_HasReceived(0));
				recieve = USART_Receive(0);
				roll_degree_rec = recieve;
				roll_degree_rec -= 90;
			}
		}
		break;
		default:
		break;
	}
	switch(rec_state) {
		case INIT_r:
		rec_state = recieve_data;
		break;
		case recieve_data:
		break;
		default:
		rec_state = INIT_r;
		break;
	}
}

void recTask()
{
	rec_Init();
	for(;;)
	{
		rec_Tick();
		vTaskDelay(0);
	}
}

void recPulse(unsigned portBASE_TYPE Priority)
{
	xTaskCreate(recTask, (signed portCHAR *)"recTask", configMINIMAL_STACK_SIZE, NULL, Priority, NULL );
}
//----------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------


//send sm -----------------------------------------------------------------------------------------------------------
enum sendState {INIT_send, send_data} send_state;

void send_Init(){
	send_state = INIT_send;
}

void send_Tick(){
	switch(send_state) {
		case INIT_send:
		break;
		case send_data:
		if(controller_data != data_to_send) {
			if(USART_IsSendReady(0)) {
				USART_Send(controller_data, 0);
				while(!USART_HasTransmitted(0));
				USART_Flush(0);
				data_to_send = controller_data;
			}
		}
		break;
		default:
		break;
	}
	switch(send_state) {
		case INIT_send:
		send_state = send_data;
		break;
		case send_data:
		break;
		default:
		send_state = INIT_send;
		break;
	}
}

void sendTask()
{
	send_Init();
	for(;;)
	{
		send_Tick();
		vTaskDelay(100);
	}
}

void sendPulse(unsigned portBASE_TYPE Priority)
{
	xTaskCreate(sendTask, (signed portCHAR *)"sendTask", configMINIMAL_STACK_SIZE, NULL, Priority, NULL );
}
//----------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------




//motor Left/Right sm---------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------
enum motorState {INIT_m, clock, counter, done_1} motor_state;

void motor_Init(){
	motor_state = INIT_m;
}

void motor_Tick(){
	//Actions
	switch(motor_state){
		case INIT_m:
		break;
		case counter:
		PORTB = stepper1[count1];
		if (count1 == 7) {
			count1 = 0;
		}
		else { count1++; }
		break;
		case clock:
		PORTB = stepper1[count1];
		if (count1 == 0) {
			count1 = 7;
		}
		else { count1 -= 1; }
		break;
		default:
		break;
	}
	//Transitions
	switch(motor_state){
		case INIT_m:
		if (start_track) {
			if (controller_data == 0x04) {
				//left
				count1 = 0;
				motor_state = clock;
			}
			if (controller_data == 0x03) {
				//right
				count1 = 7;
				motor_state = counter;
			}
		}
		else {
			if (yaw == 0x01) {
				count1 = 7;
				motor_state = clock;
			}
			if (yaw == 0x02) {
				count1 = 0;
				motor_state = counter;
			}
		}
		break;
		case clock:
		if (start_track) {
			if (controller_data == 0x00) {
				motor_state = INIT_m;
			}
		}
		else {
			if (yaw == 0x0F) {
				motor_state = INIT_m;
				yaw = 0x00;
			}
		}
		break;
		case counter:
		if (start_track) {
			if (controller_data == 0x00) {
				motor_state = INIT_m;
			}
		}
		else {
			if (yaw == 0x0F) {
				motor_state = INIT_m;
				yaw = 0x00;
			}
		}
		break;
		default:
		break;
	}
}

void motorTask()
{
	motor_Init();
	for(;;)
	{
		motor_Tick();
		vTaskDelay(3);
	}
}

void motorPulse(unsigned portBASE_TYPE Priority)
{
	xTaskCreate(motorTask, (signed portCHAR *)"motorTask", configMINIMAL_STACK_SIZE, NULL, Priority, NULL );
}
//----------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------


//motor Up/Down sm------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------
enum motorState2 {INIT_m2, clock2, counter2, clock3, counter3, done} motor_state2;

void motor_Init2(){
	motor_state2 = INIT_m2;
}

void motor_Tick2(){
	//Actions
	switch(motor_state2){
		case INIT_m2:
			break;
		case clock2:
			PORTC = stepper2[count2];
			if (count2 == 7) {
				count2 = 0;
			}
			else { count2++; }
			break;
		case counter2:
			PORTC = stepper2[count2];
			if (count2 == 0) {
				count2 = 7;
			}
			else { count2 -= 1; }
			break;
		case clock3:
			PORTC = stepper2[count2];
			if (count2 == 7) {
				count2 = 0;
			}
			else { count2++; }
			break;
		case counter3:
			PORTC = stepper2[count2];
			if (count2 == 0) {
				count2 = 7;
			}
			else { count2 -= 1; }
			break;
		case done:
			break;
		default:
			break;
	}
	//Transitions
	switch(motor_state2){
		case INIT_m2:
			if (start_track) {
				if (controller_data == 0x01) {
					count2 = 0;
					motor_state2 = clock2;
				}
				if (controller_data == 0x02) {
					count2 = 7;
					motor_state2 = counter2;
				}
			}
			else {
				if (roll_degree != roll_degree_rec) {
					degree_count = roll_degree - roll_degree_rec;
					if (degree_count > 0) {
						//go down 
						count2 = 7;
						motor_state2 = counter3;
					}
					else {
						//go up
						count2 = 0;
						motor_state2 = clock3;
					}
					//keep degree count positive
					if (degree_count < 0) {
						degree_count *= -1;
					}
					degree_count = degree_count * 2.675;
					degree_count = round(degree_count);
					degree_count = convert(degree_count);
					//PORTA = degree_count;
				}
			}
			break;
		case clock2:
			if (controller_data == 0x00) {
				motor_state2 = INIT_m2;
			}
			break;
		case counter2:
			if (controller_data == 0x00) {
				motor_state2 = INIT_m2;
			}
			break;
		case clock3:
			if (degree_count > 0) {
				degree_count -= 1;
			}
			else {
				motor_state2 = done;
			}
			break;
		case counter3:
			if (degree_count > 0) {
				degree_count -= 1;
			}
			else {
				motor_state2 = done;
			}
			break;
		case done:
			roll_degree = roll_degree_rec;
			motor_state2 = INIT_m2;
			break;
		default:
			break;
	}
}

void motorTask2()
{
	motor_Init2();
	for(;;)
	{
		motor_Tick2();
		vTaskDelay(3);
	}
}

void motorPulse2(unsigned portBASE_TYPE Priority)
{
	xTaskCreate(motorTask2, (signed portCHAR *)"motorTask2", configMINIMAL_STACK_SIZE, NULL, Priority, NULL );
}

//----------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------


//controller sm---------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------
enum controllerState {start, init, wait1, read1, wait2, wait3, final1} controller_state;

void nes_Init(){
	controller_state = start;
}

void nes_Tick(){
	switch(controller_state)
	{
		case start:
		controller_state = init;
		break;
		
		case init:
		PORTA = 0x04; // latch = 1
		controller_state = wait1;
		break;
		
		case wait1:
		if (i < 2) {  i++; }
		else if (i >= 2) { PORTA = 0x00; i = 0; controller_state = read1;} //latch = 0
		break;
		
		case read1:
		PORTA = 0x02; j++; controller_state = wait2; //clock = 1
		break;
		
		case wait2:
		if (i < 2) { i++; } // wait for 2 ms
		else if (i >= 2)
		{
			controller_state = wait3;
			temp_data = temp_data << 1;
			temp_data = temp_data + ((PINA & 0x08) >> 3);
			i = 0;
		}
		break;
		
		case wait3:
		if (i < 4) { i++; }
		else if (i >= 4 && j >= 7) { controller_state = final1; }
		else if (i >= 4 && j < 7) { PORTA = 0x00; PORTA = 0x02; i = 0; controller_state = wait2; j++; } //clock = 0 and then clock = 1
		break;
		
		case final1:
		controller_state = init;
		break;
		
		default:
		break;
		
	}
	
	switch(controller_state)
	{
		case start:
		break;
		
		case init:
		i = 0;
		j = 0;
		PORTA = 0x00;
		temp_data = 0;
		break;
		
		case wait1:
		break;
		
		case read1:
		temp_data = (PINA & 0x08) >> 3; //temp_data = data in
		break;
		
		case wait2:
		break;
		
		case wait3:
		break;
		
		case final1:
		//if nothing gets pressed, controller data shows nothing is pressed
		controller_data = 0x00;
		
		//NOT the data
		temp_data = ~temp_data;
		//first change controller data1
		if (temp_data == UP) { controller_data = 0x01; }
		else if (temp_data == DOWN) { controller_data = 0x02;}
		else if (temp_data == LEFT) { controller_data = 0x03;}
		else if (temp_data == RIGHT) { controller_data = 0x04;}
		else if (temp_data == SELECT) { controller_data = 0x05; }
		else if (temp_data == START) { controller_data = 0x06; }
		else if (temp_data == Y) { controller_data = 0x07; }
		else if (temp_data == B) { controller_data = 0x08; }
		break;
		
		default:
		break;
	}
}

void nesTask()
{
	nes_Init();
	for(;;)
	{
		nes_Tick();
		vTaskDelay(1);
	}
}

void nesPulse(unsigned portBASE_TYPE Priority)
{
	xTaskCreate(nesTask, (signed portCHAR *)"nesTask", configMINIMAL_STACK_SIZE, NULL, Priority, NULL );
}
//----------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------



int main(void)
{
	DDRD = 0xFF; PORTD = 0x00; // PORTD set to input for USART
	DDRC = 0xFF; PORTC = 0x00; // PORTC set to output for up/down
	DDRB = 0xFF; PORTB = 0x00; // PORTB set to output for left right
	//DDRA = 0xFF; PORTA = 0x00;
	DDRA = 0x06; PORTA = 0xF9; //button input
	
	roll_degree_rec = roll_degree;
	
	initUSART(0);
	//start tracker
	startPulse(1);
	//receives from USART
	recPulse(1);
	//sends usart
	sendPulse(1);
	//Left/Right Controller
	motorPulse(1);
	//Up/Down Controller
	motorPulse2(1);
	//controller
	nesPulse(1);
	//RunSchedular;
	vTaskStartScheduler();
	return 0;
}