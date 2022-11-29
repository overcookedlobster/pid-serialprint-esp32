#include <stdio.h>
#include <stdlib.h>
#include "FSM.h"
#define CLOSED 0
#define OPENING 1
#define OPENING_STOPPING 2
#define CLOSING 3
#define CLOSING_STOPPING 4
#define OPENED 5 

void fsm(uint32_t someone,  uint32_t open_b, uint32_t close_b, uint32_t *timer, float distance, float* setpoint, uint32_t* state){
	switch(*state){
		case CLOSED:
			if(open_b) *state = OPENING;
			*setpoint = 0;
			break;
		case OPENING:
			if(distance > 4) {
				*state = OPENING_STOPPING;
				}

				*setpoint = 1;
			break;
		case OPENING_STOPPING:
			if(distance > 5) {
				*state = OPENED;
					}
				*setpoint = 0.1;
			break;
		case OPENED:
			if((*timer && !open_b && !someone)||(!open_b&&!someone&&close_b)) *state = CLOSING;
			*setpoint = 0;
			break;
		case CLOSING:
			if((!someone || !open_b)&&(distance <1)) *state = CLOSING_STOPPING;	
			else if (someone || open_b) *state = OPENING;
			*timer = 0;
			*setpoint = -1;
			break;
		case CLOSING_STOPPING:
			if(distance <0){
				*state = CLOSED;
				*setpoint = -0.1;
			}
			break;
	}
}
