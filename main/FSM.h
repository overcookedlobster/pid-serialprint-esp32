#ifndef FSM_H
#define FSM_H

#define CLOSED 0          
#define OPENING 1         
#define OPENING_STOPPING 2
#define CLOSING 3         
#define CLOSING_STOPPING 4
#define OPENED 5          

void fsm(uint32_t someone,  uint32_t open_b, uint32_t close_b, uint32_t *timer, float distance, float* setpoint, uint32_t* state); 

#endif
