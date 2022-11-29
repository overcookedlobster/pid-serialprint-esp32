#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "PID.h"
#include "PID.c"
#include "FSM.h"
#include "FSM.c"

#define PID_KP 1.0f
#define PID_KI 2.3f
#define PID_KD 0.39f

#define PID_TAU 0.02f

#define PID_LIM_MIN -15.0f
#define PID_LIM_MAX 15.0f

#define PID_LIM_MIN_INT -5.0f
#define PID_LIM_MAX_INT 5.0f
#define SAMPLE_TIME_S 0.01f


static const int RX_BUF_SIZE = 1024;
#define TXD_PIN GPIO_NUM_17//(GPIO_NUM_4)
#define RXD_PIN GPIO_NUM_16//(GPIO_NUM_5)
float measurement;
float distance = 0;
#define GPIO_OUTPUT_IO_0 23 
#define GPIO_OUTPUT_PIN_SEL ((1ULL<<GPIO_OUTPUT_IO_0))
#define GPIO_INPUT_IO_0 18
#define GPIO_INPUT_IO_1 32
#define GPIO_INPUT_IO_2 33
#define GPIO_INPUT_PIN_SEL ((1ULL<<GPIO_INPUT_IO_0)|(1ULL<<GPIO_INPUT_IO_1)|(1ULL<<GPIO_INPUT_IO_2))
uint32_t timer = 0;
uint32_t level = 0;
uint32_t button_someone_flag = 0;
uint32_t button_someone = 0;
uint32_t someone = 0;
uint32_t button_open_flag = 0;
uint32_t button_open = 0;     
uint32_t open_b = 0;            
uint32_t button_close_flag = 0;
uint32_t button_close = 0;     
uint32_t close_b = 0;            
uint32_t state = 0;
PIDController pid = { PID_KP, PID_KI, PID_KD,
              PID_TAU,
              PID_LIM_MIN, PID_LIM_MAX,
              PID_LIM_MIN_INT, PID_LIM_MAX_INT,
              SAMPLE_TIME_S, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
float setpoint = 0.0f;
esp_timer_create_args_t create_args;
esp_timer_handle_t timer_handle;    
esp_timer_create_args_t create_args_1;
esp_timer_handle_t timer_handle_1;

esp_timer_create_args_t create_args_2;
esp_timer_handle_t timer_handle_2;

esp_timer_create_args_t create_args_3;
esp_timer_handle_t timer_handle_3;
void timer_expired(void *p);
void timer_expired_1(void *p);
void timer_expired_2(void *p);
void timer_expired_3(void *p);


static void rx_task(void *arg)
{
	char* data = (char*) malloc(RX_BUF_SIZE+1);
        char* pid_out = (char*) malloc(RX_BUF_SIZE+1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_2, data, RX_BUF_SIZE, 10 / portTICK_PERIOD_MS);
	printf("someone: %lu, open_b: %lu, close_b: %lu, state: %lu, setpoint:%f, distance:%f \n", someone, open_b, close_b, state, setpoint, distance); 
	sprintf(pid_out, "%f", pid.out);
        if (rxBytes > 0) {
                data[rxBytes] = 0;
		measurement = atof(data);
		PIDController_Update(&pid, setpoint, measurement);
                uart_write_bytes(UART_NUM_2, pid_out, strlen(pid_out));
                uart_write_bytes(UART_NUM_2, "\n", strlen("\n"));
		distance += measurement*SAMPLE_TIME_S;
	}
    }

    free(data);
	free(pid_out);
    }

void app_main(void)
{
gpio_config_t io_conf = {};
io_conf.intr_type = GPIO_INTR_DISABLE;
io_conf.mode = GPIO_MODE_OUTPUT;
io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
io_conf.pull_down_en = 0;
io_conf.pull_up_en = 0;
gpio_config(&io_conf);

io_conf.intr_type = GPIO_INTR_DISABLE;
io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
io_conf.mode = GPIO_MODE_INPUT;
io_conf.pull_up_en = 1;
io_conf.pull_down_en = 0;
gpio_config(&io_conf);

create_args_1.callback = timer_expired_1;
create_args_1.dispatch_method = ESP_TIMER_TASK;
create_args_1.name = "esp_timer_1";

esp_timer_create(&create_args_1, &timer_handle_1);

create_args_2.callback = timer_expired_2;
create_args_2.dispatch_method = ESP_TIMER_TASK;
create_args_2.name = "esp_timer_2";

esp_timer_create(&create_args_2, &timer_handle_2);


create_args_3.callback = timer_expired_3;
create_args_3.dispatch_method = ESP_TIMER_TASK;
create_args_3.name = "esp_timer_3";

esp_timer_create(&create_args_3, &timer_handle_3);

create_args.callback = timer_expired;
create_args.dispatch_method = ESP_TIMER_TASK;
create_args.name = "esp_timer";
esp_timer_create(&create_args, &timer_handle);


const uart_config_t uart_config = {                                        
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,                                         
    .parity = UART_PARITY_DISABLE,                                         
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,                                 
    .source_clk = UART_SCLK_DEFAULT,                                       
};
// We won't use a buffer for sending data.
uart_driver_install(UART_NUM_2, RX_BUF_SIZE * 2, 0, 0, NULL, 0);           
uart_param_config(UART_NUM_2, &uart_config);
uart_set_pin(UART_NUM_2, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
                                                                           
xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);

while(1){
//        button_someone
	button_someone = (gpio_get_level(GPIO_INPUT_IO_0))? 0:1;
        if(button_someone) esp_timer_start_once(timer_handle, 300000);
	else 
	{
                esp_timer_stop(timer_handle);
		gpio_set_level(GPIO_OUTPUT_IO_0,0);
		someone = 0;
        }


button_open = (gpio_get_level(GPIO_INPUT_IO_1))? 0:1;
if(button_open)
{
        if(button_open_flag == 0){
                esp_timer_start_once(timer_handle_1, 300000);
        }

}


else {
        esp_timer_stop(timer_handle_1);
        open_b = 0;
}
button_close= (gpio_get_level(GPIO_INPUT_IO_2))? 0:1;
if(button_close)
{
        if(button_close_flag == 0){
                esp_timer_start_once(timer_handle_2, 300000);
                button_close_flag = 1;
        }

}


else {
        esp_timer_stop(timer_handle_2);
        button_close_flag = 0;
        close_b = 0;
}
	
	if(state == OPENED){
                esp_timer_start_once(timer_handle_3, 10000000);
	}
	else {
		esp_timer_stop(timer_handle_3);
	}
	fsm(someone, open_b, close_b, &timer, distance, &setpoint, &state);

	vTaskDelay(10/portTICK_PERIOD_MS);
}}

void timer_expired(void *p)
{
someone = 1;
gpio_set_level(GPIO_OUTPUT_IO_0,1);
}
void timer_expired_1(void *p)
{
open_b = 1;
}

void timer_expired_2(void *p)
{
close_b = 1;
}


void timer_expired_3(void *p)
{
timer = 1;
}
