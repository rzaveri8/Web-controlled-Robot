#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "car-sensor.h"
#include <esp_wifi.h>
#include <esp_event_loop.h>
#include <esp_system.h>
#include <sys/param.h>
#include <http_server.h>
#include <nvs_flash.h>
#include "driver/gpio.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt.h"
#include "soc/rmt_reg.h"
#include "driver/uart.h"
#include "driver/periph_ctrl.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

#define GPIO_PWM0A_OUT 32   //Set GPIO 32 as PWM0A
#define GPIO_PWM0B_OUT 14   //Set GPIO 14 as PWM0B
#define GPIO_MOTOR1_0  13
#define GPIO_MOTOR1_1  12
#define GPIO_MOTOR2_0  27
#define GPIO_MOTOR2_1  33
#define LED_GPIO 4 //led gpio at A5

//UART
#define UART_TX_GPIO_NUM 26 // A0
#define UART_RX_GPIO_NUM 34 // A2
#define BUF_SIZE (1024)
char final[40]= "";


#define IR_LOWER 20
#define IR_UPPER 30
#define TURN_RANGE 50

#define WIFI_SSID "Group_16"
#define WIFI_PASS "smart-systems"
#define WEB_SERVER "192.168.1.127"
#define LED 21

static EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;

/*H BRIDGE & MOTOR FUNCTIONS*/
static void mcpwm_example_gpio_initialize()
{
 //only using one of the MCPWM units
   printf("initializing mcpwm gpio...\n");
   mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
   mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);
}

/**
* @brief motor moves in forward direction, with duty cycle = duty %
*/
static void brushed_motor_forward(float duty_cycle)
{

   gpio_set_level(GPIO_MOTOR1_0, 1);
   gpio_set_level(GPIO_MOTOR1_1, 0);
   gpio_set_level(GPIO_MOTOR2_0, 0);
   gpio_set_level(GPIO_MOTOR2_1, 1);



  // mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
  // mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state

   mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_cycle);
   mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, duty_cycle);
   printf("testing forward.\n");

}

/**
* @brief motor moves in backward direction, with duty cycle = duty %
*/
static void brushed_motor_backward(float duty_cycle)
{
   mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
   mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
   mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_cycle);
   mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, duty_cycle);

   gpio_set_level(GPIO_MOTOR1_0, 0);
   gpio_set_level(GPIO_MOTOR1_1, 1);
   gpio_set_level(GPIO_MOTOR2_0, 1);
   gpio_set_level(GPIO_MOTOR2_1, 0);
}

/**
* @brief motor stop
*/
static void brushed_motor_stop()
{
  // mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
  // mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state

   mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0.0);
   mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0.0);

}

static void brushed_motor_turn_left(float duty_cycle)
{
 // mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
 // mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state

  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_cycle/2);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, duty_cycle);

  gpio_set_level(GPIO_MOTOR2_0, 1);
  gpio_set_level(GPIO_MOTOR2_1, 0);
}


 static void brushed_motor_turn_right(float duty_cycle)
 {
  // mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
  // mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state

   mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, duty_cycle/2);
   mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_cycle);

   gpio_set_level(GPIO_MOTOR2_0, 1);
   gpio_set_level(GPIO_MOTOR2_1, 0);
 }

 esp_err_t forward_handler(httpd_req_t *req) {

   // GO FORWARD
   brushed_motor_forward(100.0);
   const char* response = "FORWARD";
   httpd_resp_send(req, response, strlen(response));
   return ESP_OK;
 }

 esp_err_t reverse_handler(httpd_req_t *req) {

   // GO BACKWARD
 brushed_motor_backward(100.0);
   const char* response = "REVERSE";
   httpd_resp_send(req, response, strlen(response));
   return ESP_OK;
 }

 esp_err_t left_handler(httpd_req_t *req) {

   // GO LEFT
 brushed_motor_turn_right(100.0);
   const char* response = "LEFT";
   httpd_resp_send(req, response, strlen(response));
   return ESP_OK;
 }

 esp_err_t right_handler(httpd_req_t *req) {

   // GO RIGHT
 brushed_motor_turn_left(100.0);
   const char* response = "RIGHT";
   httpd_resp_send(req, response, strlen(response));
   return ESP_OK;
 }


 esp_err_t stop_handler(httpd_req_t *req) {

   // GO RIGHT
 brushed_motor_stop();
   const char* response = "STOP";
   httpd_resp_send(req, response, strlen(response));
   return ESP_OK;
 }
 httpd_uri_t forward = {
   .uri = "/forward",
   .method = HTTP_GET,
   .handler = forward_handler,
   .user_ctx = NULL
 };

 httpd_uri_t stop = {
   .uri = "/stop",
   .method = HTTP_GET,
   .handler = stop_handler,
   .user_ctx = NULL
 };

 httpd_uri_t reverse = {
   .uri = "/reverse",
   .method = HTTP_GET,
   .handler = reverse_handler,
   .user_ctx = NULL
 };

 httpd_uri_t left = {
   .uri = "/left",
   .method = HTTP_GET,
   .handler = left_handler,
   .user_ctx = NULL
 };

 httpd_uri_t right = {
   .uri = "/right",
   .method = HTTP_GET,
   .handler = right_handler,
   .user_ctx = NULL
 };

 httpd_handle_t start_webserver(void) {
   httpd_handle_t server = NULL;
   httpd_config_t config = HTTPD_DEFAULT_CONFIG();
   // Start the httpd server
   if (httpd_start(&server, &config) == ESP_OK) {
     // Set URI handlers
     httpd_register_uri_handler(server, &forward);
     httpd_register_uri_handler(server, &reverse);
     httpd_register_uri_handler(server, &left);
     httpd_register_uri_handler(server, &right);
     httpd_register_uri_handler(server, &stop);
     return server;
   }
   return NULL;
 }

 void stop_webserver(httpd_handle_t server) {
   // Stop the httpd server
   httpd_stop(server);
 }

 static esp_err_t event_handler(void *ctx, system_event_t *event) {
   httpd_handle_t *server = (httpd_handle_t *) ctx;
   switch(event->event_id) {
   case SYSTEM_EVENT_STA_START:
     ESP_ERROR_CHECK(esp_wifi_connect());
     break;
   case SYSTEM_EVENT_STA_GOT_IP:
     /* Start the web server */
     if (*server == NULL) {
         *server = start_webserver();
     }
     break;
   case SYSTEM_EVENT_STA_DISCONNECTED:
     ESP_ERROR_CHECK(esp_wifi_connect());

     /* Stop the web server */
     if (*server) {
         stop_webserver(*server);
         *server = NULL;
     }
     break;
   default:
     break;
   }
   return ESP_OK;
 }

 static void init_wifi(void *arg) {
   tcpip_adapter_init();
   ESP_ERROR_CHECK(esp_event_loop_init(event_handler, arg));
   wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
   ESP_ERROR_CHECK(esp_wifi_init(&cfg));
   ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
   wifi_config_t wifi_config = {
     .sta = {
       .ssid = WIFI_SSID,
       .password = WIFI_PASS,
     },
   };
   printf("Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
   ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
   ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
   ESP_ERROR_CHECK(esp_wifi_start());
 }


 static void mcpwm_example_brushed_motor_control(void *arg)
 {
     //1. mcpwm gpio initialization
     mcpwm_example_gpio_initialize();

     gpio_pad_select_gpio(GPIO_MOTOR1_0);
     gpio_pad_select_gpio(GPIO_MOTOR1_1);
     gpio_pad_select_gpio(GPIO_MOTOR2_0);
     gpio_pad_select_gpio(GPIO_MOTOR2_1);
     gpio_pad_select_gpio(LED_GPIO);

     gpio_set_direction(GPIO_MOTOR1_0, GPIO_MODE_OUTPUT);
     gpio_set_direction(GPIO_MOTOR1_1, GPIO_MODE_OUTPUT);
     gpio_set_direction(GPIO_MOTOR2_0, GPIO_MODE_OUTPUT);
     gpio_set_direction(GPIO_MOTOR2_1, GPIO_MODE_OUTPUT);
     gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

     gpio_set_level(GPIO_MOTOR1_0, 0);
     gpio_set_level(GPIO_MOTOR1_1, 1);
     gpio_set_level(GPIO_MOTOR2_0, 1);
     gpio_set_level(GPIO_MOTOR2_1, 0);
     gpio_set_level(LED_GPIO, 0);

     //2. initial mcpwm configuration
     printf("Configuring Initial Parameters of mcpwm...\n");
     mcpwm_config_t pwm_config;

 	//only need to initialize one unit
     pwm_config.frequency = 100;    //frequency = 500Hz,
     pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    	pwm_config.cmpr_b = 0;	//duty cycle of PWMxB = 0
     pwm_config.counter_mode = MCPWM_UP_COUNTER;
     pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
     mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM with above settings

     static httpd_handle_t server = NULL;
     ESP_ERROR_CHECK(nvs_flash_init());
     init_wifi(&server);

     while (1) {
   		mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //sets duty type of PWM0A
   		mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0); //sets duty type of PWM0B
      vTaskDelay(100 / portTICK_PERIOD_MS);
     }
 }

/* *********************************************************************************
******************************************************************************************
***************************************************************************************************
************************************************************************************************************
****************************************************************/


// Configure UART
static void uart_init() {
  uart_config_t uart_config = {
    .baud_rate = 1200, // Slow BAUD rate
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
  };
  uart_param_config(UART_NUM_1, &uart_config);
  uart_set_pin(UART_NUM_1, UART_TX_GPIO_NUM, UART_RX_GPIO_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_set_line_inverse(UART_NUM_1,UART_INVERSE_RXD);
  uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);
}


char url_list[4][10];
int flag1 = 0;
int flag2 = 0;
int flag3 = 0;
int flag4 = 0;

// Receives task -- looks for Start byte then stores received values
int ir_rx() {
  // Buffer for input data
  uint8_t *url_part_in = (uint8_t *) malloc(BUF_SIZE);
  int len_in = uart_read_bytes(UART_NUM_1, url_part_in, BUF_SIZE, 20 / portTICK_RATE_MS);
  if (len_in > 0) {
    for (int i=0; i < 24; i++) {
      /* !!!!!!!!!!!!!!!!!!!!!!!!*/
      if (url_part_in[i] == 0x1B && i + 10 < len_in) { //NEED TO CHANGE SHIT HERE THAT HAS to DO WITH RECIEVING THE OTHER CODE SHIT
        int list_num = (int)url_part_in[i+1] - 1;
        if (list_num + 1 == 1) {
          flag1 = 1;
        } else if (list_num + 1 == 2) {
          flag2 = 1;
        } else if (list_num + 1 == 3) {
          flag3 = 1;
        } else if (list_num + 1 == 4) {
          flag4 = 1;
        }
        for (int j = 0; j < 11; j++) {
          url_list[list_num][j] = url_part_in[i+j+2];
        }
        if(flag1 ==1 && flag2 ==1  && flag3 ==1 &&  flag4 ==1) {

          for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 10; j++) {
              final[i*10 + j] = url_list[i][j];
            }
          }
          flag1 = 0;
          flag2 = 0;
          flag3 = 0;
          flag4 = 0;
          free(url_part_in);
          return 1;
        }

      }
    }

  }
  free(url_part_in);
  return 0;
}
char REQUEST[40];
void tcp_client() {
  for (int i = 0; i < 40; i++) {

    REQUEST[i] = final[i];
    putchar(REQUEST[i]);
  }
  printf("tcp_client task started \n");
  struct sockaddr_in tcpServerAddr;
  tcpServerAddr.sin_addr.s_addr = inet_addr(WEB_SERVER);
  tcpServerAddr.sin_family = AF_INET;
  // socket number
  tcpServerAddr.sin_port = htons( 3010 );
  int s, r;
  char recv_buf[64];
  //xEventGroupWaitBits(wifi_event_group,CONNECTED_BIT,false,true,portMAX_DELAY);
  s = socket(AF_INET, SOCK_STREAM, 0);
  if(s < 0) {
    printf( "... Failed to allocate socket.\n");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    return;
  }
  printf("... allocated socket\n");
    if(connect(s, (struct sockaddr *)&tcpServerAddr, sizeof(tcpServerAddr)) != 0) {
      printf( "... socket connect failed errno=%d \n", errno);
      close(s);
      vTaskDelay(4000 / portTICK_PERIOD_MS);
      return;
  }
  printf("... connected \n");
  if( write(s , REQUEST, 40) < 0)
  {
    printf( "... Send failed \n");
    close(s);
    vTaskDelay(4000 / portTICK_PERIOD_MS);
    return;
  }
  printf("... socket send success\n");
  do {
    bzero(recv_buf, sizeof(recv_buf));
    r = read(s, recv_buf, sizeof(recv_buf)-1);
    printf("RESPONSE:");
    for(int i = 0; i < r; i++) {
      putchar(recv_buf[i]);
    }
  } while(r > 0);
  /*  if (recv_buf[0] == '1') {
      gpio_set_level(BLUEPIN, 1);
      printf("URL sent\n");
      vTaskDelay(5000 / portTICK_PERIOD_MS);
      gpio_set_level(BLUEPIN, 0);
      printf("LOCKED\n");
    } else if (recv_buf[0] == '0') {
      printf("ACCESS DENIED\n");
    }
  }*/
  printf("... done reading from socket. Last read return=%d errno=%d\r\n", r, errno);
  close(s);
  printf("... new request in 5 seconds");
  vTaskDelay(5000 / portTICK_PERIOD_MS);
  printf("...tcp_client task closed\n");
}

 static void receiver_task(void *arg) {
   uart_init();
   while(1) { 
     if(ir_rx()) {
       tcp_client();
     };
     vTaskDelay(100 / portTICK_PERIOD_MS);

     }
 }



void app_main() {

  printf("Testing brushed motor...\n");
  xTaskCreate(receiver_task, "receiver_task", 4096, NULL, 5, NULL);
  xTaskCreate(mcpwm_example_brushed_motor_control, "mcpwm_example_brushed_motor_control", 4096, NULL, 5, NULL);

  printf("URL: %s.\n", final);


}
