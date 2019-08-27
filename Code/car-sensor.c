#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "soc/uart_struct.h"
#include "string.h"
#include "car-sensor.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <math.h>

#define RX 16
#define SAMPLE_NUM 64
#define DEFAULT_VREF 1100

static esp_adc_cal_characteristics_t  *adc_chars;
static const adc1_channel_t channel = ADC1_CHANNEL_3; //GPIO 39
static const adc_atten_t  atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;
static const int RX_BUF_SIZE = 256;


/* LIDAR */
// lidar initialization
void init_lidar() {
  const uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
  };
  uart_param_config(UART_NUM_1, &uart_config);
  uart_set_pin(UART_NUM_1, UART_PIN_NO_CHANGE, RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
}

// get distance from lidar
uint32_t getDistance_lidar() {
  uint32_t distance = 0;
  int argc = 0;
  uint32_t temp_dist = 0;
  uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
  int length = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
  uart_flush(UART_NUM_1);
  for (int i = 0; i < length; i++) {
    temp_dist = 0;
    if (data[i] == 0x59 && data[i+1] == 0x59) {
      temp_dist = data[i+3];
      temp_dist <<= 8;
      temp_dist += data[i+2];
      distance += temp_dist;
      argc++;
    }
  }
  if (argc > 0) {
    distance /= argc;
  } else {
    distance = 0;
  }
  return distance;
}


/* IR-RANGEFINDER */

static void check_efuse() {
  //Check TP is burned into eFuse
  if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
    printf("eFuse Two Point: Supported\n");
  } else {
    printf("eFuse Two Point: NOT supported\n");
  }
  //Check Vref is burned into eFuse
  if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
    printf("eFuse Vref: Supported\n");
  } else {
    printf("eFuse Vref: NOT supported\n");
  }
}

static void print_char_val_type(esp_adc_cal_value_t val_type) {
  if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
    printf("Characterized using Two Point Value\n");
  } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
    printf("Characterized using eFuse Vref\n");
  } else {
    printf("Characterized using Default Vref\n");
  }
}

// ir-rangefinder initialization
void init_ir() {
  check_efuse();
  //configure ADC
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(channel, atten);
  adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
  print_char_val_type(val_type);
}

// get distance from ir-rangefinder
uint32_t getDistance_ir() {
  uint32_t dist;
  //Initialize reading variable
  uint32_t val = 0;
  for (int i = 0; i < SAMPLE_NUM; i++){
    val += adc1_get_raw(channel);
  }
  val /= SAMPLE_NUM;
  uint32_t volts = esp_adc_cal_raw_to_voltage(val, adc_chars);
  dist = 146060 * (pow(volts, -1.126));
  //printf("Distance(cm): %d\n", dist);
  return dist;
}


/*
void app_main() {
  init_ir();
  init_lidar();
  while (1) {
    uint32_t dist_lidar = getDistance_lidar();
    uint32_t dist_ir = getDistance_ir();
    printf("Distance (lidar): %dcm\n", dist_lidar);
    printf("Distance (ir): %dcm\n", dist_ir);
  }
}*/
