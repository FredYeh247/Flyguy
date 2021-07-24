/* Copyright 2019 The TensorFlow Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

#include "main_functions.h"
#include "hx_drv_tflm.h"
#include "synopsys_wei_gpio.h"
#include "synopsys_wei_delay.h"
// This is the default main used on systems that have the standard C entry
// point. Other devices (for example FreeRTOS or ESP32) that have different
// requirements for entry code (like an app_main function) should specialize
// this main.cc file in a target-specific subfolder.
hx_drv_gpio_config_t hal_gpio_0;
hx_drv_gpio_config_t hal_gpio_1;
hx_drv_gpio_config_t hal_gpio_2;
hx_drv_gpio_config_t hal_led_r;
hx_drv_gpio_config_t hal_led_g;
void GPIO_INIT(void)
{
  if(hal_gpio_init(&hal_gpio_0, HX_DRV_PGPIO_0, HX_DRV_GPIO_OUTPUT, GPIO_PIN_RESET) == HAL_OK)
    hx_drv_uart_print("GPIO0 Initialized: OK\n");
  else
    hx_drv_uart_print("GPIO0 Initialized: Error\n");

  if(hal_gpio_init(&hal_gpio_1, HX_DRV_PGPIO_1, HX_DRV_GPIO_OUTPUT, GPIO_PIN_RESET) == HAL_OK)
    hx_drv_uart_print("GPIO1 Initialized: OK\n");
  else
    hx_drv_uart_print("GPIO1 Initialized: Error\n");

  if(hal_gpio_init(&hal_gpio_2, HX_DRV_PGPIO_2, HX_DRV_GPIO_OUTPUT, GPIO_PIN_RESET) == HAL_OK)
    hx_drv_uart_print("GPIO2 Initialized: OK\n");
  else
    hx_drv_uart_print("GPIO2 Initialized: Error\n");

}

int main(int argc, char* argv[]) {
  hx_drv_uart_initial(UART_BR_115200);
  GPIO_INIT();
  setup();
  int result =0;
  int send_start = 0;
  int cnt = 0;
  int poscnt = 0;
  //hx_drv_uart_print("AAAA\n");
  while (true) {

    hal_gpio_set(&hal_gpio_1, GPIO_PIN_RESET); //CLK
    
    hal_delay_ms(10);

    if(send_start == 0){
      hal_gpio_set(&hal_gpio_2, GPIO_PIN_RESET); //ENABLE
      result = loop();
      send_start=1;
     // hx_drv_uart_print("inference\n");
    }

    //up 001
    if(result==0){
      if(poscnt==0){
        hal_gpio_set(&hal_gpio_0, GPIO_PIN_RESET); //DATA
        poscnt++;
      }
      else if(poscnt==1){
        hal_gpio_set(&hal_gpio_0, GPIO_PIN_RESET);
        poscnt++;
      }
      else{
        hal_gpio_set(&hal_gpio_0, GPIO_PIN_SET);
        poscnt++;
      }
    }
    //down 010 
    else if(result==1){
      if(poscnt==0){
        hal_gpio_set(&hal_gpio_0, GPIO_PIN_RESET);
        poscnt++;
      }
      else if(poscnt==1){
        hal_gpio_set(&hal_gpio_0, GPIO_PIN_SET);
        poscnt++;
      }
      else{
        hal_gpio_set(&hal_gpio_0, GPIO_PIN_RESET);
        poscnt++;
      }
    }
    //left 011
    else if(result==2){
      if(poscnt==0){
        hal_gpio_set(&hal_gpio_0, GPIO_PIN_RESET);
        poscnt++;
      }
      else if(poscnt==1){
        hal_gpio_set(&hal_gpio_0, GPIO_PIN_SET);
        poscnt++;
      }
      else{
        hal_gpio_set(&hal_gpio_0, GPIO_PIN_SET);
        poscnt++;
      }
    }
    //right 100
    else if(result==3){
      if(poscnt==0){
        hal_gpio_set(&hal_gpio_0, GPIO_PIN_SET);
        poscnt++;
      }
      else if(poscnt==1){
        hal_gpio_set(&hal_gpio_0, GPIO_PIN_RESET);
        poscnt++;
      }
      else{
        hal_gpio_set(&hal_gpio_0, GPIO_PIN_RESET);
        poscnt++;
      }
    }
    //nothing 101
    else{
      if(poscnt==0){
        hal_gpio_set(&hal_gpio_0, GPIO_PIN_SET);
        poscnt++;
      }
      else if(poscnt==1){
        hal_gpio_set(&hal_gpio_0, GPIO_PIN_RESET);
        poscnt++;
      }
      else{
        hal_gpio_set(&hal_gpio_0, GPIO_PIN_SET);
        poscnt++;
      }
    }
    //hx_drv_uart_print("poscnt\n");
    if(poscnt==3){
      
      poscnt=0;
      send_start = 0;
    //hx_drv_uart_print("end\n");
    }
    //enable
    if(send_start==1){
      hal_gpio_set(&hal_gpio_2, GPIO_PIN_SET);
    }
    
    //CLK
    hal_gpio_set(&hal_gpio_1, GPIO_PIN_SET);
    hal_delay_ms(10);
  }

}



