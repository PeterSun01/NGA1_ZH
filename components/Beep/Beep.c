#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "Beep.h"

static void Beep_Task(void* arg)
{
    while(1)
    {
        switch(Beep_status)
        {
            case BEEP_OFF:   //关闭
                Beep_Off();
                vTaskDelay(10 / portTICK_RATE_MS);
                break;
            
            case BEEP_ALM:   //报警
                Beep_On();
                vTaskDelay(500 / portTICK_RATE_MS);
                Beep_Off();
                vTaskDelay(500 / portTICK_RATE_MS);
                break;   

            case BEEP_ERR:   //传感器故障
                Beep_On();
                vTaskDelay(900 / portTICK_RATE_MS);
                Beep_Off();
                vTaskDelay(100 / portTICK_RATE_MS);
                break;        
        }
    }
}


void Beep_Init(void)
{
    gpio_config_t io_conf;

    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO16
    io_conf.pin_bit_mask = (1<<GPIO_BEEP);
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    gpio_config(&io_conf);  

    Beep_On();
    vTaskDelay(200 / portTICK_RATE_MS);
    Beep_Off();
    vTaskDelay(200 / portTICK_RATE_MS);

    Beep_On();
    vTaskDelay(200 / portTICK_RATE_MS);   
    Beep_Off();
    Beep_status=BEEP_OFF;
    xTaskCreate(Beep_Task, "Beep_Task", 4096, NULL, 5, NULL);

}


void Beep_On(void)
{
    gpio_set_level(GPIO_BEEP, 1);
}


void Beep_Off(void)
{
    gpio_set_level(GPIO_BEEP, 0);
    
}


