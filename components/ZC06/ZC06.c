#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "Led.h"
#include "ZC06.h"
#include "Beep.h"


#define UART1_TXD  (UART_PIN_NO_CHANGE)
#define UART1_RXD  (GPIO_NUM_14)
#define UART1_RTS  (UART_PIN_NO_CHANGE)
#define UART1_CTS  (UART_PIN_NO_CHANGE)

#define BUF_SIZE    100
#define CH4_ALM_GATE    4000


//static const char *TAG = "ZC06";

void ZC06_Read_Task(void* arg);

void ZC06_Init(void)
{
    CH4_Alm_status=0;
    CH4=0;
    //配置GPIO
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask =  1 << UART1_RXD;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    
    
    uart_config_t uart_config = 
    {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, UART1_TXD, UART1_RXD, UART1_RTS, UART1_CTS);
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);
    xTaskCreate(&ZC06_Read_Task, "ZC06_Read_Task", 4096, NULL, 5, NULL);
}



unsigned char FucCheckSum(unsigned char *i,unsigned char ln)
{
    unsigned char j,tempq=0;
    i+=1;
    for(j=0;j<(ln-2);j++)
    {
        tempq+=*i;
        i++;
    }
    tempq=(~tempq)+1;
    return(tempq);
}



void ZC06_Read_Task(void* arg)
{
    uint8_t data_u1[BUF_SIZE];
    while(1)
    {
        int len1 = uart_read_bytes(UART_NUM_1, data_u1, BUF_SIZE, 20 / portTICK_RATE_MS);
        if(len1!=0)  //读取到传感器数据
        {
            // printf("zc05=");
            // for(int i=0;i<len1;i++)
            // {
            //     printf("0x%x ",data_u1[i]);
            // }
            // printf("\r\n");      
            
            if(FucCheckSum(data_u1,len1)==data_u1[len1-1])//校验成功
            {
                if((data_u1[4]&0X80)==0x80)//传感器故障
                {
                    Beep_status=BEEP_ERR;
                    printf("Sensor Error\r\n");
                }
                else
                {
                    CH4=(uint16_t)(data_u1[4]&0X7F)*256+data_u1[5];
                    if(CH4>CH4_ALM_GATE)
                    {
                        CH4_Alm_status=1;
                        Beep_status=BEEP_ALM;
                    }
                    else
                    {
                        CH4_Alm_status=0;
                        Beep_status=BEEP_OFF;
                    }
                    printf("CH4=%d\r\n",CH4);
                    printf("Free memory: %d bytes\n", esp_get_free_heap_size());
                }
            }

            len1=0;
            bzero(data_u1,sizeof(data_u1));                 
        } 
        vTaskDelay(500 / portTICK_RATE_MS);
    }   
}


