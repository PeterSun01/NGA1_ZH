
#ifndef _BEEP_H_
#define _BEEP_H_

#include "freertos/FreeRTOS.h"

extern void Beep_Init(void);
extern void Beep_On(void);
extern void Beep_Off(void);

#define GPIO_BEEP    25

uint8_t Beep_status;

#define BEEP_ALM    0X00
#define BEEP_OFF    0X01
#define BEEP_ERR    0X02


#endif

