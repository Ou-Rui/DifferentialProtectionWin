#ifndef __RS485_H__
#define __RS485_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "usart.h"
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
#define RS485_RE_Pin GPIO_PIN_7
#define RS485_RE_Port GPIOD

#define RS485_RE_LEVEL GPIO_PIN_RESET
#define RS485_DE_LEVEL GPIO_PIN_SET
/* USER CODE END Private defines */


/* USER CODE BEGIN Prototypes */
void RS485_Init(void);
void RS485_Reset(void);

GPIO_PinState RS485_Get_Mode(void);
void RS485_RE_Mode(void);
void RS485_DE_Mode(void);
// void RS485_onReceive(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

