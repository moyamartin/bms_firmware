
#ifndef __SERIE_H__
#define __SERIE_H__

#include <string.h>
#include <stdio.h>
#include "serial_defs.h"

#if defined(USE_HAL_DRIVER) && defined(STM32F407xx)
	// Support for stm32f4xx devices
	#include "stm32f4xx_hal.h"
	/* 
	 * This serial layer intend to make easier the acces to the the USART
	 * perriferic through the HAL driver, we asume that it is already
	 * initialized on the main before the infinite loop. 
	 * This design takes into account that the project has been created with
	 * the STM32CubeMX which creates the function needed to initialize the
	 * USART periferic.
	 *
	 * We use the UART5 because of our needs on hardware implementation. We
	 * asume that the USART instance used is the huart5. User should
	 * implement his own.
	 */
	extern UART_HandleTypeDef 	huart5;
	#define SERIAL_UART_INTERFACE	huart5
        #define SERIAL_BUFF_LENGTH      100
#elif defined(__AVR__)

#endif

#define SERIAL_ECHO

/**
 * @enum  serial_status
 * @brief defines several possible status of the serial layer to inform the
 * 		  user the current result of an operation
 */
enum SERIAL_status {
	SERIAL_OK = 0,
	SERIAL_FAIL = -1,
};

struct SERIAL {
    char *pBuff;
    char *pBuffActual;
    uint16_t BuffLen;
};
    

// Global funtions declarations
enum SERIAL_status serial_print_string(char *pTxSrc);
enum SERIAL_status serial_read_char(char *pRxSrc);

void SERIAL_init(void);
void SERIAL_RxHandler(void);

extern char serial_txBuff[SERIAL_BUFF_LENGTH];

#if defined (DEBUG)
#define _SERIAL_DEBUG(fmt, args...) sprintf(serial_txBuff,"%s:%s:%d: "fmt, __FILE__, __FUNCTION__, __LINE__, ##args);serial_print_string(serial_txBuff); 
#else
#define _SERIAL_DEBUG(fmt,args...){}
#endif

#endif

//
// End of file.
//
