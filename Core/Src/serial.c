#include "serial.h"

char serial_txBuff[SERIAL_BUFF_LENGTH];

/**
 * @fn	  serial_print()
 * @param [in] pTxSrc pointer to the string to be send through the serial uart
 * @brief This function takes a string and send it through the uart serial port.
 * 	         The method used to send the data is resposability of the user
 * 	         and detpends on the function implementation. 
 * 	         As an example, we have only defined the functionalty for
 * 	         STM32F407xx
 *
 * @retval SERIAL_status [SERIAL_OK|SERIAL_FAIL] 
 */
enum SERIAL_status serial_print (char *pTxSrc)
{ 
#if defined(USE_HAL_DRIVER) && defined(STM32F407xx)
    if(HAL_UART_Transmit_DMA(&SERIAL_UART_INTERFACE , (uint8_t *)pTxSrc, strlen((const char *)pTxSrc)) != HAL_OK)
    {
	return SERIAL_FAIL;
    }
    return SERIAL_OK;
#elif defined (__AVR__)

#endif
}
