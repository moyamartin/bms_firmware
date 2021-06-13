#include "serial.h"

char serial_txBuff[SERIAL_BUFF_LENGTH];
char serial_rxBuff[SERIAL_BUFF_LENGTH];

struct SERIAL SERIAL_rx;

/**
 * @fn	  serial_print_string()
 * @param [in] pTxSrc pointer to the string to be send through the serial uart
 * @brief This function takes a string and send it through the uart serial port.
 * 	         The method used to send the data is resposability of the user
 * 	         and detpends on the function implementation. 
 * 	         As an example, we have only defined the functionalty for
 * 	         STM32F407xx
 *
 * @retval SERIAL_status [SERIAL_OK|SERIAL_FAIL] 
 */
enum SERIAL_status serial_print_string (char *pTxSrc)
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

/**
 * @fn	  serial_print_char()
 * @param [in] pTxSrc pointer to the char to be send through the serial uart
 * @brief This function takes a char and send it through the uart serial port.
 * 	         The method used to send the data is resposability of the user
 * 	         and detpends on the function implementation. 
 * 	         As an example, we have only defined the functionalty for
 * 	         STM32F407xx
 *
 * @retval SERIAL_status [SERIAL_OK|SERIAL_FAIL] 
 */
enum SERIAL_status serial_print_char (char *pTxSrc)
{ 
#if defined(USE_HAL_DRIVER) && defined(STM32F407xx)
    if(HAL_UART_Transmit_DMA(&SERIAL_UART_INTERFACE , (uint8_t *)pTxSrc, 1) != HAL_OK)
    {
	return SERIAL_FAIL;
    }
    return SERIAL_OK;
#elif defined (__AVR__)

#endif
}
/**
 * @fn	  serial_read_char()
 * @param [in] pRxSrc pointer to the address where should be copied the received
 *               character through the serial uart
 * @brief This function receives a charather through the uart serial port and
 *               copies it to an specific address.
 * 	         The method used to receive the data is resposability of the user
 * 	         and detpends on the function implementation. 
 * 	         As an example, we have only defined the functionalty for
 * 	         STM32F407xx
 *
 * @retval SERIAL_status [SERIAL_OK|SERIAL_FAIL] 
 */
enum SERIAL_status serial_read_char (char *pRxSrc)
{ 
#if defined(USE_HAL_DRIVER) && defined(STM32F407xx)
    if(HAL_UART_Receive_DMA(&SERIAL_UART_INTERFACE , (uint8_t *)pRxSrc, 1) != HAL_OK)
    {
	return SERIAL_FAIL;
    }
    return SERIAL_OK;
#elif defined (__AVR__)

#endif
}

void SERIAL_init(void)
{
    SERIAL_rx.pBuff = SERIAL_rx.pBuffActual = serial_rxBuff;
    SERIAL_rx.BuffLen = SERIAL_BUFF_LENGTH;
    serial_read_char (SERIAL_rx.pBuffActual);
}
	    
void SERIAL_RxHandler(void)
{
#if defined(SERIAL_ECHO)
    serial_print_char (SERIAL_rx.pBuffActual);
#endif
    if(SERIAL_rx.pBuffActual == (SERIAL_rx.pBuff + SERIAL_rx.BuffLen))
	SERIAL_rx.pBuffActual = SERIAL_rx.pBuff;
    else
	SERIAL_rx.pBuffActual++;
    serial_read_char (SERIAL_rx.pBuffActual);
}
