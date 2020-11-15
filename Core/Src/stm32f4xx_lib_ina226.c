/*
 * TI-INA226 Bi-Directional Current and Power Monitor library
 *
 * This is a library for the TI-INA226 Bi-Directional Current and Power
 * Monitor with I2C Compatible Interface.
 *
 * Written by Federico David Ceccarelli,
 + Feel free to submite any comment to fededc88@gmail.com
 *
 * GPL license, all text here must be included in any redistribution.
 *
 */

#include "definitions.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_lib_ina226.h"
#include <stdint.h>

// I2C handle Structure definition for INA226
// TODO: should'nt these variables be static?
I2C_HandleTypeDef ina226_i2c;

_INA226_CONFIG ina226_config;

_INA226_FLAGS ina226_flags;

_INA226_BUFF ina226_tx_buff, ina226_rx_buff;

// Less Significan Bit current - Current Register resolution
// TODO: sames a before
float LSB_current;

/* Size of Transmission buffer */
#define TXBUFFERSIZE              sizeof(ina226_tx_buff)

static uint16_t ina226_read_reg(uint16_t DevAddress, uint16_t RegAddress )
{
	uint8_t rc;
	uint16_t load;

	if ((rc = HAL_I2C_Mem_Read(&ina226_i2c, DevAddress, RegAddress, 1, (uint8_t *) &load, 3, INA226_POLLING_TIMEOUT)) != HAL_OK)
		// Failed
		ina226_Error_Handler();

	ina226_rx_buff.data_h = (uint8_t) (load & 0x00FF);
	ina226_rx_buff.data_l = (uint8_t) ((load & 0xFF00)>>8);

	return ((uint16_t) ina226_rx_buff.data_h << 8 | ina226_rx_buff.data_l);
}

/**
 * @brief INA226 I2C1 Initialization Function
 * @param None
 * @retval None
 */
HAL_StatusTypeDef ina226_I2C1_Init(void) 
{
	uint8_t rc; //<- throws warning of unused variable

	/* USER CODE END I2C1_Init 1 */
	ina226_i2c.Instance = I2C1;
	ina226_i2c.Init.ClockSpeed = 100000;
	ina226_i2c.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
	ina226_i2c.Init.OwnAddress1 = 0;
	ina226_i2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	ina226_i2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	ina226_i2c.Init.OwnAddress2 = 0;
	ina226_i2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	ina226_i2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

	if (HAL_I2C_Init(&ina226_i2c) != HAL_OK) {
		//		ina226_Error_Handler(); //TODO: Revisar a ver que handler usar
	}
	/* USER CODE BEGIN I2C1_Init 2 */
	//Check comunication
	if (HAL_I2C_IsDeviceReady(&ina226_i2c, (uint16_t) INA226_I2C_ADDRESS, 10,
				INA226_POLLING_TIMEOUT) == HAL_OK) {
		rc = HAL_OK;
	}
	/* USER CODE END I2C1_Init 2 */
    return rc;
}

HAL_StatusTypeDef ina226_init(void) 
{

	//TODO: Valores de prueba - revisar

	// set configuration Register value (0x00)
	ina226_config.all =	INA226_AVG_16 						// 16 AVG Combinations
		|	INA226_CFG_SPARE					// spare
		|	INA226_VBUSCT_CONV_T_1_1ms 			// 1.1 ms Bus Voltage Conversion Time
		| 	INA226_VSHCT_CONV_T_1_1ms 			// 1.1 ms Shunt Voltage Conversion Time
		| 	INA226_MODE_SANDBVOLT_CONTINUOUS ;	// Shunt and Bus, Continuous

	// Write configuration Register (0x00) by polling
	ina226_set_config(&ina226_config);
	// Write calibration Register (0x05) by polling
	ina226_calibrate(100,15);

	// INA226 configured and ready to be used
	ina226_flags.inited = TRUE;
	//TODO: return HAL_StatusTypeDef
	return 0;
}

/**
 * @brief  Set INA226 Config Register (00h) according to the specified parameters
 *         in the _INA226_CONFIG.
 * @param  ina226_config Pointer to a _INA226_CONFIG structure that contains
 *         the configuration information for the specified register.
 * @retval HAL status
 * @TODO: is pbuff even used in this function?
 */
HAL_StatusTypeDef ina226_set_config(_INA226_CONFIG *ina226_config)
{
	uint8_t rc;
	uint8_t *pbuff;
    
	pbuff = (uint8_t*) &ina226_tx_buff;

	// Fill tx_buffer with ina226_config data
	*pbuff++ = INA226_CONFIG_REG;
	*pbuff++ = ina226_config->b_high;
	*pbuff 	 = ina226_config->b_low;

	// Transmit by I2C by polling
	if((rc = HAL_I2C_Master_Transmit(&ina226_i2c, (uint16_t) INA226_I2C_ADDRESS, 
					(uint8_t*) &ina226_tx_buff, (uint16_t) TXBUFFERSIZE, 
					INA226_POLLING_TIMEOUT)) != HAL_OK ){
		// Fallo el envio.
		ina226_Error_Handler();
	}

	return rc;
}

/**
 * @brief   Set INA226 Calibration Register (05h) for correct measurement
 * @param   Rshunt Value of the shunt conductance present in the application to 
 *          develop
 *          the configuration information for the specified register.
 * @param   current_max Maximum Expected Current
 * @retval  none
 */
HAL_StatusTypeDef ina226_calibrate( uint16_t Rshunt, float current_max){

	/*	In order for the device to report both current and power values,
	 *  the user must program the resolution of the Current Register (04h)
	 *  and the value of the shunt resistor present in the application to
	 *  develop the differential voltage applied between the input pins. */

	uint8_t rc;
	uint8_t *pbuff;
	uint16_t CAL;

	pbuff = (uint8_t*) &ina226_tx_buff;

	// Calculate Less Significant Bit current - Current Register resolution
	LSB_current = (current_max / 32768);

	// Calculate Calibration Register value
	CAL = (uint16_t) ((0.00512 * Rshunt) / LSB_current);
	// * 0.00512 is an internal fixed value used to ensure scaling is maintained 
    // properly

	// Fill tx_buffer with CAL value
	*pbuff++ = INA226_CAL_REG;
	*pbuff++ = (uint8_t) ((CAL & 0xFF00) >> 8);
	*pbuff   = (uint8_t) (CAL & 0x00FF);

	// Transmit by I2C by polling
	if ((rc = HAL_I2C_Master_Transmit(&ina226_i2c, (uint16_t) INA226_I2C_ADDRESS, 
					(uint8_t*) &ina226_tx_buff, (uint16_t) TXBUFFERSIZE, 
					INA226_POLLING_TIMEOUT)) != HAL_OK) {
		// Transmition failed
		ina226_Error_Handler();
	}

	return rc;
}

uint16_t ina226_get_config_reg(void)
{
	return ina226_read_reg(INA226_I2C_ADDRESS, INA226_CONFIG_REG);
}

float ina226_get_Vshunt(void)
{
	float v_shunt;
	v_shunt = (float) ina226_read_reg(INA226_I2C_ADDRESS, INA226_SHUNT_V_REG) * .00125;
	return v_shunt;
}

uint16_t ina226_get_calibration(void)
{
	return ina226_read_reg(INA226_I2C_ADDRESS, INA226_CAL_REG);
}

float ina226_get_Vbus(void)
{

	float Vbus;

	Vbus = ((float) ina226_read_reg(INA226_I2C_ADDRESS, INA226_BUS_V_REG ) )* 0.00125;

	return Vbus;
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void ina226_Error_Handler(void) {
	//TODO: Implementar
	//  /* Turn LED5 on */
	//  BSP_LED_On(LED5);
	while(1)
	{
	}
}

/* EOF */
