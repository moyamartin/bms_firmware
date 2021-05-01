/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
#include "battery_model.h"
#include "battery_pack.h"
#include "bq76pl536a.h"
#include "ina226.h"
#include "logging.h"
#include "main.h"

struct Pack battery_pack;
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;

struct INA226 current_sensor;
struct BQ76 battery_monitor = {
    .adc_control = {
        .ADC_ON  = 1,
        .CELL_SEL = CELL_1_6,
        .TS = BOTH,
    },
    .cb_time = {
        .CBCT = 1,
    },
    .function_config = {
        .CN = CELLS_6,
    },
};

/* MCU Initialization functions */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);

static void handle_bq76_faults(struct BQ76 * device);
static void handle_bq76_alerts(struct BQ76 * device);

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* MCU Initialization */
    __disable_irq();
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize GPIO */
    MX_GPIO_Init();
    /* Initialize I2C1 */
    MX_I2C1_Init();
    /* Initialize SPI1 */
    MX_SPI1_Init();

    /* Initialize current sensor */
    ina226_reset(&current_sensor);
    ina226_init(&current_sensor, GND_GND_ADDRESS, 0.1, 3.2f, AVG1, 
            t1100US, t1100US, SHUNT_AND_BUS_CONT, DEFAULT);
    
    /* Initialize battery monitor */
    bq76_init(&battery_monitor, 0x01, 60, MAX_VCELL, 100, MIN_VCELL, 100, 60, 
              60, 100);
    __enable_irq();
    
    // Request for an adc conversion
    bq76_swrqst_adc_convert(&battery_monitor);
    // Wait until the battery monitor finishes the conversion
    while(!battery_pack.initialized &&
          battery_monitor.data_conversion_ongoing);
    // Assume that the latest value is the OCV voltage of each cell and
    // initalized the battery pack
    init_battery_pack(&battery_pack, battery_monitor.v_cells);

    /* Infinite loop */
    _DEBUG("Start measurements\n");
    while (1)
    {
    }
}

/**
 * @fn      SystemClock_Config
 * @brief   System Clock Configuration
 * @retval  None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
        |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}


/**
 * @fn      MX_ISC1_Init
 * @brief   I2C1 Initialization Function
 * @param   None
 */
static void MX_I2C1_Init(void)
{

    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }

}

/**
 * @fn MX_SPI1_Init
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void)
{
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }

}

/**
 * @fn      MX_GPIO_Init
 * @brief   GPIO Initialization Function
 * @param   None
 * @retval  None
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(SWT_GPIO_GPIO_Port, SWT_GPIO_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(BQ76_CS_GPIO_Port, BQ76_CS_Pin, GPIO_PIN_SET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(BQ76_CONV_GPIO_Port, BQ76_CONV_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : SWT_GPIO_Pin */
    GPIO_InitStruct.Pin = SWT_GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(SWT_GPIO_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : PDM_OUT_Pin */
    GPIO_InitStruct.Pin = PDM_OUT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : BQ76_CS_Pin */
    GPIO_InitStruct.Pin = BQ76_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(BQ76_CS_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : BQ76_FAULT_Pin BQ76_ALERT_Pin */
    GPIO_InitStruct.Pin = BQ76_FAULT_Pin|BQ76_ALERT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pin : BQ76_DRDY_Pin */
    GPIO_InitStruct.Pin = BQ76_DRDY_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BQ76_DRDY_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : BQ76_CONV_Pin */
    GPIO_InitStruct.Pin = BQ76_CONV_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BQ76_CONV_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : BOOT1_Pin */
    GPIO_InitStruct.Pin = BOOT1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);

    HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);

    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == BQ76_DRDY_Pin){
        bq76_read_cells(&battery_monitor);
        battery_monitor.data_conversion_ongoing = 0;
    }
    if(GPIO_Pin == BQ76_ALERT_Pin){
        bq76_read_alert_reg(&battery_monitor);
        handle_bq76_alerts(&battery_monitor);
        bq76_read_alert_reg(&battery_monitor);
    }
    if(GPIO_Pin == BQ76_FAULT_Pin){
        bq76_read_fault_reg(&battery_monitor);
        handle_bq76_faults(&battery_monitor);
        bq76_read_fault_reg(&battery_monitor);
    }
}

static void handle_bq76_alerts(struct BQ76 * device){
    uint8_t clear_alert_flags = 0;

    if(device->alert_status.OT1){
        clear_alert_flags |= device->alert_status.OT1;
    }
    if(device->alert_status.OT2){
        clear_alert_flags |= device->alert_status.OT2 << 1;
    }
    if(device->alert_status.SLEEP){
        clear_alert_flags |= device->alert_status.SLEEP << 2;
    }
    if(device->alert_status.TSD){
        clear_alert_flags |= device->alert_status.TSD << 3;
    }
    if(device->alert_status.FORCE){
        clear_alert_flags |= device->alert_status.FORCE << 4;
    }
    if(device->alert_status.ECC_ERR){
        clear_alert_flags |= device->alert_status.ECC_ERR << 5;
    }
    if(device->alert_status.PARITY){
        clear_alert_flags |= device->alert_status.PARITY << 6;
    }
    if(device->alert_status.AR){
        clear_alert_flags |= device->alert_status.AR << 7;
    }
    bq76_clear_alert_reg(device, clear_alert_flags);
}


static void handle_bq76_faults(struct BQ76 * device)
{ 
    // flags to clear at the last stage of this function
    uint8_t clear_fault_flags = 0;

    // the battery monitor suffered a Power-On Reset
    if(device->fault_status.POR_BIT){
        // in this case we only want to inform the user over CAN and then clear
        // that POR bit
        clear_fault_flags |= device->fault_status.POR_BIT << 3;
    }
    if(device->fault_status.I_FAULT_BIT){
        // In this case, the bq76pl536a is basically broken, we cannot relie on
        // the data that's sending to the MCU, then call the Error_Handler.
        // Send this info over the CAN bus and disconnect the battery
        clear_fault_flags |= device->fault_status.I_FAULT_BIT << 5; 
    }
    if(device->fault_status.COV_BIT){
        // in this case we want to determine which cell is having an overvoltage
        // and switch off the mosfet switch
        bq76_read_cov_fault_reg(device);
        // TODO: switch off the current circulation
        clear_fault_flags |= device->fault_status.COV_BIT;
    }
    if(device->fault_status.CUV_BIT){
        bq76_read_cuv_fault_reg(device);
        // TODO: switch off the current circulation
        clear_fault_flags |= device->fault_status.CUV_BIT << 1;
    }
    if(device->fault_status.CRC_BIT){
        clear_fault_flags |= device->fault_status.CRC_BIT << 2;
    }
    bq76_clear_fault_reg(device, clear_fault_flags);
}


/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */

    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
