
#include "main.h"
#include "pin.h"
#include "stm32_handle.h"
#include "screen.h"
#include "keyboard.h"
#include "stdio.h"
#include "core_main.h"
#include "key.h"
#include "timer.h"

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);

void init_env()
{
    g_timer[TIMER_TIMEROUT1].enable = 0;
    g_timer[TIMER_TIMEROUT1].start_tick = 0;
    g_timer[TIMER_TIMEROUT1].timeout = 250;
    g_timer[TIMER_TIMEROUT1].callback = main_timeout1;

    g_timer[TIMER_TIMEROUT2].enable = 0;
    g_timer[TIMER_TIMEROUT2].start_tick = 0;
    g_timer[TIMER_TIMEROUT2].timeout = 1750;
    g_timer[TIMER_TIMEROUT2].callback = main_timeout2;

    g_timer[TIMER_TIMEROUT3].enable = 0;
    g_timer[TIMER_TIMEROUT3].start_tick = 0;
    g_timer[TIMER_TIMEROUT3].timeout = 0;
    g_timer[TIMER_TIMEROUT3].callback = main_timeout3;

    g_timer[TIMER_REPEATER].enable = 0;
    g_timer[TIMER_REPEATER].start_tick = 0;
    g_timer[TIMER_REPEATER].timeout = 200;
    g_timer[TIMER_REPEATER].callback = main_repeater;

    cur_timer = TIMER_END;

}

void init_calc()
{
    core_init(0, 0, NULL, 0);
    core_powercycle();
}

uint8_t key = 0;

void check_timeout()
{
    uint32_t cur_tick = HAL_GetTick();
    for (uint8_t timer_index = 0; timer_index < TIMER_END; timer_index++) {
        if ((g_timer[timer_index].enable == 1) && (cur_tick > g_timer[timer_index].start_tick) && (cur_tick - g_timer[timer_index].start_tick >= g_timer[timer_index].timeout)) {
            g_timer[timer_index].callback(key);
        }
    }
}
/* USER CODE END 0 */

enum key_state {
    KEY_STATE_A, 
    KEY_STATE_B, 
    KEY_STATE_C,
    KEY_STATE_D,
    KEY_STATE_E,
    KEY_STATE_F,
    KEY_STATE_G,
    KEY_STATE_H
};
// NS = non-shift
// |                                                 | shift-key press | shift-key release | NS-key press | NS-key release |  no action   |
// |       A no key press, return key_none           |       B         | ----------------- |      C       | -------------- | ------------ |
// |       B shift key press, return key_none        | --------------- |         D         |      E       | -------------- | ------------ |
// | C one non-shift key press, return the key press |       F         | ----------------- |      G       |       A        | ------------ |
// |     D shift key release, return shift key       | --------------- | ----------------- | ------------ | -------------- |       A      |
// |    E shift + key press, return the key press    | --------------- |         H         | ------------ |       B        | ------------ |
// |    F key + shift press, return the key press    | --------------- |         H         | ------------ |       B        | ------------ |
// |    G NS key 1 + NS key 2, return key_none       | --------------- | ----------------- | ------------ |       H        | return key 2 |
// |    H release one of two key, return none        |       H         | ----------------- |      G       |       A        | ------------ |
void key_get(uint8_t* key, uint8_t* key_count)
{
    static uint8_t old_key[2] = {0};
    static uint8_t old_press_num = 0;
    static enum key_state old_key_state = KEY_STATE_A;
    static enum key_state next_state = KEY_STATE_A;
    static uint8_t wait_next_state = 10;
    uint8_t new_key[2] = {0};
    uint8_t press_num;
    key_scan(new_key, 2, &press_num);
    if (press_num < old_press_num) { // key release
        if (next_state == KEY_STATE_B) {
            if (wait_next_state != 0) {
                *key_count = 1;
                *key = KEY_SHIFT;
                wait_next_state --;
            } else {
                *key_count = 0;
                *key = 0;
                wait_next_state = 10;
                old_key_state = KEY_STATE_A;
            }
        } else {
            *key_count = 0;
            *key = 0;
        }
        return;
    } else if (press_num == old_press_num) { // key unchange
        *key_count = press_num;
        switch(press_num) {
            case 0:
                *key = 0;
                return;
            case 1:
                old_key_state = KEY_STATE_A;
                if (new_key[0] == KEY_SHIFT) {
                    next_state = KEY_STATE_B;
                    *key = 0;
                } else {
                    next_state = KEY_STATE_C;
                    *key = new_key[0];
                }
                return;
            case 2:
                // return press two key mean shift key and return key press
                switch (old_key_state)
                {
                    case KEY_STATE_E:
                    case KEY_STATE_F:
                        break;
                    case KEY_STATE_G:
                        break;
                    default:
                        break;
                };
                return;
            default:
                *key_count = 0;
                *key = 0;
                return;
        }
    } else { // press one more key
        old_press_num = press_num;
        *key_count = press_num;
        // from zero to one
        // press one key
        if (press_num == 1) {
            old_key[0] = new_key[0];
            old_key[1] = new_key[1];
            if (new_key[0] == KEY_SHIFT) {
                next_state = KEY_STATE_B;
                *key = 0;
            } else {
                next_state = KEY_STATE_C;
               *key = new_key[0];
            }
        } else {
            if (old_key[0] == new_key[0]) {
                old_key[0] = new_key[0];
                old_key[1] = new_key[1];
            } else {
                old_key[1] = new_key[0];
                old_key[0] = new_key[1];
            }
            if (old_key[0] == KEY_SHIFT) {
                // return press two key mean shift key and return key press
                old_key_state = KEY_STATE_B;
                next_state = KEY_STATE_E;
                *key = old_key[1];
                *key_count = 2;
            } else if (old_key[1] == KEY_SHIFT) {
                old_key_state = KEY_STATE_C;
                next_state = KEY_STATE_F;
                *key = old_key[0];
                *key_count = 1;
            } else {
                old_key_state = KEY_STATE_C;
                next_state = KEY_STATE_G;
                if (wait_next_state != 0) {
                    *key_count = 0;
                    *key = 0;
                    wait_next_state --;
                } else {
                    *key_count = 1;
                    *key = old_key[1];
                    wait_next_state = 10;
                    old_key_state = KEY_STATE_G;
                }
            }
        }
    }
}

/**
* @brief  The application entry point.
* @retval int
*/
int main(void)
{
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();
    /* Configure the system clock */
    SystemClock_Config();
    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_SPI2_Init();
    MX_TIM3_Init();
    MX_USART2_UART_Init();

    int last_keynum = 0;
    int key_enqueued = 0;
    int key_repeat = 0;
    uint8_t has_more_key = 0;
    uint8_t key_count = 0;

    LcdDispEnable();
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    ClearLCD();

    init_env();
    init_calc();
    /* Infinite loop */
    while (1)
    {
        check_timeout();
        key_get(&key, &key_count);
        // TODO: TWO key press need adjust hardward setting 
        // TODO: the key when press two key is behave difference in win free 42 need adjust
        int keyrunning = 1;
        if (key != last_keynum) {
            if (key != 0) {
                end_timer();
                // key press
                keyrunning = core_keydown(key, &key_enqueued, &key_repeat);
                if (!key_enqueued) {
                    core_keyup();
                }
                if (key_count == 2) {
                    keyrunning = core_keydown(KEY_SHIFT, &key_enqueued, &key_repeat);
                    if (!key_enqueued) {
                        core_keyup();
                    }
                }
                if (g_timer[TIMER_TIMEROUT3].enable == 1 && key != 28) {
                    end_timer3();
                    core_timeout3(0);
                }
                if (!keyrunning) {
                    if (key_repeat != 0) {
                        cur_timer = TIMER_REPEATER;
                        start_repeater(key_repeat == 1 ? 1000 : 500);
                    } else if (!key_enqueued) {
                        cur_timer = TIMER_TIMEROUT1;
                        start_timer1();
                    }
                }
            }
            last_keynum = key;
        }
        if (LCD_CHANGE == 1) {
            OutputBuff();
        }
    }
}

/**
* @brief System Clock Configuration
* @retval None
*/
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 10;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure the main internal regulator output voltage
    */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
* @brief SPI2 Initialization Function
* @param None
* @retval None
*/
static void MX_SPI2_Init(void)
{
    /* USER CODE BEGIN SPI2_Init 0 */

    /* USER CODE END SPI2_Init 0 */

    /* USER CODE BEGIN SPI2_Init 1 */

    /* USER CODE END SPI2_Init 1 */
    /* SPI2 parameter configuration*/
    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi2.Init.NSS = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_LSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial = 7;
    hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    if (HAL_SPI_Init(&hspi2) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI2_Init 2 */

    /* USER CODE END SPI2_Init 2 */

}

/**
* @brief TIM3 Initialization Function
* @param None
* @retval None
*/
static void MX_TIM3_Init(void)
{

    /* USER CODE BEGIN TIM3_Init 0 */

    /* USER CODE END TIM3_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    /* USER CODE BEGIN TIM3_Init 1 */

    /* USER CODE END TIM3_Init 1 */
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 9999;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 199;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 100;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM3_Init 2 */

    /* USER CODE END TIM3_Init 2 */
    HAL_TIM_MspPostInit(&htim3);

}

/**
* @brief USART2 Initialization Function
* @param None
* @retval None
*/
static void MX_USART2_UART_Init(void)
{

    /* USER CODE BEGIN USART2_Init 0 */

    /* USER CODE END USART2_Init 0 */

    /* USER CODE BEGIN USART2_Init 1 */

    /* USER CODE END USART2_Init 1 */
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART2_Init 2 */

    /* USER CODE END USART2_Init 2 */
}

/**
* @brief GPIO Initialization Function
* @param None
* @retval None
*/
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, COL6_Pin|COL5_Pin|COL4_Pin|COL1_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, DISP_Pin|SCS_Pin|COL3_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(COL2_GPIO_Port, COL2_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : ROW1_Pin ROW7_Pin ROW6_Pin */
    GPIO_InitStruct.Pin = ROW1_Pin|ROW7_Pin|ROW6_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : ROW2_Pin ROW3_Pin ROW4_Pin */
    GPIO_InitStruct.Pin = ROW2_Pin|ROW3_Pin|ROW4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : COL6_Pin COL5_Pin COL4_Pin COL1_Pin */
    GPIO_InitStruct.Pin = COL6_Pin|COL5_Pin|COL4_Pin|COL1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : ROW5_Pin */
    GPIO_InitStruct.Pin = ROW5_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(ROW5_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : DISP_Pin SCS_Pin COL3_Pin */
    GPIO_InitStruct.Pin = DISP_Pin|SCS_Pin|COL3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin : COL2_Pin */
    GPIO_InitStruct.Pin = COL2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(COL2_GPIO_Port, &GPIO_InitStruct);

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
