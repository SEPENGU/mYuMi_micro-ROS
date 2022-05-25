/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microxrcedds_c/config.h>
#include <std_msgs/msg/header.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <uxr/client/transport.h>

#include "FreeRTOS.h"
#include "task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STRING_BUFFER_LEN 50
#define DUTY_MAX 60000
#define FREQ_MS 80

#define RCCHECK(fn)                                                                      \
    {                                                                                    \
        rcl_ret_t temp_rc = fn;                                                          \
        if ((temp_rc != RCL_RET_OK)) {                                                   \
            printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
            vTaskDelete(NULL);                                                           \
        }                                                                                \
    }
#define RCSOFTCHECK(fn)                                                                    \
    {                                                                                      \
        rcl_ret_t temp_rc = fn;                                                            \
        if ((temp_rc != RCL_RET_OK)) {                                                     \
            printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
        }                                                                                  \
    }
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 800 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for lowTask */
osThreadId_t lowTaskHandle;
const osThreadAttr_t lowTask_attributes = {
    .name = "lowTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityBelowNormal,
};
/* Definitions for idleTask */
osThreadId_t idleTaskHandle;
const osThreadAttr_t idleTask_attributes = {
    .name = "idleTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void *argument);
void StartLowTask(void *argument);
void StartIdleTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
int _write(int file, char *data, int len) {
    if ((file != 1) && (file != 2)) {  // STDOUT_FILENO, STDERR_FILENO
        return -1;
    }

    // arbitrary timeout 1000
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart3, (uint8_t *)data, len, 1000);

    // return # of bytes written - as best we can tell
    return (status == HAL_OK ? len : 0);
}
*/
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */
#ifdef CONFIG_PERCEPIO_TRACERECORDER_ENABLED
    // vTraceEnable(TRC_START);
    vTraceEnable(TRC_INIT);
#endif
    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART3_UART_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    /* USER CODE BEGIN 2 */

    /* USER CODE END 2 */

    /* Init scheduler */
    osKernelInitialize();

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* creation of defaultTask */
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

    /* creation of lowTask */
    lowTaskHandle = osThreadNew(StartLowTask, NULL, &lowTask_attributes);

    /* creation of idleTask */
    idleTaskHandle = osThreadNew(StartIdleTask, NULL, &idleTask_attributes);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
    /* USER CODE END RTOS_EVENTS */

    /* Start scheduler */
    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */
    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {
    /* USER CODE BEGIN TIM1_Init 0 */

    /* USER CODE END TIM1_Init 0 */

    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_IC_InitTypeDef sConfigIC = {0};

    /* USER CODE BEGIN TIM1_Init 1 */

    /* USER CODE END TIM1_Init 1 */
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 72;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 0xffff;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_IC_Init(&htim1) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM1_Init 2 */

    /* USER CODE END TIM1_Init 2 */
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {
    /* USER CODE BEGIN TIM2_Init 0 */

    /* USER CODE END TIM2_Init 0 */

    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    /* USER CODE BEGIN TIM2_Init 1 */

    /* USER CODE END TIM2_Init 1 */
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 23;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 60000;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 2000;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM2_Init 2 */

    /* USER CODE END TIM2_Init 2 */
    HAL_TIM_MspPostInit(&htim2);
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {
    /* USER CODE BEGIN USART3_Init 0 */

    /* USER CODE END USART3_Init 0 */

    /* USER CODE BEGIN USART3_Init 1 */

    /* USER CODE END USART3_Init 1 */
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart3) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART3_Init 2 */

    /* USER CODE END USART3_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {
    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Stream1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
    /* DMA1_Stream3_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOE, TRIG_Pin | BLINK_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : TRIG_Pin BLINK_Pin */
    GPIO_InitStruct.Pin = TRIG_Pin | BLINK_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0;  // is the first value captured ?
double Distance = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
    {
        if (Is_First_Captured == 0)  // if the first value is not captured
        {
            IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read the first value
            Is_First_Captured = 1;                                     // set the first captured as true
            // Now change the polarity to falling edge
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
        }

        else if (Is_First_Captured == 1)  // if the first is already captured
        {
            IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value
            __HAL_TIM_SET_COUNTER(htim, 0);                            // reset the counter

            if (IC_Val2 > IC_Val1) {
                Difference = IC_Val2 - IC_Val1;
            }

            else if (IC_Val1 > IC_Val2) {
                Difference = (0xffff - IC_Val1) + IC_Val2;
            }

            Distance = Difference * .0343 / 2;
            Is_First_Captured = 0;  // set it back to false

            // set polarity to rising edge
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
            __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
        }
    }
}

enum { NS_PER_SECOND = 1000000000 };

double sub_timespec(struct timespec t1, struct timespec t2) {
    double nsec = t1.tv_nsec - t2.tv_nsec;
    double sec = t1.tv_sec - t2.tv_sec;
    if (sec > 0 && nsec < 0) {
        nsec += NS_PER_SECOND;
        sec--;
    } else if (sec < 0 && nsec > 0) {
        nsec -= NS_PER_SECOND;
        sec++;
    }
    return sec + nsec;
}

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

rcl_publisher_t ping_pub;
rcl_subscription_t pong_sub;
rcl_publisher_t peng_pub;

std_msgs__msg__Header msg_header;

int state;
#ifdef CONFIG_PERCEPIO_TRACERECORDER_ENABLED
traceString chnState;
#endif

struct timespec ts;

void ping_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (state >= 1) {
        struct timespec currT;
        clock_gettime(CLOCK_REALTIME, &currT);
        if ((currT.tv_sec - ts.tv_sec) > 1) {
            state = 0;
            // printf("State reset\r\n");
        }
    }
    if (timer != NULL && state < 1) {
        HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
        osDelay(10);
        HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low

        __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);

        sprintf(msg_header.frame_id.data, "%f", Distance);
        msg_header.frame_id.size = strlen(msg_header.frame_id.data);

        // Fill the message timestamp
        clock_gettime(CLOCK_REALTIME, &ts);
        msg_header.stamp.sec = ts.tv_sec;
        msg_header.stamp.nanosec = ts.tv_nsec;

        printf("Ping data: %s\r\n", msg_header.frame_id.data);
        state = 1;
#ifdef CONFIG_PERCEPIO_TRACERECORDER_ENABLED
        vTracePrintF(chnState, "%d", state);
#endif
        // HAL_GPIO_WritePin(BLINK_GPIO_Port, BLINK_Pin, state);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, state * DUTY_MAX);
        RCCHECK(rcl_publish(&ping_pub, (const void *)&msg_header, NULL));
    }
}

void pong_sub_callback(const void *msgin) {
    struct timespec currT;
    clock_gettime(CLOCK_REALTIME, &currT);

    const std_msgs__msg__Header *msg = (const std_msgs__msg__Header *)msgin;
    if (msg_header.stamp.sec == msg->stamp.sec && msg_header.stamp.nanosec == msg->stamp.nanosec) {
        state = 0;
#ifdef CONFIG_PERCEPIO_TRACERECORDER_ENABLED
        vTracePrintF(chnState, "%d", state);
#endif
        // HAL_GPIO_WritePin(BLINK_GPIO_Port, BLINK_Pin, state);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, state * DUTY_MAX);

        double RTT = sub_timespec(currT, ts);
        // printf("Pong data: %s, RTT: %fs\r\n", msg->frame_id.data, RTT);

        RCSOFTCHECK(rcl_publish(&peng_pub, (const void *)&msg_header, NULL));
    } else {
        state = 0.5;
#ifdef CONFIG_PERCEPIO_TRACERECORDER_ENABLED
        vTracePrintF(chnState, "%d", state);
#endif
        // HAL_GPIO_WritePin(BLINK_GPIO_Port, BLINK_Pin, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, state * DUTY_MAX);
    }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
bool cubemx_transport_open(struct uxrCustomTransport *transport);
bool cubemx_transport_close(struct uxrCustomTransport *transport);
size_t cubemx_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err);
size_t cubemx_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err);

void *microros_allocate(size_t size, void *state);
void microros_deallocate(void *pointer, void *state);
void *microros_reallocate(void *pointer, size_t size, void *state);
void *microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void *state);
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument) {
    /* USER CODE BEGIN 5 */
    clock_gettime(CLOCK_REALTIME, &ts);

    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

#ifdef CONFIG_PERCEPIO_TRACERECORDER_ENABLED
    chnState = xTraceRegisterString("state");
#endif

    // micro-ROS configuration

    rmw_uros_set_custom_transport(true, (void *)&huart3, cubemx_transport_open, cubemx_transport_close,
                                  cubemx_transport_write, cubemx_transport_read);

    rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
    freeRTOS_allocator.allocate = microros_allocate;
    freeRTOS_allocator.deallocate = microros_deallocate;
    freeRTOS_allocator.reallocate = microros_reallocate;
    freeRTOS_allocator.zero_allocate = microros_zero_allocate;

    if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
        // printf("Error on default allocators (line %d)\n", __LINE__);
    }

    // micro-ROS app
    while (RMW_RET_OK != rmw_uros_ping_agent(1000, 1))
        ;

    allocator = rcl_get_default_allocator();

    // create init_options
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    size_t domain_id = (size_t)50;
    rcl_init_options_set_domain_id(&init_options, domain_id);
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    // RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    rclc_node_init_default(&node, "ping_f7_node", "", &support);

    const rosidl_message_type_support_t *type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header);
    RCCHECK(rclc_publisher_init_best_effort(&ping_pub, &node, type_support, "/mros/ping"));
    RCCHECK(rclc_subscription_init_best_effort(&pong_sub, &node, type_support, "/ros2/pong"));
    RCCHECK(rclc_publisher_init_best_effort(&peng_pub, &node, type_support, "/mros/peng"));

    // Create a 3 seconds ping timer,
    rcl_timer_t timer;
    RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(FREQ_MS), ping_timer_callback));

    // Create executor
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &pong_sub, &msg_header, &pong_sub_callback, ON_NEW_DATA));

    // Create and allocate the pingpong messages
    msg_header.frame_id.data = (char *)malloc(sizeof(char) * STRING_BUFFER_LEN);
    msg_header.frame_id.size = 0;
    msg_header.frame_id.capacity = STRING_BUFFER_LEN;

    /* Infinite loop */
    for (;;) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(FREQ_MS / 2));
        osDelay(FREQ_MS / 2);
    }

    // free resources
    RCCHECK(rcl_publisher_fini(&ping_pub, &node));
    RCCHECK(rcl_subscription_fini(&pong_sub, &node));
    RCCHECK(rcl_publisher_fini(&peng_pub, &node));
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);
    /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartLowTask */
/**
 * @brief Function implementing the lowTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartLowTask */
void StartLowTask(void *argument) {
    /* USER CODE BEGIN StartLowTask */
    // traceString chnLow = xTraceRegisterString("Low");;
    // bool lowOn = false;
    /* Infinite loop */
    for (;;) {
        // lowOn = true;
        // vTracePrintF(chnLow, "%d", lowOn);
        for (int i = 0; i < 3000; i++)
            ;
        // lowOn = false;
        // vTracePrintF(chnLow, "%d", lowOn);
        osDelay(777);
    }
    /* USER CODE END StartLowTask */
}

/* USER CODE BEGIN Header_StartIdleTask */
/**
 * @brief Function implementing the idleTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartIdleTask */
void StartIdleTask(void *argument) {
    /* USER CODE BEGIN StartIdleTask */
    // traceString chnIdle = xTraceRegisterString("Idle");;
    // bool idleOn = false;
    /* Infinite loop */
    for (;;) {
        // idleOn = true;
        // vTracePrintF(chnIdle, "%d", idleOn);
        // idleOn = false;
        // vTracePrintF(chnIdle, "%d", idleOn);
        osDelay(1234);
    }
    /* USER CODE END StartIdleTask */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM7 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM7) {
        HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */

    /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
