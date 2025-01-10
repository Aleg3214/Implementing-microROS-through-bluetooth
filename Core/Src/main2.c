/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <uxr/client/profile/transport/custom/custom_transport.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define UXR_CONFIG_SERIAL_TRANSPORT_MTU 512

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BLUETOOTH_TIMEOUT 100 // Timeout in milliseconds
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1; // Bluetooth UART
UART_HandleTypeDef huart2; // Debug UART

/* USER CODE BEGIN PV */
uint8_t rxData;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
// Custom transport functions
bool my_custom_transport_open(uxrCustomTransport* transport);
bool my_custom_transport_close(uxrCustomTransport* transport);
size_t my_custom_transport_write(uxrCustomTransport* transport, const uint8_t* buffer, size_t length, uint8_t* errcode);
size_t my_custom_transport_read(uxrCustomTransport* transport, uint8_t* buffer, size_t length, int timeout, uint8_t* errcode);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Custom transport implementation
struct custom_args {
    UART_HandleTypeDef* uart_handle;  // UART handle for communication
};

// Callback implementations
bool my_custom_transport_open(uxrCustomTransport* transport) {
    struct custom_args* args = (struct custom_args*)transport->args;
    if (HAL_UART_Init(args->uart_handle) != HAL_OK) {
        return false;
    }
    return true;
}

bool my_custom_transport_close(uxrCustomTransport* transport) {
    struct custom_args* args = (struct custom_args*)transport->args;
    if (HAL_UART_DeInit(args->uart_handle) != HAL_OK) {
        return false;
    }
    return true;
}

size_t my_custom_transport_write(uxrCustomTransport* transport, const uint8_t* buffer, size_t length, uint8_t* errcode) {
    struct custom_args* args = (struct custom_args*)transport->args;
    if (HAL_UART_Transmit(args->uart_handle, (uint8_t*)buffer, length, HAL_MAX_DELAY) == HAL_OK) {
        return length;
    } else {
        if (errcode) *errcode = 1;  // Set error code if the write fails
        return 0;
    }
}

size_t my_custom_transport_read(uxrCustomTransport* transport, uint8_t* buffer, size_t length, int timeout, uint8_t* errcode) {
    struct custom_args* args = (struct custom_args*)transport->args;
    if (HAL_UART_Receive(args->uart_handle, buffer, length, timeout) == HAL_OK) {
        return length;
    } else {
        if (errcode) *errcode = 1;  // Set error code if the read fails
        return 0;
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/
	   HAL_Init();
	    SystemClock_Config();
	    MX_GPIO_Init();
	    MX_USART1_UART_Init();  // UART for Bluetooth

	    // Custom transport setup
	    struct custom_args args = { .uart_handle = &huart1 };
	    uxrCustomTransport transport;

	    // Set the callbacks for the transport
	    uxr_set_custom_transport_callbacks(&transport, false,  // No framing
	        my_custom_transport_open,
	        my_custom_transport_close,
	        my_custom_transport_write,
	        my_custom_transport_read);

	    // Initialize the custom transport
	    if (!uxr_init_custom_transport(&transport, &args)) {
	        Error_Handler();  // Handle initialization failure
	    }
    // Initialize micro-ROS
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_node_t node;

    // Setup micro-ROS
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "micro_ros_node", "", &support);

    /* USER CODE END 2 */

    /* Infinite loop */
    while (1) {
        /* USER CODE BEGIN WHILE */
        // micro-ROS executor or custom logic here
        rclc_spin_node(&node);
        /* USER CODE END WHILE */
    }
    /* USER CODE BEGIN 3 */
    /* USER CODE END 3 */
}

/* USER CODE BEGIN Additional Configuration Functions */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        // Handle Bluetooth data
        HAL_UART_Receive_IT(&huart1, &rxData, 1);
    }
}
/* USER CODE END Additional Configuration Functions */
