#include "mymain.h"

VESC vesc_motor[4];
DJI_Motor dji_motor[4];
uint8_t receive_handle[19];
uint8_t receive_imu[7];
uint8_t receive_dt35[19];
uint8_t test[4];
HandleData handle_data;

Speed world_speed;
Speed body_speed;
Helm_chassis helm_chassis;
// uint8_t rx_data[8]; //vesc电机反馈
// FDCAN_RxHeaderTypeDef rx_header;

static void FDCAN1_Init(void)
{
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_Start(&hfdcan1);
}

// static void FDCAN2_Init(void)
// {
//	HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
//	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
//	HAL_FDCAN_Start(&hfdcan2);
// }

// static void FDCAN3_Init(void)
//{
//	HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
//	HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
//	HAL_FDCAN_Start(&hfdcan3);
// }

void StartchassisTask(void *argument)
{
    My_init();
    DJI_calibration();
    osDelay(5);
    helm_chassis_ready();

    for (;;)
    {
        //        for (uint8_t i = 0; i < 4; i++) // 测试用
        //        {
        //            vesc_set_rpm(&vesc_motor[i], 0.0f);
        //        }
        //        for (uint8_t i = 0; i < 4; i++)
        //        {
        //            DJI2006_speed(&dji_motor[3], 5.0f);
        //        }
        //        for(uint8_t i = 0; i < 4; i++)
        //        {
        //            DJI2006_position(&dji_motor[1], 180.00f);
        //        }
        car_angle.actual_angle = Imu_angle;
        if (osMessageQueueGet(handleQueueHandle, receive_handle, NULL, 0U) == osOK)
        {
            Handle_Analysis(receive_handle);
        }
        if (osMessageQueueGet(imuQueueHandle, receive_imu, NULL, 0U) == osOK)
        {
            Imu_Analysis(receive_imu);
        }
        if (osMessageQueueGet(imuQueueHandle, receive_dt35, NULL, 0U) == osOK)
        {
            DT35_Analysis(receive_dt35);
        }
        // World_to_body(&world_speed, &body_speed, Imu_angle);
        test[0] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
        test[1] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
        test[2] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
        test[3] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
        helm_calculate(&body_speed, &helm_chassis);
        osDelay(1);
    }
}

void Startchassis_can(void *argument)
{
    for (;;)
    {
        if (!ready_target_initialized)
        {
            osDelay(1);
            continue;
        }
        osDelay(20);
		for (uint8_t i = 0; i < 4; i++)
		{
			DJI2006_position(&dji_motor[i], helm_chassis.set_angle[i]);
		}
		osDelay(10);
		for (uint8_t i = 0; i < 4; i++)
		{
			vesc_set_rpm(&vesc_motor[i], helm_chassis.helm_speed[i]);
		}
        osDelay(1);
    }
}

void My_init(void)
{
    Vesc_Init(&vesc_motor[0], &hfdcan1, 53);
    Vesc_Init(&vesc_motor[1], &hfdcan1, 54);
    Vesc_Init(&vesc_motor[2], &hfdcan1, 51);
    Vesc_Init(&vesc_motor[3], &hfdcan1, 52);

    DJI2006_Init(&dji_motor[0], &hfdcan1, 1);
    DJI2006_Init(&dji_motor[1], &hfdcan1, 2);
    DJI2006_Init(&dji_motor[2], &hfdcan1, 3);
    DJI2006_Init(&dji_motor[3], &hfdcan1, 4);

    FDCAN1_Init();

    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, receive_handle, sizeof(receive_handle));
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);

    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, receive_handle, sizeof(receive_handle));
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);

    HAL_UARTEx_ReceiveToIdle_DMA(&huart7, receive_imu, sizeof(receive_imu));
    __HAL_DMA_DISABLE_IT(huart7.hdmarx, DMA_IT_HT);
}

void Handle_Analysis(uint8_t *data)
{
    uint16_t crc = 0;
    if (data[0] == 0xAA && data[18] == 0xBB)
    {
        for (uint8_t i = 1; i <= 16; i++)
        {
            crc += data[i];
        }
        if ((crc & 0xFF) == data[17])
        {
            // 获取摇杆值
            for (uint8_t i = 0; i < 8; i++)
            {
                handle_data.rocker.u8[i] = data[i + 1];
            }

            for (uint8_t i = 0; i < 8; i++)
            {
                handle_data.shoulder.u8[i] = data[i + 9];
            }
            // 其他按键暂不解析
        }
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{

    if (huart->Instance == USART1)
    {
        if (Size == sizeof(receive_handle))
        {
            osMessageQueuePut(handleQueueHandle, receive_handle, 0U, 0U);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, receive_handle, sizeof(receive_handle));
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }

    if (huart->Instance == USART2)
    {
        if (Size == sizeof(receive_dt35))
        {
            osMessageQueuePut(handleQueueHandle, receive_dt35, 0U, 0U);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, receive_dt35, sizeof(receive_dt35));
        __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
    }

    if (huart->Instance == UART7)
    {
        if (Size == sizeof(receive_imu))
        {
            osMessageQueuePut(imuQueueHandle, receive_imu, 0U, 0U);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(&huart7, receive_imu, sizeof(receive_imu));
        __HAL_DMA_DISABLE_IT(huart7.hdmarx, DMA_IT_HT);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, receive_handle, sizeof(receive_handle));
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }

    if (huart->Instance == USART2)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, receive_dt35, sizeof(receive_handle));
        __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
    }

    if (huart->Instance == UART7)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart7, receive_imu, sizeof(receive_imu));
        __HAL_DMA_DISABLE_IT(huart7.hdmarx, DMA_IT_HT);
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U)
    {
        FDCAN_RxHeaderTypeDef rx_header;
        uint8_t rx_data[8];
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
        {
            if (rx_header.IdType == FDCAN_STANDARD_ID)
            {
                for (int i = 0; i < 4; i++)
                {
                    DJI2006_Analysisdata(&dji_motor[i], &rx_header, rx_data);
                }
            }
            else
            {
                for (int i = 0; i < 4; i++)
                {
                    VESC_Analysisdata(&vesc_motor[i], &rx_header, rx_data);
                }
            }
        }
    }
}
