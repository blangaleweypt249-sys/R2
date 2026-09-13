#include "small_mcu.h"

#define MAIN_COMMAND_CAN_ID_BASE  0x300 //小板收
#define STEER_STATUS_CAN_ID_BASE  0x380 //小板发
#define STEER_2006_STATUS_CAN_ID_BASE 0x390 //2006舵向反馈
#define MAIN_COMMAND_TIMEOUT_MS   2000

uint32_t Small_MCU_Last_Tick;
AS5047P_HandleTypeDef Small_MCU_SteerSensor;
int32_t Small_MCU_Wheel_Total;
uint16_t Small_MCU_Wheel_Last_Raw;
uint8_t Small_MCU_Wheel_First_Read;
static uint32_t Small_MCU_Last_Steer_Status_Tick;

static void Small_MCU_Parse_Command(uint8_t *data, uint32_t dlc)
{
    uint8_t motor_id;
    uint8_t mode;
    int16_t target;

    if (dlc < 4)
    {
        return;
    }

    motor_id = data[0];
    mode = data[1];
    target = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    C610_Set_Command(motor_id, mode, target);
    Small_MCU_Last_Tick = HAL_GetTick();
}

void Small_MCU_Init(void)
{
    C610_Init(&hcan2);
    //C610_Start_Home_All();
    
    /* AS5047P_Init(&Small_MCU_SteerSensor, &hspi1, GPIOA, GPIO_PIN_4); */
    USER_CAN1_INIT(&hcan1);
    USER_CAN2_INIT(&hcan2);
    /* 调试：上电后先请求舵向回到 0 度，并立即发送一次控制帧。 */
    // C610_Set_Command(1U, C610_MODE_Position, 200);
    // C610_Control();
    Small_MCU_Last_Tick = HAL_GetTick();

    if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK)
    {
        Error_Handler();
    }
}

//
void Small_MCU_Task(void)
{
    uint32_t now = HAL_GetTick();

    /*
    if ((now - Small_MCU_Last_Angle_Tick) >= 5)
    {
        AS5047P_Read(&Small_MCU_SteerSensor);

        if (Small_MCU_SteerSensor.error == 0)
        {
            if (Small_MCU_Wheel_First_Read != 0)
            {
                Small_MCU_Wheel_Last_Raw = Small_MCU_SteerSensor.raw_angle;
                Small_MCU_Wheel_First_Read = 0;
            }
            else
            {
                int32_t delta = (int32_t)Small_MCU_SteerSensor.raw_angle - Small_MCU_Wheel_Last_Raw;
                
                if (delta > 8192) 
                    delta -= 16384;
                if (delta < -8192) 
                 delta += 16384;

                Small_MCU_Wheel_Total += delta;
                Small_MCU_Wheel_Last_Raw = Small_MCU_SteerSensor.raw_angle;
            }
        }
        Small_MCU_Last_Angle_Tick = now;
    }
    */

    /*
    if ((now - Small_MCU_Last_Status_Tick) >= 10)
    {
        can_frame_t frame = {0};
        int16_t angle_cdeg = (int16_t)(Small_MCU_SteerSensor.angle * 100.0f);

        frame.id = STEER_STATUS_CAN_ID_BASE + SMALL_MCU_NODE_ID;
        frame.id_type = CAN_STANDARD;
        frame.dlc = 8U;
        frame.data[0] = (uint8_t)(Small_MCU_SteerSensor.raw_angle >> 8);
        frame.data[1] = (uint8_t)Small_MCU_SteerSensor.raw_angle;
        frame.data[2] = (uint8_t)(angle_cdeg >> 8);
        frame.data[3] = (uint8_t)angle_cdeg;
        frame.data[4] = (uint8_t)(Small_MCU_Wheel_Total >> 8);
        frame.data[5] = (uint8_t)Small_MCU_Wheel_Total;
        frame.data[6] = Small_MCU_SteerSensor.error;
        frame.data[7] = SMALL_MCU_NODE_ID;
        (void)CAN_Send(&hcan1, &frame);
        Small_MCU_Last_Status_Tick = now;

    */
    if ((now - Small_MCU_Last_Steer_Status_Tick) >= 10)
    {
        //2006 舵向反馈 0.01 编码器 转速 在线状态
        {
            can_frame_t steer_frame = {0};
            int16_t steer_angle_cdeg = (int16_t)(C610.motor[0].angle_deg * 100.0f);
            steer_frame.id = STEER_2006_STATUS_CAN_ID_BASE + SMALL_MCU_NODE_ID;
            steer_frame.id_type = CAN_STANDARD;
            steer_frame.dlc = 8U;
            steer_frame.data[0] = (uint8_t)(steer_angle_cdeg >> 8);
            steer_frame.data[1] = (uint8_t)steer_angle_cdeg;
            steer_frame.data[2] = (uint8_t)(C610.motor[0].last_encoder >> 8);
            steer_frame.data[3] = (uint8_t)C610.motor[0].last_encoder;
            steer_frame.data[4] = (uint8_t)(C610.motor[0].speed_rpm >> 8);
            steer_frame.data[5] = (uint8_t)C610.motor[0].speed_rpm;
            steer_frame.data[6] = C610.motor[0].is_online;
            steer_frame.data[7] = SMALL_MCU_NODE_ID;
            (void)CAN_Send(&hcan1, &steer_frame);
        }
        Small_MCU_Last_Steer_Status_Tick = now;
    }
}


//定时器回调
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        uint32_t now = HAL_GetTick();

        C610_Control();
    }
}

//CAN回调
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header = {0};
    uint8_t rx_data[8] = {0};

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    if ((rx_header.IDE != CAN_ID_STD) || (rx_header.RTR != CAN_RTR_DATA))
    {
        return;
    }

    if (hcan->Instance == CAN1)
    {
        if ((rx_header.StdId == (MAIN_COMMAND_CAN_ID_BASE + SMALL_MCU_NODE_ID)) ||
            (rx_header.StdId == MAIN_COMMAND_CAN_ID_BASE)) /* 0x300 可作广播 */
        {
            Small_MCU_Parse_Command(rx_data, rx_header.DLC);
        }
    }
    else if (hcan->Instance == CAN2)
    {
        C610_Receive(rx_header.StdId, rx_data);
    }
}
