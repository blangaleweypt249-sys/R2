#include "CAN_PUB.h"

#if USER_CAN_HAS_FDCAN

void USER_FDCAN1_INIT(FDCAN_HandleTypeDef *hfdcan)
{
    HAL_FDCAN_ConfigGlobalFilter(hfdcan,FDCAN_ACCEPT_IN_RX_FIFO0,FDCAN_ACCEPT_IN_RX_FIFO0,FDCAN_FILTER_REMOTE,FDCAN_FILTER_REMOTE);

    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_Start(hfdcan);
}

bool FDCAN_Send(void *user_context,const can_frame_t *frame)
{
    FDCAN_TxHeaderTypeDef tx_header = {0};

    static const uint32_t fdcan_dlc[9] =
    {
        FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2,
        FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5,
        FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8
    };

    if ((user_context == NULL) || (frame == NULL) || (frame->dlc > 8U))
    {
        return false;
    }

    tx_header.Identifier = frame->id;
    tx_header.IdType = (frame->id_type == CAN_EXTENDED) ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = fdcan_dlc[frame->dlc];
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    return HAL_FDCAN_AddMessageToTxFifoQ((FDCAN_HandleTypeDef *)user_context, &tx_header, (uint8_t *)frame->data) == HAL_OK;
}

bool FDCAN_Read(FDCAN_HandleTypeDef *hfdcan, can_frame_t *frame)
{
    FDCAN_RxHeaderTypeDef rx_header = {0};

    if ((hfdcan == NULL) || (frame == NULL))
    {
        return false;
    }

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, frame->data) != HAL_OK)
    {
        return false;
    }

    if (rx_header.RxFrameType != FDCAN_DATA_FRAME)
    {
        return false;
    }

    frame->id = rx_header.Identifier;
    frame->id_type = (rx_header.IdType == FDCAN_EXTENDED_ID) ? CAN_EXTENDED : CAN_STANDARD;
    // STM32 HAL 将 FDCAN 的 DLC 放在 DataLength 的第 16~19 位。
    frame->dlc = (uint8_t)((rx_header.DataLength >> 16) & 0x0FU);
    return frame->dlc <= 8U;
}

#endif

#if USER_CAN_HAS_CAN

static void USER_CAN_INIT(CAN_HandleTypeDef *hcan, uint32_t filter_bank)
{
    CAN_FilterTypeDef filter = {0};

    filter.FilterBank = filter_bank;
    filter.FilterIdHigh = 0;
    filter.FilterIdLow = 0;
    filter.FilterMaskIdHigh = 0;
    filter.FilterMaskIdLow = 0;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(hcan, &filter) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_CAN_Start(hcan) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }
}

void USER_CAN1_INIT(CAN_HandleTypeDef *hcan)
{
    USER_CAN_INIT(hcan, 0U);
}

void USER_CAN2_INIT(CAN_HandleTypeDef *hcan)
{
    USER_CAN_INIT(hcan, 14U);
}

bool CAN_Send(void *user_context,const can_frame_t *frame)
{
    CAN_TxHeaderTypeDef tx_header = {0};
    uint32_t tx_mailbox = 0;

    if ((user_context == NULL) || (frame == NULL) || (frame->dlc > 8U))
    {
        return false;
    }

    tx_header.IDE = (frame->id_type == CAN_EXTENDED) ? CAN_ID_EXT : CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = frame->dlc;

    if (frame->id_type == CAN_EXTENDED)
    {
        tx_header.ExtId = frame->id;
    }
    else
    {
        tx_header.StdId = frame->id;
    }

    return HAL_CAN_AddTxMessage((CAN_HandleTypeDef *)user_context, &tx_header, (uint8_t *)frame->data, &tx_mailbox) == HAL_OK;
}

bool CAN_Read(CAN_HandleTypeDef *hcan, can_frame_t *frame)
{
    CAN_RxHeaderTypeDef rx_header = {0};

    if ((hcan == NULL) || (frame == NULL))
    {
        return false;
    }

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, frame->data) != HAL_OK)
    {
        return false;
    }

    if (rx_header.RTR != CAN_RTR_DATA)
    {
        return false;
    }

    frame->id_type = (rx_header.IDE == CAN_ID_EXT) ? CAN_EXTENDED : CAN_STANDARD;
    frame->id = (frame->id_type == CAN_EXTENDED) ? rx_header.ExtId : rx_header.StdId;
    frame->dlc = (uint8_t)rx_header.DLC;
    return frame->dlc <= 8U;
}

#endif
