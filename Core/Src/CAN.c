/*
 * can_comm.c
 */
#include "CAN.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <math.h>

/* ------------------------------------------------------------------ */
/* RX ring buffer (ISR = producer, one task = consumer)                */
/* ------------------------------------------------------------------ */
static CAN_Message can_queue[QUEUE_SIZE];
static volatile uint32_t write_index = 0;   /* written only by the ISR  */
static volatile uint32_t read_index  = 0;   /* written only by the task */
static volatile uint32_t drop_count  = 0;

static TaskHandle_t rx_task = NULL;

HAL_StatusTypeDef CAN_Comm_Init(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef f = {0};

    /* Accept-all filter into FIFO0. Tighten this to your DBC IDs later
     * to cut ISR load. */
    f.FilterBank           = 0;
    f.FilterMode           = CAN_FILTERMODE_IDMASK;
    f.FilterScale          = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh         = 0x0000;
    f.FilterIdLow          = 0x0000;
    f.FilterMaskIdHigh     = 0x0000;
    f.FilterMaskIdLow      = 0x0000;
    f.FilterFIFOAssignment = CAN_RX_FIFO0;
    f.FilterActivation     = CAN_FILTER_ENABLE;
    f.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(hcan, &f) != HAL_OK) return HAL_ERROR;
    if (HAL_CAN_Start(hcan) != HAL_OK)            return HAL_ERROR;
    return HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void CAN_Comm_SetRxTask(void *task_handle)
{
    rx_task = (TaskHandle_t)task_handle;
}

/* Overrides the weak HAL callback. Define this only once in the project. */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef hdr;
    uint8_t data[8];
    bool got_any = false;

    /* Drain everything pending in the FIFO */
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hdr, data) != HAL_OK)
            break;

        uint32_t next = (write_index + 1) % QUEUE_SIZE;
        if (next == read_index)          /* buffer full: drop frame */
        {
            drop_count++;
            continue;
        }

        CAN_Message *slot = &can_queue[write_index];
        uint8_t len = (hdr.DLC > 8) ? 8 : (uint8_t)hdr.DLC;

        memset(slot->data, 0, sizeof slot->data);
        memcpy(slot->data, data, len);
        slot->dlc = len;

        if (hdr.IDE == CAN_ID_EXT) {
            slot->id = hdr.ExtId;
            slot->is_extended = 1;
        } else {
            slot->id = hdr.StdId;
            slot->is_extended = 0;
        }

        write_index = next;              /* publish after slot is filled */
        got_any = true;
    }

    /* Wake the CAN RX task */
    if (got_any && rx_task != NULL)
    {
        BaseType_t woken = pdFALSE;
        vTaskNotifyGiveFromISR(rx_task, &woken);
        portYIELD_FROM_ISR(woken);
    }
}

bool CAN_Comm_Pop(CAN_Message *out)
{
    if (read_index == write_index) return false;     /* empty */

    *out = can_queue[read_index];
    read_index = (read_index + 1) % QUEUE_SIZE;
    return true;
}

uint32_t CAN_Comm_GetDropCount(void)
{
    return drop_count;
}

HAL_StatusTypeDef CAN_Comm_Send(CAN_HandleTypeDef *hcan, uint32_t id,
                                bool is_extended, const uint8_t *data,
                                uint8_t len)
{
    CAN_TxHeaderTypeDef hdr = {0};
    uint32_t mailbox;

    if (len > 8) len = 8;
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0) return HAL_BUSY;

    if (is_extended) {
        hdr.IDE   = CAN_ID_EXT;
        hdr.ExtId = id;
    } else {
        hdr.IDE   = CAN_ID_STD;
        hdr.StdId = id;
    }
    hdr.RTR = CAN_RTR_DATA;
    hdr.DLC = len;
    hdr.TransmitGlobalTime = DISABLE;

    return HAL_CAN_AddTxMessage(hcan, &hdr, (uint8_t *)data, &mailbox);
}

/* ------------------------------------------------------------------ */
/* Signal decoding                                                     */
/* ------------------------------------------------------------------ */
uint64_t CAN_Message_ToU64(const CAN_Message *msg)
{
    uint64_t raw = 0;
    for (int i = 7; i >= 0; i--)
        raw = (raw << 8) | msg->data[i];     /* byte 0 -> bits 0..7 */
    return raw;
}

uint64_t extract_bits(uint64_t raw_data, int start, int length)
{
    if (length <= 0 || start < 0 || start >= 64) return 0;
    if (length >= 64) return raw_data >> start;

    uint64_t mask = (1ULL << length) - 1;
    return (raw_data >> start) & mask;
}

int64_t extract_signed(uint64_t raw_data, int start, int length)
{
    if (length <= 0) return 0;
    uint64_t v = extract_bits(raw_data, start, length);

    if (length < 64 && (v & (1ULL << (length - 1))))
        v |= ~((1ULL << length) - 1);        /* sign-extend */
    return (int64_t)v;
}

int32_t extract_signed32(uint64_t raw_data, int start)
{
    return (int32_t)extract_signed(raw_data, start, 32);
}

float extract_float32(uint64_t raw_data, int start)
{
    uint32_t bits = (uint32_t)extract_bits(raw_data, start, 32);
    float f;
    memcpy(&f, &bits, sizeof f);
    return f;
}

double decode_physical_value(uint64_t raw_data, int start, int length,
                             double offset, double scale)
{
    uint64_t raw = extract_bits(raw_data, start, length);
    return offset + scale * (double)raw;
}

double decode_physical_signed(uint64_t raw_data, int start, int length,
                              double offset, double scale)
{
    int64_t raw = extract_signed(raw_data, start, length);
    return offset + scale * (double)raw;
}

/* ------------------------------------------------------------------ */
/* Analog inputs                                                       */
/* ------------------------------------------------------------------ */
uint32_t Read_ADC_Value(ADC_HandleTypeDef *hadc, bool convert)
{
    uint32_t adc_val = 0;

    HAL_ADC_Start(hadc);
    if (HAL_ADC_PollForConversion(hadc, 10) == HAL_OK)
        adc_val = HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);

    if (!convert) return adc_val;

    /* Clamp to the sensor's physical range, then map to 0-100 % */
    if (adc_val > THROTTLE_ADC_MAX) adc_val = THROTTLE_ADC_MAX;
    if (adc_val < THROTTLE_ADC_MIN) adc_val = THROTTLE_ADC_MIN;

    return (THROTTLE_ADC_MAX - adc_val) * 100u /
           (THROTTLE_ADC_MAX - THROTTLE_ADC_MIN);
}

float readTempC(int adc)
{
    if (adc <= 0 || adc >= (int)ADC_MAX) return NAN;   /* open/short */

    float r    = R_FIXED * (ADC_MAX - (float)adc) / (float)adc;
    float invT = 1.0f / T0 + logf(r / R0) / BETA;
    float tempC = 1.0f / invT - 273.15f;
    return tempC * 9.0f / 5.0f + 32.0f;   /* returns Fahrenheit */
}