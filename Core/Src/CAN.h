/*
 * can_comm.h
 * CAN RX ring buffer, DBC signal decoding, CAN TX helper, throttle ADC and
 * thermistor helpers for the DAQ board (STM32L4 bxCAN + FreeRTOS).
 */
#ifndef CAN_COMM_H
#define CAN_COMM_H

#include "main.h"          /* pulls in the HAL (CAN, ADC) headers */
#include <stdint.h>
#include <stdbool.h>

/* ------------------------------------------------------------------ */
/* Configuration                                                       */
/* ------------------------------------------------------------------ */
#define QUEUE_SIZE      64          /* RX ring buffer depth (frames)   */

/* CAN data decoding constants from the provided DBC file */
#define TIME_SCALE      0.001
#define TIME_OFFSET     1577840400.0

#define POS_SCALE       0.000001
#define LAT_OFFSET      (-90.0)
#define LONG_OFFSET     (-180.0)

#define SPEED_SCALE     0.001

#define ACCEL_SCALE     0.125
#define ACCEL_OFFSET    (-64.0)
#define ANGULAR_SCALE   0.25
#define ANGULAR_OFFSET  (-256.0)

/* Throttle potentiometer calibration (12-bit ADC, inverted pot) */
#define THROTTLE_ADC_MIN   370u     /* reading at 100 % throttle */
#define THROTTLE_ADC_MAX   4095u    /* reading at 0 % throttle   */

/* Thermistor divider constants */
#define R_FIXED     10000.0f
#define R0          5000.0f         /* thermistor resistance at 25 C */
#define T0          298.15f
#define BETA        3960.0f         /* fit for ~25-85 C              */
#define ADC_MAX     4095.0f         /* 12-bit                        */

/* ------------------------------------------------------------------ */
/* Types                                                               */
/* ------------------------------------------------------------------ */
typedef struct {
    uint8_t  is_extended;
    uint8_t  dlc;
    uint32_t id;
    uint8_t  data[8];
} CAN_Message;

/* ------------------------------------------------------------------ */
/* CAN RX / TX                                                         */
/* ------------------------------------------------------------------ */

/* Configure an accept-all filter, start the peripheral and enable the
 * FIFO0 message-pending interrupt. Call once before osKernelStart(). */
HAL_StatusTypeDef CAN_Comm_Init(CAN_HandleTypeDef *hcan);

/* Register the task to wake (via task notification) when frames arrive.
 * Pass the handle returned by osThreadCreate(). Optional: if never set,
 * the consumer must poll CAN_Comm_Pop(). */
void CAN_Comm_SetRxTask(void *task_handle);

/* Pop the oldest received frame. Returns true if one was available.
 * Call from ONE consumer task only (single-producer/single-consumer). */
bool CAN_Comm_Pop(CAN_Message *out);

/* Number of frames dropped because the ring buffer was full. */
uint32_t CAN_Comm_GetDropCount(void);

/* Send a frame (standard or extended ID). Returns HAL_BUSY if all three
 * TX mailboxes are full. Call only from the CAN TX task. */
HAL_StatusTypeDef CAN_Comm_Send(CAN_HandleTypeDef *hcan, uint32_t id,
                                bool is_extended, const uint8_t *data,
                                uint8_t len);

/* ------------------------------------------------------------------ */
/* Signal decoding (little-endian / Intel DBC signals)                 */
/* ------------------------------------------------------------------ */

/* Pack the 8 data bytes of a frame into a 64-bit word (byte 0 = LSB). */
uint64_t CAN_Message_ToU64(const CAN_Message *msg);

uint64_t extract_bits(uint64_t raw_data, int start, int length);
int64_t  extract_signed(uint64_t raw_data, int start, int length);
int32_t  extract_signed32(uint64_t raw_data, int start);
float    extract_float32(uint64_t raw_data, int start);

/* offset + scale * unsigned raw value */
double decode_physical_value(uint64_t raw_data, int start, int length,
                             double offset, double scale);
/* offset + scale * signed raw value */
double decode_physical_signed(uint64_t raw_data, int start, int length,
                              double offset, double scale);

/* ------------------------------------------------------------------ */
/* Analog inputs                                                       */
/* ------------------------------------------------------------------ */

/* Blocking single conversion. If convert is true, returns throttle 0-100 %,
 * otherwise the raw 12-bit value. */
uint32_t Read_ADC_Value(ADC_HandleTypeDef *hadc, bool convert);

/* Thermistor temperature in degrees Fahrenheit (NaN if adc is invalid). */
float readTempC(int adc);

#endif /* CAN_COMM_H */