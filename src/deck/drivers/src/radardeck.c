/* radardeck.c – UART‑based pose deck (cleaned)                        */
/*
 * Protocol (18 bytes @ 1 M Bd, 8‑N‑1 on UART1 PB6/PB7):
 *   0xA5  float x  float y  float z  float stdDev  crc8
 *
 * IO‑1 (PB8) is held LOW during Crazyflie boot and set HIGH once the
 * radio/CRTP link is up; the radar MCU must start transmitting only
 * after detecting the rising edge.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "deck.h"
#include "uart1.h"
#include "FreeRTOS.h"
#include "task.h"
#include "system.h"            // systemWaitStart()

#include "stabilizer_types.h"
#include "estimator.h"
#include "log.h"

#define BAUDRATE  1000000u
#define PKT_SYNC  0xA5
#define PKT_LEN   (1 + 4*4 + 1)   // 18 bytes

static float lastX, lastY, lastZ;
static uint8_t poseValid;

/* ----------------------- CRC‑8 helper ------------------------------*/
static uint8_t crc8_maxim(const uint8_t *d, size_t n)
{
  uint8_t c = 0;
  for (size_t i = 0; i < n; i++) {
    uint8_t b = d[i];
    for (uint8_t j = 0; j < 8; j++) {
      uint8_t mix = (c ^ b) & 1;
      c >>= 1;
      if (mix) c ^= 0x8C;
      b >>= 1;
    }
  }
  return c;
}

/* ----------------------- worker task -------------------------------*/
static void radarTask(void *arg)
{
  (void)arg;

  systemWaitStart();                 // wait until CRTP/radio is alive

  pinMode(DECK_GPIO_IO1, OUTPUT);
  digitalWrite(DECK_GPIO_IO1, HIGH);   // keep radar MCU silent
  uart1Init(BAUDRATE);               // enable UART only now!
  digitalWrite(DECK_GPIO_IO1, LOW); // raise ready flag for radar MCU

  uint8_t buf[PKT_LEN];
  size_t  idx = 0;

  for (;;) {
    uint8_t byte;
    uart1Getchar(&byte);             // blocking read

    if (idx == 0) {
      if (byte == PKT_SYNC) buf[idx++] = byte;
    } else {
      buf[idx++] = byte;
      if (idx == PKT_LEN) {
        if (crc8_maxim(&buf[1], PKT_LEN - 2) == buf[PKT_LEN - 1]) {
          float *f = (float *)&buf[1];
          positionMeasurement_t m = {
            .x = f[0], .y = f[1], .z = f[2],
            .stdDev = f[3],
            .source = MeasurementSourceLocationService,
          };
          estimatorEnqueuePosition(&m);

          lastX = f[0]; lastY = f[1]; lastZ = f[2]; poseValid = 1;
        }
        idx = 0;
      }
    }
  }
}

/* ----------------------- deck init ---------------------------------*/
static void radarInit(DeckInfo *info)
{
  xTaskCreate(radarTask, "radarUart", 256, NULL, 1, NULL);
}

static const DeckDriver radarDeck = {
  .vid = 0xB0,
  .pid = 0x0D,
  .name = "radarDeck",
  .requiredEstimator = StateEstimatorTypeKalman,
  .init = radarInit,
};
DECK_DRIVER(radarDeck);

/* ----------------------- logging -----------------------------------*/
LOG_GROUP_START(radar)
LOG_ADD(LOG_FLOAT, x, &lastX)
LOG_ADD(LOG_FLOAT, y, &lastY)
LOG_ADD(LOG_FLOAT, z, &lastZ)
LOG_ADD(LOG_UINT8, valid, &poseValid)
LOG_GROUP_STOP(radar)
