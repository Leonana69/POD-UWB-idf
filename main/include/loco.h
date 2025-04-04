#ifndef __LOCO_H__
#define __LOCO_H__

#ifdef __cplusplus
extern "C" {
#endif
#include "types.h"
#include "driver/spi_master.h"

#define SPI_HOST    SPI2_HOST  // Use SPI2 or SPI3, SPI1 is typically for flash
#define PIN_NUM_MISO GPIO_NUM_8
#define PIN_NUM_MOSI GPIO_NUM_9
#define PIN_NUM_CLK  GPIO_NUM_7
#define PIN_NUM_CS   GPIO_NUM_3
#define PIN_NUM_IRQ  GPIO_NUM_4
#define PIN_NUM_RST  GPIO_NUM_6

#include "libdw1000.h"
#define LOCODECK_NR_OF_TDOA2_ANCHORS 8
// Timestamp counter frequency
#define LOCODECK_TS_FREQ (499.2e6 * 128)

#define LOCODECK_ANTENNA_OFFSET 154.6   // In meters
#define SPEED_OF_LIGHT (299792458.0)
#define LOCODECK_ANTENNA_DELAY  ((LOCODECK_ANTENNA_OFFSET * LOCODECK_TS_FREQ) / SPEED_OF_LIGHT) // In radio ticks

typedef enum uwbEvent_e {
  eventTimeout,
  eventPacketReceived,
  eventPacketSent,
  eventReceiveTimeout,
  eventReceiveFailed,
} uwbEvent_t;

typedef uint64_t locoAddress_t;

// Callbacks for uwb algorithms
typedef struct uwbAlgorithm_s {
    void (*init)(dwDevice_t *dev);
    void (*onEvent)(dwDevice_t *dev, uwbEvent_t event);
    bool (*getAnchorPosition)(const uint8_t anchorId, point_t* position);
    // TODO: remove the following two functions
    uint8_t (*getAnchorIdList)(uint8_t unorderedAnchorList[], const int maxListSize);
    uint8_t (*getActiveAnchorIdList)(uint8_t unorderedAnchorList[], const int maxListSize);
} uwbAlgorithm_t;

void dw1000_init();

#ifdef __cplusplus
}
#endif

#endif // __LOCO_H__