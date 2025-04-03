#ifndef __LOCO_H__
#define __LOCO_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/spi_master.h"
#define SPI_HOST    SPI2_HOST  // Use SPI2 or SPI3, SPI1 is typically for flash
#define PIN_NUM_MISO GPIO_NUM_8
#define PIN_NUM_MOSI GPIO_NUM_9
#define PIN_NUM_CLK  GPIO_NUM_7
#define PIN_NUM_CS   GPIO_NUM_3
#define PIN_NUM_IRQ  GPIO_NUM_4
#define PIN_NUM_RST  GPIO_NUM_6

/* x,y,z vector */
struct vec3_s {
    uint32_t timestamp; // Timestamp when the data was computed
    float x;
    float y;
    float z;
};

typedef struct vec3_s point_t;

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

typedef enum {
    lpsMode_auto = 0,
    lpsMode_TWR = 1,
    lpsMode_TDoA2 = 2,
    lpsMode_TDoA3 = 3,
} lpsMode_t;

typedef struct {
    uint8_t dest;
    uint8_t length;
    uint8_t data[30];
} lpsLppShortPacket_t;

typedef struct {
    // The status of anchors. A bit field (bit 0 - anchor 0, bit 1 - anchor 1 and so on)
    // where a set bit indicates that an anchor reentry has been detected
    volatile uint16_t rangingState;
  
    // Requested and current ranging mode
    lpsMode_t userRequestedMode;
    lpsMode_t currentRangingMode;
  
    // State of the ranging mode auto detection
    bool modeAutoSearchDoInitialize;
    bool modeAutoSearchActive;
    uint32_t nextSwitchTick;
} lpsAlgoOptions_t;

bool locoDeckGetAnchorPosition(const uint8_t anchorId, point_t* position);
uint8_t locoDeckGetAnchorIdList(uint8_t unorderedAnchorList[], const int maxListSize);
uint8_t locoDeckGetActiveAnchorIdList(uint8_t unorderedAnchorList[], const int maxListSize);

// Callbacks for uwb algorithms
typedef struct uwbAlgorithm_s {
    void (*init)(dwDevice_t *dev);
    uint32_t (*onEvent)(dwDevice_t *dev, uwbEvent_t event);
    bool (*isRangingOk)();
    bool (*getAnchorPosition)(const uint8_t anchorId, point_t* position);
    uint8_t (*getAnchorIdList)(uint8_t unorderedAnchorList[], const int maxListSize);
    uint8_t (*getActiveAnchorIdList)(uint8_t unorderedAnchorList[], const int maxListSize);
} uwbAlgorithm_t;

typedef struct {
    const locoAddress_t anchorAddress[LOCODECK_NR_OF_TDOA2_ANCHORS];

    point_t anchorPosition[LOCODECK_NR_OF_TDOA2_ANCHORS];
} lpsTdoa2AlgoOptions_t;


void dw1000_init();

#ifdef __cplusplus
}
#endif

#endif // __LOCO_H__