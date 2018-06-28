#include "hal.h"
#include "aseba_bridge.h"
#include "aseba_can_interface.h"

#include "can-net.h"
#include "consts.h"
#include "main.h"
#include "chprintf.h"
#include "leds.h"

static bool is_bridge = false;

static void aseba_bridge_uart_to_can(void *p);
static void aseba_bridge_can_to_uart(void *p);

typedef union {
    uint8_t u8[2];
    uint16_t u16;
} uint16_8_t;

void aseba_bridge(void *stream)
{
    is_bridge = true;
    static THD_WORKING_AREA(uart_to_can_wa, 1024);
    chThdCreateStatic(uart_to_can_wa, sizeof(uart_to_can_wa), NORMALPRIO,
                      aseba_bridge_uart_to_can, (void *)stream);

    static THD_WORKING_AREA(can_to_uart_wa, 1024);
    chThdCreateStatic(can_to_uart_wa, sizeof(can_to_uart_wa), NORMALPRIO,
                      aseba_bridge_can_to_uart, (void *)stream);

}
static THD_FUNCTION(aseba_bridge_uart_to_can, arg)
{
    chRegSetThreadName("aseba uart -> can");
    BaseSequentialStream *stream = (BaseSequentialStream *)arg;

    uint16_8_t source, length;
    uint8_t data[ASEBA_MAX_PACKET_SIZE];

    while (true) {
        streamRead(stream, length.u8, sizeof(length));
        streamRead(stream, source.u8, sizeof(source));
        streamRead(stream, data, length.u16 + 2);

        aseba_can_lock();
        AsebaCanSendSpecificSource(data, length.u16 + 2, source.u16);
        aseba_can_unlock();
    }
}

static THD_FUNCTION(aseba_bridge_can_to_uart, arg)
{
    chRegSetThreadName("aseba can -> uart");
    BaseSequentialStream *stream = (BaseSequentialStream *)arg;
    uint16_8_t source, length;
    uint8_t data[ASEBA_MAX_PACKET_SIZE];

    while (true) {
        length.u16 = AsebaCanRecv(data,
                                  ASEBA_MAX_INNER_PACKET_SIZE,
                                  &source.u16);

        if (length.u16 > 0) {
            setLed(BLUE_LED, LED_NO_POWER);
            /* Aseba transmits length minus the type. */
            length.u16 -= 2;

            streamWrite(stream, length.u8, sizeof(source));
            streamWrite(stream, source.u8, sizeof(source));
            streamWrite(stream, data, length.u16 + 2);
        }
        chThdSleepMilliseconds(1);
    }
}

uint16 AsebaShouldDropPacket(uint16 source, const uint8* data) {
    (void)source;
    (void)data;
    return 0;
}

bool aseba_is_bridge(void)
{
    return is_bridge;
}
