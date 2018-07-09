/**
 * @file    aseba_can_interface.c
 * @brief   USB Serial to CAN Aseba bridge functions
 * 
 * @source          adapted from aseba_can_interface.c of e-puck2_main-processor project
 * @written by      Eliot Ferragni
 * @creation date   28.06.2018
 */

#include "ch.h"
#include "hal.h"

#include "can-net.h"
#include "vm.h"

#include "aseba_can_interface.h"

#define ASEBA_CAN_SEND_QUEUE_SIZE       1024
#define ASEBA_CAN_RECEIVE_QUEUE_SIZE    1024



CanFrame aseba_can_send_queue[ASEBA_CAN_SEND_QUEUE_SIZE];
CanFrame aseba_can_receive_queue[ASEBA_CAN_RECEIVE_QUEUE_SIZE];

static THD_WORKING_AREA(can_rx_thread_wa, 256);
static THD_FUNCTION(can_rx_thread, arg)
{
    (void)arg;
    chRegSetThreadName("CAN rx");
    while (1) {
        CANRxFrame rxf;
        CanFrame aseba_can_frame;
        msg_t m = canReceive(&CAN_ASEBA, CAN_ANY_MAILBOX, &rxf, TIME_MS2I(1000));
        if (m != MSG_OK) {
            continue;
        }
        if (rxf.IDE) {
            continue; // no extended id frames
        }
        if (rxf.RTR) {
            continue; // no remote transmission request frames
        }
        aseba_can_frame.id = rxf.SID;
        aseba_can_frame.len = rxf.DLC;
        int i;
        for (i = 0; i < aseba_can_frame.len; i++) {
            aseba_can_frame.data[i] = rxf.data8[i];
        }
        AsebaCanFrameReceived(&aseba_can_frame);
    }
}

void can_init(void)
{

    /* CAN cell init.
     * Setting the bitrate to 1MBit. APB1 = 48MHz,
     * prescaler = 3 -> 16MHz time quanta frequency.
     * 1tq sync + 9tq bit segment1 (TS1) + 6tq bit segment2 (TS2) =
     * 16time quanta per bit period, therefor 16MHz/16 = 1MHz
     */
    static const CANConfig can1_config = {
        .mcr = CAN_MCR_ABOM | CAN_MCR_TXFP,
        .btr = CAN_BTR_SJW(1-1) | CAN_BTR_TS1(9-1) | CAN_BTR_TS2(6-1) | CAN_BTR_BRP(3-1)
    };

    canStart(&CAN_ASEBA, &can1_config);
}

void aseba_can_rx_dropped(void)
{
}

void aseba_can_tx_dropped(void)
{
}

void aseba_can_send_frame(const CanFrame *frame)
{
    aseba_can_lock();
    CANTxFrame txf;
    txf.DLC = frame->len;
    txf.RTR = 0;
    txf.IDE = 0;
    txf.SID = frame->id;

    int i;
    for (i = 0; i < frame->len; i++) {
        txf.data8[i] = frame->data[i];
    }

    canTransmit(&CAN_ASEBA, CAN_ANY_MAILBOX, &txf, TIME_MS2I(100));
    chThdSleepMilliseconds(1);

    AsebaCanFrameSent();
    aseba_can_unlock();
}

// Returns true if there is enough space to send the frame
int aseba_can_is_frame_room(void)
{
    return can_lld_is_tx_empty(&CAN_ASEBA, CAN_ANY_MAILBOX);
}

void aseba_can_start(uint16 id)
{
    can_init();
    chThdCreateStatic(can_rx_thread_wa,
                      sizeof(can_rx_thread_wa),
                      NORMALPRIO + 1,
                      can_rx_thread,
                      NULL);
    AsebaCanInit(id, aseba_can_send_frame, aseba_can_is_frame_room,
                 aseba_can_rx_dropped, aseba_can_tx_dropped,
                 aseba_can_send_queue, ASEBA_CAN_SEND_QUEUE_SIZE,
                 aseba_can_receive_queue, ASEBA_CAN_RECEIVE_QUEUE_SIZE);
}

static MUTEX_DECL(can_lock);

void aseba_can_lock(void)
{
    chMtxLock(&can_lock);
}

void aseba_can_unlock(void)
{
    chMtxUnlock(&can_lock);
}
