/*
 * emsuart.cpp
 *
 * The low level UART code for ESP8266 to read and write to the EMS bus via uart
 * Paul Derbyshire - https://github.com/proddy/EMS-ESP
 */

#include "emsuart.h"
#include "ems.h"
#include "ets_sys.h"
#include "osapi.h"

#include <Arduino.h>
#include <user_interface.h>

_EMSRxBuf emsRxBufs[EMS_MAXBUFFERS];
volatile int8_t emsRxBufReadIdx;
volatile int8_t emsRxBufWriteIdx;

os_event_t recvTaskQueue[EMSUART_recvTaskQueueLen]; // our Rx queue

//
// Main interrupt handler
// Important: do not use ICACHE_FLASH_ATTR !
//
static void ICACHE_RAM_ATTR emsuart_rx_intr_handler(void * para) {
    static uint8_t length;
    static uint8_t uart_buffer[EMS_MAXBUFFERSIZE];
    bool complete = false;

    // is a new buffer? if so init the thing for a new telegram
    if (EMS_Sys_Status.emsRxStatus == EMS_RX_STATUS_IDLE) {
        EMS_Sys_Status.emsRxStatus = EMS_RX_STATUS_BUSY; // status set to busy
        length                     = 0;
    }

    // fill IRQ buffer, by emptying Rx FIFO
    while ((USS(EMSUART_UART) >> USRXC) & 0xFF) {
        if (length == EMS_MAXBUFFERSIZE) {
            EMS_Sys_Status.emsRxStatus = EMS_RX_STATUS_ERROR;
            length = 0;
        }
        uart_buffer[length++] = USF(EMSUART_UART);
    }

    if (length == 1 && uart_buffer[0] != 0 && (uart_buffer[0] & 0x80) == 0) {
        uart_buffer[length++] = 0;
        complete = true;
    }

    // clear Rx FIFO full and Rx FIFO timeout interrupts
    if (U0IS & (1 << UIFF)) {
        U0IC = (1 << UIFF);
    }
    if (U0IS & (1 << UITO)) {
        U0IC = (1 << UITO);
    }
    if (U0IS & (1 << UIOF)) {
        U0IC = (1 << UIOF);
        // UART FIFO overflowed, discard the rest of the message
        EMS_Sys_Status.emsRxStatus = EMS_RX_STATUS_ERROR;
        length = 0;
    }

    // BREAK detection = End of EMS data block
    if (USIS(EMSUART_UART) & ((1 << UIBD))) {
        U0IC = (1 << UIBD); // INT clear the BREAK detect interrupt
        complete = true;
    }

    if (complete) {
        if (length > 1) {
            _EMSRxBuf * pEMSRxBuf = &emsRxBufs[emsRxBufWriteIdx];

            // copy data into transfer buffer, excluding the break
            os_memcpy(pEMSRxBuf->buffer, &uart_buffer, length - 1);

            if (emsRxBufReadIdx < 0) {
                // first message
                emsRxBufReadIdx = emsRxBufWriteIdx;
            } else if (pEMSRxBuf->length > 0 && emsRxBufReadIdx == emsRxBufWriteIdx) {
                // circular buffer overrun, move read index to the next oldest message
                emsRxBufReadIdx = (emsRxBufReadIdx + 1) % EMS_MAXBUFFERS;
            }

            pEMSRxBuf->length = length - 1;
            emsRxBufWriteIdx = (emsRxBufWriteIdx + 1) % EMS_MAXBUFFERS;

            // call emsuart_recvTask() at next opportunity
            system_os_post(EMSUART_recvTaskPrio, 0, 0);
        }

        // set the status flag stating BRK has been received and we can start a new package
        EMS_Sys_Status.emsRxStatus = EMS_RX_STATUS_IDLE;
    }
}

/*
 * system task triggered on BRK interrupt
 * Read commands are all asynchronous
 * When a buffer is full it is sent to the ems_parseTelegram() function in ems.cpp. This is the hook
 */
static void ICACHE_RAM_ATTR emsuart_recvTask(os_event_t * events) {
    _EMSRxBuf copy = { .length = 0 };

    // temporarily disable interrupt handler that would write to the buffers
    ETS_UART_INTR_DISABLE();

    if (emsRxBufReadIdx >= 0) {
        // get next free EMS Receive buffer
        _EMSRxBuf * pCurrent = &emsRxBufs[emsRxBufReadIdx];

        if (pCurrent->length > 0) {
            // copy the current buffer
            os_memcpy(copy.buffer, pCurrent->buffer, pCurrent->length);
            copy.length = pCurrent->length;

            // empty buffer and move to the next one
            pCurrent->length = 0;
            emsRxBufReadIdx = (emsRxBufReadIdx + 1) % EMS_MAXBUFFERS;
        }
    }

    // re-enable interrupt handler before processing the copied data
    ETS_UART_INTR_ENABLE();

    ems_parseTelegram(copy.buffer, copy.length);
}

/*
 * init UART0 driver
 */
void ICACHE_FLASH_ATTR emsuart_init() {
    ETS_UART_INTR_DISABLE();
    ETS_UART_INTR_ATTACH(NULL, NULL);

    // reset EMS Receive buffers
    for (int i = 0; i < EMS_MAXBUFFERS; i++) {
        emsRxBufs[i].length = 0;
    }
    emsRxBufReadIdx = -1;
    emsRxBufWriteIdx = 0;

    // pin settings
    PIN_PULLUP_DIS(PERIPHS_IO_MUX_U0TXD_U);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_U0TXD);
    PIN_PULLUP_DIS(PERIPHS_IO_MUX_U0RXD_U);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0RXD_U, FUNC_U0RXD);

    // set 9600, 8 bits, no parity check, 1 stop bit
    USD(EMSUART_UART)  = (UART_CLK_FREQ / EMSUART_BAUD);
    USC0(EMSUART_UART) = EMSUART_CONFIG; // 8N1

    // flush everything left over in buffer, this clears both rx and tx FIFOs
    uint32_t tmp = ((1 << UCRXRST) | (1 << UCTXRST)); // bit mask
    USC0(EMSUART_UART) |= (tmp);                      // set bits
    USC0(EMSUART_UART) &= ~(tmp);                     // clear bits

    // conf1 params
    // UCTOE = RX TimeOut enable (default is 1)
    // UCTOT = RX TimeOut Threshold (7bit) = want this when no more data after 2 characters. (default is 2)
    // UCFFT = RX FIFO Full Threshold (7 bit) = want this to be 32 for a maximum size packet. (default was 127).
    USC1(EMSUART_UART) = 0;                                              // reset config first
    USC1(EMSUART_UART) = (1 << UCFFT) | (0 << UCTOE); // enable interupts

    // set interrupts for triggers
    USIC(EMSUART_UART) = 0xffff; // clear all interupts
    USIE(EMSUART_UART) = 0;      // disable all interrupts

    // enable rx break, fifo full, fifo overflow and timeout.
    // not frame error UIFR (because they are too frequent)
    USIE(EMSUART_UART) = (1 << UIBD) | (1 << UIFF) | (1 << UIOF);

    // set up interrupt callbacks for Rx
    system_os_task(emsuart_recvTask, EMSUART_recvTaskPrio, recvTaskQueue, EMSUART_recvTaskQueueLen);

    // disable esp debug which will go to Tx and mess up the line
    // system_set_os_print(0); // https://github.com/espruino/Espruino/issues/655

    ETS_UART_INTR_ATTACH(emsuart_rx_intr_handler, NULL);
    ETS_UART_INTR_ENABLE();

    // swap Rx and Tx pins to use GPIO13 (D7) and GPIO15 (D8) respectively
    system_uart_swap();
}

/*
 * stop UART0 driver
 */
void ICACHE_FLASH_ATTR emsuart_stop() {
    ETS_UART_INTR_DISABLE();
    ETS_UART_INTR_ATTACH(NULL, NULL);
}

/*
 * Send a BRK signal
 * Which is a 11-bit set of zero's (11 cycles)
 */
void ICACHE_FLASH_ATTR emsuart_tx_brk() {
    // must make sure Tx FIFO is empty
    while (((USS(EMSUART_UART) >> USTXC) & 0xff) != 0)
        ;

    // To create a 11-bit <BRK> we set TXD_BRK bit so the break signal will
    // automatically be sent when the tx fifo is empty
    USC0(EMSUART_UART) |= (1 << UCBRK);  // set bit
    delayMicroseconds(EMS_Sys_Status.emsBreakTime);  // 2070 - based on trial and error using an oscilloscope
    USC0(EMSUART_UART) &= ~(1 << UCBRK); // clear bit
}

/*
 * Send to Tx, ending with a <BRK>
 */
void ICACHE_FLASH_ATTR emsuart_tx_buffer(uint8_t * buf, uint8_t len) {
    for (uint8_t i = 0; i < len; i++) {
        USF(EMSUART_UART) = buf[i];
    }
    emsuart_tx_brk();
}

/*
 * Send the Poll (our own ID) to Tx as a single byte and end with a <BRK>
 */
void ICACHE_FLASH_ATTR emsaurt_tx_poll() {
    USF(EMSUART_UART) = 0x80 | EMS_Sys_Status.emsMyId;
    emsuart_tx_brk();
}
