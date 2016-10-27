/* * OSS-7 - An opensource implementation of the DASH7 Alliance Protocol for ultra
 * lowpower wireless sensor communication
 *
 * Copyright 2015 University of Antwerp
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*! \file efm32pg1b_uart.c
 *
 *  \author jeremie@wizzilab.com
 *  \author maarten.weyn@uantwerpen.be
 *  \author contact@christophe.vg
 */

#include <string.h>

#include "em_usart.h"
#include "em_cmu.h"
#include "em_gpio.h"
//#include "em_usbd.h"
#include "em_gpio.h"
#include "hwgpio.h"
#include "hwuart.h"
#include <assert.h>
#include "hwsystem.h"
#include "efm32pg1b_pins.h"

#include "platform.h"

#define UARTS     3   // 2 USARTs + 1 LEUART
#define LOCATIONS 32

typedef struct {
  IRQn_Type  tx;
  IRQn_Type  rx;
} uart_irq_t;

typedef struct {
  uint32_t location;
  pin_id_t tx;
  pin_id_t rx;
} uart_pins_t;

#define UNDEFINED_LOCATION {                      \
  .location = 0,                                  \
  .tx       = { .port = 0,         .pin =  0 },   \
  .rx       = { .port = 0,         .pin =  0 }    \
}

// configuration of uart/location mapping to tx and rx pins
static uart_pins_t location[UARTS][LOCATIONS] = {
  {
    // USART 0
    {
      .location = 0,
      .tx     = { .port = gpioPortA, .pin = 0 },
      .rx     = { .port = gpioPortA, .pin = 1 }
    },
    {
      .location = 1,
      .tx     = { .port = gpioPortA, .pin = 1 },
      .rx     = { .port = gpioPortA, .pin = 2 }
    },
    {
      .location = 2,
      .tx     = { .port = gpioPortA, .pin = 2 },
      .rx     = { .port = gpioPortA, .pin = 3 }
    },
    {
      .location = 3,
      .tx     = { .port = gpioPortA, .pin = 3 },
      .rx     = { .port = gpioPortA, .pin = 4 }
    },
    {
      .location = 4,
      .tx     = { .port = gpioPortA, .pin = 4 },
      .rx     = { .port = gpioPortA, .pin = 5 }
    },
    {
      .location = 5,
      .tx     = { .port = gpioPortA, .pin = 5 },
      .rx     = { .port = gpioPortB, .pin = 11 }
    },
    {
      .location = 6,
      .tx     = { .port = gpioPortB, .pin = 11 },
      .rx     = { .port = gpioPortB, .pin = 12 }
    },
    {
      .location = 7,
      .tx     = { .port = gpioPortB, .pin = 12 },
      .rx     = { .port = gpioPortB, .pin = 13 }
    },
    {
      .location = 8,
      .tx     = { .port = gpioPortB, .pin = 13 },
      .rx     = { .port = gpioPortB, .pin = 14 }
    },
    {
      .location = 9,
      .tx     = { .port = gpioPortB, .pin = 14 },
      .rx     = { .port = gpioPortB, .pin = 15 }
    },
    {
      .location = 10,
      .tx     = { .port = gpioPortB, .pin = 15 },
      .rx     = { .port = gpioPortC, .pin = 6 }
    },
    {
      .location = 11,
      .tx     = { .port = gpioPortC, .pin = 6 },
      .rx     = { .port = gpioPortC, .pin = 7 }
    },
    {
      .location = 12,
      .tx     = { .port = gpioPortC, .pin = 7 },
      .rx     = { .port = gpioPortC, .pin = 8 }
    },
    {
      .location = 13,
      .tx     = { .port = gpioPortC, .pin = 8 },
      .rx     = { .port = gpioPortC, .pin = 9 }
    },
    {
      .location = 14,
      .tx     = { .port = gpioPortC, .pin = 9 },
      .rx     = { .port = gpioPortC, .pin = 10 }
    },
    {
      .location = 15,
      .tx     = { .port = gpioPortC, .pin = 10 },
      .rx     = { .port = gpioPortC, .pin = 11 }
    },
    {
      .location = 16,
      .tx     = { .port = gpioPortC, .pin = 11 },
      .rx     = { .port = gpioPortD, .pin = 9 }
    },
    {
      .location = 17,
      .tx     = { .port = gpioPortD, .pin = 9 },
      .rx     = { .port = gpioPortD, .pin = 10 }
    },
    {
      .location = 18,
      .tx     = { .port = gpioPortD, .pin = 10 },
      .rx     = { .port = gpioPortD, .pin = 11 }
    },
    {
      .location = 19,
      .tx     = { .port = gpioPortD, .pin = 11 },
      .rx     = { .port = gpioPortD, .pin = 12 }
    },
    {
      .location = 20,
      .tx     = { .port = gpioPortD, .pin = 12 },
      .rx     = { .port = gpioPortD, .pin = 13 }
    },
    {
      .location = 21,
      .tx     = { .port = gpioPortD, .pin = 13 },
      .rx     = { .port = gpioPortD, .pin = 14 }
    },
    {
      .location = 22,
      .tx     = { .port = gpioPortD, .pin = 14 },
      .rx     = { .port = gpioPortD, .pin = 15 }
    },
    {
      .location = 23,
      .tx     = { .port = gpioPortD, .pin = 15 },
      .rx     = { .port = gpioPortF, .pin = 0 }
    },
    {
      .location = 24,
      .tx     = { .port = gpioPortF, .pin = 0 },
      .rx     = { .port = gpioPortF, .pin = 1 }
    },
    {
      .location = 25,
      .tx     = { .port = gpioPortF, .pin = 1 },
      .rx     = { .port = gpioPortF, .pin = 2 }
    },
    {
      .location = 26,
      .tx     = { .port = gpioPortF, .pin = 2 },
      .rx     = { .port = gpioPortF, .pin = 3 }
    },
    {
      .location = 27,
      .tx     = { .port = gpioPortF, .pin = 3 },
      .rx     = { .port = gpioPortF, .pin = 4 }
    },
    {
      .location = 28,
      .tx     = { .port = gpioPortF, .pin = 4 },
      .rx     = { .port = gpioPortF, .pin = 5 }
    },
    {
      .location = 29,
      .tx     = { .port = gpioPortF, .pin = 5 },
      .rx     = { .port = gpioPortF, .pin = 6 }
    },
    {
      .location = 30,
      .tx     = { .port = gpioPortF, .pin = 6 },
      .rx     = { .port = gpioPortF, .pin = 7 }
    },
    {
      .location = 31,
      .tx     = { .port = gpioPortF, .pin = 7 },
      .rx     = { .port = gpioPortA, .pin = 0 }
    }
  },
  {
    // USART 1
    {
      .location = 0,
      .tx     = { .port = gpioPortA, .pin = 0 },
      .rx     = { .port = gpioPortA, .pin = 1 }
    },
    {
      .location = 1,
      .tx     = { .port = gpioPortA, .pin = 1 },
      .rx     = { .port = gpioPortA, .pin = 2 }
    },
    {
      .location = 2,
      .tx     = { .port = gpioPortA, .pin = 2 },
      .rx     = { .port = gpioPortA, .pin = 3 }
    },
    {
      .location = 3,
      .tx     = { .port = gpioPortA, .pin = 3 },
      .rx     = { .port = gpioPortA, .pin = 4 }
    },
    {
      .location = 4,
      .tx     = { .port = gpioPortA, .pin = 4 },
      .rx     = { .port = gpioPortA, .pin = 5 }
    },
    {
      .location = 5,
      .tx     = { .port = gpioPortA, .pin = 5 },
      .rx     = { .port = gpioPortB, .pin = 11 }
    },
    {
      .location = 6,
      .tx     = { .port = gpioPortB, .pin = 11 },
      .rx     = { .port = gpioPortB, .pin = 12 }
    },
    {
      .location = 7,
      .tx     = { .port = gpioPortB, .pin = 12 },
      .rx     = { .port = gpioPortB, .pin = 13 }
    },
    {
      .location = 8,
      .tx     = { .port = gpioPortB, .pin = 13 },
      .rx     = { .port = gpioPortB, .pin = 14 }
    },
    {
      .location = 9,
      .tx     = { .port = gpioPortB, .pin = 14 },
      .rx     = { .port = gpioPortB, .pin = 15 }
    },
    {
      .location = 10,
      .tx     = { .port = gpioPortB, .pin = 15 },
      .rx     = { .port = gpioPortC, .pin = 6 }
    },
    {
      .location = 11,
      .tx     = { .port = gpioPortC, .pin = 6 },
      .rx     = { .port = gpioPortC, .pin = 7 }
    },
    {
      .location = 12,
      .tx     = { .port = gpioPortC, .pin = 7 },
      .rx     = { .port = gpioPortC, .pin = 8 }
    },
    {
      .location = 13,
      .tx     = { .port = gpioPortC, .pin = 8 },
      .rx     = { .port = gpioPortC, .pin = 9 }
    },
    {
      .location = 14,
      .tx     = { .port = gpioPortC, .pin = 9 },
      .rx     = { .port = gpioPortC, .pin = 10 }
    },
    {
      .location = 15,
      .tx     = { .port = gpioPortC, .pin = 10 },
      .rx     = { .port = gpioPortC, .pin = 11 }
    },
    {
      .location = 16,
      .tx     = { .port = gpioPortC, .pin = 11 },
      .rx     = { .port = gpioPortD, .pin = 9 }
    },
    {
      .location = 17,
      .tx     = { .port = gpioPortD, .pin = 9 },
      .rx     = { .port = gpioPortD, .pin = 10 }
    },
    {
      .location = 18,
      .tx     = { .port = gpioPortD, .pin = 10 },
      .rx     = { .port = gpioPortD, .pin = 11 }
    },
    {
      .location = 19,
      .tx     = { .port = gpioPortD, .pin = 11 },
      .rx     = { .port = gpioPortD, .pin = 12 }
    },
    {
      .location = 20,
      .tx     = { .port = gpioPortD, .pin = 12 },
      .rx     = { .port = gpioPortD, .pin = 13 }
    },
    {
      .location = 21,
      .tx     = { .port = gpioPortD, .pin = 13 },
      .rx     = { .port = gpioPortD, .pin = 14 }
    },
    {
      .location = 22,
      .tx     = { .port = gpioPortD, .pin = 14 },
      .rx     = { .port = gpioPortD, .pin = 15 }
    },
    {
      .location = 23,
      .tx     = { .port = gpioPortD, .pin = 15 },
      .rx     = { .port = gpioPortF, .pin = 0 }
    },
    {
      .location = 24,
      .tx     = { .port = gpioPortF, .pin = 0 },
      .rx     = { .port = gpioPortF, .pin = 1 }
    },
    {
      .location = 25,
      .tx     = { .port = gpioPortF, .pin = 1 },
      .rx     = { .port = gpioPortF, .pin = 2 }
    },
    {
      .location = 26,
      .tx     = { .port = gpioPortF, .pin = 2 },
      .rx     = { .port = gpioPortF, .pin = 3 }
    },
    {
      .location = 27,
      .tx     = { .port = gpioPortF, .pin = 3 },
      .rx     = { .port = gpioPortF, .pin = 4 }
    },
    {
      .location = 28,
      .tx     = { .port = gpioPortF, .pin = 4 },
      .rx     = { .port = gpioPortF, .pin = 5 }
    },
    {
      .location = 29,
      .tx     = { .port = gpioPortF, .pin = 5 },
      .rx     = { .port = gpioPortF, .pin = 6 }
    },
    {
      .location = 30,
      .tx     = { .port = gpioPortF, .pin = 6 },
      .rx     = { .port = gpioPortF, .pin = 7 }
    },
    {
      .location = 31,
      .tx     = { .port = gpioPortF, .pin = 7 },
      .rx     = { .port = gpioPortA, .pin = 0 }
    }
  },
  {
    // LEUART0
    {
      .location = 0,
      .tx     = { .port = gpioPortA, .pin = 0 },
      .rx     = { .port = gpioPortA, .pin = 1 }
    },
    {
      .location = 1,
      .tx     = { .port = gpioPortA, .pin = 1 },
      .rx     = { .port = gpioPortA, .pin = 2 }
    },
    {
      .location = 2,
      .tx     = { .port = gpioPortA, .pin = 2 },
      .rx     = { .port = gpioPortA, .pin = 3 }
    },
    {
      .location = 3,
      .tx     = { .port = gpioPortA, .pin = 3 },
      .rx     = { .port = gpioPortA, .pin = 4 }
    },
    {
      .location = 4,
      .tx     = { .port = gpioPortA, .pin = 4 },
      .rx     = { .port = gpioPortA, .pin = 5 }
    },
    {
      .location = 5,
      .tx     = { .port = gpioPortA, .pin = 5 },
      .rx     = { .port = gpioPortB, .pin = 11 }
    },
    {
      .location = 6,
      .tx     = { .port = gpioPortB, .pin = 11 },
      .rx     = { .port = gpioPortB, .pin = 12 }
    },
    {
      .location = 7,
      .tx     = { .port = gpioPortB, .pin = 12 },
      .rx     = { .port = gpioPortB, .pin = 13 }
    },
    {
      .location = 8,
      .tx     = { .port = gpioPortB, .pin = 13 },
      .rx     = { .port = gpioPortB, .pin = 14 }
    },
    {
      .location = 9,
      .tx     = { .port = gpioPortB, .pin = 14 },
      .rx     = { .port = gpioPortB, .pin = 15 }
    },
    {
      .location = 10,
      .tx     = { .port = gpioPortB, .pin = 15 },
      .rx     = { .port = gpioPortC, .pin = 6 }
    },
    {
      .location = 11,
      .tx     = { .port = gpioPortC, .pin = 6 },
      .rx     = { .port = gpioPortC, .pin = 7 }
    },
    {
      .location = 12,
      .tx     = { .port = gpioPortC, .pin = 7 },
      .rx     = { .port = gpioPortC, .pin = 8 }
    },
    {
      .location = 13,
      .tx     = { .port = gpioPortC, .pin = 8 },
      .rx     = { .port = gpioPortC, .pin = 9 }
    },
    {
      .location = 14,
      .tx     = { .port = gpioPortC, .pin = 9 },
      .rx     = { .port = gpioPortC, .pin = 10 }
    },
    {
      .location = 15,
      .tx     = { .port = gpioPortC, .pin = 10 },
      .rx     = { .port = gpioPortC, .pin = 11 }
    },
    {
      .location = 16,
      .tx     = { .port = gpioPortC, .pin = 11 },
      .rx     = { .port = gpioPortD, .pin = 9 }
    },
    {
      .location = 17,
      .tx     = { .port = gpioPortD, .pin = 9 },
      .rx     = { .port = gpioPortD, .pin = 10 }
    },
    {
      .location = 18,
      .tx     = { .port = gpioPortD, .pin = 10 },
      .rx     = { .port = gpioPortD, .pin = 11 }
    },
    {
      .location = 19,
      .tx     = { .port = gpioPortD, .pin = 11 },
      .rx     = { .port = gpioPortD, .pin = 12 }
    },
    {
      .location = 20,
      .tx     = { .port = gpioPortD, .pin = 12 },
      .rx     = { .port = gpioPortD, .pin = 13 }
    },
    {
      .location = 21,
      .tx     = { .port = gpioPortD, .pin = 13 },
      .rx     = { .port = gpioPortD, .pin = 14 }
    },
    {
      .location = 22,
      .tx     = { .port = gpioPortD, .pin = 14 },
      .rx     = { .port = gpioPortD, .pin = 15 }
    },
    {
      .location = 23,
      .tx     = { .port = gpioPortD, .pin = 15 },
      .rx     = { .port = gpioPortF, .pin = 0 }
    },
    {
      .location = 24,
      .tx     = { .port = gpioPortF, .pin = 0 },
      .rx     = { .port = gpioPortF, .pin = 1 }
    },
    {
      .location = 25,
      .tx     = { .port = gpioPortF, .pin = 1 },
      .rx     = { .port = gpioPortF, .pin = 2 }
    },
    {
      .location = 26,
      .tx     = { .port = gpioPortF, .pin = 2 },
      .rx     = { .port = gpioPortF, .pin = 3 }
    },
    {
      .location = 27,
      .tx     = { .port = gpioPortF, .pin = 3 },
      .rx     = { .port = gpioPortF, .pin = 4 }
    },
    {
      .location = 28,
      .tx     = { .port = gpioPortF, .pin = 4 },
      .rx     = { .port = gpioPortF, .pin = 5 }
    },
    {
      .location = 29,
      .tx     = { .port = gpioPortF, .pin = 5 },
      .rx     = { .port = gpioPortF, .pin = 6 }
    },
    {
      .location = 30,
      .tx     = { .port = gpioPortF, .pin = 6 },
      .rx     = { .port = gpioPortF, .pin = 7 }
    },
    {
      .location = 31,
      .tx     = { .port = gpioPortF, .pin = 7 },
      .rx     = { .port = gpioPortA, .pin = 0 }
    }
  }
};

// references to registered handlers
static uart_rx_inthandler_t handler[UARTS];

// private definition of the UART handle, passed around publicly as a pointer
struct uart_handle {
  uint8_t              idx;
  USART_TypeDef*       channel;
  CMU_Clock_TypeDef    clock;
  uart_irq_t           irq;
  uart_pins_t*         pins;
  uint32_t             baudrate;
};

// private storage of handles, pointers to these records are passed around
static uart_handle_t handle[UARTS] = {
  {
    .idx     = 0,
    .channel = USART0,
    .clock   = cmuClock_USART0,
    .irq     = { .tx = USART0_TX_IRQn, .rx = USART0_RX_IRQn }
  },
  {
    .idx     = 1,
    .channel = USART1,
    .clock   = cmuClock_USART1,
    .irq     = { .tx = USART1_TX_IRQn, .rx = USART1_RX_IRQn }
  },
  {
    .idx     = 2,
    .channel = (USART_TypeDef *)LEUART0,
    .clock   = cmuClock_LEUART0,
    .irq     = { .tx = LEUART0_IRQn, .rx = LEUART0_IRQn }
  },
};

uart_handle_t* uart_init(uint8_t idx, uint32_t baudrate, uint8_t pins) {
  // configure pins
  handle[idx].pins     = &location[idx][pins];
  handle[idx].baudrate = baudrate;
  
  // configure UART TX pin as digital output
  hw_gpio_configure_pin(handle[idx].pins->tx, false, gpioModePushPull, 0);
  // configure UART RX pin as input (no filter)
  hw_gpio_configure_pin(handle[idx].pins->rx, false, gpioModeInput, 0);

  return &handle[idx];
}

bool uart_enable(uart_handle_t* uart) {
  // CMU_ClockEnable(cmuClock_GPIO,    true); // TODO future use: hw_gpio_enable
  CMU_ClockEnable(uart->clock, true);

  USART_InitAsync_TypeDef uartInit = {
    .enable       = usartDisable,   // wait to enable the transceiver
    .refFreq      = 0,              // setting refFreq to 0 will invoke the
                                    // CMU_ClockFreqGet() function and measure
                                    // the HFPER clock
    .baudrate     = uart->baudrate, // desired baud rate
    .oversampling = usartOVS16,     // set oversampling value to x16
    .databits     = usartDatabits8, // 8 data bits
    .parity       = usartNoParity,  // no parity bits
    .stopbits     = usartStopbits1, // 1 stop bit
    .mvdis        = false,          // use majority voting
    .prsRxEnable  = false,          // not using PRS input
    .prsRxCh      = usartPrsRxCh0,  // doesn't matter which channel we select
  };

  USART_InitAsync(uart->channel, &uartInit);
  
  // clear RX/TX buffers and shift regs, enable transmitter and receiver pins
  uart->channel->ROUTEPEN = USART_ROUTEPEN_RXPEN | USART_ROUTEPEN_TXPEN;
  uart->channel->ROUTELOC0 = (uart->pins->location << _USART_ROUTELOC0_RXLOC_SHIFT) | (uart->pins->location << _USART_ROUTELOC0_TXLOC_SHIFT);
  USART_IntClear(uart->channel, (uart->idx == 2) ? _LEUART_IF_MASK : _USART_IF_MASK);
  NVIC_ClearPendingIRQ(uart->irq.rx);
  NVIC_ClearPendingIRQ(uart->irq.tx);

  USART_Enable(uart->channel, usartEnable);

  return true;
}

bool uart_disable(uart_handle_t* uart) {
  // reset route to make sure that TX pin will become low after disable
  uart->channel->ROUTEPEN = _USART_ROUTEPEN_RESETVALUE;

  USART_Enable(uart->channel, usartDisable);
  CMU_ClockEnable(uart->clock, false);
  // CMU_ClockEnable(cmuClock_GPIO, false); // TODO future use: hw_gpio_disable

  return true;
}

void uart_set_rx_interrupt_callback(uart_handle_t* uart,
                                    uart_rx_inthandler_t rx_handler)
{
  handler[uart->idx] = rx_handler;
}

void uart_send_byte(uart_handle_t* uart, uint8_t data) {
#ifdef PLATFORM_USE_USB_CDC
		uint16_t timeout = 0;
		while(USBD_EpIsBusy(0x81) && timeout < 100){
			timeout++;
			hw_busy_wait(1000);
		};
		uint32_t tempData = data;
		int ret = USBD_Write( 0x81, (void*) &tempData, 1, NULL);
#else
  while(!(uart->channel->STATUS & (1 << 6))); // wait for TX buffer to empty
	uart->channel->TXDATA = data;
#endif
}

void uart_send_bytes(uart_handle_t* uart, void const *data, size_t length) {
#ifdef PLATFORM_USE_USB_CDC
    // print misaliged bytes first as individual bytes.
		int8_t* tempData = (int8_t*) data;
		while(((uint32_t)tempData & 3) && (length > 0)) {
			uart_send_byte(uart, tempData[0]);
			tempData++;
			length--;
		}

		if (length > 0)
		{
			uint16_t timeout = 0;
			while(USBD_EpIsBusy(0x81) && timeout < 100){
				timeout++;
				hw_busy_wait(1000);
			};
			int ret = USBD_Write( 0x81, (void*) tempData, length, NULL);
		}
#else
  for(size_t i = 0; i < length; i++)	{
		uart_send_byte(uart, ((uint8_t const*)data)[i]);
	}
#endif
}

void uart_send_string(uart_handle_t* uart, const char *string) {
  uart_send_bytes(uart, string, strnlen(string, 100));
}

error_t uart_rx_interrupt_enable(uart_handle_t* uart) {
  if(handler[uart->idx] == NULL) { return EOFF; }
  USART_IntClear(uart->channel, (uart->idx == 2) ? _LEUART_IF_MASK : _USART_IF_MASK);
  USART_IntEnable(uart->channel, (uart->idx == 2) ? _LEUART_IF_RXDATAV_MASK : _USART_IF_RXDATAV_MASK);
  NVIC_ClearPendingIRQ(uart->irq.tx);
  NVIC_ClearPendingIRQ(uart->irq.rx);
  NVIC_EnableIRQ(uart->irq.rx);
  return SUCCESS;
}

void uart_rx_interrupt_disable(uart_handle_t* uart) {
  USART_IntClear(uart->channel, (uart->idx == 2) ? _LEUART_IF_MASK : _USART_IF_MASK);
  USART_IntDisable(uart->channel, (uart->idx == 2) ? _LEUART_IF_RXDATAV_MASK : _USART_IF_RXDATAV_MASK);
  NVIC_ClearPendingIRQ(uart->irq.rx);
  NVIC_ClearPendingIRQ(uart->irq.tx);
  NVIC_DisableIRQ(uart->irq.rx);
}

void USART0_RX_IRQHandler(void) {
  if(handle[2].channel->STATUS & USART_STATUS_RXDATAV) {
    handler[2](USART_Rx(handle[2].channel));
    USART_IntClear(handle[2].channel, USART_IF_RXDATAV);
  }
}

void USART1_RX_IRQHandler(void) {
  if(handle[3].channel->STATUS & USART_STATUS_RXDATAV) {
    handler[3](USART_Rx(handle[3].channel));
    USART_IntClear(handle[3].channel, USART_IF_RXDATAV);
  }
}

void LEUART0_RX_IRQHandler(void) {
  if(handle[0].channel->STATUS & LEUART_STATUS_RXDATAV) {
    handler[0](USART_Rx(handle[0].channel));
    USART_IntClear(handle[0].channel, LEUART_IF_RXDATAV);
  }
}
