// SPDX-License-Identifier: MIT
/*
 * Copyright 2021 Álvaro Fernández Rojas <noltari@gmail.com>
 */

#include <hardware/irq.h>
#include <hardware/structs/sio.h>
#include <hardware/uart.h>
#include <hardware/spi.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>
#include <string.h>
#include <tusb.h>

#if !defined(MIN)
#define MIN(a, b) ((a > b) ? b : a)
#endif /* MIN */

#define LED_PIN 25

#define BUFFER_SIZE 256

#define DEF_BIT_RATE 115200
#define DEF_STOP_BITS 1
#define DEF_PARITY 0
#define DEF_DATA_BITS 8

#define SPI_FREQ_HZ (2000 * 1000)
#define SPI_INST spi0

typedef struct {
	cdc_line_coding_t usb_lc;
	mutex_t lc_mtx;
	uint8_t uart_buffer[BUFFER_SIZE];
	uint32_t uart_pos;
	mutex_t uart_mtx;
	uint8_t usb_buffer[BUFFER_SIZE];
	uint32_t usb_pos;
	mutex_t usb_mtx;
} uart_data_t;

static uart_data_t UART_DATA[1];

void usb_read_bytes(void) {
	uint32_t len = tud_cdc_n_available(0);

	if (len) {
		uart_data_t *ud = &UART_DATA[0];

		mutex_enter_blocking(&ud->usb_mtx);

		len = MIN(len, BUFFER_SIZE - ud->usb_pos);
		if (len) {
			uint32_t count;

			count = tud_cdc_n_read(0, ud->usb_buffer, len);
			/* ud->usb_pos += count; */
		}

		mutex_exit(&ud->usb_mtx);
	}
}

void usb_write_bytes(void) {
	uart_data_t *ud = &UART_DATA[0];

	if (ud->uart_pos) {
		uint32_t count;

		mutex_enter_blocking(&ud->uart_mtx);

		count = tud_cdc_n_write(0, ud->uart_buffer, ud->uart_pos);
		if (count < ud->uart_pos) {
			memcpy(ud->uart_buffer, &ud->uart_buffer[count],
			       ud->uart_pos - count);
		}
		ud->uart_pos -= count;

		mutex_exit(&ud->uart_mtx);

		if (count)
			tud_cdc_n_write_flush(0);
	}
}

void usb_cdc_process(void)
{
	uart_data_t *ud = &UART_DATA[0];

	mutex_enter_blocking(&ud->lc_mtx);
	tud_cdc_n_get_line_coding(0, &ud->usb_lc);
	mutex_exit(&ud->lc_mtx);

	usb_read_bytes();
	usb_write_bytes();
}

void core1_entry(void)
{
	tusb_init();

	while (1) {
		tud_task();

		if (tud_cdc_n_connected(0)) {
			usb_cdc_process();
		}
	}
}

void insert_byte(uint8_t byte)
{
}

void uart_read_bytes(void) {
	static uint8_t act = 0;
	uart_data_t *ud = &UART_DATA[0];

	/* Drop bytes if we can't receive */
	if (ud->uart_pos >= BUFFER_SIZE) return;

	/* wait until SPI bytes are clocked in */
	/* TODO: use whole buf instead of byte by byte */
	uint8_t byte;

	do {
		spi_read_blocking(SPI_INST, 0xFF, &byte, 1);

		mutex_enter_blocking(&ud->uart_mtx);
		ud->uart_buffer[ud->uart_pos] = byte;
		ud->uart_pos += 1;
		mutex_exit(&ud->uart_mtx);

		act ^= 0x01;
		gpio_put(LED_PIN, act);
	} while (spi_is_readable(SPI_INST));
}

void init_uart_data(void) {
	uart_data_t *ud = &UART_DATA[0];

	/* Pinmux */
    spi_init(SPI_INST, SPI_FREQ_HZ);
	/* This line is necessary when CS is not asserted for every byte. Else SPI
	 * IP uses that to latch the data interrupt. */
    spi_set_format(SPI_INST, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    spi_set_slave(SPI_INST, true);

    gpio_set_function(0, GPIO_FUNC_SPI); /* MISO: pin 1 */
    gpio_set_function(1, GPIO_FUNC_SPI); /* CS:   pin 2 */
    gpio_set_function(2, GPIO_FUNC_SPI); /* SCK:  pin 4 */
    gpio_set_function(3, GPIO_FUNC_SPI); /* MOSI: pin 5 */

	/* USB CDC LC */
	ud->usb_lc.bit_rate = DEF_BIT_RATE;
	ud->usb_lc.data_bits = DEF_DATA_BITS;
	ud->usb_lc.parity = DEF_PARITY;
	ud->usb_lc.stop_bits = DEF_STOP_BITS;

	/* Buffer */
	ud->uart_pos = 0;
	ud->usb_pos = 0;

	/* Mutex */
	mutex_init(&ud->lc_mtx);
	mutex_init(&ud->uart_mtx);
	mutex_init(&ud->usb_mtx);
}

int main(void)
{
	init_uart_data();

	gpio_init(LED_PIN);
	gpio_set_dir(LED_PIN, GPIO_OUT);

	multicore_launch_core1(core1_entry);

	while (1) {
		uart_read_bytes();
	}

	return 0;
}
