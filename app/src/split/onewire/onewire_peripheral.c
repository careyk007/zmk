/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * @brief Implement a one-wire protocol for wired split communication.
 * 
 * This is a custom implementation of a one-wire communication protocol. It currently only supports sending data
 * one direction (half-duplex) from the peripheral board to the central board.
 * 
 * At a high-level overview, the packets will be framed by a start bit, followed by a length byte, followed by
 * the actual data.
 * 
 * When idle, the line is pulled high on both ends. To initiate a transaction, the peripheral will drive the line
 * low for one transmission period (ONE_PERIOD_US). After this period, the peripheral will drive the line high
 * or low indicating a bit value of 1 or 0, respectively, for one transmission period.
 * 
 * The central device will set an interrupt for the falling edge of the transaction start bit. When the interrupt
 * occurs, the central will immediately disable the falling edge iterrupt and wait for one and a half transmission
 * periods before beginning sampling the data line every transmission period thereafter. By waiting one and a half
 * transmission periods before starting sampling, the central should be sampling in the middle of the peripheral's
 * transmission period, allowing a stable signal to be sampled.
 */

#include <zephyr/types.h>
#include <sys/util.h>

#include <logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <device.h>
#include <drivers/gpio.h>

#include <zmk/matrix.h>
#include <zmk/split/onewire/onewire.h>


#define ONE_PERIOD_US   (50)
#define HALF_PERIOD_US  (ONE_PERIOD_US / 2)

#define FRAMING_SYMBOL  (0x55)

/*
 * Devicetree helper macro which gets the 'flags' cell from a 'gpios'
 * property, or returns 0 if the property has no 'flags' cell.
 */

#define FLAGS_OR_ZERO(node)                             \
    COND_CODE_1(DT_PHA_HAS_CELL(node, gpios, flags),    \
            (DT_GPIO_FLAGS(node, gpios)),               \
            (0)

#define ONEWIRE_NODE   DT_ALIAS(onewire0)
#if DT_NODE_HAS_STATUS(ONEWIRE_NODE, okay) && DT_NODE_HAS_PROP(ONEWIRE_NODE, gpios)
#define ONEWIRE_GPIO_LABEL DT_GPIO_LABEL(ONEWIRE_NODE, gpios)
#define ONEWIRE_GPIO_PIN   DT_GPIO_PIN(ONEWIRE_NODE, gpios)
#define ONEWIRE_GPIO_FLAGS (GPIO_OUTPUT | FLAGS_OR_ZERO(ONEWIRE_NODE))
#else
#error "Unsupported board: onewire0 devicetree node is not defined"
#define ONEWIRE_GPIO_LABEL ""
#define ONEWIRE_GPIO_PIN   0
#define ONEWIRE_GPIO_FLAGS 0
#endif

#define POSITION_STATE_DATA_LEN     16
static u8_t num_of_positions = ZMK_KEYMAP_LEN;
static u8_t position_state[POSITION_STATE_DATA_LEN];

static struct device *onewire_dev;

/******************************************************************************
 * Peripheral functions
 ******************************************************************************/
static void onewire_send_byte(u8_t byte) {
    for (int i = 0; i < sizeof(byte); ++i) {
        gpio_pin_set(onewire_dev, ONEWIRE_GPIO_PIN, (byte & 0x01));
        byte >>= 1;
        k_busy_wait(ONE_PERIOD_US);
    }
}

static u8_t onewire_read_byte(void) {
    u8_t byte = 0;
    for (int i = 0; i < sizeof(byte); ++i) {
        int pin_value = gpio_pin_get(onewire_dev, ONEWIRE_GPIO_PIN);
        if (pin_value >= 0) {
            byte |= pin_value << i;
        } else {
            log("Pin read error: %d", pin_value);
        }
        k_busy_wait(ONE_PERIOD_US);
    }
    return byte;
}

static void onewire_start_transmission(u8_t length) {
    onewire_send_byte(FRAMING_SYMBOL);
    onewire_send_byte(length);
}

static void onewire_end_transmission(u8_t length) {
    gpio_pin_set(onewire_port, onewire_pin, 1);
}

static void send_onewire_data(u8_t *data, u8_t length) {
    onewire_start_transmission(length);
    /* Send data bytes */
    for (int i = 0; i < length; ++i) {
        onewire_send_byte(data[i]);
    }
    onewire_end_transmission();
}

int zmk_split_onewire_position_pressed(u8_t position) {
    WRITE_BIT(position_state[position/8], position % 8, true);
    send_onewire_data(position_state, sizeof(position_state))
}

int zmk_split_onewire_position_released(u8_t position) {
    WRITE_BIT(position_state[position/8], position % 8, false);
    send_onewire_data(position_state, sizeof(position_state))
}

static int onewire_init(struct device *_arg) {
    onewire_dev = device_get_binding(ONEWIRE_GPIO_LABEL);
    if (onewire_dev == NULL) {
        log("Didn't find onewire device %s\n", ONEWIRE_GPIO_LABEL);
        return NULL;
    }

    ret = gpio_pin_configure(onewire_dev, ONEWIRE_GPIO_PIN, ONEWIRE_GPIO_FLAGS);
    if (ret != 0) {
        log("Error %d: failed to configure onewire device %s pin %d\n",
               ret, ONEWIRE_GPIO_LABEL, ONEWIRE_GPIO_PIN);
        return NULL;
    }

    /* Setup pin for peripheral */
    gpio_pin_set(onewire_dev, ONEWIRE_GPIO_PIN, 1);

    log("Set up onewire at %s pin %d\n", ONEWIRE_GPIO_LABEL, ONEWIRE_GPIO_PIN);
}

SYS_INIT(onewire_init, APPLICATION, CONFIG_ZMK_BLE_INIT_PRIORITY);