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

static struct gpio_callback onewire_callback;
static void onewire_cb(struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    gpio_pin_interrupt_configure(onewire_dev, ONEWIRE_GPIO_PIN, GPIO_INT_DISABLE);

    u8_t changed_positions[POSITION_STATE_DATA_LEN];

    k_busy_wait(ONE_PERIOD_US + HALF_PERIOD_US);
    u8_t data_length = onewire_read_byte();

    if (data_length < sizeof(position_state)) {
        for (int i = 0; i < data_length; i++) {
            position_state[i] = onewire_read_byte();
        }
    } else {
        log("Error, data length was too long: %d", data_length);
    }
    
    for (int i = 0; i < POSITION_STATE_DATA_LEN; i++) {
        changed_positions[i] = ((u8_t *)data)[i] ^ position_state[i];
        position_state[i] = ((u8_t *)data)[i];
    }

    for (int i = 0; i < POSITION_STATE_DATA_LEN; i++) {
        for (int j = 0; j < 8; j++) {
            if (changed_positions[i] & BIT(j)) {
                u32_t position = (i * 8) + j;
                bool pressed = position_state[i] & BIT(j);
                struct position_state_changed *pos_ev = new_position_state_changed();
                pos_ev->position = position;
                pos_ev->state = pressed;
                pos_ev->timestamp = k_uptime_get();

                LOG_DBG("Trigger key position state change for %d", position);
                ZMK_EVENT_RAISE(pos_ev);
            }
        }
    }

    gpio_pin_interrupt_configure(onewire_dev, ONEWIRE_GPIO_PIN, GPIO_INT_EDGE_TO_INACTIVE);
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


    /* Setup pin for central */
    gpio_pin_interrupt_configure(onewire_dev, ONEWIRE_GPIO_PIN, GPIO_INT_EDGE_TO_INACTIVE);
    gpio_init_callback(&onewire_callback, onewire_cb, BIT(ONEWIRE_GPIO_PIN));
    gpio_add_callback(onewire_dev, &onewire_callback);

    log("Set up onewire at %s pin %d\n", ONEWIRE_GPIO_LABEL, ONEWIRE_GPIO_PIN);
}

SYS_INIT(onewire_init, APPLICATION, CONFIG_ZMK_BLE_INIT_PRIORITY);