/*
 *  BSD 3-Clause License
 *
 * Copyright (c) 2025, Northern Mechatronics, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <string.h>

#include <FreeRTOS.h>
#include <task.h>

#include <am_mcu_apollo.h>
#include <am_util.h>
#include <am_bsp.h>

#include "led.h"

#include "application_task.h"
#include "application_lrm.h"

const uint16_t lrm_pan_id = 0xFDF3;
char lrm_network_name[] = "NM MeshConnect";
const char lrm_extended_pan_id[] = "\x61\xc2\x0b\xcf\x65\xcb\x82\x56";
const char lrm_mesh_local_prefix[] = "\xfd\x58\x47\xf8\x0c\xd8\x54\xc4";
const char lrm_network_key[] = "\xcd\x2a\x30\xa1\x17\x1f\x30\x85\x7b\xb9\x54\x09\x01\xcc\x17\x58";
const char lrm_pksc[] = "\x57\xc0\xe7\x78\x49\xc8\xd8\x4e\x72\x6c\x9a\x84\x91\x8d\x68\x51";
const uint16_t lrm_channel = 2;

const uint32_t lrm_radio_spreading_factor = 8;
const uint32_t lrm_radio_bandwidth = 500;
const uint32_t lrm_radio_transmit_power_dbm = 22;
const uint32_t lrm_radio_frequency_hz = 915000000;

static char udp_bind_address[] = "::";
static char udp_send_socket_address[] = "ff03::1";
static uint16_t udp_bind_port = 1234;
static lrm_udp_socket udp_socket;

static lrm_context_t *lrm_context_handle;

static udp_packet_t udp_downlink_packet;
static uint8_t udp_downlink_payload[UDP_PAYLOAD_MAX_SIZE];

static void on_lrm_role_changed(lrm_context_t *context, lrm_device_role_e role)
{
    led_command_t led_command = {
        .ui32Handle = comms_led_handle,
        .ui32Id = LED_EFFECT_OFF,
        .ui32Repeat = 0,
    };

    switch(role)
    {
    case LRM_DEVICE_ROLE_DISABLED:
        led_command.ui32Id = LED_EFFECT_OFF;
        led_send(&led_command);
        break;

    case LRM_DEVICE_ROLE_DETACHED:
        led_command.ui32Id = LED_EFFECT_BREATHING;
        led_send(&led_command);
        break;

    case LRM_DEVICE_ROLE_LEADER:
        led_command.ui32Id = LED_EFFECT_PULSE3;
        led_send(&led_command);
        break;

    case LRM_DEVICE_ROLE_ROUTER:
        led_command.ui32Id = LED_EFFECT_PULSE2;
        led_send(&led_command);
        break;

    case LRM_DEVICE_ROLE_CHILD:
        led_command.ui32Id = LED_EFFECT_PULSE1;
        led_send(&led_command);
        break;

    default:
        led_command.ui32Id = LED_EFFECT_SOS;
        led_send(&led_command);
        break;
    }
}

static void on_lrm_udp_receive(void *context, lrm_message *message, const lrm_message_info *message_info)
{
    uint16_t offset = lrm_message_get_offset(message);

    udp_downlink_packet.payload = udp_downlink_payload;
    memcpy(udp_downlink_packet.addr, lrm_message_info_get_peer_addr(message_info), LRM_IP6_ADDRESS_SIZE);
    udp_downlink_packet.port = lrm_message_info_get_peer_port(message_info);
    udp_downlink_packet.size = lrm_message_get_length(message) - offset;
    udp_downlink_packet.rssi = lrm_message_get_rss(message);

    lrm_message_read(
        message, offset, udp_downlink_packet.payload, udp_downlink_packet.size);

    application_send(APP_MSG_LRM_RX);
}

void application_lrm_setup(void)
{
    lrm_network_config_t lrm_config;

    udp_downlink_packet.payload = udp_downlink_payload;

    while (lrm_status_get() == LRM_STATUS_NOT_READY)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    lrm_context_handle = lrm_init();
    if (!lrm_is_configured(lrm_context_handle))
    {
        lrm_config.networkName = (char *)lrm_network_name;
        lrm_config.panId = lrm_pan_id;
        lrm_config.extendedPanId = lrm_extended_pan_id;
        lrm_config.meshLocalPrefix = lrm_mesh_local_prefix;

        lrm_config.channel = lrm_channel;
        lrm_config.networkKey = lrm_network_key;
        lrm_config.pskc = lrm_pksc;

        lrm_network_configure(lrm_context_handle, &lrm_config);
    }

    lrm_radio_set_cfg(lrm_context_handle, LRM_RADIO_CFG_FREQUENCY, lrm_radio_frequency_hz);
    lrm_radio_set_cfg(lrm_context_handle, LRM_RADIO_CFG_SPREADING_FACTOR, lrm_radio_spreading_factor);
    lrm_radio_set_cfg(lrm_context_handle, LRM_RADIO_CFG_BANDWIDTH, lrm_radio_bandwidth);
    lrm_radio_set_cfg(lrm_context_handle, LRM_RADIO_CFG_TX_POWER, lrm_radio_transmit_power_dbm);

    lrm_set_role_changed_callback(lrm_context_handle, on_lrm_role_changed);

    lrm_radio_set(LRM_RADIO_ON);
    lrm_error_e ret = lrm_network_start(lrm_context_handle);
    if (ret != LRM_ERROR_NONE)
    {
        lrm_radio_set(LRM_RADIO_OFF);
        return;
    }

    if (lrm_is_network_started(lrm_context_handle))
    {
        lrm_udp_open(lrm_context_handle, &udp_socket, on_lrm_udp_receive, NULL);
        lrm_udp_bind(lrm_context_handle, &udp_socket, udp_bind_address, udp_bind_port);
    }
    else
    {
        lrm_radio_set(LRM_RADIO_OFF);
    }
}

void lrm_radio_init()
{
    am_hal_gpio_pinconfig(AM_BSP_GPIO_PETAL_CORE_nLORA_EN, g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_state_write(AM_BSP_GPIO_PETAL_CORE_nLORA_EN, AM_HAL_GPIO_OUTPUT_SET);
}

void lrm_radio_set(lrm_radio_t enable)
{
    if (enable == LRM_RADIO_ON)
    {
        am_hal_gpio_state_write(AM_BSP_GPIO_PETAL_CORE_nLORA_EN, AM_HAL_GPIO_OUTPUT_CLEAR);
    }
    else
    {
        am_hal_gpio_state_write(AM_BSP_GPIO_PETAL_CORE_nLORA_EN, AM_HAL_GPIO_OUTPUT_SET);
    }
}

void application_lrm_factory_reset()
{
    lrm_factory_reset(lrm_context_handle);
}

void application_lrm_receive(udp_packet_t **packet)
{
    *packet = &udp_downlink_packet;
}

void application_lrm_transmit(uint8_t *data, size_t len)
{
    if (!lrm_is_network_started(lrm_context_handle))
    {
        return;
    }

    lrm_udp_send(lrm_context_handle, &udp_socket, udp_send_socket_address, udp_bind_port, data, len);
}
