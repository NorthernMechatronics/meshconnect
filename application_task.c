/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2024, Northern Mechatronics, Inc.
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
#include <am_mcu_apollo.h>
#include <am_util.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include "am_bsp.h"

#include "button.h"
#include "led.h"

#include "gpio_cli.h"

#include "application_task.h"
#include "application_task_cli.h"

#include "application_lrm.h"

#define LED_RED AM_BSP_GPIO_LED0
#define LED_RED_TIMER_NUMBER    (1)
#define LED_RED_TIMER_SEGMENT   AM_HAL_CTIMER_TIMERA
#define LED_RED_TIMER_INTERRUPT AM_HAL_CTIMER_INT_TIMERA1C0

uint32_t led_red_handle;
uint32_t comms_led_handle;

#define APPLICATION_QUEUE_MAX_SIZE           (10)

static TaskHandle_t application_task_handle;
static QueueHandle_t application_queue_handle;

static void on_button0_pressed(void)
{
    application_send(APP_MSG_BUTTON0_PRESSED);
}

static void on_button1_pressed(void)
{
    application_send(APP_MSG_BUTTON1_PRESSED);
}

static void on_led_red_ctimer(void)
{
    led_interrupt_service(led_red_handle);
}

static void handle_button_short_press(void)
{
    uint8_t data = 0x01;
    application_lrm_transmit(&data, sizeof(data));
}

static void handle_lrm_receive(void)
{
    udp_packet_t *packet;
    application_lrm_receive(&packet);

    am_util_stdio_printf("\r\nPacket Received\r\n");
    am_util_stdio_printf("  HOST: ");
    for (int i = 0; i < LRM_IP6_ADDRESS_SIZE; i++)
    {
        if (((i % 2) == 0) && (i > 0))
        {
            am_util_stdio_printf(":");
        }
        am_util_stdio_printf("%02x", packet->addr[i]);
    }
    am_util_stdio_printf("\n\r");
    am_util_stdio_printf("  PORT: %u\r\n", packet->port);
    am_util_stdio_printf("  RSSI: %d\r\n", packet->rssi);
    am_util_stdio_printf("  SIZE: %u\r\n", packet->size);
    am_util_stdio_printf("  PAYLOAD:");
    for (int i = 0; i < packet->size; i++)
    {
        if ((i % 8) == 0)
        {
            am_util_stdio_printf("\n\r    ");
        }
        am_util_stdio_printf("%02x ", packet->payload[i]);
    }
    am_util_stdio_printf("\n\r");

    am_hal_gpio_state_write(AM_BSP_GPIO_LED4, AM_HAL_GPIO_OUTPUT_SET);
    vTaskDelay(pdMS_TO_TICKS(100));
    am_hal_gpio_state_write(AM_BSP_GPIO_LED4, AM_HAL_GPIO_OUTPUT_CLEAR);
}

static void setup_button(void)
{
    uint32_t handle;
    button_config(&handle, AM_BSP_GPIO_BUTTON0, g_AM_BSP_GPIO_BUTTON0, 1);
    button_sequence_register(handle, 1, 0b0, on_button0_pressed);

    button_config(&handle, AM_BSP_GPIO_BUTTON1, g_AM_BSP_GPIO_BUTTON1, 1);
    button_sequence_register(handle, 1, 0b0, on_button1_pressed);
    button_sequence_register(handle, 9, 0b000111000, application_lrm_factory_reset);
}

static void setup_led(void)
{
    // Configure the LED that is connected to a GPIO with CTIMER output.
    const led_config_t led_red_cfg = {
        .ui32Number    = LED_RED_TIMER_NUMBER,
        .ui32Segment   = LED_RED_TIMER_SEGMENT,
        .ui32Interrupt = LED_RED_TIMER_INTERRUPT,
        .ui32ActiveLow = 0,
        .ui32Pin       = LED_RED,
        .pfnInterruptService = on_led_red_ctimer,
    };

    led_config(&led_red_handle, &led_red_cfg);

    comms_led_handle = led_red_handle;
}

static void application_task_setup(void)
{
    am_hal_gpio_pinconfig(AM_BSP_GPIO_LED0, g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_state_write(AM_BSP_GPIO_LED0, AM_HAL_GPIO_OUTPUT_CLEAR);

    am_hal_gpio_pinconfig(AM_BSP_GPIO_LED1, g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_state_write(AM_BSP_GPIO_LED1, AM_HAL_GPIO_OUTPUT_CLEAR);

    am_hal_gpio_pinconfig(AM_BSP_GPIO_LED2, g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_state_write(AM_BSP_GPIO_LED2, AM_HAL_GPIO_OUTPUT_CLEAR);

    am_hal_gpio_pinconfig(AM_BSP_GPIO_LED3, g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_state_write(AM_BSP_GPIO_LED3, AM_HAL_GPIO_OUTPUT_CLEAR);

    am_hal_gpio_pinconfig(AM_BSP_GPIO_LED4, g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_state_write(AM_BSP_GPIO_LED4, AM_HAL_GPIO_OUTPUT_CLEAR);

    am_hal_gpio_pinconfig(AM_BSP_GPIO_PETAL_CORE_nLORA_EN, g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_state_write(AM_BSP_GPIO_PETAL_CORE_nLORA_EN, AM_HAL_GPIO_OUTPUT_SET);

    am_hal_gpio_pinconfig(AM_BSP_GPIO_PETAL_DEV_IO_EN, g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_state_write(AM_BSP_GPIO_PETAL_DEV_IO_EN, AM_HAL_GPIO_OUTPUT_SET);

    setup_button();
    setup_led();

    application_lrm_setup();
}

static void application_task_loop(void)
{
    application_message_t message;

    if (application_queue_handle == NULL)
    {
        return;
    }

    if (xQueueReceive(application_queue_handle, &message, portMAX_DELAY) == pdFALSE)
    {
        return;
    }

    switch(message)
    {
    case APP_MSG_BUTTON0_PRESSED:
    case APP_MSG_BUTTON1_PRESSED:
        handle_button_short_press();
        break;
    
    case APP_MSG_LRM_RX:
        handle_lrm_receive();
        break;
    
    default:
        break;
    }
}

static void application_task(void *parameter)
{
#if defined(CLI_ENABLE)
    gpio_cli_register();
    application_task_cli_register();
#endif
    application_task_setup();
    while (1)
    {
        application_task_loop();
    }
}

void application_task_create(uint32_t priority)
{
    application_queue_handle = xQueueCreate(APPLICATION_QUEUE_MAX_SIZE, sizeof(application_message_t));
    xTaskCreate(application_task, "application", 512, 0, priority, &application_task_handle);
}

void application_send(application_message_t message)
{
    BaseType_t context_switch;

    if (application_queue_handle == NULL)
    {
        return;
    }

    if (xPortIsInsideInterrupt() == pdTRUE)
    {
        context_switch = pdFALSE;
        xQueueSendFromISR(application_queue_handle, &message, &context_switch);
        portYIELD_FROM_ISR(context_switch);
    }
    else
    {
        xQueueSend(application_queue_handle, &message, portMAX_DELAY);
    }
}