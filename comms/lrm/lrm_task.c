/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2023-2024, Northern Mechatronics, Inc.
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

#include <stdio.h>

#include <am_util.h>

#include <FreeRTOS.h>
#include <task.h>

#include "lrm.h"
#include "lrm_api.h"
#include "lrm_task.h"
#include "lrm_task_cli.h"

static TaskHandle_t lrm_task_handle;
static lrm_context_t *lrm_context_handle;
static lrm_status_t lrm_status = LRM_STATUS_NOT_READY;
static char lrm_cli_buffer[384];

void lrm_event_pending(void)
{
    if (xPortIsInsideInterrupt() == pdTRUE)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(lrm_task_handle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    else
    {
        xTaskNotifyGive(lrm_task_handle);
    }
}

static int cli_output_callback(void *context, const char *fmt, va_list args)
{
    vsnprintf(lrm_cli_buffer, sizeof(lrm_cli_buffer), fmt, args);
    return am_util_stdio_printf(lrm_cli_buffer);
}

static void lrm_task_setup()
{
    // Initialize the LoRa mesh stack
    lrm_sys_init(0, NULL);

    // Initialize a LoRa mesh instance
    lrm_radio_set(LRM_RADIO_ON);
    lrm_context_handle = lrm_init();
    lrm_radio_set(LRM_RADIO_OFF);

    // Initialize the LoRa mesh cli
    lrm_cli_init(lrm_context_handle, cli_output_callback, NULL);

    lrm_status = LRM_STATUS_READY;
}

static void lrm_task(void *pvParameters)
{
#if defined(CLI_ENABLE)
    lrm_task_cli_register();
#endif

    lrm_task_setup();

    lrm_lock(lrm_context_handle);
    while (!lrm_sys_pseudo_reset_was_requested())
    {
        lrm_tasklets_process(lrm_context_handle);
        if (!lrm_tasklets_are_pending(lrm_context_handle))
        {
            lrm_unlock(lrm_context_handle);
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            lrm_lock(lrm_context_handle);
        }
        lrm_sys_process_drivers(lrm_context_handle);
    }
    lrm_unlock(lrm_context_handle);

    lrm_finalize(lrm_context_handle);
}

void lrm_task_create(uint32_t ui32Priority)
{
    lrm_status = LRM_STATUS_NOT_READY;
    lrm_radio_init();
    xTaskCreate(lrm_task, "LoRa Mesh", 2048, 0, ui32Priority, &lrm_task_handle);
}

lrm_status_t lrm_status_get(void)
{
    return lrm_status;
}

__attribute ((weak)) void lrm_radio_init(void)
{
}

__attribute ((weak)) void lrm_radio_set(lrm_radio_t enable)
{
}
