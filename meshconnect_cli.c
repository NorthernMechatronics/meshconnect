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
#include <stdlib.h>
#include <string.h>

#include <am_mcu_apollo.h>
#include <am_util.h>

#include <FreeRTOS.h>
#include <FreeRTOS_CLI.h>
#include <timers.h>

#include "application_lrm.h"

#include "meshconnect_cli.h"

#define COMMAND_LINE_BUFFER_MAX     (128)

static portBASE_TYPE meshconnect_cli_entry(char *pui8OutBuffer,
                                                size_t ui32OutBufferLength,
                                                const char *pui8Command);

static CLI_Command_Definition_t meshconnect_cli_definition = {
    (const char *const) "mc",
    (const char *const) "mc     :  MeshConnect Application Layer Commands.\r\n",
    meshconnect_cli_entry,
    -1};

static size_t argc;
static char *argv[8];
static char argz[COMMAND_LINE_BUFFER_MAX];
static uint8_t lrm_transmit_buffer[UDP_PAYLOAD_MAX_SIZE];
static TimerHandle_t periodic_transmit_timer = NULL;

void meshconnect_cli_register()
{
    FreeRTOS_CLIRegisterCommand(&meshconnect_cli_definition);
    argc = 0;
}

static void periodic_transmit_callback(TimerHandle_t handle)
{
    uint32_t ui32Count = (uint32_t)pvTimerGetTimerID(handle);
    ui32Count++;
    vTimerSetTimerID(handle, (void *)ui32Count);

    am_util_stdio_sprintf((char *)lrm_transmit_buffer, "%d", ui32Count);
    uint32_t length = strlen((char *)lrm_transmit_buffer);
    application_lrm_transmit(lrm_transmit_buffer, length);
}

static void convert_hex_string(const char *in, size_t inlen, uint8_t *out, size_t *outlen)
{
    size_t n = 0;
    char cNum[3];
    *outlen = 0;
    while (n < inlen)
    {
        switch (in[n])
        {
        case '\\':
            n++;
            switch (in[n])
            {
            case 'x':
                n++;
                memset(cNum, 0, 3);
                memcpy(cNum, &in[n], 2);
                n++;
                out[*outlen] = strtol(cNum, NULL, 16);
                break;
            }
            break;
        default:
            out[*outlen] = in[n];
            break;
        }
        *outlen = *outlen + 1;
        n++;
    }
}

static void help(char *pui8OutBuffer, size_t argc, char **argv)
{
    am_util_stdio_printf("\r\nusage: mc <command>\r\n");
    am_util_stdio_printf("\r\n");
    am_util_stdio_printf("supported commands are:\r\n");
    am_util_stdio_printf("  help     display help message\r\n");
    am_util_stdio_printf("  periodic <start|stop> [period in seconds]\r\n");
    am_util_stdio_printf("           periodically transmit an incrementing counter\r\n");
    am_util_stdio_printf("  send     <payload>\r\n");
    am_util_stdio_printf("           transmit a packet over a socket\r\n");
}

static void periodic(char *pui8OutBuffer, size_t argc, char **argv)
{
    uint32_t ui32Period;
    if (argc < 3)
    {
        return;
    }

    if (strcmp(argv[2], "stop") == 0)
    {
        if (periodic_transmit_timer)
        {
            xTimerStop(periodic_transmit_timer, portMAX_DELAY);
            xTimerDelete(periodic_transmit_timer, portMAX_DELAY);
            periodic_transmit_timer = NULL;
        }
    }
    else if (strcmp(argv[2], "start") == 0)
    {
        if (argc == 3)
        {
            ui32Period = 300;
        }
        else
        {
            ui32Period = strtol(argv[3], NULL, 10);
        }

        if (periodic_transmit_timer == NULL)
        {
            periodic_transmit_timer = xTimerCreate("MeshConnect periodic",
                                                   pdMS_TO_TICKS(ui32Period * 1000),
                                                   pdTRUE,
                                                   (void *)0,
                                                   periodic_transmit_callback);
            xTimerStart(periodic_transmit_timer, portMAX_DELAY);
        }
        else
        {
            xTimerChangePeriod(periodic_transmit_timer, pdMS_TO_TICKS(ui32Period * 1000), portMAX_DELAY);
        }
        periodic_transmit_callback(periodic_transmit_timer);
    }
}

static void send(char *pui8OutBuffer, size_t argc, char **argv)
{
    if (argc < 3)
    {
        am_util_stdio_printf("usage: mc send <data>\r\n");
        return;
    }

    size_t length;
    convert_hex_string(argv[argc - 1], strlen(argv[argc - 1]), lrm_transmit_buffer, &length);

    application_lrm_transmit(lrm_transmit_buffer, length);
}

portBASE_TYPE
meshconnect_cli_entry(char *pui8OutBuffer, size_t ui32OutBufferLength, const char *pui8Command)
{
    pui8OutBuffer[0] = 0;

    memset(argz, 0, COMMAND_LINE_BUFFER_MAX);
    strcpy(argz, pui8Command);
    FreeRTOS_CLIExtractParameters(argz, &argc, argv);

    if (strcmp(argv[1], "help") == 0)
    {
        help(pui8OutBuffer, argc, argv);
    }
    else if (strcmp(argv[1], "periodic") == 0)
    {
        periodic(pui8OutBuffer, argc, argv);
    }
    else if (strcmp(argv[1], "send") == 0)
    {
        send(pui8OutBuffer, argc, argv);
    }
    else
    {
        help(pui8OutBuffer, argc, argv);
    }

    return pdFALSE;
}
