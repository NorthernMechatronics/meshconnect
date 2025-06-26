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

#include <string.h>

#include <FreeRTOS.h>
#include <FreeRTOS_CLI.h>

#include <am_util.h>

#include "lrm_api.h"
#include "lrm_task_cli.h"

static portBASE_TYPE lrm_task_cli_entry(char *pui8OutBuffer,
                                                size_t ui32OutBufferLength,
                                                const char *pui8Command);

static CLI_Command_Definition_t lora_mesh_task_cli_definition = {
    (const char *const) "lrm",
    (const char *const) "lrm    :  LoRa Mesh commands\r\n",
    lrm_task_cli_entry,
    -1};

static size_t argc;
static char *argv[8];
static char argz[128];

void lrm_task_cli_register()
{
    FreeRTOS_CLIRegisterCommand(&lora_mesh_task_cli_definition);
}

static void help(char *pui8OutBuffer, size_t argc, char **argv)
{
    am_util_stdio_printf("\r\nusage: lrm <command>\r\n\r\n");
    am_util_stdio_printf("supported commands:\r\n\r\n");
    lrm_cli_input_line(lrm_init(), true, "help");
}

static portBASE_TYPE
lrm_task_cli_entry(char *pui8OutBuffer, size_t ui32OutBufferLength, const char *pui8Command)
{
    pui8OutBuffer[0] = 0;

    strcpy(argz, pui8Command);
    FreeRTOS_CLIExtractParameters(argz, &argc, argv);
    strcpy(argz, pui8Command);

    if (argc < 2)
    {
        help(pui8OutBuffer, argc, argv);
    }
    else
    {
        lrm_cli_input_line(lrm_init(), true, argv[1]);
    }

    return pdFALSE;
}
