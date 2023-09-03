/*
 * Copyright (c) 2014, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
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

/**
 * \file
 *
 * \brief Sensirion SHTC1 demo helper functions.
 *
 * The demo_tools contains helper functions needed by the example. Delay
 * functions, port pins (used for LED/button), usart communication and
 * i2c communication initialization are here.
 *
 * There is also a function provided to the main application which allows to
 * directly send a string (printf-style) over the serial connection to
 * the terminal on host PC.
 */

#include <asf.h>
#include <stdarg.h>
#include "demo_tools.h"
#include "i2c_master.h"

/* used to init usart communication to host PC */
struct usart_module usart_instance;

/* used to init I2C software module. */
struct i2c_master_module i2c_master_instance;

void configure_i2c_master(void)
{
    /* initialize config structure and software module */
    struct i2c_master_config config_i2c_master;
    i2c_master_get_config_defaults(&config_i2c_master);

    /* change buffer timeout to something longer */
    config_i2c_master.buffer_timeout = 10000;

    /* initialize and enable device with config. */
    i2c_master_init(&i2c_master_instance, SERCOM2, &config_i2c_master);
    i2c_master_enable(&i2c_master_instance);
}





void shtc1_demo_init()
{
    //delay_init();
    //configure_port_pins();
    //configure_usart();
    configure_i2c_master();
}

