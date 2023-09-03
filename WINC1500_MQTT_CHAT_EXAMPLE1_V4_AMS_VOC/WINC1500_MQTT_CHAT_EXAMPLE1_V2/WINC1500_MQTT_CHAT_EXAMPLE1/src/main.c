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
 * \brief SHTC1 eXplained pro board application example
 *
 * Uses Sensirion SHTC1 extension board for SAMD20 board
 * LED0 and BUTTON0 included on SAMD20 board.
 *
 * Reads out the sensor measurements in a loop. If the button is pressed or
 * on power up, a reference average humidity value is formed out of 3
 * measurements (LED blinks 3 times fast).
 *
 * If breath (or air-touch how we call it) is detected, the LED turns on.
 * A trivial algorithm is used for detection or air-touch, a comparison of
 * (last measured relative humidity - ref. relative humidity value) > TRESHOLD
 *
 * In every measurement loop, a dew point is calculated and sent back over the
 * serial connection to the host PC. It can be seen within Atmel Studio if the
 * "Terminal Window" extension is installed or in any other terminal
 * application (e.g. Terra Term). The COM port used can be found within the
 * device manager, it is called "EDBG Virtual COM Port".
 *
 */

#include <asf.h>
#include <math.h>
#include "demo_tools.h"
#include "shtc1.h"

#define MEASUREMENT_INTERVAL_MS 200
#define AIR_TOUCH_DETECT_THRESHOLD 0.1f

int main(void)
{
    system_init();

    /**
     * initialize pins for button and led,
     * initializes I2C communication with
     * shtc1 extension board plugged to EXT2 position and
     * initialize usart communication to terminal on host PC
     */
    shtc1_demo_init();

    /* values read from sensor, encoded as integer numbers */
    int temp, rh;

    /**
     * temperature (°C), dew point (°C),
     * and relative humidity (%RH)
     */
    float temp_f, rh_f, dew_f, k_f;

    /* average reference value to detect air-touch */
    float rh_avg_f = 0.0f;

    /* statuses of I2C communication */
    enum status_code shtc1_connected;
    enum status_code shtc1_read_status;

    delay_s(1);

    print_to_terminal("\r\nstarting...\r\n");

    /* test if sensor is connected */
    shtc1_connected = shtc1_probe(&i2c_master_instance);
    print_to_terminal("sensor is %s\r\n",
            shtc1_connected ? "present" : "not present");

    /* main loop */
    while (true)
    {
        /* check if button pressed or program starts, possibly
         * new reference humidity value is read for breath detection.
         *
         * this is signalized to user by blinking the led three times
         */
        if (rh_avg_f <= 0.0f || !port_pin_get_input_level(BUTTON_0_PIN))
        {
            rh_avg_f = 0.0f;
            for (int i = 0; i < 3; i++)
            {
                shtc1_read_hpm_sync(&i2c_master_instance, &temp, &rh);
                rh_avg_f += rh / 3000.0f;
                port_pin_set_output_level(LED_0_PIN, false);
                delay_ms(50);
                port_pin_set_output_level(LED_0_PIN, true);
                delay_ms(50);
            }
            print_to_terminal("threshold humidity = %4.2f %%RH \r\n", rh_avg_f);
            delay_ms(50);
        }

        /**
         * perform measurement in high precision mode, clock stretching and
         * update temperature & humidity. On success calculate dew point.
         */
        shtc1_read_status = shtc1_read_hpm_sync(&i2c_master_instance, &temp, &rh);
        if (shtc1_read_status == STATUS_OK)
        {
            temp_f = temp / 1000.0f;
            rh_f = rh / 1000.0f;
            k_f = (log10(rh_f) - 2) / 0.4343 + (17.62 * temp_f) / (243.12 + temp_f);
            dew_f = 243.12 * k_f / (17.62 - k_f);
            print_to_terminal("temp = %4.2f \260C, rh = %4.2f %%RH,"
                    " dew point= %4.2f \260C\n", temp_f, rh_f, dew_f);

            /* turn on LED if humidity threshold is exceeded */
            if (rh_f - rh_avg_f > AIR_TOUCH_DETECT_THRESHOLD)
                port_pin_set_output_level(LED_0_PIN, false);
            else
                port_pin_set_output_level(LED_0_PIN, true);
        }
        else
        {
            print_to_terminal("Measurement failed. Extension-board disconnected?\n");
            port_pin_set_output_level(LED_0_PIN, true);
        }

        /* delay until the next iteration */
        delay_ms(MEASUREMENT_INTERVAL_MS);
    }
}
