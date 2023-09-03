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
 * \brief AMS VOC driver interface
 *
 * This module provides access to the AMS VOC functionality over the ASF I2C
 * interface. 
 */

#include <asf.h>
#include "ams_voc.h"
#include "i2c_master.h"

const uint8_t AMS_ADDRESS_AND_READ  = 0x5a;

enum status_code ams_sensor_read(struct i2c_master_module *i2c_master_instance_ptr, int *voc, int *co2)
{
    uint8_t data[9];
    struct i2c_master_packet packet = {
	    .address = AMS_ADDRESS_AND_READ,
	    .data_length = sizeof(data),
	    .data = data,
	    .ten_bit_address = false,
	    .high_speed = false,
    };
    // stop and nack enabled
	enum status_code ret = i2c_master_read_packet_wait(i2c_master_instance_ptr, &packet);
    //print_to_terminal("ret in  shtc1_read_async_result = 0x%x\n", ret);
    
    if (ret)
		return ret;
    if (data[2])
	{
		printf("data[2] status = 0x%x\n", data[2]);	
		return STATUS_ERR_BAD_DATA;
	}
    
	*co2 = (data[0] << 8) + data[1] ;
	*voc = (data[7] << 8) + data[8] ;

    return STATUS_OK;	
	
};




