/*
 *  Copyright (C) 2013 Sensirion AG, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *
 * You may obtain a copy of the License at
 *       http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * \file
 *
 * \brief AMS VOC driver interface
 *
 * This module provides access to the AMS VOC functionality over the ASF I2C
 * interface. 
 */

#ifndef AMS_H_
#define AMS_H_

#include "status_codes.h"

/**
 * Performs a measurement of the AMS VOC sensor. 
 *
 * @param i2c_master_instance the i2c master instance pointer
 * @param voc the address for the result of the voc equivalent measurement
 * @param co2 the address for the result of the co2 equipvalent measurement
 * @return     STATUS_OK if the command was successful, else an error code.
 */
enum status_code ams_sensor_read(struct i2c_master_module *i2c_master_instance_ptr,
        int *voc, int *co2);

#endif /* AMS_H_ */
