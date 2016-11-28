/* * OSS-7 - An opensource implementation of the DASH7 Alliance Protocol for ultra
 * lowpower wireless sensor communication
 *
 * Copyright 2015 University of Antwerp
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "hwleds.h"
#include "hwsystem.h"
#include "scheduler.h"
#include "timer.h"
#include "assert.h"
#include "platform.h"

#include <stdio.h>
#include <stdlib.h>

#include "hwlcd.h"
#include "hwadc.h"
#include "d7ap_stack.h"
#include "fs.h"
#include "log.h"

#include "userbutton.h"
#include "platform_sensors.h"
#include "platform_lcd.h"

#define SENSOR_FILE_ID           0x40
#define SENSOR_FILE_SIZE         4
#define ACTION_FILE_ID           0x41

#define SENSOR_UPDATE	TIMER_TICKS_PER_SEC * 3

// Toggle different operational modes
void userbutton_callback(button_id_t button_id)
{
  log_print_string("\nButton: %d", button_id);
	// lcd_write_line(4, "Button: %d\n", button_id);
}

void execute_sensor_measurement()
{
  static uint32_t val;
  val++;

  led_toggle(1);
  log_print_string("\nval = %d", val & 0xFF);
  // lcd_write_line(3, "val = %d\n", val & 0xFF);

  fs_write_file(SENSOR_FILE_ID, 0, (uint8_t*)&val, SENSOR_FILE_SIZE);

  timer_post_task_delay(&execute_sensor_measurement, SENSOR_UPDATE);
}

void init_user_files()
{
    // file 0x40: contains our sensor data + configure an action file to be executed upon write
    fs_file_header_t file_header = (fs_file_header_t){
        .file_properties.action_protocol_enabled = 1,
        .file_properties.action_file_id = ACTION_FILE_ID,
        .file_properties.action_condition = ALP_ACT_COND_WRITE,
        .file_properties.storage_class = FS_STORAGE_VOLATILE,
        .file_properties.permissions = 0, // TODO
        .length = SENSOR_FILE_SIZE
    };

    fs_init_file(SENSOR_FILE_ID, &file_header, NULL);

    // configure file notification using D7AActP: write ALP command to broadcast changes made to file 0x40 in file 0x41
    // first generate ALP command consisting of ALP Control header, ALP File Data Request operand and D7ASP interface configuration
    alp_control_regular_t alp_ctrl = {
        .group = false,
        .response_requested = false,
        // .operation = ALP_OP_READ_FILE_DATA
        .operation = ALP_OP_WRITE_FILE_DATA
    };

    // alp_operand_file_data_request_t file_data_request_operand = {
    alp_operand_file_data_t file_data_operand = {
        .file_offset = {
            .file_id = SENSOR_FILE_ID,
            .offset = 0
        },
        .provided_data_length = SENSOR_FILE_SIZE,
    };

    d7asp_master_session_config_t session_config = {
        .qos = {
            .qos_resp_mode = SESSION_RESP_MODE_NO,
            .qos_nls                 = false,
            .qos_stop_on_error       = false,
            .qos_record              = false
        },
        .dormant_timeout = 0,
        .addressee = {
            .ctrl = {
              .id_type = ID_TYPE_BCAST,
              .access_class = 0
            },
            .id = 0
        }
    };

    // finally, register D7AActP file
    // fs_init_file_with_D7AActP(ACTION_FILE_ID, &session_config, (alp_control_t*)&alp_ctrl, (uint8_t*)&file_data_request_operand);
    fs_init_file_with_D7AActP(ACTION_FILE_ID, &session_config, (alp_control_t*)&alp_ctrl, (uint8_t*)&file_data_operand);
}

void bootstrap()
{
	log_print_string("Device booted\n");

    dae_access_profile_t access_classes[1] = {
        {
            .control_scan_type_is_foreground = false,
            .control_csma_ca_mode = CSMA_CA_MODE_UNC,
            .control_number_of_subbands = 1,
            .subnet = 0x00,
            .scan_automation_period = 0,
            .transmission_timeout_period = 50,
            .subbands[0] = (subband_t){
                .channel_header = {
                    .ch_coding = PHY_CODING_PN9,
                    .ch_class = PHY_CLASS_NORMAL_RATE,
                    .ch_freq_band = PHY_BAND_868
                },
                .channel_index_start = 16,
                .channel_index_end = 16,
                .eirp = 10,
                .ccao = 0
            }
        }
    };

    fs_init_args_t fs_init_args = (fs_init_args_t){
        .fs_user_files_init_cb = &init_user_files,
        .access_profiles_count = 1,
        .access_profiles = access_classes
    };

    d7ap_stack_init(&fs_init_args, NULL, false, NULL);

    initSensors();

    ubutton_register_callback(0, &userbutton_callback);
    ubutton_register_callback(1, &userbutton_callback);

    sched_register_task(&execute_sensor_measurement);

    timer_post_task_delay(&execute_sensor_measurement, TIMER_TICKS_PER_SEC);

    lcd_write_string("EFM32 Sensor\n");
}

