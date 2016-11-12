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

/*
 * \author	glenn.ergeerts@uantwerpen.be
 * \author  maarten.weyn@uantwerpen.be
 * \author  contact@christophe.vg
 */


#include "string.h"
#include "stdio.h"
#include "stdint.h"

#include "hwleds.h"
#include "log.h"
#include "random.h"

#include "d7ap_stack.h"
#include "dll.h"
#include "alp_cmd_handler.h"

#ifdef HAS_LCD
#include "hwlcd.h"
#endif

#ifdef FRAMEWORK_LOG_ENABLED
#ifdef HAS_LCD
		#define DPRINT(...) log_print_string(__VA_ARGS__); lcd_write_string(__VA_ARGS__)
	#else
		#define DPRINT(...) log_print_string(__VA_ARGS__)
	#endif

#else
	#ifdef HAS_LCD
		#define DPRINT(...) lcd_write_string(__VA_ARGS__)
	#else
		#define DPRINT(...)
	#endif
#endif

#define SENSOR_FILE_ID           0x40
#define SENSOR_FILE_SIZE         4
#define ACTION_FILE_ID           0x41

#define REPORTING_INTERVAL       1 // seconds
#define REPORTING_INTERVAL_TICKS TIMER_TICKS_PER_SEC * REPORTING_INTERVAL

void execute_sensor_measurement() {
#if HW_NUM_LEDS >= 1
    led_toggle(0);
#endif
    // use the counter value for now instead of 'real' sensor
    uint32_t val = timer_get_counter_value();
    DPRINT("\nval = %d", val);
    // file 0x40 is configured to use D7AActP trigger an ALP action which 
    // broadcasts this file data on Access Class 0
    fs_write_file(0x40, 0, (uint8_t*)&val, 4);
    timer_post_task_delay(&execute_sensor_measurement, REPORTING_INTERVAL_TICKS);
}


void bootstrap() {
    DPRINT("Device booted at time: %d\n", timer_get_counter_value());

    sched_register_task(&execute_sensor_measurement);
    timer_post_task_delay(&execute_sensor_measurement, REPORTING_INTERVAL_TICKS);
}
