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

/* \file
 *
 * Interface to the lcd of the gecko board. This file is NOT a part of the
 * 'HAL' interface since not every platform has the same LCD options
 *
 * The EFM32WG 'Wonder Gecko' has additional LCD options to generic API
 *
 */
#ifndef __PLATFORM_LCD_H_
#define __PLATFORM_LCD_H_

#include "link_c.h"
#include "types.h"
#include "platform.h"

void lcd_write_number(int value);

void lcd_write_line(int line_nr, const char* format, ...);

// void lcd_show_battery_indication(int batteryLevel); // 0 to 4

// void lcd_write_temperature(int temperature, bool celcius); // temperature in centi degrees


#endif
