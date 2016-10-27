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

#ifndef __EFM32PG1B_PINS_H_
#define __EFM32PG1B_PINS_H_
#include "hwgpio.h"
//GPIO port/pin definitions for the EFM32PG1B
//TABLE 4.3 in Datasheet
//port A
extern pin_id_t const A0;
extern pin_id_t const A1;
extern pin_id_t const A2;
extern pin_id_t const A3;
extern pin_id_t const A4;
extern pin_id_t const A5;
//port B
extern pin_id_t const B11;
extern pin_id_t const B12;
extern pin_id_t const B13;
extern pin_id_t const B14;
extern pin_id_t const B15;
//port C
extern pin_id_t const C6;
extern pin_id_t const C7;
extern pin_id_t const C8;
extern pin_id_t const C9;
extern pin_id_t const C10;
extern pin_id_t const C11;
//port D
extern pin_id_t const D9;
extern pin_id_t const D10;
extern pin_id_t const D11;
extern pin_id_t const D12;
extern pin_id_t const D13;
extern pin_id_t const D14;
extern pin_id_t const D15;
//port E
//no pins available externally
//port F
extern pin_id_t const F0;
extern pin_id_t const F1;
extern pin_id_t const F2;
extern pin_id_t const F3;
extern pin_id_t const F4;
extern pin_id_t const F5;
extern pin_id_t const F6;
extern pin_id_t const F7;


#endif //__EFM32PG1B_PINS_H_
