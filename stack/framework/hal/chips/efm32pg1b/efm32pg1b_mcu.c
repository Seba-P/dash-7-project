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

/*! \file efm32pg1b_mcu.c
 *
 *  \author glenn.ergeerts@uantwerpen.be
 *  \author daniel.vandenakker@uantwerpen.be
 *  \author maarten.weyn@uantwerpen.be
 *
 */

#include "em_cmu.h"
#include "em_emu.h"
#include "em_chip.h"
#include "platform.h"

void __efm32pg1b_mcu_init()
{
    /* Chip errata */
    CHIP_Init();

    EMU_DCDCInit_TypeDef dcdcInit = EMU_DCDCINIT_DEFAULT;
    EMU_DCDCInit(&dcdcInit);

#ifdef HW_USE_HFXO //40MHz
    // init clock with HFXO (external)
    CMU_HFXOInit_TypeDef hfxoInit = CMU_HFXOINIT_DEFAULT;
    hfxoInit.ctuneSteadyState = 0x142;

    //CMU_ClockDivSet(cmuClock_HF, cmuClkDiv_2);		// 20 MHZ
    CMU_HFXOInit(&hfxoInit);    //40MHz
    CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO); // Select HF XTAL osc as system clock source. 48MHz XTAL, but we divided the system clock by 2, therefore our HF clock will be 24MHz
    //CMU_ClockDivSet(cmuClock_HFPER, cmuClkDiv_4); // TODO set HFPER clock divider (used for SPI) + disable gate clock when not used?   
    CMU_OscillatorEnable(cmuOsc_HFRCO, false, false);
#else
    // init clock with HFRCO (internal)
    CMU_HFRCOFreqSet(cmuHFRCOFreq_38M0Hz); //there is no 21MHz band set in efm32pg1b
    CMU_OscillatorEnable(cmuOsc_HFRCO, true, true);
    CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFRCO);
#endif
}

