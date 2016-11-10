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

/*! \file efm32pg1b_timer.c
 *
 *  \author jeremie@wizzilab.com
 *  \author daniel.vandenakker@uantwerpen.be
 *
 */

#include <stdbool.h>
#include <stdint.h>

#include "em_cmu.h"
#include "em_rtcc.h"
#include "em_int.h"
#include "em_gpio.h"

#include "hwtimer.h"
#include "hwatomic.h"
#include "efm32pg1b_mcu.h"

#include "log.h"

/**************************************************************************//**
 * @brief  Start LFRCO for RTC
 * Starts the low frequency RC oscillator (LFRCO) and routes it to the RTC
 *****************************************************************************/
void startLfxoForRtcc(uint8_t freq)
{
	log_print_string("\nHERE(): DONE");
	/* Enabling clock to the interface of the low energy modules */
	CMU_ClockEnable(cmuClock_CORELE, true);
	log_print_string("\n2(): DONE");
	#ifdef HW_USE_LFXO
		/* Starting LFXO and waiting until it is stable */
		CMU_OscillatorEnable(cmuOsc_LFXO, true, true);
		/* Routing the LFRCO clock to the RTC */
	    CMU_ClockSelectSet(cmuClock_LFE,cmuSelect_LFXO);
	#else
	    // init clock with LFRCO (internal)
		/* Starting LFRCO and waiting until it is stable */
		CMU_OscillatorEnable(cmuOsc_LFRCO, true, true);
		/* Routing the LFRCO clock to the RTC */
		CMU_ClockSelectSet(cmuClock_LFE,cmuSelect_LFRCO);
	#endif

	    uint32_t lf = CMU_ClockFreqGet(cmuClock_LFE);
	    log_print_string("\nCMU_ClockFreqGet(): DONE");

    /* Set Clock prescaler */
    // if(freq == HWTIMER_FREQ_1MS)
    // 	CMU_ClockDivSet(cmuClock_RTCC, cmuClkDiv_32);
    // else
    // 	CMU_ClockDivSet(cmuClock_RTCC, cmuClkDiv_1);

    CMU_ClockEnable(cmuClock_RTCC, true);
    log_print_string("\nCMU_ClockEnable(): DONE");

    uint32_t rtcc = CMU_ClockFreqGet(cmuClock_RTCC);
}

static timer_callback_t compare_f = 0x0;
static timer_callback_t overflow_f = 0x0;
static bool timer_inited = false;
/**************************************************************************//**
 * @brief Enables LFACLK and selects LFXO as clock source for RTC.
 *        Sets up the RTC to count at 1024 Hz.
 *        The counter should not be cleared on a compare match and keep running.
 *        Interrupts should be cleared and enabled.
 *        The counter should run.
 *****************************************************************************/
error_t hw_timer_init(hwtimer_id_t timer_id, uint8_t frequency, timer_callback_t compare_callback, timer_callback_t overflow_callback)
{
    if(timer_id >= HWTIMER_NUM)
    	return ESIZE;
    if(timer_inited)
    	return EALREADY;
    if(frequency != HWTIMER_FREQ_1MS && frequency != HWTIMER_FREQ_32K)
    	return EINVAL;
	
    start_atomic();
		compare_f = compare_callback;
		overflow_f = overflow_callback;
		timer_inited = true;

		/* Configuring clocks in the Clock Management Unit (CMU) */
		startLfxoForRtcc(frequency);
		log_print_string("\nstartLfxoForRtcc(): DONE");

		RTCC_Init_TypeDef rtccInit = RTCC_INIT_DEFAULT;
		rtccInit.enable   = false;   /* Don't enable RTC after init has run */
		// rtccInit.comp0Top = true;   /* Clear counter on compare 0 match: cmp 0 is used to limit the value of the rtc to 0xffff */
		// rtccInit.debugRun = false;   /* Counter shall not keep running during debug halt. */


		/* Initialize the RTC */
		RTCC_Init(&rtccInit);
		log_print_string("\nRTCC_Init(): DONE");


		//disable all rtc interrupts while we're still configuring
		RTCC_IntDisable(RTCC_IEN_OF | RTCC_IEN_CC0 | RTCC_IEN_CC1);
		RTCC_IntClear(RTCC_IFC_OF | RTCC_IFC_CC0 | RTCC_IFC_CC1);
		log_print_string("\nRTCC_IntClear(): DONE");

		
		//Set maximum value for the RTC
		//RTC_CompareSet( 0, 0x0000FFFF );
		RTCC_CCChConf_TypeDef rtccChInit = RTCC_CH_INIT_COMPARE_DEFAULT;

		RTCC_ChannelInit(0, &rtccChInit);
		RTCC_ChannelCCVSet(0, 0x0000FFFF);
		log_print_string("\nRTCC_ChannelCCVSet(): DONE");

		//RTC_CounterReset();
		RTCC_Unlock();
		RTCC->PRECNT  = _RTCC_PRECNT_RESETVALUE;
 		RTCC->CNT     = _RTCC_CNT_RESETVALUE;
 		RTCC->TIME    = _RTCC_TIME_RESETVALUE;
 		RTCC->DATE    = _RTCC_DATE_RESETVALUE;

		RTCC_IntEnable(RTCC_IEN_CC0);

		NVIC_EnableIRQ(RTCC_IRQn);
		RTCC_Enable(true);
		log_print_string("\nRTCC_Enable(): DONE");

    end_atomic();

    //hw_gpio_configure_pin({gpioPortD, 13}, 0, gpioModePushPull, 0);
    return SUCCESS;
}

hwtimer_tick_t hw_timer_getvalue(hwtimer_id_t timer_id)
{
	if(timer_id >= HWTIMER_NUM || (!timer_inited))
		return 0;
	else
	{
		uint32_t value = (uint16_t)(RTCC->CNT & 0x0000FFFF);
		return value;
	}
}

error_t hw_timer_schedule(hwtimer_id_t timer_id, hwtimer_tick_t tick )
{
	if(timer_id >= HWTIMER_NUM)
		return ESIZE;
	if(!timer_inited)
		return EOFF;

	start_atomic();
	   RTCC_IntDisable(RTCC_IEN_CC1);
	   //RTC_CompareSet( 1, tick );
	   RTCC_CCChConf_TypeDef rtccChInit = RTCC_CH_INIT_COMPARE_DEFAULT;

	   RTCC_ChannelInit(1, &rtccChInit);
	   RTCC_ChannelCCVSet(1, tick);
	   
	   RTCC_IntClear(RTCC_IFC_CC1);
	   RTCC_IntEnable(RTCC_IEN_CC1);
	end_atomic();
}

error_t hw_timer_cancel(hwtimer_id_t timer_id)
{
	if(timer_id >= HWTIMER_NUM)
		return ESIZE;
	if(!timer_inited)
		return EOFF;

	start_atomic();
	   RTCC_IntDisable(RTCC_IEN_CC1);
	   RTCC_IntClear(RTCC_IFC_CC1);
	end_atomic();
}

error_t hw_timer_counter_reset(hwtimer_id_t timer_id)
{
	if(timer_id >= HWTIMER_NUM)
		return ESIZE;
	if(!timer_inited)
		return EOFF;

	start_atomic();
		RTCC_IntDisable(RTCC_IEN_CC0 | RTCC_IEN_CC1);
		RTCC_IntClear(RTCC_IFC_CC0 | RTCC_IFC_CC1);
		//RTC_CounterReset();
		RTCC_Unlock();
		RTCC->PRECNT  = _RTCC_PRECNT_RESETVALUE;
 		RTCC->CNT     = _RTCC_CNT_RESETVALUE;
 		RTCC->TIME    = _RTCC_TIME_RESETVALUE;
 		RTCC->DATE    = _RTCC_DATE_RESETVALUE;

		RTCC_IntEnable(RTCC_IEN_CC0);
	end_atomic();

}

bool hw_timer_is_overflow_pending(hwtimer_id_t timer_id)
{
    if(timer_id >= HWTIMER_NUM)
	return false;
    start_atomic();
	//CC0 is used to limit thc RTC to 16 bits -> use this one to check
	bool is_pending = !!((RTCC_IntGet() & RTCC->IEN) & RTCC_IFS_CC0);
    end_atomic();
    return is_pending;	
}
bool hw_timer_is_interrupt_pending(hwtimer_id_t timer_id)
{
    if(timer_id >= HWTIMER_NUM)
	return false;

    start_atomic();
	bool is_pending = !!((RTCC_IntGet() & RTCC->IEN) & RTCC_IFS_CC1);
    end_atomic();
    return is_pending;	
}



INT_HANDLER(RTCC_IRQHandler)
{
	//retrieve flags. We 'OR' this with the enabled interrupts
	//since the CC1 flag may be set if it wasn't used before (compare register == 0 -> ifs flag set regardless of whether interrupt is enabled)
	//by AND ing with the IEN we make sure we only consider the flags of the ENABLED interrupts
	uint32_t flags = (RTCC_IntGet() & RTCC->IEN);
	RTCC_IntClear(RTCC_IFC_OF | RTCC_IFC_CC0 | RTCC_IFC_CC1);

	//evaluate flags to see which one(s) fired:
	if((flags & RTCC_IFS_CC0) && (overflow_f != 0x0))
		overflow_f();
	if((flags & RTCC_IFS_CC1))
	{
		RTCC_IntDisable(RTCC_IEN_CC1);
		
		static uint16_t lcd_toggle_pin;
		lcd_toggle_pin = (lcd_toggle_pin + 1) & 0x3FF;
		if(lcd_toggle_pin == 0)
			GPIO_PinOutToggle(gpioPortD, 13); // LCD_PIN_EXTCOMIN

		if(compare_f != 0x0)
			compare_f();
	}
}
