/**
 * @file    main.c
 * @brief   Demonstrates the SysTick timer and interrupt. Toggles LED0 every SysTick period.
 */

/*******************************************************************************
 * Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 * $Date: 2018-08-09 18:45:02 -0500 (Thu, 09 Aug 2018) $
 * $Revision: 36818 $
 *
 ******************************************************************************/

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include "mxc_config.h"
#include "mxc_sys.h"
#include "nvic_table.h"
#include "board.h"
#include "led.h"
#include "mxc_delay.h"
#include "lp.h"

#include "tmr.h"
#include "tmr_utils.h"

#include <stdbool.h>
#include <math.h>

/***** Definitions *****/
#define MAX32660_EVSYS_LED								0
#define GPIO_PORT_OUT               			PORT_0
#define GPIO_PIN_OUT                			PIN_12

#define RX_DEMODULATE_PORT_IN       			PORT_0
#define RX_DEMODULATE_PIN_IN        			PIN_5

#define CONT_TIMER 												MXC_TMR1   /* Timer used by ContinuousTimer */

#define NANO_CLK_POWER_DOWN_SLOPE 				0.030627257F
#define NANO_CLK_POWER_DOWN_INTERCEPT	 		-603.4746337F

#define LP_HFIO_CLK_POWER_DOWN_SLOPE 			1.003914467F
#define LP_HFIO_CLK_POWER_DOWN_INTERCEPT	-80.48833525F

/************************* Variable Definitions *************************/

gpio_cfg_t gpio_out;															// P0_12
gpio_cfg_t gpio_rx_demodulate;										// P0_5
gpio_cfg_t gpio_interrupt_status_rx_demodulate;
		 

//extern int mxc_delay_start(unsigned long us);
//extern void mxc_delay_handler(void);
bool is_mcu_powered_down = FALSE;
float divisor = 1000000; //995851;
bool TIMER_FINISHED = FALSE;
/**
 * Enumeration type for system state select.
 */
	typedef enum {
		POWERED_DOWN_MCU_AFE4404,
		POWERED_DOWN_AFE4404,
		ACTIVE_ALL
	} system_state_t;
	
/************************* Functions *************************/

//void SysTick_Handler(void)
//{
//    //Toggle LED0 every systick period
//    LED_Toggle(0);
//		if (GPIO_OutGet(&gpio_out))
//			GPIO_OutClr(&gpio_out);
//		else
//			GPIO_OutSet(&gpio_out);
////		LED_Off(0);
//}

void gpio_isr_rx_demod(void *cbdata) // I2C interrupt will be triggered by ADC_RDY going High for 200ns
{
	static uint16_t isr_cnt = 0;
	isr_cnt++;
	printf("\nGPIO ISR: %d\n", isr_cnt);
	LED_Toggle(MAX32660_EVSYS_LED);
	GPIO_OutToggle(&gpio_out);
	
//	int64_t rx_demod_period = SysTick->VAL - systick_prev;
//	uint8_t bits = 0;
//	static uint8_t bit_idx = 0;
//	if(rx_demod_period > RX_DEMOD_PERIOD*(1-RX_DEMOD_ALLOWABLE_ERROR) && rx_demod_period < RX_DEMOD_PERIOD*(1+RX_DEMOD_ALLOWABLE_ERROR))
//		rx_demod_toggle_cnt++;
//	
//	if (rx_demod_toggle_cnt >= RX_DEMOD_TOGGLE_CNT_MIN && FALSE == rx_demod_data_rdy){
//		rx_demod_record_data = TRUE;
//	}
//	else if(TRUE == rx_demod_record_data && rx_demod_toggle_cnt < sizeof(uint32_t)){
//		bits = round(rx_demod_period/(float)RX_DEMOD_PERIOD);
//		while(bits > 0){
//			bits--;
//			bitWrite(rx_demod_data, bit_idx, GPIO_InGet(&gpio_rx_demodulate));
//			bit_idx++;
//		}
//	}
//	else if(TRUE == rx_demod_record_data && bit_idx >= sizeof(uint32_t)){
//		rx_demod_data_rdy  = TRUE;
//		rx_demod_toggle_cnt = 0;
//		bit_idx = 0;
//	}
//	
//	systick_prev = SysTick->VAL;
}

void ContinuousTimer_Handler(void)
{
    /* Clear interrupt */
    TMR_IntClear(CONT_TIMER);
		TMR_Disable(CONT_TIMER);
}

	void power_down_mcu(void){
	LP_EnableSysRAM3LightSleep();
	LP_EnableSysRAM2LightSleep();
	LP_EnableSysRAM1LightSleep();
	LP_DisableSysRAM0LightSleep(); // Stack and global variables are in RAM0
	LP_DisableSRAM3();
	LP_DisableSRAM2();
	LP_DisableSRAM1();
	LP_EnableSRAM0(); // Stack and global variables are in RAM0
	LP_DisableBlockDetect();
	SYS_Clock_Select(SYS_CLOCK_NANORING, MXC_TMR0);
	LP_ClearWakeStatus();
	LP_EnterSleepMode();
}

void power_up_mcu(void){
	SYS_Clock_Select(SYS_CLOCK_HIRC, MXC_TMR0);
	LP_EnableBlockDetect();	
	LP_EnableSRAM1();	
	LP_EnableSRAM2();
	LP_EnableSRAM3();
	LP_DisableSysRAM1LightSleep();
	LP_DisableSysRAM2LightSleep();
	LP_DisableSysRAM3LightSleep();
}

void ContinuousTimer(uint32_t period_ticks)
{
    tmr_cfg_t tmr; 

    TMR_Disable(CONT_TIMER);
    
    TMR_Init(CONT_TIMER, TMR_PRES_1, 0);
    
    tmr.mode = TMR_MODE_CONTINUOUS;
    tmr.cmp_cnt = period_ticks;
    tmr.pol = 0;
    TMR_Config(CONT_TIMER, &tmr);

    TMR_Enable(CONT_TIMER);
}


int main(void)
{
		/* Setup output pin on P0_12. */
		gpio_out.port = GPIO_PORT_OUT;
    gpio_out.mask = GPIO_PIN_OUT;
    gpio_out.pad = GPIO_PAD_PULL_UP;
    gpio_out.func = GPIO_FUNC_OUT;
    GPIO_Config(&gpio_out);
		GPIO_OutClr(&gpio_out);
	
		/* Setup output pin on P0_5. */
		gpio_rx_demodulate.port = RX_DEMODULATE_PORT_IN;
		gpio_rx_demodulate.mask = RX_DEMODULATE_PIN_IN;
		gpio_rx_demodulate.pad = GPIO_PAD_NONE; // GPIO_PAD_PULL_DOWN ???
		gpio_rx_demodulate.func = GPIO_FUNC_IN;
		GPIO_Config(&gpio_rx_demodulate);

		GPIO_RegisterCallback(&gpio_rx_demodulate, gpio_isr_rx_demod, &gpio_interrupt_status_rx_demodulate);
		GPIO_IntConfig(&gpio_interrupt_status_rx_demodulate, GPIO_INT_EDGE, GPIO_INT_BOTH);
		GPIO_IntEnable(&gpio_rx_demodulate);
		NVIC_EnableIRQ((IRQn_Type)MXC_GPIO_GET_IRQ(RX_DEMODULATE_PORT_IN));
	
    printf("\n************ max32660_evsys_lp_timer Setup ****************\n");
    uint32_t error;
	
//		SYS_Clock_Select(SYS_CLOCK_NANORING, MXC_TMR0);
		
		uint32_t us_delay = 100; //round(340000/2);
		uint32_t us_delay_calibrated = round(LP_HFIO_CLK_POWER_DOWN_SLOPE*((float)us_delay) + LP_HFIO_CLK_POWER_DOWN_INTERCEPT);
		float measure_frequency = .3;
		uint32_t interstitial_period_us = round(1/measure_frequency*1000000);
		uint32_t interstitial_period_us_calibrated = round(NANO_CLK_POWER_DOWN_SLOPE*((float)interstitial_period_us) + NANO_CLK_POWER_DOWN_INTERCEPT);
    
//    error = SYS_SysTick_Config(sysTicks, USE_SYSTEM_CLK, MXC_TMR0);
		
    printf("SysTick (SystemCoreClock) Clock = %d Hz\n", SYS_SysTick_GetFreq());
		printf("PeripheralClock = %d Hz\n", PeripheralClock);
		printf("Delay = %dus\n", us_delay);
		printf("Delay Calibrated = %dus\n", us_delay_calibrated);
		uint32_t ticks = (uint32_t)(((double)us_delay_calibrated * (double)SystemCoreClock) / divisor);
		printf("Delay Calibrated Ticks SysTick = %d\n", ticks);
    printf("Interstitial Period = %dus\n", interstitial_period_us);
		printf("Interstitial Period Calibrated = %dus\n", interstitial_period_us_calibrated);
		ticks = (uint32_t)(((double)interstitial_period_us_calibrated * (double)SystemCoreClock) / divisor);
		printf("Interstitial Period Ticks TMR1 = %d\n", ticks);
    
    if (error != E_NO_ERROR) {
        printf("ERROR: Ticks is not valid");
        LED_On(2);
    }
    
    is_mcu_powered_down = FALSE;
		system_state_t sys_state = POWERED_DOWN_MCU_AFE4404;
    NVIC_SetVector(TMR1_IRQn, ContinuousTimer_Handler);
    NVIC_EnableIRQ(TMR1_IRQn);
    while (1) {
			switch (sys_state){
				case POWERED_DOWN_MCU_AFE4404:
					power_up_mcu();
					LED_On(MAX32660_EVSYS_LED);
					GPIO_OutSet(&gpio_out);	
//					ticks = (uint32_t)(((double)us * (double)SystemCoreClock) / divisor);
//					printf("SysTick Period POWERED_DOWN_MCU_AFE4404  = %d ticks\n", ticks);
					mxc_delay(us_delay_calibrated);
					GPIO_OutClr(&gpio_out);
					LED_Off(MAX32660_EVSYS_LED);
					mxc_delay(us_delay_calibrated);
					sys_state = ACTIVE_ALL;
					break;
				case POWERED_DOWN_AFE4404:
					mxc_delay(us_delay_calibrated);
					break;
				case ACTIVE_ALL:
//					ticks = (uint32_t)(((double)us*4 * (double)SystemCoreClock) / divisor);
//					printf("SysTick Period ACTIVE_ALL = %d ticks\n", ticks);
//					printf("Entering Power Down\n");
					
//					LED_On(0);
					/* The delay function can also be used for similar functionality */
					GPIO_OutClr(&gpio_out);	
					LED_Off(MAX32660_EVSYS_LED);			
					ContinuousTimer(interstitial_period_us_calibrated);
					power_down_mcu();
//					printf("Exiting Power Down\n");
					sys_state = POWERED_DOWN_MCU_AFE4404;
					break;
			}
//			if(FALSE == is_mcu_powered_down){
////				printf("Here0\n");
//				is_mcu_powered_down = TRUE;
//				mxc_delay_start(us);
//			}
			

		}
}
