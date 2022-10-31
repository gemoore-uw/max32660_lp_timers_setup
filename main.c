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
#include <string.h>

/***** Definitions *****/
#define MAX32660_EVSYS_LED								0

#define GPIO_PORT_OUT               			PORT_0
#define GPIO_PIN_OUT                			PIN_12

#define RX_DEMODULATE_PORT_IN       			PORT_0
#define RX_DEMODULATE_PIN_IN        			PIN_5

#define GPIO_PORT_SQUARE_WAVE							PORT_0
#define GPIO_PIN_SQUARE_WAVE							PIN_11

#define CONT_TIMER 												MXC_TMR1   /* Timer used by ContinuousTimer */

#define NANO_CLK_POWER_DOWN_SLOPE 				0.030627257F
#define NANO_CLK_POWER_DOWN_INTERCEPT	 		-603.4746337F

#define LP_HFIO_CLK_POWER_DOWN_SLOPE 			1.003914467F
#define LP_HFIO_CLK_POWER_DOWN_INTERCEPT	-80.48833525F

/************************* Variable Definitions *************************/

gpio_cfg_t gpio_out;															// P0_12
//gpio_cfg_t gpio_square_wave;															// P0_11
gpio_cfg_t gpio_rx_demodulate;										// P0_5
gpio_cfg_t gpio_interrupt_status_rx_demodulate;
		 

//extern int mxc_delay_start(unsigned long us);
//extern void mxc_delay_handler(void);
bool is_mcu_powered_down = FALSE;
float divisor = 1000000; //995851;
bool TIMER_FINISHED = FALSE;
bool is_rx_demod_counter_running = FALSE;
/**
 * Enumeration type for system state select.
 */
	typedef enum {
		POWERED_DOWN_MCU_AFE4404,
		RX_DEMODULATE,
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
double running_avg = 0;
#define MAX_RX_DEMOD_SEQ_BITS				16
#define RX_DEMOD_PREAMBLE_BITS			8
uint32_t elapsed_time[MAX_RX_DEMOD_SEQ_BITS];
bool rx_demod_sequence_level[MAX_RX_DEMOD_SEQ_BITS];
uint16_t rx_demod_isr_cnt = 0;
bool rx_demod_is_preamble_good = FALSE;
uint32_t rx_demod_bit_period_us = 1e6;
void gpio_isr_rx_demod(void *cbdata)
{
	static uint32_t running_sum_elapsed_time = 0;
	
	//	LED_Toggle(MAX32660_EVSYS_LED);
//	GPIO_OutToggle(&gpio_out);
	
	if(rx_demod_isr_cnt == 0){
		TMR_SW_Start(MXC_TMR2, 0);
		running_sum_elapsed_time = 0;
		for(int8_t idx = MAX_RX_DEMOD_SEQ_BITS-1; idx >= 0; idx--){
			rx_demod_sequence_level[idx] = 0;
			elapsed_time[idx] = 0;
		}
//		printf("\n\n");
		//Avoiding starting on a one to zero transition
		if(!((bool)GPIO_InGet(&gpio_rx_demodulate)))
			rx_demod_isr_cnt++;
		else
			rx_demod_isr_cnt = 0;
		return;
	}
	else if (rx_demod_isr_cnt > MAX_RX_DEMOD_SEQ_BITS){
		TMR_SW_Stop(MXC_TMR2);
		return;
	}
	
	memmove(&elapsed_time[1], &elapsed_time[0], (MAX_RX_DEMOD_SEQ_BITS-1)*sizeof(uint32_t));
	elapsed_time[0] = TMR_TO_Elapsed(MXC_TMR2)-running_sum_elapsed_time; //TMR_SW_Stop(MXC_TMR2); //??? TMR_TO_Elapsed(MXC_TMR2)
	running_sum_elapsed_time = running_sum_elapsed_time + elapsed_time[0];
//	for(int8_t idx = MAX_RX_DEMOD_SEQ_BITS-1; idx >= 0; idx--){
//		printf("%d,", elapsed_time[idx]);
//	}
//	printf("\n");
	
	// Checking the preamble
	if(RX_DEMOD_PREAMBLE_BITS == rx_demod_isr_cnt){
		rx_demod_is_preamble_good = TRUE;
		for(int8_t idx = RX_DEMOD_PREAMBLE_BITS-1; idx >= 0; idx--){
			if(idx > 0){
				if(rx_demod_sequence_level[idx] == rx_demod_sequence_level[idx-1]){
					rx_demod_is_preamble_good = FALSE;
					rx_demod_isr_cnt = 0;
					TMR_SW_Stop(MXC_TMR2);
					printf("Bad Preamble at rx_demod_bit_period_us[%d]=%d and rx_demod_bit_period_us[%d]=%d\n",idx,(int8_t) rx_demod_sequence_level[idx], idx-1,(int8_t) rx_demod_sequence_level[idx-1]);
					for(int8_t idx = MAX_RX_DEMOD_SEQ_BITS-1; idx >= 0; idx--){
						printf("%d,", (uint8_t)rx_demod_sequence_level[idx]);
					}
					printf("\n");
					return;
				}
			}
		}
	}
	
	if(RX_DEMOD_PREAMBLE_BITS >= rx_demod_isr_cnt){
		rx_demod_bit_period_us = round(running_sum_elapsed_time/(float)(rx_demod_isr_cnt));
//		printf("\nrx_demod_bit_period_us = %d| rx_demod_isr_cnt = %d\n\n", rx_demod_bit_period_us, rx_demod_isr_cnt);
	}

	for(uint8_t idx = 0; idx < round(elapsed_time[0]/((float)rx_demod_bit_period_us)); idx++){
		memmove(&rx_demod_sequence_level[1], &rx_demod_sequence_level[0], (MAX_RX_DEMOD_SEQ_BITS-1)*sizeof(bool));
	}
	rx_demod_sequence_level[0] = (bool)GPIO_InGet(&gpio_rx_demodulate);
	
//	for(int8_t idx = MAX_RX_DEMOD_SEQ_BITS-1; idx >= 0; idx--){
//		printf("%d,", (uint8_t)rx_demod_sequence_level[idx]);
//	}
//	printf("\n");
		
	rx_demod_isr_cnt++;

		
	//	printf("\nGPIO ISR Count: %d\n", rx_demod_isr_cnt);
	// !!! ToDo: Correct for TMR2 overrun
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
    gpio_out.pad = GPIO_PAD_NONE;
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
		GPIO_IntConfig(&gpio_rx_demodulate, GPIO_INT_EDGE, GPIO_INT_BOTH);
		GPIO_IntEnable(&gpio_rx_demodulate);
		NVIC_EnableIRQ((IRQn_Type)MXC_GPIO_GET_IRQ(RX_DEMODULATE_PORT_IN));
	
    printf("\n************ max32660_evsys_lp_timer Setup ****************\n");
    uint32_t error;
	
//		SYS_Clock_Select(SYS_CLOCK_NANORING, MXC_TMR0);
		
		uint32_t us_delay = 5000000; //round(340000/2);
		uint32_t us_delay_calibrated = round(LP_HFIO_CLK_POWER_DOWN_SLOPE*((float)us_delay) + LP_HFIO_CLK_POWER_DOWN_INTERCEPT);
		float measure_frequency = .3;
		uint32_t interstitial_period_us = round(1/measure_frequency*1000000);
		uint32_t interstitial_period_us_calibrated = round(NANO_CLK_POWER_DOWN_SLOPE*((float)interstitial_period_us) + NANO_CLK_POWER_DOWN_INTERCEPT);
    
//    error = SYS_SysTick_Config(sysTicks, USE_SYSTEM_CLK, MXC_TMR0);
		
    printf("SysTick (SystemCoreClock) Clock = %d Hz\n", SYS_SysTick_GetFreq());
		printf("Peripheral Clock = %d Hz\n", PeripheralClock);
		printf("Timer Clock = %d Hz\n", SYS_TMR_GetFreq(MXC_TMR0));
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
    LED_Off(MAX32660_EVSYS_LED);
    is_mcu_powered_down = FALSE;
		system_state_t sys_state = RX_DEMODULATE;
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
				case RX_DEMODULATE:
//					GPIO_OutToggle(&gpio_square_wave);
					mxc_delay(us_delay_calibrated);
//					for(uint8_t idx = 0; idx < MAX_RX_DEMOD_SEQ_BITS; idx++){
//						running_avg = running_avg+elapsed_time[idx];
//						printf("%d,", elapsed_time[idx]);
//					}
//					printf("\n");
//					for(uint8_t idx = 0; idx < MAX_RX_DEMOD_SEQ_BITS; idx++){
//						printf("%d,", rx_demod_sequence_level[idx]);
//					}
//					printf("\n");
//					running_avg = (running_avg+elapsed_time)/(double)2.0;
					running_avg = running_avg/MAX_RX_DEMOD_SEQ_BITS;
//					printf("\rRunning Average: %g\n", running_avg);
					if(rx_demod_isr_cnt >= MAX_RX_DEMOD_SEQ_BITS || TMR_TO_Elapsed(MXC_TMR2) > rx_demod_bit_period_us*MAX_RX_DEMOD_SEQ_BITS*1.1){
						
						//manchester_decode(&elapsed_time[0], &rx_demod_sequence_level[0], rx_demod_isr_cnt);
						//afe4404_reg_write (uint8_t reg_address, uint32_t data) //Note: largest possible data entry = LHS 20 DYNAMIC1_FIELD (e.g. 21st bit)
						printf("\nrx_demod_bit_period_us = %d\n\n", rx_demod_bit_period_us);
						printf("RX Demodulate Sequence: ");
						for(int8_t idx = MAX_RX_DEMOD_SEQ_BITS-1; idx >= 0; idx--){
							running_avg = running_avg+elapsed_time[idx];
							printf("%d,", elapsed_time[idx]);
						}
						printf("\n");
						for(int8_t idx = MAX_RX_DEMOD_SEQ_BITS-1; idx >= 0; idx--){
							printf("%d,", (uint8_t)rx_demod_sequence_level[idx]);
						}
						printf("\n");
						rx_demod_isr_cnt = 0;
						TMR_SW_Stop(MXC_TMR2);
					}
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
