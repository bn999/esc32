/*
    This file is part of AutoQuad ESC32.

    AutoQuad ESC32 is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    AutoQuad ESC32 is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with AutoQuad ESC32.  If not, see <http://www.gnu.org/licenses/>.

    Copyright © 2011, 2012  Bill Nesbitt
*/

#include "main.h"
#include "rcc.h"
#include "config.h"
#include "timer.h"
#include "fet.h"
#include "serial.h"
#include "pwm.h"
#include "adc.h"
#include "run.h"
#include "cli.h"
#include "ow.h"

digitalPin *errorLed, *statusLed;
#ifdef ESC_DEBUG
digitalPin *tp;
#endif

volatile uint32_t minCycles, idleCounter, totalCycles;
volatile uint8_t state, inputMode;

char buf[64];

void main(void) {
    rccInit();
    timerInit();
    configInit();
    adcInit();
    fetInit();
    serialInit();
    runInit();
    cliInit();
    owInit();

    runDisarm(REASON_STARTUP);
    inputMode = ESC_INPUT_PWM;

    fetSetDutyCycle(0);
    fetBeep(200, 100);
    fetBeep(300, 100);
    fetBeep(400, 100);
    fetBeep(500, 100);
    fetBeep(400, 100);
    fetBeep(300, 100);
    fetBeep(200, 100);

    pwmInit();

    statusLed = digitalInit(GPIO_STATUS_LED_PORT, GPIO_STATUS_LED_PIN);
    digitalHi(statusLed);
    errorLed = digitalInit(GPIO_ERROR_LED_PORT, GPIO_ERROR_LED_PIN);
    digitalHi(errorLed);
#ifdef ESC_DEBUG
    tp = digitalInit(GPIO_TP_PORT, GPIO_TP_PIN);
    digitalLo(tp);
#endif

    // self calibrating idle timer loop
    {
        volatile unsigned long cycles;
        volatile unsigned int *DWT_CYCCNT = (int *)0xE0001004;
        volatile unsigned int *DWT_CONTROL = (int *)0xE0001000;
        volatile unsigned int *SCB_DEMCR = (int *)0xE000EDFC;

        *SCB_DEMCR = *SCB_DEMCR | 0x01000000;
        *DWT_CONTROL = *DWT_CONTROL | 1;	// enable the counter

	minCycles = 0xffff;
        while (1) {
            idleCounter++;

	    NOPS_4;

            cycles = *DWT_CYCCNT;
            *DWT_CYCCNT = 0;		    // reset the counter

            // record shortest number of instructions for loop
	    totalCycles += cycles;
            if (cycles < minCycles)
                minCycles = cycles;
        }
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line) {
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  while (1) {
    NOP;
  }
}
#endif

void HardFault_Handler(void) {
    FET_PANIC;
    while (1)
	;
}

void MemManage_Handler(void) {
    FET_PANIC;
    while (1)
	;
}

void BusFault_Handler(void) {
    FET_PANIC;
    while (1)
	;
}

void UsageFault_Handler(void) {
    FET_PANIC;
    while (1)
	;
}

void reset_wait(void) {
    FET_PANIC;
    while (1)
	;
}


