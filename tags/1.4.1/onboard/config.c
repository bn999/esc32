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

#include "config.h"
#include "fet.h"
#include "pwm.h"
#include "adc.h"
#include "run.h"
#include "serial.h"
#include "stm32f10x_flash.h"
#include "stm32f10x_rcc.h"
#include <string.h>
#include <math.h>

float p[CONFIG_NUM_PARAMS];

const char *configParameterStrings[] = {
    "CONFIG_VERSION",
    "STARTUP_MODE",
    "BAUD_RATE",
    "PTERM",
    "ITERM",
    "FF1TERM",
    "FF2TERM",
    "CL1TERM",
    "CL2TERM",
    "CL3TERM",
    "CL4TERM",
    "CL5TERM",
    "SHUNT_RESISTANCE",
    "MIN_PERIOD",
    "MAX_PERIOD",
    "BLANKING_MICROS",
    "ADVANCE",
    "START_VOLTAGE",
    "GOOD_DETECTS_START",
    "BAD_DETECTS_DISARM",
    "MAX_CURRENT",
    "SWITCH_FREQ",
    "MOTOR_POLES",
    "PWM_MIN_PERIOD",
    "PWM_MAX_PERIOD",
    "PWM_MIN_VALUE",
    "PWM_LO_VALUE",
    "PWM_HI_VALUE",
    "PWM_MAX_VALUE",
    "PWM_MIN_START",
    "PWM_RPM_SCALE",
    "FET_BRAKING"
};

const char *configFormatStrings[] = {
    "%f",	    // CONFIG_VERSION
    "%.0f",	    // STARTUP_MODE
    "%.0f baud",    // BAUD_RATE
    "%.3f",	    // PTERM
    "%.5f",	    // ITERM
    "%+e",	    // FF1TERM
    "%+e",	    // FF2TERM
    "%+e",	    // CL1TERM
    "%+e",	    // CL2TERM
    "%+e",	    // CL3TERM
    "%+e",	    // CL4TERM
    "%+e",	    // CL5TERM
    "%.3f mohms",   // SHUNT_RESISTANCE
    "%.0f us",	    // MIN_PERIOD
    "%.0f us",	    // MAX_PERIOD
    "%.0f us",	    // BLANKING_MICROS
    "%.2f Degs",    // ADVANCE
    "%.2f Volts",   // START_VOLTAGE
    "%.0f",	    // GOOD_DETECTS_START
    "%.0f",	    // BAD_DETECTS_DISARM
    "%.1f Amps",    // MAX_CURRENT
    "%.1f KHz",	    // SWITCH_FREQ
    "%.0f",	    // MOTOR_POLES
    "%.0f us",	    // PWM_MIN_PERIOD
    "%.0f us",	    // PWM_MAX_PERIOD
    "%.0f us",	    // PWM_MIN_VALUE
    "%.0f us",	    // PWM_LO_VALUE
    "%.0f us",	    // PWM_HI_VALUE
    "%.0f us",	    // PWM_MAX_VALUE
    "%.0f us",	    // PWM_MIN_START
    "%.0f RPM",	    // PWM_RPM_SCALE
    "%.0f"	    // FET_BRAKING
};

// AXi 2820/14, APC 14x4.7
//#define DEFAULT_MAX_CURRENT		35.0
//#define DEFAULT_FF1TERM		+8.772974e-08
//#define DEFAULT_FF2TERM		+1.195871e-03
//#define DEFAULT_CL1TERM		-2.534735e-01
//#define DEFAULT_CL2TERM		+1.213829e-03
//#define DEFAULT_CL3TERM		+6.616300e-02
//#define DEFAULT_CL4TERM		-5.980994e-05
//#define DEFAULT_CL5TERM		+5.675677e-01


// AXi 2212/34, APC 10x4.7
//#define DEFAULT_MAX_CURRENT		15.0
//#define DEFAULT_PTERM			0.25
//#define DEFAULT_PWM_RPM_SCALE		6500
//#define DEFAULT_FF1TERM              +7.23870e-08
//#define DEFAULT_FF2TERM              +1.40690e-03
//#define DEFAULT_CL1TERM              -3.05210e-01
//#define DEFAULT_CL2TERM              +1.54300e-03
//#define DEFAULT_CL3TERM              -1.42560e-02
//#define DEFAULT_CL4TERM              -9.82150e-05
//#define DEFAULT_CL5TERM              +1.49340e+00

// Scorpion custom 3008, custom CF 13x4 3 blade
//#define DEFAULT_MAX_CURRENT		10.0
//#define DEFAULT_FF1TERM		+1.019580e-07
//#define DEFAULT_FF2TERM		+2.137348e-03
//#define DEFAULT_CL1TERM		+3.114897e-01
//#define DEFAULT_CL2TERM		+1.840782e-03
//#define DEFAULT_CL3TERM		-7.671915e-02
//#define DEFAULT_CL4TERM		-5.943240e-05
//#define DEFAULT_CL5TERM		+1.566824e+00

// Scorpion custom 3008, custom CF 14x8 2 blade
//#define DEFAULT_MAX_CURRENT		10.0
//#define DEFAULT_FF1TERM		+1.460333e-07
//#define DEFAULT_FF2TERM		+2.027514e-03
//#define DEFAULT_CL1TERM		+4.384707e-01
//#define DEFAULT_CL2TERM		+1.895964e-03
//#define DEFAULT_CL3TERM		+2.488202e-01
//#define DEFAULT_CL4TERM		-7.540044e-05
//#define DEFAULT_CL5TERM		+8.006618e-01

// Scorpion S2505-1200KV, APC 8x4.5
//#define DEFAULT_MAX_CURRENT		6.0
//#define DEFAULT_PTERM			0.25
//#define DEFAULT_MOTOR_POLES		12
//#define DEFAULT_FF1TERM		+2.136259e-08
//#define DEFAULT_FF2TERM		+7.304788e-04
//#define DEFAULT_CL1TERM		+2.173815e-01
//#define DEFAULT_CL2TERM		+7.996285e-04
//#define DEFAULT_CL3TERM		+8.779305e-02
//#define DEFAULT_CL4TERM		-4.019083e-05
//#define DEFAULT_CL5TERM		+8.509188e-01

// Avroto 2814/11s, APC 14x4.7SF
//#define DEFAULT_FF1TERM		+1.998525e-07
//#define DEFAULT_FF2TERM		+9.514660e-04
//#define DEFAULT_MAX_CURRENT		20.0
//#define DEFAULT_PWM_RPM_SCALE		5500
//#define DEFAULT_CL1TERM		-2.301245e-02
//#define DEFAULT_CL2TERM		+1.381919e-03
//#define DEFAULT_CL3TERM		+2.570513e-01
//#define DEFAULT_CL4TERM		-1.064467e-04
//#define DEFAULT_CL5TERM		+5.656897e-01

// Avroto 2814/11s, APC 12x3.8SF
//#define DEFAULT_FF1TERM		+1.028551e-07
//#define DEFAULT_FF2TERM		+9.977381e-04
//#define DEFAULT_MAX_CURRENT		20.0
//#define DEFAULT_PWM_RPM_SCALE		7500
//#define DEFAULT_CL1TERM		-8.272934e-01
//#define DEFAULT_CL2TERM		+1.385665e-03
//#define DEFAULT_CL3TERM		+1.335600e-01
//#define DEFAULT_CL4TERM		-9.868111e-05
//#define DEFAULT_CL5TERM		+1.173475e+00

// Avroto 2814/11s, APC 11x4.7SF
//#define DEFAULT_FF1TERM		+7.562404e-08
//#define DEFAULT_FF2TERM		+1.116254e-03
//#define DEFAULT_MAX_CURRENT		20.0
//#define DEFAULT_PWM_RPM_SCALE		8125
//#define DEFAULT_CL1TERM		-7.778534e-01
//#define DEFAULT_CL2TERM		+1.385586e-03
//#define DEFAULT_CL3TERM		+1.559464e-01
//#define DEFAULT_CL4TERM		-9.598113e-05
//#define DEFAULT_CL5TERM		+1.091966e+00

// RCTimer 5010-14, RCTimer 15x5.5 CF
//#define DEFAULT_FF1TERM		+1.663210e-07
//#define DEFAULT_FF2TERM		+3.019136e-03
//#define DEFAULT_MAX_CURRENT		10.0
//#define DEFAULT_PWM_RPM_SCALE		4200
//#define DEFAULT_CL1TERM		+4.023476e-01
//#define DEFAULT_CL2TERM		+2.972955e-03
//#define DEFAULT_CL3TERM		+3.942016e-01
//#define DEFAULT_CL4TERM		-2.607788e-04
//#define DEFAULT_CL5TERM		+1.195345e+00


void configInit(void) {
    float ver;

    configLoadDefault();

    ver = *(float *)FLASH_WRITE_ADDR;

    if (isnan(ver))
	configWriteFlash();
    else if (ver >= p[CONFIG_VERSION])
	configReadFlash();
    else if (p[CONFIG_VERSION] > ver)
	configWriteFlash();
}

// recalculate constants with bounds checking
void configRecalcConst(void) {
    adcSetConstants();
    fetSetConstants();
    runSetConstants();
    pwmSetConstants();
    serialSetConstants();
}

int configSetParamByID(int i, float value) {
    int ret = 0;

    if (i >= 0 && i < CONFIG_NUM_PARAMS) {
	p[i] = value;
	configRecalcConst();

	ret = 1;
    }

    return ret;
}

int configSetParam(char *param, float value) {
    int ret = 0;
    int i;

    for (i = 0; i < CONFIG_NUM_PARAMS; i++) {
	if (!strncasecmp(configParameterStrings[i], param, strlen(configParameterStrings[i]))) {
	    configSetParamByID(i, value);
	    ret = 1;
	    break;
	}
    }

    return ret;
}

int configGetId(char *param) {
    int i;

    for (i = 0; i < CONFIG_NUM_PARAMS; i++)
	if (!strncasecmp(configParameterStrings[i], param, strlen(configParameterStrings[i])))
	    return i;

    return -1;
}

float configGetParam(char *param) {
    int i;

    i = configGetId(param);

    if (i >= 0)
	return p[i];
    else
	return __float32_nan;
}

void configLoadDefault(void) {
    p[CONFIG_VERSION] = DEFAULT_CONFIG_VERSION;
    p[STARTUP_MODE] = DEFAULT_STARTUP_MODE;
    p[BAUD_RATE] = DEFAULT_BAUD_RATE;
    p[PTERM] = DEFAULT_PTERM;
    p[ITERM] = DEFAULT_ITERM;
    p[FF1TERM] = DEFAULT_FF1TERM;
    p[FF2TERM] = DEFAULT_FF2TERM;
    p[CL1TERM] = DEFAULT_CL1TERM;
    p[CL2TERM] = DEFAULT_CL2TERM;
    p[CL3TERM] = DEFAULT_CL3TERM;
    p[CL4TERM] = DEFAULT_CL4TERM;
    p[CL5TERM] = DEFAULT_CL5TERM;
    p[SHUNT_RESISTANCE] = DEFAULT_SHUNT_RESISTANCE;
    p[MIN_PERIOD] = DEFAULT_MIN_PERIOD;
    p[MAX_PERIOD] = DEFAULT_MAX_PERIOD;
    p[BLANKING_MICROS] = DEFAULT_BLANKING_MICROS;
    p[ADVANCE] = DEFAULT_ADVANCE;
    p[START_VOLTAGE] = DEFAULT_START_VOLTAGE;
    p[GOOD_DETECTS_START] = DEFAULT_GOOD_DETECTS_START;
    p[BAD_DETECTS_DISARM] = DEFAULT_BAD_DETECTS_DISARM;
    p[MAX_CURRENT] = DEFAULT_MAX_CURRENT;
    p[SWITCH_FREQ] = DEFAULT_SWITCH_FREQ;
    p[MOTOR_POLES] = DEFAULT_MOTOR_POLES;
    p[PWM_MIN_PERIOD] = DEFAULT_PWM_MIN_PERIOD;
    p[PWM_MAX_PERIOD] = DEFAULT_PWM_MAX_PERIOD;
    p[PWM_MIN_VALUE] = DEFAULT_PWM_MIN_VALUE;
    p[PWM_LO_VALUE] = DEFAULT_PWM_LO_VALUE;
    p[PWM_HI_VALUE] = DEFAULT_PWM_HI_VALUE;
    p[PWM_MAX_VALUE] = DEFAULT_PWM_MAX_VALUE;
    p[PWM_MIN_START] = DEFAULT_PWM_MIN_START;
    p[PWM_RPM_SCALE] = DEFAULT_PWM_RPM_SCALE;
    p[FET_BRAKING] = DEFAULT_FET_BRAKING;

    configRecalcConst();
}

int configWriteFlash(void) {
    uint16_t prevReloadVal;
    FLASH_Status FLASHStatus;
    uint32_t address;
    int ret = 0;

    prevReloadVal = runIWDGInit(999);

    // Startup HSI clock
    RCC_HSICmd(ENABLE);
    while (RCC_GetFlagStatus(RCC_FLAG_HSIRDY) != SET)
	runFeedIWDG();

    FLASH_Unlock();

    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    if ((FLASHStatus = FLASH_ErasePage(FLASH_WRITE_ADDR)) == FLASH_COMPLETE) {
	address = 0;
	while (FLASHStatus == FLASH_COMPLETE && address < sizeof(p)) {
	    if ((FLASHStatus = FLASH_ProgramWord(FLASH_WRITE_ADDR + address, *(uint32_t *)((char *)p + address))) != FLASH_COMPLETE)
		break;

	    address += 4;
	}

	ret = 1;
    }

    FLASH_Lock();

    runFeedIWDG();

    // Shutdown HSI clock
    RCC_HSICmd(DISABLE);
    while (RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == SET)
	;

    runIWDGInit(prevReloadVal);

    return ret;
}

void configReadFlash(void) {
    memcpy(p, (char *)FLASH_WRITE_ADDR, sizeof(p));

    configRecalcConst();
}
