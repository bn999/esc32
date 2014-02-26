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

    Copyright © 2011-2014  Bill Nesbitt
*/

#define DEFAULT_PORT		"/dev/cu.usbserial-FTELPQ1S"
#define DEFAULT_BAUD		230400

enum binaryCommands {
    BINARY_COMMAND_NOP = 0,
    BINARY_COMMAND_ARM,
    BINARY_COMMAND_CLI,
    BINARY_COMMAND_CONFIG,
    BINARY_COMMAND_DISARM,
    BINARY_COMMAND_DUTY,
    BINARY_COMMAND_PWM,
    BINARY_COMMAND_RPM,
    BINARY_COMMAND_SET,
    BINARY_COMMAND_START,
    BINARY_COMMAND_STATUS,
    BINARY_COMMAND_STOP,
    BINARY_COMMAND_TELEM_RATE,
    BINARY_COMMAND_VERSION,
    BINARY_COMMAND_TELEM_VALUE,
    BINARY_COMMAND_GET_PARAM_ID,
    BINARY_COMMAND_ACK = 250,
    BINARY_COMMAND_NACK
};

enum binaryValues {
    BINARY_VALUE_NONE = 0,
    BINARY_VALUE_AMPS,
    BINARY_VALUE_VOLTS_BAT,
    BINARY_VALUE_VOLTS_MOTOR,
    BINARY_VALUE_RPM,
    BINARY_VALUE_DUTY,
    BINARY_VALUE_COMM_PERIOD,
    BINARY_VALUE_BAD_DETECTS,
    BINARY_VALUE_ADC_WINDOW,
    BINARY_VALUE_IDLE_PERCENT,
    BINARY_VALUE_STATE,
    BINARY_VALUE_AVGA,
    BINARY_VALUE_AVGB,
    BINARY_VALUE_AVGC,
    BINARY_VALUE_AVGCOMP,
    BINARY_VALUE_FETSTEP,
    BINARY_VALUE_NUM
};

enum configParameters {
    CONFIG_VERSION = 0,
    STARTUP_MODE,
    BAUD_RATE,
    PTERM,
    ITERM,
    FF1TERM,
    FF2TERM,
    CL1TERM,
    CL2TERM,
    CL3TERM,
    CL4TERM,
    CL5TERM,
    SHUNT_RESISTANCE,
    MIN_PERIOD,
    MAX_PERIOD,
    BLANKING_MICROS,
    ADVANCE,
    START_VOLTAGE,
    GOOD_DETECTS_START,
    BAD_DETECTS_DISARM,
    MAX_CURRENT,
    SWITCH_FREQ,
    MOTOR_POLES,
    PWM_MIN_PERIOD,
    PWM_MAX_PERIOD,
    PWM_MIN_VALUE,
    PWM_LO_VALUE,
    PWM_HI_VALUE,
    PWM_MAX_VALUE,
    PWM_MIN_START,
    PWM_RPM_SCALE,
    FET_BRAKING,
    PNFAC,
    INFAC,
    THR1TERM,
    THR2TERM,
    START_ALIGN_TIME,
    START_ALIGN_VOLTAGE,
    START_STEPS_NUM,
    START_STEPS_PERIOD,
    START_STEPS_ACCEL,
    PWM_LOWPASS,
    RPM_MEAS_LP,
    SERVO_DUTY,
    SERVO_P,
    SERVO_D,
    SERVO_MAX_RATE,
    SERVO_SCALE,
    ESC_ID,
    DIRECTION,
    CONFIG_NUM_PARAMS
};
