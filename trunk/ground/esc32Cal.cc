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

#include "esc32.h"
#include "serial.h"
#include "plplot/plplot.h"
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <termios.h>
#include <fcntl.h>
#include <getopt.h>
#include <stdlib.h>
#include <pthread.h>
#include <math.h>
#include <signal.h>

//#define ESC32_DEBUG

//#define EIGEN_NO_DEBUG
//#define EIGEN_DONT_VECTORIZE
#define EIGEN_DONT_PARALLELIZE  // don't use openmp

#include <Eigen/Core>
#include <Eigen/LU>

using namespace Eigen;

#define DEFAULT_MAX_AMPS        30.0
#define MAX_TELEM_STORAGE	200000

unsigned char checkInA, checkInB;
unsigned char checkOutA, checkOutB;
char sendBuf[128];
unsigned char sendBufPtr;
volatile float telemValueAvgs[BINARY_VALUE_NUM];
volatile float telemValueMaxs[BINARY_VALUE_NUM];
volatile float telemData[256][BINARY_VALUE_NUM];
float *telemStorage;
volatile int telemStorageNum;
float maxAmps;
int runR2V, runCL;
FILE* telemOutFile;
volatile unsigned char lastAck;
volatile unsigned short lastSeqId = -1;
volatile short paramId;
pthread_t threadIn;
pthread_mutex_t threadMutex = PTHREAD_MUTEX_INITIALIZER;
serialStruct_t *s;
unsigned short commandSeqId = 1;

char port[256];
unsigned int baud;

void esc32Send(void) {
	serialWrite(s, sendBuf, sendBufPtr);
}

void esc32OutChecksum(unsigned char c) {
	checkOutA += c;
	checkOutB += checkOutA;
}

void esc32InChecksum(unsigned char c) {
	checkInA += c;
	checkInB += checkInA;
}

unsigned char esc32GetChar(serialStruct_t *s) {
	unsigned char c;

	c = serialRead(s);
	esc32InChecksum(c);

	return c;
}

void esc32SendChar(unsigned char c) {
	sendBuf[sendBufPtr++] = c;
	esc32OutChecksum(c);
}

void esc32SendShort(unsigned short i) {
	unsigned char j;
	unsigned char *c = (unsigned char *)&i;

	for (j = 0; j < sizeof(short); j++)
		esc32SendChar(*c++);
}

void esc32SendFloat(float f) {
	unsigned char j;
	unsigned char *c = (unsigned char *)&f;

	for (j = 0; j < sizeof(float); j++)
		esc32SendChar(*c++);
}

unsigned short esc32GetShort(serialStruct_t *s) {
	unsigned short d;
	unsigned char *c = (unsigned char *)&d;
	unsigned int i;

	for (i = 0; i < sizeof(unsigned short); i++)
		*c++ = esc32GetChar(s);

	return d;
}

float esc32GetFloat(serialStruct_t *s) {
	float f;
	unsigned char *c = (unsigned char *)&f;
	unsigned int i;

	for (i = 0; i < sizeof(float); i++)
		*c++ = esc32GetChar(s);

	return f;
}

void *esc32Read(void *ipt) {
	unsigned short seqId;
        unsigned char c;
	int rows, cols;
	int n;
        int i, j;

        while (1) {
		thread_read_start:

		c = esc32GetChar(s);
		if (c != 'A')
			goto thread_read_start;

		c = esc32GetChar(s);
		if (c != 'q')
			goto thread_read_start;

		c = esc32GetChar(s);
		checkInA = checkInB = 0;

		if (c == 'C') {
			n = esc32GetChar(s);	// count
			c = esc32GetChar(s);	// command
			seqId = esc32GetShort(s);

			if (n > 3)
				paramId = esc32GetShort(s);

			if (serialRead(s) != checkInA)
				goto thread_read_start;

			if (serialRead(s) != checkInB)
				goto thread_read_start;

			if (c == BINARY_COMMAND_ACK) {
				lastSeqId = seqId;
				lastAck = 1;
#ifdef ESC32_DEBUG
				printf("Ack [%d]\n", seqId);
#endif
			}
			else if (c == BINARY_COMMAND_NACK) {
				lastSeqId = seqId;
				lastAck = 0;
#ifdef ESC32_DEBUG
				printf("Nack [%d]\n", seqId);
#endif
			}
			else if (c == BINARY_COMMAND_GET_PARAM_ID) {
				lastSeqId = seqId;
			}
			else {
				printf("Unkown command [%d]\n", c);
			}
		}
		if (c == 'T') {
			rows = esc32GetChar(s);
			cols = esc32GetChar(s);

			for (i = 0; i < rows; i++)
				for (j = 0; j < cols; j++)
					telemData[i][j] = esc32GetFloat(s);

			if (serialRead(s) != checkInA)
				goto thread_read_start;

			if (serialRead(s) != checkInB)
				goto thread_read_start;

			// update averages
			for (i = 0; i < rows; i++)
				for (j = 0; j < cols; j++)
					telemValueAvgs[j] -= (telemValueAvgs[j] - telemData[i][j]) * 0.01;

			// update max values
			for (i = 0; i < rows; i++)
				for (j = 0; j < cols; j++)
					if (telemValueMaxs[j] < telemData[i][j])
						telemValueMaxs[j] = telemData[i][j];

			// save to memory
			for (i = 0; i < rows; i++) {
				for (j = 0; j < cols; j++)
					telemStorage[MAX_TELEM_STORAGE*j + telemStorageNum] = telemData[i][j];
				telemStorageNum++;
			}

			// output to stream
			if (telemOutFile) {
				for (i = 0; i < rows; i++) {
					for (j = 0; j < cols; j++) {
						if (j != 0)
							fprintf(telemOutFile, ", ");
						fprintf(telemOutFile, "%f", telemData[i][j]);
					}
					fprintf(telemOutFile, "\n");
					fflush(telemOutFile);
				}
			}
		}
	}
}

unsigned short esc32SendCommand(unsigned char command, float param1, float param2, int n) {
	checkOutA = checkOutB = 0;
	sendBufPtr = 0;

#ifdef ESC32_DEBUG
        printf("Send %d [%d] - ", command, commandSeqId);
#endif
        sendBuf[sendBufPtr++] = 'A';
        sendBuf[sendBufPtr++] = 'q';
        esc32SendChar(1 + 2 + n*sizeof(float));
        esc32SendChar(command);
        esc32SendShort(commandSeqId++);
        if (n > 0)
                esc32SendFloat(param1);
        if (n > 1)
                esc32SendFloat(param2);
        sendBuf[sendBufPtr++] = checkOutA;
        sendBuf[sendBufPtr++] = checkOutB;
        esc32Send();
#ifdef ESC32_DEBUG
        printf("%d bytes\n", sendBufPtr);
#endif

	return (commandSeqId - 1);
}

void esc32Usage(void) {
	fprintf(stderr, "usage: esc32Cal <-h> <-a amps> <-p device_file> <-b baud_rate> <-t telemtry_out_file> [--cl --r2v]\n");
}

unsigned int esc32Options(int argc, char **argv) {
	int ch;

	strncpy(port, DEFAULT_PORT, sizeof(port));
	baud = DEFAULT_BAUD;
	maxAmps = DEFAULT_MAX_AMPS;

	/* options descriptor */
	static struct option longopts[] = {
		{ "help",	no_argument,		NULL,           'h' },
		{ "port",	required_argument,	NULL,           'p' },
		{ "baud",	required_argument,      NULL,           's' },
		{ "amps",	required_argument,      NULL,           'a' },
		{ "r2v",	no_argument,		NULL,           'r' },
		{ "cl",		no_argument,		NULL,           'c' },
		{ "telem_file",	required_argument,      NULL,           't' },
		{ NULL,         0,                      NULL,           0 }
	};

	while ((ch = getopt_long(argc, argv, "hp:b:a:rct:", longopts, NULL)) != -1)
		switch (ch) {
		case 'h':
			esc32Usage();
			exit(0);
			break;
		case 'p':
			strncpy(port, optarg, sizeof(port));
			break;
		case 'b':
			baud = atoi(optarg);
			break;
		case 'a':
			maxAmps = atof(optarg);
			break;
		case 'r':
			runR2V++;
			break;
		case 'c':
			runCL++;
			break;
		case 't':
			telemOutFile = fopen(optarg, "w");
			if (telemOutFile == NULL) {
				fprintf(stderr, "esc32Cal: cannot open output file '%s', aborting\n", optarg);
				exit(0);
			}
			break;
		default:
			esc32Usage();
			return 0;
	}
	argc -= optind;
	argv += optind;

	if (!runR2V && !runCL) {
		fprintf(stderr, "esc32Cal: requires either --r2v or --cl option, aborting\n");
		exit(0);
	}
	return 1;
}

int esc32SendReliably(unsigned char command, float param1, float param2, int n) {
	unsigned short seqId;
	int ret = 0;
	int i, j;

	j = 0;
	do {
		seqId = esc32SendCommand(command, param1, param2, n);

		i = 0;
		do {
			usleep(1000);
			i++;
		} while (lastSeqId != seqId && i < 500);

		j++;
	} while (lastSeqId != seqId && j < 5);

	if (lastSeqId == seqId)
		ret = lastAck;

	return ret;
}

int16_t esc32GetParamId(const char *name) {
	unsigned short seqId;
	int id;
	int i, j, k;

	j = 0;
	do {
		seqId = commandSeqId++;

		checkOutA = checkOutB = 0;
		sendBufPtr = 0;

		sendBuf[sendBufPtr++] = 'A';
		sendBuf[sendBufPtr++] = 'q';
		esc32SendChar(1 + 16 + 2);
		esc32SendChar(BINARY_COMMAND_GET_PARAM_ID);
		esc32SendShort(seqId);
		for (i = 0; i < 16; i++)
			esc32SendChar(name[i]);

		sendBuf[sendBufPtr++] = checkOutA;
		sendBuf[sendBufPtr++] = checkOutB;
		esc32Send();

		k = 0;
		do {
			usleep(1000);
			k++;
		} while (lastSeqId != seqId && k < 500);

		j++;
	} while (lastSeqId != seqId && j < 5);

	if (lastSeqId == seqId)
		id = paramId;
	else
		id = -1;

	return id;
}

short esc32SetParamByName(const char *name, float value) {
	short int paramId;

	paramId = esc32GetParamId(name);

	if (paramId < 0)
		return -1;

	
	return esc32SendReliably(BINARY_COMMAND_SET, paramId, value, 2);
}

void rpmToVoltageGraph(MatrixXd &data, MatrixXd &b, int n) {
	char s[256];
	float xMin, xMax;
	float yMin, yMax;
	double r;
	int k;

	PLFLT *xVals;
	PLFLT *yVals;

        xVals = (PLFLT *)calloc(1000, sizeof(PLFLT));
        yVals = (PLFLT *)calloc(1000, sizeof(PLFLT));

	xMin = +999999999.99;
	xMax = -999999999.99;
	yMin = +999999999.99;
	yMax = -999999999.99;

	for (k = 0; k < n; k++) {
		if (data(k, 0) < xMin)
			xMin = data(k, 0);
		if (data(k, 0) > xMax)
			xMax = data(k, 0);
		if (data(k, 1) < yMin)
			yMin = data(k, 1);
		if (data(k, 1) > yMax)
			yMax = data(k, 1);
	}

	plinit();
	plcol0(15);
	plenv(xMin, xMax, yMin, yMax, 0, 0);
	pllab("RPM", "Volts", "RPM vs Voltage");

	sprintf(s, "FF1TERM  %+e", b(0, 0));
	plptex(xMin+(xMax-xMin)/25.0, yMax-(yMax-yMin)/10.0, 10, 0, 0, s);

	sprintf(s, "FF2TERM  %+e", b(1, 0));
	plptex(xMin+(xMax-xMin)/25.0, yMax-(yMax-yMin)/5.0, 10, 0, 0, s);

	// plot data
	for (k = 0; k < n; k++) {
		xVals[k] = data(k, 0);
		yVals[k] = data(k, 1);
	}
	plcol0(1);
	plpoin(n, xVals, yVals, '+');

	// plot calculated
	k = 0;
	for (r = 0; r <= xMax; r += (xMax / 1000.0)) {
		xVals[k] = r;
		yVals[k] = b(0,0)*r*r + b(1,0)*r;
		k++;
	}
	plcol0(3);
	plline(1000, xVals, yVals);

	plend();

	free(xVals);
	free(yVals);
}

void rpmToVoltage(void) {
	MatrixXd A(2,2);
	MatrixXd c(2,1);
	MatrixXd ab(2,1);
	MatrixXd data(100, 3);
	float f;
	int j = 0;
	int i;

	printf("Starting...\n");
	esc32SendReliably(BINARY_COMMAND_START, 0.0, 0.0, 0);
	sleep(1);
	printf("Reducing RPM...\n");
	esc32SendReliably(BINARY_COMMAND_DUTY, 4.0, 0.0, 1);
	sleep(4);
	esc32SendReliably(BINARY_COMMAND_TELEM_RATE, 1000.0, 0.0, 1);

	// reset max current
	telemValueMaxs[2] = 0.0;

	printf("\n%5s %5s %5s\n", "RPM", "VOLTS", "AMPS");
	for (f = 4; f <= 100.0; f += 2.0) {
                esc32SendReliably(BINARY_COMMAND_DUTY, f, 0.0, 1);
		usleep((useconds_t)((100.0f - f) / 3.0f * 1e6 * 0.15));
		data(j,0) = telemValueAvgs[0];
		data(j,1) = telemValueAvgs[1];
		data(j,2) = telemValueAvgs[2];
		printf("%5.0f %5.2f %5.2f\n", data(j,0), data(j,1), data(j,2));

		j++;

		if (telemValueMaxs[2] > maxAmps)
			break;
	}
	esc32SendReliably(BINARY_COMMAND_TELEM_RATE, 0.0, 0.0, 1);
	printf("Stopping...\n");
	esc32SendReliably(BINARY_COMMAND_STOP, 0.0, 0.0, 0);

	printf("Calculating...\n");

	// calculate a & b where
	// y = ax^2 + bx

	A.setZero();
	c.setZero();
	for (i = 0; i < j; i++) {
		A(0, 0) += data(i,0)*data(i,0)*data(i,0)*data(i,0);
		A(0, 1) += data(i,0)*data(i,0)*data(i,0);
		A(1, 0) += data(i,0)*data(i,0)*data(i,0);
		A(1, 1) += data(i,0)*data(i,0);

		c(0) += data(i,0)*data(i,0)*data(i,1);
		c(1) += data(i,0)*data(i,1);
	}

	ab = A.inverse() * c;

	for (i = 0; i < 2; i++)
		printf("#define DEFAULT_FF%dTERM\t\t%+e\n", i+1, ab(i, 0));

	rpmToVoltageGraph(data, ab, j);
}

void stepUp(float start, float end) {
        esc32SendReliably(BINARY_COMMAND_DUTY, start, 0.0, 1);
        sleep(2);

        esc32SendReliably(BINARY_COMMAND_TELEM_RATE, 1000.0, 0.0, 1);
        esc32SendReliably(BINARY_COMMAND_DUTY, end, 0.0, 1);
        usleep(200000);
        esc32SendReliably(BINARY_COMMAND_TELEM_RATE, 0.0, 0.0, 1);
        usleep(250000);
}

void currentLimitGraph(MatrixXd &b) {
	char s[256];
	int k;
	float xMin, xMax;
	float yMin, yMax;

	PLFLT *xVals;
	PLFLT *yVals;

        xVals = (PLFLT *)calloc(telemStorageNum, sizeof(PLFLT));
        yVals = (PLFLT *)calloc(telemStorageNum, sizeof(PLFLT));

	xMin = +999999999.99;
	xMax = -999999999.99;
	yMin = +999999999.99;
	yMax = -999999999.99;
	for (k = 0; k < telemStorageNum; k++) {
		if (telemStorage[MAX_TELEM_STORAGE*0 + k] < xMin)
			xMin = telemStorage[MAX_TELEM_STORAGE*0 + k];
		if (telemStorage[MAX_TELEM_STORAGE*0 + k] > xMax)
			xMax = telemStorage[MAX_TELEM_STORAGE*0 + k];
		if (telemStorage[MAX_TELEM_STORAGE*1 + k] < yMin)
			yMin = telemStorage[MAX_TELEM_STORAGE*1 + k];
		if (telemStorage[MAX_TELEM_STORAGE*1 + k] > yMax)
			yMax = telemStorage[MAX_TELEM_STORAGE*1 + k];
	}

	plinit();
	plcol0(15);
	plenv(xMin, xMax, yMin, yMax, 0, 0);
	pllab("RPM", "Volts", "Current calculated RPM vs Voltage");

	for (k = 0; k < 5; k++) {
		sprintf(s, "CL%dTERM  %+e", k+1, b(k, 0));
		plptex(xMax-(xMax-xMin)/2.5, yMin + (yMax-yMin)/2.0 - ((yMax-yMin)/12.0)*(k+1), 10, 0, 0, s);
	}

	// first plot actual data
	for (k = 0; k < telemStorageNum; k++) {
		xVals[k] = telemStorage[MAX_TELEM_STORAGE*0 + k];
		yVals[k] = telemStorage[MAX_TELEM_STORAGE*1 + k];
	}
	plcol0(1);
	plpoin(telemStorageNum, xVals, yVals, -1);

	// next plot calculated data
	for (k = 0; k < telemStorageNum; k++) {
		xVals[k] = telemStorage[MAX_TELEM_STORAGE*0 + k];
		yVals[k] = b(0,0) +
			b(1,0)*telemStorage[MAX_TELEM_STORAGE*0 + k] +
			b(2,0)*telemStorage[MAX_TELEM_STORAGE*2 + k] +
			b(3,0)*telemStorage[MAX_TELEM_STORAGE*0 + k] * sqrt(fabs(telemStorage[MAX_TELEM_STORAGE*2 + k])) +
			b(4,0)*sqrt(fabs(telemStorage[MAX_TELEM_STORAGE*2 + k]));
	}
	plcol0(3);
	plpoin(telemStorageNum, xVals, yVals, -1);

	plend();

	free(xVals);
	free(yVals);
}

void currentLimiter(void) {
	MatrixXd A;
	MatrixXd c;
	MatrixXd ab;
	MatrixXd X;
	int m,n;
	int i, j, k;

	telemStorageNum = 0;

	printf("Starting...\n");
	esc32SendReliably(BINARY_COMMAND_START, 0.0, 0.0, 0);
	sleep(1);

	for (i = 10; i <= 90; i += 5) {
		// reset max current
		telemValueMaxs[2] = 0.0;

		for (j = i+5; j <= 100; j += 5) {
			stepUp((float)i, (float)j);

			printf("Duty %d to %d, MAX current = %f\n", i, j, telemValueMaxs[2]);

			// break if last try went overcurrent
			if (telemValueMaxs[2] > maxAmps)
				break;
		}

		// break if the first try went overcurrent
		if (telemValueMaxs[2] > maxAmps && j == i+5)
			break;
	}

	printf("Stopping...\n");
	esc32SendReliably(BINARY_COMMAND_STOP, 0.0, 0.0, 0);

	printf("Calculating...\n");

	n = telemStorageNum;
	m = 5;

	X.setZero(n, m);
	A.setZero(m, m);
	c.setZero(m, 1);
	
	for (k = 0; k < n; k++) {
		X(k, 0) = 1.0;
		X(k, 1) = telemStorage[MAX_TELEM_STORAGE*0 + k];
		X(k, 2) = telemStorage[MAX_TELEM_STORAGE*2 + k];
		X(k, 3) = telemStorage[MAX_TELEM_STORAGE*0 + k]*sqrt(fabs(telemStorage[MAX_TELEM_STORAGE*2 + k]));
		X(k, 4) = sqrt(fabs(telemStorage[MAX_TELEM_STORAGE*2 + k]));
	}

	for (i = 0; i < m; i++) {
		for (j = 0; j < m; j++)
			for (k = 0; k < n; k++)
				A(i, j) += X(k, i) * X(k, j);

		for (k = 0; k < n; k++)
			c(i, 0) += X(k, i) * telemStorage[MAX_TELEM_STORAGE*1 + k];
	}

	ab = A.inverse() * c;

	for (i = 0; i < m; i++)
		printf("#define DEFAULT_CL%dTERM\t\t%+e\n", i+1, ab(i, 0));

	currentLimitGraph(ab);
}

void signal_callback_handler(int signum) {
	printf("Caught signal %d\n",signum);
	esc32SendReliably(BINARY_COMMAND_DISARM, 0.0, 0.0, 0);
	esc32SendCommand(BINARY_COMMAND_CLI, 0.0, 0.0, 0);
	exit(signum);
}

int main(int argc, char **argv) {
	float f;
	int i;

	telemStorage = (float *)calloc(MAX_TELEM_STORAGE, sizeof(float)*3);

	// init
	if (!esc32Options(argc, argv)) {
		fprintf(stderr, "Init failed, aborting\n");
		return 0;
	}

	if ((s = initSerial(port, baud, 0)) == 0) {
		printf("Cannot open port '%s', aborting.\n", port);
		return 0;
	}

        if (pthread_create(&threadIn, NULL, esc32Read, (void *)NULL)) {
                serialFree(s);
                fprintf(stderr, "esc32: cannot create input thread\n");
                return 0;
        }

	serialPrint(s, "\n");
	usleep(100000);
	serialPrint(s, "binary\n");
	usleep(100000);

	if (!esc32SendReliably(BINARY_COMMAND_NOP, 0.0, 0.0, 0)) {
		fprintf(stderr, "Cannot communicate with ESC, aborting...\n");
		return 0;
	}

	esc32SendReliably(BINARY_COMMAND_ARM, 0.0, 0.0, 0);
	esc32SendReliably(BINARY_COMMAND_STOP, 0.0, 0.0, 0);
	esc32SendReliably(BINARY_COMMAND_TELEM_RATE, 0.0, 0.0, 1);
	esc32SendReliably(BINARY_COMMAND_TELEM_VALUE, 0.0, BINARY_VALUE_RPM, 2);
	esc32SendReliably(BINARY_COMMAND_TELEM_VALUE, 1.0, BINARY_VALUE_VOLTS_MOTOR, 2);
	esc32SendReliably(BINARY_COMMAND_TELEM_VALUE, 2.0, BINARY_VALUE_AMPS, 2);
//	esc32SendReliably(BINARY_COMMAND_SET, MAX_CURRENT, 0.0, 2);
	esc32SetParamByName("MAX_CURRENT", 0.0);


	// disarm motor if interrupted
	signal(SIGINT, signal_callback_handler);

	if (runR2V)
		rpmToVoltage();

	if (runCL)
		currentLimiter();

	esc32SendReliably(BINARY_COMMAND_TELEM_RATE, 0.0, 0.0, 1);
	esc32SendCommand(BINARY_COMMAND_CLI, 0.0, 0.0, 0);

	return 1;
}
