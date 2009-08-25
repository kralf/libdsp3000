/***************************************************************************
                        libdsp3000.h  -  description
                            -------------------
  begin                : June 2006
  copyright            : (C) 2006 by Pierre Lamon
  email                : pierre.lamon@epfl.ch

  description          : library api for the optical gyro
 ***************************************************************************/
#ifndef LIBDSP3000_H
#define LIBDSP3000_H

#include <stdio.h>
#include "libdsp3000_struct.h"

#define CR 0x0D
#define LF 0x0A
#define DSP_SIZE  21
#define DSP_SIZE_BUFF        (2*DSP_SIZE)
#define DSP_CMD_SWITCH_RATE  'R'
#define DSP_CMD_SWITCH_INC   'A'
#define DSP_CMD_SWITCH_INT   'P'
#define DSP_CMD_ZERO_INT     'Z'
#define DSP_CHAR_VALID       '1'
#define DSP_CHAR_FAULT       '0'
#define DSP_EARTH_RATE       (DEG2RAD(15.04107)/(3600.0))

int   DSP3000_OpenPort(char *device, int timeout);
int   DSP3000_ClosePort();
int   DSP3000_ParseStream(char *rec_buf, size_t buflen, TIMEVAL *stamp);
int   DSP3000_ParseStreamBloc(char *rec_buf, TIMEVAL *stamp);
EBOOL DSP3000_FillContainer(const char *rec_buf, TIMEVAL *stamp, MODULENAME type, unsigned long meas_id, DSP3000_Data *cntr);
EBOOL DSP3000_SwitchMode(MODULENAME mode);
EBOOL DSP3000_SwitchToRate();
EBOOL DSP3000_SwitchToIncremental();
EBOOL DSP3000_SwitchToIntegrated();
EBOOL DSP3000_ZeroIntAngle();
void  DSP3000_PrintData(DSP3000_Data cntr);
void  DSP3000_DumpHeader(FILE *strm, const char *hostname);
void  DSP3000_DumpData(FILE *strm, DSP3000_Data *cntr);
EBOOL DSP3000_ReadDataFromFile(FILE *strm, DSP3000_Data *cntr);
EBOOL DSP3000_EarthRateCompensation(TIMEVAL curr_time, double latitude, double *compensation);
TIMEVAL DSP3000_GetTimeOrigin();
void  DSP3000_Resync();
EBOOL DSP3000_LaunchCaptureThread(MODULENAME dsp_mode, EBOOL root);
void  DSP3000_GetData(DSP3000_Data *pdata);
EBOOL DSP3000_KillCaptureThread();
EBOOL DSP3000_IsThreadRunning();

#endif
