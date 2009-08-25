/***************************************************************************
                        libdsp3000_struct.h  -  description
                            -------------------
  begin                : June 2006
  copyright            : (C) 2006 by Pierre Lamon
  email                : pierre.lamon@epfl.ch

  description          : structures definition for the optical gyro
 ***************************************************************************/
#ifndef LIBDSP3000_STRUCT_H
#define LIBDSP3000_STRUCT_H

#include <elrob/Etypes.h>

typedef struct _DSP3000_Data{
  MODULENAME    type;
  double        data;
  long          meas_id;
  int           valid;
  TIMEVAL       avail_time;
  TIMEVAL       meas_time;
}DSP3000_Data;

#endif
