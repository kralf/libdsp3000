/***************************************************************************
                        libdsp3000.c  -  description
                            -------------------
  begin                : June 2006
  copyright            : (C) 2006 by Pierre Lamon
  email                : pierre.lamon@epfl.ch

  description          : interface with the dsp3000
 ***************************************************************************/
#include <stdlib.h>
#include <termios.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>

#include "libdsp3000.h"
#include <elrob/Edebug.h>
#include <elrob/Emacros.h>
#include <elrob/serial_easy.h>
#include <elrob/Etime.h>

static pthread_mutex_t  m_mutex_dsp = PTHREAD_MUTEX_INITIALIZER; /* mutex for the continuous capture thread */
static pthread_attr_t   m_thread_attr_dsp;                       /* the main thread attribute */
static pthread_t        m_tty_thread_dsp;                        /* the main thread structure */

static DSP3000_Data     m_shm_dsp;                               /* shared memory */
static int              m_ttyS = -1;
static TIMEVAL          m_time_origin = {-1.0,-1.0};
static EBOOL            m_cont_capt = EFALSE;
static MODULENAME       m_thread_params = MODULE_NAME_DSP3000_INT;

inline TIMEVAL DSP3000_GetTimeOrigin()
{
  return m_time_origin;
}

int DSP3000_OpenPort(char *device, int timeout)
{
  EDBG("Open %s serial port\n", device);
  m_ttyS = OpenSerial(device, B38400, timeout,0);
  return m_ttyS;
}

int DSP3000_ClosePort()
{
  if(m_ttyS == -1) return -1;
  return CloseSerial(m_ttyS);
}

inline EBOOL SendCommand(char cmd)
{
  if (SERIAL_WRITE(m_ttyS,&cmd,1) != 1) return 0;
  return 1;
}

EBOOL DSP3000_SwitchToRate()
{
  return SendCommand(DSP_CMD_SWITCH_RATE);
}

EBOOL DSP3000_SwitchToIncremental()
{
  return SendCommand(DSP_CMD_SWITCH_INC);
}

EBOOL DSP3000_SwitchToIntegrated()
{
  return SendCommand(DSP_CMD_SWITCH_INT);
}

EBOOL DSP3000_SwitchMode(MODULENAME mode)
{
  switch(mode){
    case MODULE_NAME_DSP3000_RATE: return DSP3000_SwitchToRate();
    case MODULE_NAME_DSP3000_INC:  return DSP3000_SwitchToIncremental();
    case MODULE_NAME_DSP3000_INT:  return DSP3000_SwitchToIntegrated();
    default: return EFALSE;
  }

  return EFALSE;
}

/* set the angle to zero (+ get the current time) */
EBOOL DSP3000_ZeroIntAngle()
{
  Time_gettimeofday(&m_time_origin);

  return SendCommand(DSP_CMD_ZERO_INT);
}

/* Add this to the integrated angle to compensate for earth rotation */
EBOOL DSP3000_EarthRateCompensation(TIMEVAL curr_time, double latitude, double *compensation)
{
  double delay;
  *compensation = 0.0;

  if(m_time_origin.tv_sec == -1.0) {
    EERR("DSP3000_EarthRateCompensation -> Time origin not initialized\n");
    return EFALSE;
  }

  if(Time_Compare(curr_time, m_time_origin) == TIME_COMP_SMALLER) {
    EERR("DSP3000_EarthRateCompensation -> curr_time < m_time_origin\n");
    EERR("curr_time is %.6f and m_time_origin  is %.6f\n", Time_FromTimeval(curr_time), Time_FromTimeval(m_time_origin));
    return EFALSE;
  }

  delay = Time_DiffDbl(curr_time, m_time_origin);
  *compensation = DSP_EARTH_RATE * delay * sin(latitude);

  return ETRUE;
}

/* return 0 if not valid */
inline int DSP3000_ParseStream(char *rec_buf, size_t buflen, TIMEVAL *stamp)
{
  char  *ptr;

  rec_buf[0] = '\0';
  ptr = rec_buf;

  while(1)
  {
    /* Read bytes one by one */
    if(SERIAL_READ(m_ttyS,ptr,1) != 1) {
      rec_buf[0] = '\0';
      return 0;
    }

    /* detects the end of a message */
    if(*ptr == CR)
    {
      Time_gettimeofday(stamp);  /* stamp here */
      ptr++;

      if(ptr-rec_buf != DSP_SIZE) {
        rec_buf[0] = '\0';

        return 0;
      }

      *ptr = '\0';
      return 1;
    }
    else {
      if(ptr-rec_buf > DSP_SIZE) {
        rec_buf[0] = '\0';
        return 0;
      }
      else
        ptr++;
    }

    if(ptr > (char *) (rec_buf + buflen))
    {
      EERR("DSP3000_ParseStream -> Buffer overflow!\n");
      rec_buf[0] = '\0';
      return 0;
    }
  }

  return 0;
}

void DSP3000_Resync()
{
  char c = ' ';

  while(SERIAL_READ(m_ttyS,&c,1) == 1 && c != LF) Time_SleepDbl(0.000010);
}

int DSP3000_ParseStreamBloc(char *rec_buf, TIMEVAL *stamp)
{
  rec_buf[0] = '\0';

  /* Read DSP_SIZE bytes */
  if(SERIAL_READ(m_ttyS,rec_buf,DSP_SIZE) != DSP_SIZE) {
      EERR("Could not read %i byte\n", DSP_SIZE);
      rec_buf[0] = '\0';
      return 0;
  }

  Time_gettimeofday(stamp);

  if(rec_buf[DSP_SIZE-1] != LF) return 0;

  rec_buf[DSP_SIZE] = '\0';
  return 1;
}

inline EBOOL DSP3000_FillContainer(const char *rec_buf, TIMEVAL *stamp, MODULENAME type, unsigned long meas_id, DSP3000_Data *cntr)
{
  char validity_char = DSP_CHAR_FAULT;
  /*  char *ptr = rec_buf;*/
  int  idx = 0;

  cntr->avail_time = *stamp;
  cntr->meas_time  = *stamp;
  cntr->valid      = 0;
  cntr->meas_id    = meas_id;

  if(type != MODULE_NAME_DSP3000_RATE && type != MODULE_NAME_DSP3000_INC && type != MODULE_NAME_DSP3000_INT){
    EERR("DSP3000_FillContainer -> error (type = %i)\n", type);
    return EFALSE;
  }

  cntr->type = type;

  /* remove spaces */
  while(rec_buf[idx] == ' ' || rec_buf[idx] == CR) idx++;

  /* if no . (dot) is found there is a problem */
  if(strchr(rec_buf+idx, '.') == NULL) return EFALSE;

  /* finally convert ASCII to numerical data */
  if(sscanf(rec_buf+idx,"%lf   %c", &cntr->data, &validity_char) != 2) return EFALSE;

  /* Convert in radians */
  cntr->data = DEG2RAD(cntr->data);

  if(validity_char == DSP_CHAR_VALID)
    cntr->valid = 1;
  else
    cntr->valid = 0;

  return (EBOOL) (cntr->valid);
}

void DSP3000_PrintData(DSP3000_Data cntr)
{
  EPRINT("*** DSP3000_Data struct ***\n");
  EPRINT("Type : %i\n", cntr.type);
  EPRINT("Data : %.10f [rad] or [rad/s] (%.6f [deg] of [deg/s])\n", cntr.data, RAD2DEG(cntr.data));
  EPRINT("Valid: %i\n", cntr.valid);
  EPRINT("Avail time: %.6f\n", Time_FromTimeval(cntr.avail_time));
  EPRINT("Meas time: %.6f\n", Time_FromTimeval(cntr.meas_time));
}

void DSP3000_DumpHeader(FILE *strm, const char *hostname)
{
  if(hostname != NULL)
    DUMP_HOSTNAME(strm,hostname);

  fprintf(strm,"# DSP3000 format is : \n");
  fprintf(strm,"# 1:  data row number \n");
  fprintf(strm,"# 2:  measured time [s]\n");
  fprintf(strm,"# 3:  available time [s]\n");
  fprintf(strm,"# 4:  validity\n");
  fprintf(strm,"# 5:  measurement id\n");
  fprintf(strm,"# 6:  measurement (depends on 1)\n");
}

void DSP3000_DumpData(FILE *strm, DSP3000_Data *cntr)
{
  fprintf(strm,"%i\t%f\t%f\t%i\t%li\t%.10f",
        (int) cntr->type,
        Time_FromTimeval(cntr->meas_time),
        Time_FromTimeval(cntr->avail_time),
        cntr->valid,
        cntr->meas_id,
        cntr->data);
}

EBOOL DSP3000_ReadDataFromFile(FILE *strm, DSP3000_Data *cntr)
{
  double meas_time;
  double avail_time;
  int    res;
  int    type;

  res = fscanf(strm,"%i\t%lf\t%lf\t%i\t%li\t%lf\n",
               &type,
               &meas_time,
               &avail_time,
               &cntr->valid,
               &cntr->meas_id,
               &cntr->data);

  if(res != 6){
    EERR("DSP3000_ReadDataFromFile -> Could not read all the field\n");
    cntr->valid = 0;
    return EFALSE;
  }

  cntr->type = (MODULENAME) type;
  cntr->meas_time  = Time_FromDbl(meas_time);
  cntr->avail_time = Time_FromDbl(avail_time);

  return ETRUE;
}

/* ---  Thread functions */
inline void DSP_Sem_p()
{
  if(pthread_mutex_lock(&m_mutex_dsp) != 0) EERR("Sem_p failed\n");
}

inline void DSP_Sem_v()
{
  if(pthread_mutex_unlock(&m_mutex_dsp) != 0) EERR("Sem_v failed\n");
}

EBOOL DSP_InitThreadAttr(pthread_attr_t *attr, EBOOL root)
{
  int    res;
  struct sched_param    sched;

  /* Increase the priority of the thread (for better time stamping) */
  res = pthread_attr_init(attr);
  if (res != 0)
  {
    perror("InitThreadAttr -> pthread_attr_init error!\n");
    return EFALSE;
  }

  /* set the priority; others are unchanged */
  if(root)
  {
    res = pthread_attr_setinheritsched(attr, PTHREAD_EXPLICIT_SCHED);
    if (res != 0)
    {
      perror("InitThreadAttr -> pthread_attr_setinheritsched error!\n");
      return EFALSE;
    }

    res = pthread_attr_setschedpolicy(attr, SCHED_FIFO);
    if (res != 0)
    {
      perror("InitThreadAttr -> pthread_attr_setschedpolicy error!\n");
      return EFALSE;
    }

    sched.sched_priority = sched_get_priority_max(SCHED_FIFO);

    if(sched.sched_priority == -1)
    {
      perror("InitThreadAttr -> sched_get_priority_max failed!\n");
      return EFALSE;
    }

    EDBG("InitThreadAttr -> Set sched priority to %i\n", sched.sched_priority);

    /* set the new scheduling param */
    if (pthread_attr_setschedparam (attr, &sched) != 0)
    {
      perror("InitThreadAttr -> pthread_attr_setschedparam error!\n");
      return EFALSE;
    }
  }
  return ETRUE;
}

static void *ttyThreadCapture (void *params)
{
  char          rec_buf[DSP_SIZE_BUFF];
  MODULENAME    dsp_mode = *((MODULENAME *) params);
  DSP3000_Data  gyro_data;
  TIMEVAL       stamp;
  unsigned long meas_id = 0;

  while(m_cont_capt)
  {
    if(DSP3000_ParseStream(rec_buf, DSP_SIZE_BUFF, &stamp))
    {
      if(DSP3000_FillContainer(rec_buf, &stamp, dsp_mode, meas_id++, &gyro_data))
      {
        DSP_Sem_p();
        m_shm_dsp = gyro_data;
        DSP_Sem_v();
      }
      else {
        EERR("DSP3000_FillContainer: not valid!\n");
        DSP3000_PrintData(gyro_data);
      }
    }
    else {
      if(meas_id > 2) EERR("ParseStream error! (%s)\n", rec_buf);
    }
  }

  return NULL;
}

EBOOL DSP3000_IsThreadRunning()
{
  return m_cont_capt;
}

void DSP3000_GetData(DSP3000_Data  *pdata)
{
  DSP_Sem_p();
  *pdata = m_shm_dsp;
  DSP_Sem_v();
}

EBOOL DSP3000_LaunchCaptureThread(MODULENAME dsp_mode, EBOOL root)
{
  int res;

  if(m_ttyS == -1) {
    fprintf(stderr, "DSP3000_LaunchCaptureThread -> Port not open\n");
    return EFALSE;
  }

  DSP_InitThreadAttr(&m_thread_attr_dsp, root);
  m_cont_capt = ETRUE;
  m_thread_params = dsp_mode;

  res = pthread_create(&m_tty_thread_dsp, &m_thread_attr_dsp, ttyThreadCapture, (void *) &m_thread_params);
  if (res != 0)
  {
    perror("DSP3000_LaunchCaptureThread -> pthread_create error!\n");
    EPRINT("Are you root ?\n");
    return EFALSE;
  }

  return ETRUE;
}

EBOOL DSP_KillThread(pthread_t *thread, pthread_attr_t *attr)
{
  int  res;
  void *value_ptr;

  EDBG("Killing thread ...\n");
  res = pthread_join(*thread, &value_ptr);

  if(res != 0) {
    perror("KillThread -> could not join thread!");
    return EFALSE;
  }

  res = pthread_attr_destroy(attr);
  if(res != 0) {
    perror("KillThread -> destroy thread attr!");
    return EFALSE;
  }

  return ETRUE;
}

EBOOL DSP3000_KillCaptureThread()
{
  m_cont_capt = EFALSE;
  return DSP_KillThread(&m_tty_thread_dsp, &m_thread_attr_dsp);
}







