//
// These defines were moved from the rs274ngc.hh header to here because they
// are private to the interpreter. External programs should not use this.
//
// Original header states:
// Derived from a work by Thomas Kramer
// License: GPL Version 2
// System: Linux
// Copyright (c) 2004 All rights reserved.
//
#ifndef RS274NGC_INTERP_LOG_HH
#define RS274NGC_INTERP_LOG_HH

#include <nml_intf/debugflags.h>

#define DBG(level,fmt,args...)                  \
    do {                                        \
      if (level < _setup.loggingLevel) {	\
	fprintf(stderr,fmt, ## args);		\
      }                                         \
    } while (0)

// print to if RS274NGC/LOG_LEVEL > 1:

#define MSG(fmt,args...)                        \
    do {                                        \
      DBG(0, fmt, ##args);                      \
    } while (0)


#undef DEBUG_EMC


#define _logDebug(mask,dlflags,level, fmt, args...)	\
    do {						\
	if (((mask & _setup.debugmask) &&		\
	     (level < _setup.loggingLevel)) ||		\
            (mask & EMC_DEBUG_UNCONDITIONAL)) {		\
	    doLog(dlflags,				\
		  __FILE__,				\
		  __LINE__ ,				\
		  fmt "\n",				\
		  ## args);				\
        }						\
    } while(0)

//#define logDebug(fmt, args...)  _logDebug(EMC_DEBUG_INTERP,LOG_FILENAME,1,fmt, ## args)
#define logDebug(fmt, args...)  _logDebug(EMC_DEBUG_INTERP,0,1,fmt, ## args)

#define logConfig(fmt, args...) _logDebug(EMC_DEBUG_CONFIG,0,1,fmt, ## args)
#define logOword(fmt, args...)  _logDebug(EMC_DEBUG_OWORD,0,1,fmt, ## args)
#define logRemap(fmt, args...)  _logDebug(EMC_DEBUG_REMAP,0,1,fmt, ## args)
#define logPy(fmt, args...)     _logDebug(EMC_DEBUG_PYTHON,0,1,fmt, ## args)
#define logNP(fmt, args...)     _logDebug(EMC_DEBUG_NAMEDPARAM,0,1,fmt, ## args)
#define logStateTags(fmt, args...) \
    _logDebug(EMC_DEBUG_STATE_TAGS,0,1,fmt, ## args)

// log always
#define Log(fmt, args...)       _logDebug(EMC_DEBUG_UNCONDITIONAL,LOG_PID|LOG_FILENAME,-1,fmt, ## args)
#define Error(fmt, args...)       _logDebug(EMC_DEBUG_UNCONDITIONAL,0,-1,fmt, ## args)


#endif
