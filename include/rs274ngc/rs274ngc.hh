/********************************************************************
* Description: rs274ngc.hh
*
*   Derived from a work by Thomas Kramer
*
* Author:
* License: GPL Version 2
* System: Linux
*
* Copyright (c) 2004 All rights reserved.
*
********************************************************************/
#ifndef __LINUXCNC_RS274NGC_RS274NGC_HH
#define __LINUXCNC_RS274NGC_RS274NGC_HH

#include "interp_base.hh"
#include "interp_fwd.hh"

// Declare class so that we can use it in the typedef.
class Interp;
typedef int (Interp::*read_function_pointer) (char *, int *, block_pointer, double *);

#endif
