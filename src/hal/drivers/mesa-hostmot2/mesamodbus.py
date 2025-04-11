#!/usr/bin/env python
#
# Build hm2_modbus control command binary data file
# Copyright (C) 2025 B. Stultiens
#
# This program is free software; you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation; either version 2 of the License, or (at your option) any later
# version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
# details.
#
# You should have received a copy of the GNU General Public License along with
# this program; if not, write to the Free Software Foundation, Inc., 51
# Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
#

#
# XML format
# <mesamodbus [comm param override attributes]>
#   <devices>
#     <device address="0x01" name="mydev"><description /></device>
#     ...
#   </devices>
#   <initlist>
#     <command device="mydev" function="6" address="0x0034" /><data val="0x0001" /><data... /><description /></command>
#     <command delay="2000" />
#     ...
#   </initlist>
#   <commands>
#     <!-- <command> can have a local 'timeout' override attribute -->
#     <command device="mydev" function="MBCMD_R_COILS" address="0x0000" count="8" name="state"><description /></command>
#     <command device="mydev" function="MBCMD_W_COILS" address="0x0000" count="8" name="relay">
#       <pin name="bulb-light"><description /></pin>
#       <pin name="shinyshiny" />
#       <pin name="vroooom" />
#       ...
#     </command>
#     <command delay="2000" />
#      ...
#   </commands>
# </mesamodbus>
#
# The <description> tag is just for comments. And, a future graphical interface
# may use it to make stuff more explanatory.
#
# Mandatory <command> attributes:
# - device="mydev"    A reference to the devices list
# - function="6"      The Modbus function to perform
# - address="0x1234"  The Modbus address
#
# Optional <command> attributes:
# - bcanswer="1"  Broadcasts may set the attribute to signal a reply being sent
#                 from the slave, even though it should keep quiet.
# - noanswer="1"  Non-broadcasts may set the  attribute to indicate that no
#                 answer is expected to be received.
# - timesout="1"  Command reply timeout is not supposed to be treated as an
#                 error. This can happen for devices that may or may not answer.
# - scale="1"     Add scale/offset HAL pins. This is the default for pin type
#                 'haltype="HAL_FLOAT"' (disable using 'scaled="0"'). Only
#                 valid on hal types HAL_S32, HAL_S64 and HAL_FLOAT.
# - clamp="1"     Clamp values to their native size. Default on, disable using
#                 'clamp="0"'.
# - resend="1"    Force writes to be resend every loop, even if no data has
#                 changed. This helps devices that expect regular writes for,
#                 for example, a watchdog.
#
# 'modbustype=...' attribute in <command> and <data> tags for non-bit functions:
# - {S,U,F}_<order> where order is one of:
#   * AB, BA
#   * ABCD, BADC, CDAB, DCBA
#   * ABCDEFGH, BADCFEHG, CDABGHEF, DCBAHGFE, EFGHABCD, FEHGBADC, GHEFCDAB, HGFEDCBA
# The default modbustype is U_AB.
#
# 'haltype=... attribute in <command> tag for non-bit functions:
# - HAL_FLOAT (HAL_FLT)
# - HAL_S32
# - HAL_U32
# - HAL_S64
# - HAL_U64
# The name HAL_BIT is recognized but only allowed in bit functions (where it is
# implicit anyway).
#

import sys
import os
import math
import getopt
import xml.etree.ElementTree as ET
import xml.parsers.expat as XP
import re
import struct

# HAL names are a bit picky
pinpattern  = re.compile("^[a-z][a-z0-9-.]*[a-z0-9]$")
namepattern = re.compile("^[a-z][a-z0-9-]*[a-z0-9]$")

devices  = None    # The list of devices

inputfilename = "unknown-filename" # file being processed
verbose = False
errorflag = False

# Default parameters for serial communication.
# These can be overridden in the root tag as attributes.
configparams = {'baudrate'  : '9600',
                'parity'    : 'E',      # Translates into N=0, O=1, E=2
                'stopbits'  : '1',
                'duplex'    : 'HALF',
                'rxdelay'   : 'AUTO',
                'txdelay'   : 'AUTO',
                'drivedelay': '1',
                'interval'  : '0',
                'timeout'   : 'AUTO' }

# Allowed values for the parity attribute
PARITIES = {'N':    '0', 'O':   '1', 'E':    '2',
            'NONE': '0', 'ODD': '1', 'EVEN': '2',
            '0':    '0', '1':   '1', '2':    '2' }

DUPLEXES = {'HALF': '0', '0': '0',
            'FULL': '1', '1': '1'}

CONFIGLIMITS = {'baudrate'  : [1200, 1000000],
                'parity'    : [0, 2],
                'stopbits'  : [1, 2],
                'duplex'    : [0, 1],   # half/full
                'rxdelay'   : [15, 1020],
                'txdelay'   : [17, 1020],
                'drivedelay': [0, 31],
                'interval'  : [0, 1000000000],  # As-fast-as possible to 1000 seconds
                'timeout'   : [5000, 5000000] } # 5 milliseconds to 5 seconds (can override in <command>)

# XXX: Keep in sync with hal.h
HAL_BIT = 1
HAL_FLT = 2 # HAL_FLOAT
HAL_S32 = 3
HAL_U32 = 4
HAL_PRT = 5 # HAL_PORT, unused
HAL_S64 = 6
HAL_U64 = 7

# XXX: Keep in sync with hm2_modbus.c
MBT_AB       = 0x00
MBT_BA       = 0x01
MBT_ABCD     = 0x02
MBT_BADC     = 0x03
MBT_CDAB     = 0x04
MBT_DCBA     = 0x05
MBT_ABCDEFGH = 0x06
MBT_BADCFEHG = 0x07
MBT_CDABGHEF = 0x08
MBT_DCBAHGFE = 0x09
MBT_EFGHABCD = 0x0a
MBT_FEHGBADC = 0x0b
MBT_GHEFCDAB = 0x0c
MBT_HGFEDCBA = 0x0d
MBTBYTESIZES = [2, 2, 4, 4, 4, 4, 8, 8, 8, 8, 8, 8, 8, 8]

MBT_U        = 0x00
MBT_S        = 0x10
MBT_F        = 0x20

MBT_X_MASK   = 0x0f
MBT_T_MASK   = 0xf0

U_AB   = MBT_U | MBT_AB
U_BA   = MBT_U | MBT_BA
S_AB   = MBT_S | MBT_AB
S_BA   = MBT_S | MBT_BA
F_AB   = MBT_F | MBT_AB
F_BA   = MBT_F | MBT_BA
U_ABCD = MBT_U | MBT_ABCD
U_BADC = MBT_U | MBT_BADC
U_CDAB = MBT_U | MBT_CDAB
U_DCBA = MBT_U | MBT_DCBA
S_ABCD = MBT_S | MBT_ABCD
S_BADC = MBT_S | MBT_BADC
S_CDAB = MBT_S | MBT_CDAB
S_DCBA = MBT_S | MBT_DCBA
F_ABCD = MBT_F | MBT_ABCD
F_BADC = MBT_F | MBT_BADC
F_CDAB = MBT_F | MBT_CDAB
F_DCBA = MBT_F | MBT_DCBA

# Possible values of the 'haltype' attribute
HALTYPES = { 'HAL_BIT': HAL_BIT, 'HAL_FLOAT': HAL_FLT, 'HAL_FLT': HAL_FLT,
             'HAL_S32': HAL_S32, 'HAL_U32':   HAL_U32,
             'HAL_S64': HAL_S64, 'HAL_U64':   HAL_U64 }

# Reverse map of HALTYPES
HALNAMES = { HAL_BIT: 'HAL_BIT', HAL_FLT: 'HAL_FLOAT',
             HAL_S32: 'HAL_S32', HAL_U32: 'HAL_U32',
             HAL_S64: 'HAL_S64', HAL_U64: 'HAL_U64' }

# Possible values of the 'modbustype' attribute
# [typeId, maxCount, nWords]
MBTYPES = { 'S_AB':       [MBT_S | MBT_AB,      125, 1], 'S_BA':       [MBT_S | MBT_BA,      125, 1],
            'U_AB':       [MBT_U | MBT_AB,      125, 1], 'U_BA':       [MBT_U | MBT_BA,      125, 1],
            'F_AB':       [MBT_F | MBT_AB,      125, 1], 'F_BA':       [MBT_F | MBT_BA,      125, 1],
            'S_ABCD':     [MBT_S | MBT_ABCD,     62, 2], 'S_BADC':     [MBT_S | MBT_BADC,     62, 2],
            'S_CDAB':     [MBT_S | MBT_CDAB,     62, 2], 'S_DCBA':     [MBT_S | MBT_DCBA,     62, 2],
            'U_ABCD':     [MBT_U | MBT_ABCD,     62, 2], 'U_BADC':     [MBT_U | MBT_BADC,     62, 2],
            'U_CDAB':     [MBT_U | MBT_CDAB,     62, 2], 'U_DCBA':     [MBT_U | MBT_DCBA,     62, 2],
            'F_ABCD':     [MBT_F | MBT_ABCD,     62, 2], 'F_BADC':     [MBT_F | MBT_BADC,     62, 2],
            'F_CDAB':     [MBT_F | MBT_CDAB,     62, 2], 'F_DCBA':     [MBT_F | MBT_DCBA,     62, 2],
            'S_ABCDEFGH': [MBT_S | MBT_ABCDEFGH, 31, 4], 'S_BADCFEHG': [MBT_S | MBT_BADCFEHG, 31, 4],
            'S_CDABGHEF': [MBT_S | MBT_CDABGHEF, 31, 4], 'S_DCBAHGFE': [MBT_S | MBT_DCBAHGFE, 31, 4],
            'S_EFGHABCD': [MBT_S | MBT_EFGHABCD, 31, 4], 'S_FEHGBADC': [MBT_S | MBT_FEHGBADC, 31, 4],
            'S_GHEFCDAB': [MBT_S | MBT_GHEFCDAB, 31, 4], 'S_HGFEDCBA': [MBT_S | MBT_HGFEDCBA, 31, 4],
            'U_ABCDEFGH': [MBT_U | MBT_ABCDEFGH, 31, 4], 'U_BADCFEHG': [MBT_U | MBT_BADCFEHG, 31, 4],
            'U_CDABGHEF': [MBT_U | MBT_CDABGHEF, 31, 4], 'U_DCBAHGFE': [MBT_U | MBT_DCBAHGFE, 31, 4],
            'U_EFGHABCD': [MBT_U | MBT_EFGHABCD, 31, 4], 'U_FEHGBADC': [MBT_U | MBT_FEHGBADC, 31, 4],
            'U_GHEFCDAB': [MBT_U | MBT_GHEFCDAB, 31, 4], 'U_HGFEDCBA': [MBT_U | MBT_HGFEDCBA, 31, 4],
            'F_ABCDEFGH': [MBT_F | MBT_ABCDEFGH, 31, 4], 'F_BADCFEHG': [MBT_F | MBT_BADCFEHG, 31, 4],
            'F_CDABGHEF': [MBT_F | MBT_CDABGHEF, 31, 4], 'F_DCBAHGFE': [MBT_F | MBT_DCBAHGFE, 31, 4],
            'F_EFGHABCD': [MBT_F | MBT_EFGHABCD, 31, 4], 'F_FEHGBADC': [MBT_F | MBT_FEHGBADC, 31, 4],
            'F_GHEFCDAB': [MBT_F | MBT_GHEFCDAB, 31, 4], 'F_HGFEDCBA': [MBT_F | MBT_HGFEDCBA, 31, 4] }

# Reverse map of MBTYPES
MBNAMES = { MBT_S | MBT_AB:       'S_AB',       MBT_S | MBT_BA:       'S_BA',
            MBT_U | MBT_AB:       'U_AB',       MBT_U | MBT_BA:       'U_BA',
            MBT_F | MBT_AB:       'F_AB',       MBT_F | MBT_BA:       'F_BA',
            MBT_S | MBT_ABCD:     'S_ABCD',     MBT_S | MBT_BADC:     'S_BADC',
            MBT_S | MBT_CDAB:     'S_CDAB',     MBT_S | MBT_DCBA:     'S_DCBA',
            MBT_U | MBT_ABCD:     'U_ABCD',     MBT_U | MBT_BADC:     'U_BADC',
            MBT_U | MBT_CDAB:     'U_CDAB',     MBT_U | MBT_DCBA:     'U_DCBA',
            MBT_F | MBT_ABCD:     'F_ABCD',     MBT_F | MBT_BADC:     'F_BADC',
            MBT_F | MBT_CDAB:     'F_CDAB',     MBT_F | MBT_DCBA:     'F_DCBA',
            MBT_S | MBT_ABCDEFGH: 'S_ABCDEFGH', MBT_S | MBT_BADCFEHG: 'S_BADCFEHG',
            MBT_S | MBT_CDABGHEF: 'S_CDABGHEF', MBT_S | MBT_DCBAHGFE: 'S_DCBAHGFE',
            MBT_S | MBT_EFGHABCD: 'S_EFGHABCD', MBT_S | MBT_FEHGBADC: 'S_FEHGBADC',
            MBT_S | MBT_GHEFCDAB: 'S_GHEFCDAB', MBT_S | MBT_HGFEDCBA: 'S_HGFEDCBA',
            MBT_U | MBT_ABCDEFGH: 'U_ABCDEFGH', MBT_U | MBT_BADCFEHG: 'U_BADCFEHG',
            MBT_U | MBT_CDABGHEF: 'U_CDABGHEF', MBT_U | MBT_DCBAHGFE: 'U_DCBAHGFE',
            MBT_U | MBT_EFGHABCD: 'U_EFGHABCD', MBT_U | MBT_FEHGBADC: 'U_FEHGBADC',
            MBT_U | MBT_GHEFCDAB: 'U_GHEFCDAB', MBT_U | MBT_HGFEDCBA: 'U_HGFEDCBA',
            MBT_F | MBT_ABCDEFGH: 'F_ABCDEFGH', MBT_F | MBT_BADCFEHG: 'F_BADCFEHG',
            MBT_F | MBT_CDABGHEF: 'F_CDABGHEF', MBT_F | MBT_DCBAHGFE: 'F_DCBAHGFE',
            MBT_F | MBT_EFGHABCD: 'F_EFGHABCD', MBT_F | MBT_FEHGBADC: 'F_FEHGBADC',
            MBT_F | MBT_GHEFCDAB: 'F_GHEFCDAB', MBT_F | MBT_HGFEDCBA: 'F_HGFEDCBA' }

R_COILS     =  1
R_INPUTS    =  2
R_REGISTERS =  3
R_INPUTREGS =  4
W_COIL      =  5
W_REGISTER  =  6
W_COILS     = 15
W_REGISTERS = 16

# Map function names/numbers to value
# [funcId, maxCount]
FUNCTIONS = { 'R_COILS'    : [R_COILS,    2000],  '1': [R_COILS,    2000],
              'R_INPUTS'   : [R_INPUTS,   2000],  '2': [R_INPUTS,   2000],
              'R_REGISTERS': [R_REGISTERS, 125],  '3': [R_REGISTERS, 125],
              'R_INPUTREGS': [R_INPUTREGS, 125],  '4': [R_INPUTREGS, 125],
              'W_COIL'     : [W_COIL,        1],  '5': [W_COIL,        1],
              'W_REGISTER' : [W_REGISTER,    1],  '6': [W_REGISTER,    1],
              'W_COILS'    : [W_COILS,    2000], '15': [W_COILS,    2000],
              'W_REGISTERS': [W_REGISTERS, 125], '16': [W_REGISTERS, 125] }

FUNCNAMES = { R_COILS: 'R_COILS', R_INPUTREGS: 'R_INPUTREGS', R_INPUTS: 'R_INPUTS', R_REGISTERS: 'R_REGISTERS',
              W_COIL:  'W_COIL',  W_REGISTER:  'W_REGISTER',  W_COILS:  'W_COILS',  W_REGISTERS: 'W_REGISTERS' }

WRITEFUNCTIONS = [ 'W_COIL', 'W_REGISTER', 'W_COILS', 'W_REGISTERS' ]

# These functions always use HAL_BIT
BITFUNCTIONS = [R_COILS, R_INPUTS, W_COIL, W_COILS]


# XXX: keep in sync with hm2_modbus.h
MAXDELAY = 60000000    # Max one minute delay between init commands (in microseconds)
MAXPINNAME = 24        # Max chars for a name

# Command flags for handling quirks and options
# XXX: keep in sync with hm2_modbus.h
MBCCB_CMDF_TIMESOUT = 0x0001
MBCCB_CMDF_BCANSWER = 0x0002
MBCCB_CMDF_NOANSWER = 0x0004
MBCCB_CMDF_SCALE    = 0x0008
MBCCB_CMDF_CLAMP    = 0x0010
MBCCB_CMDF_RESEND   = 0x0020
MBCCB_CMDF_INITMASK = 0x0007 # sum of allowed flags in init
MBCCB_CMDF_MASK     = 0x003f # sum of all above flags

#
# Program invocation message
#
def usage():
    print("""Build hm2_modbus binary control command file from XML source description.
Usage:
  mesamodbus [-h] [-o outfile] infile.mbccs

Options:
  -h|--help              This message
  -o file|--output=file  Write output to 'file'. Defaults to 'mesamodbus.output.mbccb'
  -v|--verbose           Verbose messages
""")
    sys.exit(2)

#
# Print to stderr with prefix
#
def perr(*args, **kwargs):
    global inputfilename
    global errorflag
    print("{}: error: ".format(inputfilename), file=sys.stderr, end='')
    print(*args, file=sys.stderr, **kwargs)
    errorflag = True

def pwarn(*args, **kwargs):
    global inputfilename
    print("{}: warning: ".format(inputfilename), file=sys.stderr, end='')
    print(*args, file=sys.stderr, **kwargs)

#
# Parse an int
#
def checkInt(s, k):
    if k not in s:
        return None
    try:
        val = int(s[k], 0)
    except ValueError as err:
        perr("Invalid integer '{}'".format(s))
        return None
    return val

#
# Parse a float
#
def checkFlt(s, k):
    if k not in s:
        return None
    try:
        val = float(s[k])
    except ValueError as err:
        perr("Invalid float '{}'".format(s))
        return None
    except OverflowError as err:
        perr("Invalid float range '{}'".format(s))
        return None
    return val

#
# Mangle 16 bit bytesex
#
def mangle16(d, mtype):
    mtype &= MBT_X_MASK
    if MBT_AB == mtype:
        return d;
    if MBT_BA == mtype:
        return d[::-1];
    sys.exit("Invalid mtype '{}' in mangle16".format(mtype))

#
# Mangle 32 bit bytesex
#
def mangle32(d, mtype):
    mtype &= MBT_X_MASK
    if MBT_ABCD == mtype:
        return d         # Source was in big endian
    if MBT_DCBA == mtype:
        return d[::-1]   # Reverse is little endian
    if MBT_BADC == mtype:
        d[0], d[1], d[2], d[3] = d[1], d[0], d[3], d[2]
        return d
    if MBT_CDAB == mtype:
        d[0], d[1], d[2], d[3] = d[2], d[3], d[0], d[1]
        return d
    sys.exit("Invalid mtype '{}' in mangle32".format(mtype))

#
# Mangle 64 bit bytesex
#
def mangle64(d, mtype):
    mtype &= MBT_X_MASK
    if MBT_ABCDEFGH == mtype:
        return d         # Source was in big endian
    if MBT_HGFEDCBA == mtype:
        return d[::-1]   # Reverse is little endian
    if MBT_BADCFEHG == mtype:
        d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7] = d[1], d[0], d[3], d[2], d[5], d[4], d[7], d[6]
        return d
    if MBT_CDABGHEF == mtype:
        d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7] = d[2], d[3], d[0], d[1], d[6], d[7], d[4], d[5]
        return d
    if MBT_DCBAHGFE == mtype:
        d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7] = d[3], d[2], d[1], d[0], d[7], d[6], d[5], d[5]
        return d
    if MBT_EFGHABCD == mtype:
        d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7] = d[4], d[5], d[6], d[7], d[0], d[1], d[2], d[3]
        return d
    if MBT_FEHGBADC == mtype:
        d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7] = d[5], d[4], d[7], d[6], d[1], d[0], d[3], d[2]
        return d
    if MBT_GHEFCDAB == mtype:
        d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7] = d[6], d[7], d[4], d[5], d[2], d[3], d[0], d[1]
        return d
    sys.exit("Invalid mtype '{}' in mangle64".format(mtype))

#
# Create a list of flags
#
def flagList(flags):
    if 0 == flags:
        return "<none>"
    l = ['timesout', 'bcanswer', 'noanswer', 'scale', 'clamp', 'resend']
    return ','.join([l[v] for v in range(len(l)) if flags & (1 << v)])

#
# Merge and check comms config from root element attributes
#
def verifyConfigParams(n):
    global configparams
    err = False
    # Merge attributes
    configparams = configparams | n;
    # Make it case insensitive
    configparams = { k: v.upper() for k,v in configparams.items() }
    # Fixup parity
    if configparams['parity'] not in PARITIES:
        perr("Attribute 'parity' must be 'E', 'O' or 'N'.")
        err = True
        configparams['parity'] = 'E'
    configparams['parity'] = PARITIES[configparams['parity']]
    # Fixup duplex
    if configparams['duplex'] not in DUPLEXES:
        perr("Attribute 'duplex' must be 'full' or 'half'.")
        err = True
        configparams['parity'] = 'FULL'
    configparams['duplex'] = DUPLEXES[configparams['duplex']]

    if 'AUTO' == configparams['rxdelay'] or 'AUTO' == configparams['txdelay']:
        # rxdelay and txdelay are bit-time based for baudrates <= 19200. The
        # timeouts are fixed at higher baudrates (remark in bottom of section
        # 2.5.1.1):
        #  - t1.5 =  750 microseconds
        #  - t3.5 = 1750 microseconds
        # We only use t3.5.
        try:
            baudrate = int(configparams['baudrate'], 0)
        except ValueError as e:
            perr("Baudrate must be an integer number".format(k))
            return True
        if baudrate > 582000:
            # with this baudrate, bittimes would be 1020
            pwarn("Baudrate > 582000 will make inter-frame timer overflow. Setting rxdelay and txdelay to maximum.")
            pwarn("You must manually set both rxdelay and txdelay attributes to override.")
            if 'AUTO' == configparams['rxdelay']:
                configparams['rxdelay'] = '1020'
            if 'AUTO' == configparams['txdelay']:
                configparams['txdelay'] = '1020'
        else:
            bittimes = (1750 * baudrate + 999999) // 1000000
            try:
                par = int(configparams['parity'], 0)
                stp = int(configparams['stopbits'], 0)
                bittimes = 10
                bittimes += 1 if par != 0 else 0
                bittimes += 1 if stp > 1 else 0
                bittimes = math.ceil(bittimes * 3.5)
                if 'AUTO' == configparams['rxdelay']:
                    configparams['rxdelay'] = "{}".format(bittimes - 1)
                if 'AUTO' == configparams['txdelay']:
                    configparams['txdelay'] = "{}".format(bittimes + 1)
            except ValueError as e:
                err = True

    # Set timeout to zero for auto calculation
    if 'AUTO' == configparams['timeout']:
        configparams['timeout'] = '0'

    # convert to int and check against the min/max values
    for k, v in configparams.items():
        # convert to integer
        try:
            p = int(v, 0)
            # Test limits
            if (p < CONFIGLIMITS[k][0] or p > CONFIGLIMITS[k][1]) and not (k == 'timeout' and p == 0):
                perr("Attribute '{}' in <mesamodbus> must be between {} and {} (is set to {})"
                    .format(k, CONFIGLIMITS[k][0], CONFIGLIMITS[k][1], p))
                err = True
            configparams[k] = p
        except ValueError as e:
            perr("Attribute '{}' must be an integer number".format(k))
            err = True
    return err

#
# Parse the <devices> tag content
#
def handleDevices(devs):
    devlist  = {'broadcast': 0}   # map device name to address
    addrlist = {0: 'broadcast'}   # map address to device name
    for dev in devs:
        ldl = "devices/device[{}]".format(1 + len(devlist))
        # Attribute name must exist and of proper format
        if 'name' not in dev.attrib:
            perr("Missing 'name' attribute in {}".format(ldl))
            continue
        name = dev.attrib['name']
        if not namepattern.match(name):
            perr("Attribute 'name' contains invalid characters {}".format(ldl))
            continue
        if len(name) > MAXPINNAME:
            perr("Attribute 'name' longer than {} characters in {}".format(MAXPINNAME, ldl))
            continue

        # Device address in byte value
        address = checkInt(dev.attrib, 'address')
        if None == address:
            perr("Attribute 'address' missing or has non-integer content in {}".format(ldl))
            continue
        # Modbus device address, 0=broadcast, 248..255=reserved
        if address < 1 or address > 255:
            perr("Address '{}' out of range [1..247] in {}".format(address, ldl))
            continue
        if address > 247:
            pwarn("Address '{}' is a Modbus reserved value. Valid range [1..247] in {}".format(address, ldl))

        # Don't allow duplicates in the list
        if name in devlist:
            perr("Device name '{}' already defined in {}".format(ldl))
            continue
        if address in addrlist:
            perr("Device address {} already used for device '{}' in {}".format(address, addrlist[address], ldl))
            continue

        addrlist[address] = name
        devlist[name] = address
        if verbose:
            print("Device bus ID 0x{0:02x} ({0:3}) ==> '{1}'".format(address, name))
    # end for dev in devs:

    # We should have one device on top of broadcast
    if len(devlist) < 2:
        pwarn("No devices defined other than broadcast")

    return devlist

#
# Parse optional attribute flags
#
def parseOptFlags(dev, attrs, defflags):
    flags = defflags
    flags |= MBCCB_CMDF_TIMESOUT if 'timesout' in attrs and attrs['timesout'] == '1' else 0
    flags |= MBCCB_CMDF_BCANSWER if 'bcanswer' in attrs and attrs['bcanswer'] == '1' else 0
    flags |= MBCCB_CMDF_NOANSWER if 'noanswer' in attrs and attrs['noanswer'] == '1' else 0
    flags |= MBCCB_CMDF_SCALE    if 'scale'    in attrs and attrs['scale']    == '1' else 0
    flags |= MBCCB_CMDF_CLAMP    if 'clamp'    in attrs and attrs['clamp']    == '1' else 0
    flags |= MBCCB_CMDF_RESEND   if 'resend'   in attrs and attrs['resend']   == '1' else 0
    # Scale and clamp can be default on. This allows them to be turned off.
    flags &= (~MBCCB_CMDF_SCALE & 0xffff) if 'scale' in attrs and attrs['scale'] == '0' else 0xffff
    flags &= (~MBCCB_CMDF_CLAMP & 0xffff) if 'clamp' in attrs and attrs['clamp'] == '0' else 0xffff
    if 'broadcast' != dev and 0 != (flags & MBCCB_CMDF_BCANSWER):
        perr("Cannot use 'bcanswer' attribute on non-broadcast target '{}'".format(dev));
    return flags

#
# Calculate the timeout for a command based on the number of bytes
# sent/receives in worst case scenario, double it for processing and add
# state-machine overhead.
#
def calcTimeout(function, count, mtype):
    # Find out the byte-size of the command+reply
    if function in [R_COILS, R_INPUTS]:
        n = 8 + 5 + ((count + 7) // 8)
    elif function in [W_COILS]:
        n = 9 + ((count + 7) // 8) + 8
    elif function in [R_REGISTERS, R_INPUTREGS]:
        n = 8 + 3 + count * MBTBYTESIZES[mtype & MBT_X_MASK]
    elif function in [W_REGISTER, W_COIL]:
        n = 8 + 8
    elif function in [W_REGISTERS]:
        n = 9 + count * MBTBYTESIZES[mtype & MBT_X_MASK] + 8
    else:
        sys.exit("Unknown function '{}' in calcTimeout, aborting...".format(function))
    bits = 1 + 8 + 1
    bits += 1 if configparams['parity'] != 0 else 0
    bits += 1 if configparams['stopbits'] == 1 else 0
    bits *= n   # Number of bits send+received
    # Worst case transmission is one character per just under 2.5 character
    # times (because of max. 1.5 inter-character gap allowance).
    bits *= 2.5
    # We allow the whole lot to take twice as long
    # Convert to microseconds based on baudrate
    # and add twice the state-machine base overhead based on a 1kHz servo-thread
    return int((2 * bits * 1000000.0 + configparams['baudrate'] - 1) / configparams['baudrate']) + 2*8000

#
# Parse the <initlist> tag content
#
def handleInits(inits):
    initlist = []
    for cmd in inits:
        lil = "initlist/command[{}]".format(1 + len(initlist))
        if cmd.tag != 'command':
            perr("Expected <command> tag as child of {}".format(lil))
            continue
        if 'delay' in cmd.attrib:
            # A delay between init commands
            delay = checkInt(cmd.attrib, 'delay')
            if delay < 0 or delay > MAXDELAY:
                perr("Attribute 'delay' out of range [0..{}] in {}".format(MAXDELAY, lil))
                continue
            initlist.append({'delay': delay})
            if verbose:
                print("Init {:2}: delay {} microseconds".format(len(initlist), delay))
        else:
            # An init command
            if 'device' not in cmd.attrib:
                perr("Attribute 'device' missing in {}".format(lil))
                continue

            # Device must be known
            device = cmd.attrib['device']
            if device not in devices:
                perr("Device name '{}' not found in devices list {}".format(device, lil))
                continue
            if 'function' not in cmd.attrib:
                perr("Attribute 'function' missing in {}".format(lil))
                continue

            # Get optional attrib flags
            flags = parseOptFlags(device, cmd.attrib, 0)
            if 0 != flags & ~MBCCB_CMDF_INITMASK:
                pwarn("Additional flags '0x{:04x}' (allowed=0x{:04x}) in {}".format(flags, MBCCB_CMDF_INITMASK, lil))

            # Find timeout to use
            if 'timeout' in cmd.attrib:
                # A specific timeout for this command
                timeout = checkInt(cmd.attrib, 'timeout')
                if timeout < 0 or timeout > MAXDELAY:
                    perr("Attribute 'timeout' out of range [0..{}] {}".format(MAXDELAY, lil))
                    continue
            else:
                global configparams
                timeout = configparams['timeout']   # Use global timeout

            # Must have a function
            # Only functions tagged are allowed
            function = cmd.attrib['function'].upper()
            if function not in FUNCTIONS:
                perr("Function '{}' out of range {} in {}".format(function, str(list(FUNCTIONS.keys())), lil))
                continue
            maxcount = FUNCTIONS[function][1]
            function = FUNCTIONS[function][0]

            # Need a target address
            address  = checkInt(cmd.attrib, 'address')
            if None == address:
                perr("Attribute 'address' invalid or missing in {}".format(lil))
                continue
            if address < 0 or address > 65535:
                perr("Address '{}' out of range [0..65535] in {}".format(address, lil))
                continue

            # Data to be sent with the command
            datalist = []
            dlnbytes = 0;
            err = False
            for data in cmd:
                ldl = "{}/data[{}]".format(lil, (1 + len(datalist)))
                if data.tag == 'description':
                    continue    # Ignored
                if data.tag != 'data':
                    perr("Expected <data> tag instead of '{}' as child in {}".format(data.tag, lil))
                    err = True
                    break
                # Write function must have data
                if FUNCNAMES[function] not in WRITEFUNCTIONS:
                    perr("Only write function can have <data> tag(s) in {}".format(data.tag, lil))
                    err = True
                    break
                # Get the type of the data
                mtype = data.attrib['modbustype'] if 'modbustype' in data.attrib else 'U_AB'
                if mtype not in MBTYPES:
                    perr("Invalid modbustype '{}' in {}".format(mtype, lil))
                    err = True
                    break
                mtype = MBTYPES[mtype][0]

                if MBT_F == (mtype & MBT_T_MASK):
                    # Handle floating point target
                    if function not in [W_REGISTER, W_REGISTERS]:
                        perr("Floating point data only possible for W_REGISTER(6)/W_REGISTERS(16) functions in {}".format(ldl))
                        err = True
                        break
                    if W_REGISTER == function and mtype not in [F_AB, F_BA]:
                        perr("Floating point value requires 16-bit type for W_REGISTER/6) in {}".format(ldl))
                        err = True
                        break

                    value = checkFlt(data.attrib, 'value')
                    if None == value:
                        perr("Attribute 'value' invalid or missing in {}".format(ldl))
                        err = True
                        break
                    if mtype in [F_AB, F_BA]:
                        # 16-bit float
                        if value < -65504.0 or value > 65504.0:
                            perr("Attribute 'value' out of range [-65504.0,+65504.0] for 16-bit float {}".format(ldl))
                            err = True
                            break
                        datalist.append(mangle16(struct.pack(">e", value), mtype))
                        dlnbytes += 2
                    elif mtype in [F_ABCD, F_BADC, F_CDAB, F_DCBA]:
                        # 32-bit float
                        if value <= -3.4e38 or value >= +3.4e38:    # +/- 3.4028234664e38
                            perr("Attribute 'value' out of range [-3.4e38,+3.4e38] for 32-bit float {}".format(ldl))
                            err = True
                            break
                        datalist.append(mangle32(struct.pack(">f", value), mtype))
                        dlnbytes += 4
                    else: # value is ok. otherwise we'd get an OverflowError on conversion
                        # 64-bit float
                        # XXX: +/inf and NaN are also fine?
                        datalist.append(mangle64(struct.pack(">d", value), mtype))
                        dlnbytes += 8
                else: # no modbustype attribute or MBT_U or MBT_S
                    value = checkInt(data.attrib, 'value')
                    if None == value:
                        perr("Attribute 'value' invalid or missing in {}".format(ldl))
                        err = True
                        break
                    # Write single coil usually has special value
                    # We allow deviation because some devices allow it too
                    if W_COIL == function:
                        if 'modbustype' in data.attrib or mtype != U_AB:
                            pwarn("Attribute 'modbustype' ignored for W_COIL(5) and always set to U_AB in {}".format(ldl))
                        if value != 0 and value != 0xff00:
                            pwarn("Data value '{}' for W_COIL(5) is normally expected to be 0 or 0xff00 (65280) in {}".format(value, ldl))
                        datalist.append(struct.pack('>H', value));
                        dlnbytes += 2
                    elif W_COILS == function:
                        if 'modbustype' in data.attrib or mtype != U_AB:
                            perr("Attribute 'modbustype' ignored for W_COILS(15) and always set to binary bits in {}".format(ldl))
                            err = True
                            break;
                        if 0 != value and 1 != value:
                            perr("Function W_COILS(15) only accepts binary data in range [0,1] in {}".format(ldl))
                            err = True
                            break;
                        datalist.append(struct.pack('>B', value));
                        dlnbytes += 1
                    elif W_REGISTER == function:
                        if mtype not in [U_AB, U_BA, S_AB, S_BA]:
                            perr("Attribute 'modbustype' invalid for W_REGISTER(6), expected one of (U_AB,U_BA,S_AB,S_BA) in {}".format(ldl))
                            err = True
                            break;
                        if mtype in [U_AB, U_BA]:
                            # 16-bit unsigned
                            if value < 0 or value > 65535:
                                perr("Value '{}' outside valid range [0,65535] for W_REGISTER(6) in {}".format(value, ldl))
                                err = True
                                break;
                            datalist.append(mangle16(struct.pack('>H', value), mtype));
                            dlnbytes += 2
                        else:
                            # 16-bit signed
                            if value < -32768 or value > +32767:
                                perr("Value '{}' outside valid range [-32768,+32767] for W_REGISTER(6) in {}".format(value, ldl))
                                err = True
                                break;
                            datalist.append(mangle16(struct.pack('>h', value), mtype));
                            dlnbytes += 2
                    else: # function == W_REGISTERS
                        if mtype in [U_AB, U_BA]:
                            # 16-bit unsigned
                            if value < 0 or value > 65535:
                                perr("Value '{}' outside valid range [0,65535] for W_REGISTER(6) in {}".format(value, ldl))
                                err = True
                                break;
                            datalist.append(mangle16(struct.pack('>H', value), mtype));
                            dlnbytes += 2
                        elif mtype in [S_AB, S_BA]:
                            # 16-bit signed
                            if value < -32768 or value > +32767:
                                perr("Value '{}' outside valid range [-32768,+32767] for W_REGISTER(16) in {}".format(value, ldl))
                                err = True
                                break;
                            datalist.append(mangle16(struct.pack('>h', value), mtype));
                            dlnbytes += 2
                        elif mtype in [U_ABCD, U_BADC, U_CDAB, U_DCBA]:
                            # 32-bit unsigned
                            if value < 0 or value > 0xffffffff:
                                perr("Value '{}' outside valid range [0,4294967295] for W_REGISTERS(16) in {}".format(value, ldl))
                                err = True
                                break;
                            datalist.append(mangle32(struct.pack(">I", value), mtype))
                            dlnbytes += 4
                        elif mtype in [S_ABCD, S_BADC, S_CDAB, S_DCBA]:
                            # 32-bit signed
                            if value < -2147483648 or value > +2147483647:
                                perr("Value '{}' outside valid range [-2147483648,+2147483647] for W_REGISTERS(16) in {}".format(value, ldl))
                                err = True
                                break;
                            datalist.append(mangle32(struct.pack(">i", value), mtype))
                            dlnbytes += 4
                        elif MBT_S == mtype & MBT_T_MASK:
                            # 64-bit signed
                            if value < 0x8000000000000000 or value > 0x7fffffffffffffff:
                                perr("Value '{}' outside valid 64-bit range [{},{}] for W_REGISTERS(16) in {}"
                                        .format(value, 0x8000000000000000, 0x7fffffffffffffff, ldl))
                                err = True
                                break;
                            datalist.append(mangle64(struct.pack(">q", value), mtype))
                            dlnbytes += 8
                        else: # MBT_U == mtype & MBT_T_MASK
                            # 64-bit unsigned
                            if value > 0xffffffffffffffff:
                                perr("Value '{}' outside valid 64-bit range [0,{}] for W_REGISTERS(16) in {}"
                                        .format(value, 0xffffffffffffffff, ldl))
                                err = True
                                break;
                            datalist.append(mangle64(struct.pack(">Q", value), mtype))
                            dlnbytes += 8
                    # end if [functions]
                # end if MBT_x types
            # end for data in cmd:
            if err:
                continue

            # Read functions must have a count
            if function not in WRITEFUNCTIONS:
                if function in [R_COILS, R_REGISTERS, R_INPUTREGS]:
                    if count < 1 or count > maxcount:
                        print("Attribute 'count' '{}' out of range [1-{}] in {}".format(count, maxcount, lil))
                        continue
                else:
                    count = 1
            else:
                count = dlnbytes // 2 if function != R_COILS else dlnbytes

            # We cannot write beyond the address region in multi-writes
            if address + maxcount > 65536:
                maxcount = 65536 - address

            if W_COILS != function and 0 != dlnbytes & 1:
                perr("Odd number of data bytes for function {}({}) in {}".format(FUNCNAMES[function], function, lil))
                continue

            if count < 1 or count > maxcount:
                perr("Data element count '{}' is out of range [1..{}] for function {}({}) in {}"
                     .format(b, maxcount, FUNCNAMES[function], function, lil))
                continue

            if 0 == timeout:
                timeout = calcTimeout(function, count, MBT_AB);

            initlist.append({'device': device, 'mbid': devices[device], 'function': function,
                             'address': address, 'data': datalist, 'timeout': timeout, 'flags': flags })
            if verbose:
                print("Init {:2}: {} {}({}) addr=0x{:04x} flags={} timeout={} data="
                      .format(len(initlist), device, FUNCNAMES[function], function, address, flagList(flags), timeout), end='')
                print(','.join((''.join("{:02x}".format(c) for c in d)) for d in datalist))

        # endif
    # end for cmd in init:

    return initlist

#
# Parse the <commands> tag content
#
def handleCommands(commands):
    cmdlist = []
    pinlistall = []
    for cmd in commands:
        lcl = "commands/command[{}]".format(1 + len(cmdlist))
        if cmd.tag != 'command':
            perr("Expected <command> tag as child of <commands>")
            continue

        if 'delay' in cmd.attrib:
            # A delay between commands
            delay = checkInt(cmd.attrib, 'delay')
            if delay < 0 or delay > MAXDELAY:
                perr("Attribute 'delay' out of range [0..{}] {}".format(MAXDELAY, lil))
                continue
            cmdlist.append({'delay': delay})
            if verbose:
                print("Command {:2}: delay {} microseconds".format(len(cmdlist), delay))
        else:
            # Device must be known
            if 'device' not in cmd.attrib:
                perr("Attribute 'device' missing in {}".format(lcl))
                continue
            device = cmd.attrib['device']
            if device not in devices:
                perr("Device name '{}' not found in devices list {}".format(device, lcl))
                continue

            # Find timeout to use
            if 'timeout' in cmd.attrib:
                # A specific timeout for this command
                timeout = checkInt(cmd.attrib, 'timeout')
                if timeout < 0 or timeout > MAXDELAY:
                    perr("Attribute 'timeout' out of range [0..{}] {}".format(MAXDELAY, lil))
                    continue
            else:
                global configparams
                timeout = configparams['timeout']   # Use global timeout

            # Must have a target address
            address = checkInt(cmd.attrib, 'address')
            if None == address:
                perr("Attribute 'address' invalid or missing in {}".format(lcl))
                continue
            if address < 0 or address > 65535:
                perr("Address '{}' out of range [0..65535] in {}".format(address, lcl))
                continue

            # The command function to perform
            if 'function' not in cmd.attrib:
                perr("Attribute 'function' missing in {}".format(lcl))
                continue
            function = cmd.attrib['function'].upper()
            # check depends on unicast or broadcast
            if 'broadcast' == device:
                if function not in WRITEFUNCTIONS:
                    perr("Function '{}' not available from {} for broadcast in {}".format(function, str(list(WRITEFUNCTIONS.keys())), lcl))
                    continue
            else:
                if function not in FUNCTIONS:
                    perr("Function '{}' not defined in {} in {}".format(function, str(list(FUNCTIONS.keys())), lcl))
                    continue
            function = FUNCTIONS[function][0]

            # How many pins to read/write
            if W_REGISTER == function and 'count' not in cmd.attrib:
                count = 1
            else:
                count = checkInt(cmd.attrib, 'count')

            if function in BITFUNCTIONS:
                # Coils and inputs are binary and always map to HAL_BIT
                if 'modbustype' in cmd.attrib:
                    pwarn("Attribute 'modbustype' ignored for bit functions ({}) in {}".format(str(list(BITFUNCTIONS.keys())), lcl))
                if 'haltype' in cmd.attrib and cmd.attrib['haltype'].upper() != 'HAL_BIT':
                    pwarn("Attribute 'haltype' ignore for bit functions (always HAL_BIT) in {}".format(lcl))
                nwords   = 1
                maxcount = 2000
                mtype    = 0
                haltype  = HAL_BIT
            else:
                # Others must specify a type
                if 'haltype' not in cmd.attrib:
                    perr("Attribute 'haltype' missing in {}".format(lcl))
                    continue
                haltype = cmd.attrib['haltype'].upper()
                if haltype not in HALTYPES:
                    perr("Invalid haltype '{}' must be one of {} in {}".format(haltype, str(list(HALTYPES.keys())), lcl))
                    continue
                if 'modbustype' not in cmd.attrib:
                    pwarn("Attribute 'modbustype' missing in {}. Defaulting to U_AB.".format(lcl))
                    mtype = 'U_AB';
                else:
                    mtype = cmd.attrib['modbustype'].upper()
                if mtype not in MBTYPES:
                    perr("Invalid modbustype '{}' must be one of {} in {}".format(mtype, str(list(MBTYPES.keys())), lcl))
                    continue
                nwords   = MBTYPES[mtype][2]
                maxcount = MBTYPES[mtype][1]
                mtype    = MBTYPES[mtype][0]
                haltype  = HALTYPES[haltype]
                if W_REGISTER == function:
                    if (mtype & MBT_X_MASK) not in [MBT_AB, MBT_BA]:
                        perr("Function W_REGISTER(6) requires a 16-bit modbustype (S_AB,S_BA,U_AB,U_BA,F_AB,F_BA) in {}".format(lcl))
                        continue
                    if 1 != count:
                        perr("Function W_REGISTER(6) must have a count of one (1) in {}".format(lcl))
                        continue

            # Get optional attrib flags
            # scale is default for float pins
            # clamp is default on all pins
            sflag  = MBCCB_CMDF_SCALE if haltype == HAL_FLT else 0
            sflag |= MBCCB_CMDF_CLAMP if function not in BITFUNCTIONS else 0
            flags = parseOptFlags(device, cmd.attrib, sflag)
            if 0 != flags & ~MBCCB_CMDF_MASK:
                pwarn("Additional flags '0x{:04x}' (allowed=0x{:04x}) in {}".format(flags, MBCCB_CMDF_MASK, lcl))

            if (flags & MBCCB_CMDF_SCALE) and haltype in [HAL_U32, HAL_U64]:
                pwarn("Unsigned hal types cannot be scaled, disabling in {}".format(lcl))
                flags &= ~MBCCB_CMDF_SCALE & 0xffff

            if ((0 == flags & MBCCB_CMDF_SCALE) and
                       ((MBT_S == (mtype & MBT_T_MASK) and haltype in [HAL_U32, HAL_U64])
                     or (MBT_U == (mtype & MBT_T_MASK) and haltype in [HAL_S32, HAL_S64]))):
                pwarn("Signedness mismatch between haltype={} and modbustype={} may give wrong results in {}"
                        .format(HALNAMES[haltype], MBNAMES[mtype], lcl))

            # Don't wrap address
            if maxcount * nwords + address > 65536:
                maxcount = (65536 - address) // nwords

            pinlist = []
            err = False
            for pin in cmd:
                lpl = "{}/pin[{}]".format(lcl, (1 + len(pinlist)))
                if pin.tag == 'description':
                    continue
                if pin.tag != 'pin':
                    perr("Expected <pin> tag {}".format(lcl))
                    err = True
                    break
                if 'name' not in pin.attrib:
                    perr("Attribute 'name' missing in {}".format(lpl))
                    err = True
                    break
                pinname = pin.attrib['name']
                if not pinpattern.match(pinname):
                    perr("Attribute 'name' contains invalid characters in {}".format(lpl))
                    err = True
                    break
                if len(pinname) > MAXPINNAME:
                    perr("Attribute 'name' longer than {} characters in {}".format(MAXPINNAME, lpl))
                    err = True
                    break
                pintag = "{}.{}".format(device, pinname)
                if pintag in pinlistall:
                    perr("Pin name '{}' already in use in {}".format(pintag, lpl))
                    err = True
                    break
                pinlist.append(pintag)
                pinlistall.append(pintag)

            if err:
                continue

            if len(pinlist) <= 0:
                # If no <pin> list, then we require a count
                if None == count:
                    perr("Attribute 'count' invalid or missing in {}".format(lcl))
                    continue
            else:
                # We have a <pin> list; it may need to be appended
                if None == count:
                    count = len(pinlist)    # pin list determines count
                elif len(pinlist) > count:
                    perr("Number of pins '{}' larger than count '{}' in {}".format(len(pinlist), count, lcl))
                    continue

            if count < 1 or count > maxcount:
                perr("Number of pins '{}' out of range [1..{}] in {}".format(count, maxcount, lcl))
                continue

            # Append default naming scheme "name-XX"
            if len(pinlist) < count:
                if 'name' not in cmd.attrib:
                    perr("Attribute 'name' invalid or missing in {}".format(lcl))
                    continue
                pinbase = cmd.attrib['name']
                if not pinpattern.match(pinbase):
                    perr("Attribute 'name' contains invalid characters {}".format(lcl))
                    continue
                if len(pinbase) > MAXPINNAME - 3:
                    perr("Attribute 'name' longer than {} characters in {}".format(MAXPINNAME - 3, lcl))
                    continue
                # Fill pin list with generated names
                err = False
                for i in range(len(pinlist), count):
                    pintag = "{}.{}-{:02d}".format(device, pinbase, i)
                    if pintag in pinlistall:
                        perr("Pin name '{}' already in use in {}".format(pintag, lcl))
                        err = True
                        break
                    pinlist.append(pintag)
                    pinlistall.append(pintag)
                if err:
                    continue

            if 0 == timeout:
                timeout = calcTimeout(function, count, mtype);

            cmdlist.append({'device': device, 'mbid': devices[device], 'function': function, 'timeout': timeout,
                            'address': address, 'mtype': mtype, 'htype': haltype, 'count': count, 'pins': pinlist, 'flags': flags })
            if verbose:
                mbn = 'BIT' if haltype == HAL_BIT else MBNAMES[mtype]
                print("Command {:2}: {} {}({}) addr=0x{:04x} flags={} {}<=>{} timeout={}"
                    .format(len(cmdlist), device, FUNCNAMES[function], function, address, flagList(flags), HALNAMES[haltype], mbn, timeout))
                io = "in " if FUNCNAMES[function] in WRITEFUNCTIONS else "out"
                for p in range(len(pinlist)):
                    print("  pin {:2} ({}): {}".format(p+1, io, pinlist[p]))
                    if flags & MBCCB_CMDF_SCALE:
                        print("              : {}.offset".format(pinlist[p]))
                        print("              : {}.scale".format(pinlist[p]))
                        if FUNCNAMES[function] not in WRITEFUNCTIONS:
                            print("              : {}.scaled".format(pinlist[p]))
        # endif
    # end for cmd in commands:

    return cmdlist

#
# Main program
#
def main():
    # Get the command-line options
    try:
        opts, args = getopt.getopt(sys.argv[1:], "ho:v", ["help", "output=", "verbose"])
    except getopt.GetoptError as err:
        print(err, file=sys.stderr)  # Something like "option -a not recognized"
        return 1

    # Check the options
    output = "mesamodbus.output.mbccb"
    global verbose
    for o, a in opts:
        if o in ("-v", "--verbose"):
            verbose = True
        elif o in ("-o", "--output"):
            output = a
        elif o in ("-h", "--help"):
            usage()
        else:
            print("Unhandled option: '{}'".format(o), file=sys.syderr);
            return 1

    if len(args) < 1:
        print("Must have one file argument.", file=sys.stderr);
        return 1
    if len(args) > 1:
        print("Cannot do more than one file at a time.", file=sys.stderr);
        return 1

    global inputfilename
    inputfilename = args[0]

    # Parse the XML source
    try:
        mbccs = ET.parse(inputfilename);
    except ET.ParseError as err:
        perr("line {}, char {}: {}".format(*err.position, XP.ErrorString(err.code)))
        return 1

    root = mbccs.getroot();
    if root.tag != "mesamodbus":
        perr("Expected <mesamodbus> root tag")
        return 1

    # Check the comms attributes
    if verifyConfigParams(root.attrib):
        return 1

    if verbose:
        print("Communication parameters:")
        print("  baudrate  : {}".format(configparams['baudrate']))
        print("  parity    : {}".format(['None', 'Odd', 'Even'][configparams['parity']]))
        print("  stopbits  : {}".format(configparams['stopbits']))
        print("  rxdelay   : {} bits".format(configparams['rxdelay']))
        print("  txdelay   : {} bits".format(configparams['txdelay']))
        print("  drivedelay: {} bits".format(configparams['drivedelay']))
        if 0 == configparams['drivedelay']:
            print("  interval  : 0 (as fast as possible)".format(configparams['interval']))
        else:
            print("  interval  : {} microseconds".format(configparams['interval']))
        #print("  timeout   : {} microseconds".format(configparams['timeout']))

    # Parse the nodes
    global devices
    global errorflag
    initlist = None    # Modbus devices init commands
    commands = None    # Modbus I/O commands
    for node in root:
        if node.tag == "devices":
            if None != devices:
                perr("Multiple <devices> tags")
                continue
            devices = handleDevices(node)
            if None == devices:
                return 1
        else:
            if None == devices:
                perr("The <devices> tag must be declared first")
                break   # This error is fatal, we need the devices list

            if node.tag == 'initlist':
                if None != initlist:
                    perr("Multiple <initlist> tags")
                    continue
                initlist = handleInits(node)
                if None == initlist:
                    errorflag = True
                    continue
            elif node.tag == 'commands':
                if None != commands:
                    perr("Multiple <commands> tags")
                    continue
                commands = handleCommands(node)
                if None == commands:
                    errorflag = True
                    continue
            else:
                perr("Invalid/unknown tag '{}'".format(node.tag))

    # Quit if a parse error occurred
    if errorflag:
        return 1

    # Now construct the binary image
    npins = 0   # statistics (all mirrors and lies)
    ilb = []    # initlist binary hm2_modbus_mbccb_cmd_t
    cmb = []    # commands binary hm2_modbus_mbccb_cmd_t
    dlb = []    # datalist binary fragments
    dlblen = 0  # datalist binary accumulated length
    #
    # The init commands are pre-packed, including mbid and function code. They
    # only need the CRC attached, which is done in the hm2_modbus module.
    #
    # typedef struct {
    #   rtapi_u8  mbid;    // Modbus device ID
    #   rtapi_u8  func;    // Function code, 0 for delay
    #   rtapi_u16 addr;    // Address, 0 in init
    #   rtapi_u16 pincnt;  // Number of pins, 0 in init
    #   rtapi_u8  htype;   // HAL data type, 0 for init
    #   rtapi_u8  mtype;   // Modbus data type, 0 for init
    #   rtapi_u16 flags;   // Mostly quirks to handle, see MBCCB_CMDF_* defines
    #   rtapi_u16 unused1;
    #   rtapi_u32 unused2[3];
    #   rtapi_u32 timeout; // Response timeout or delay in microseconds
    #   rtapi_u32 dataptr; // Pin names, packet data for init
    # } hm2_modbus_mbccb_cmds_t;
    #
    for i in initlist:
        if 'delay' in i:
            ilb.append(struct.pack(">BBHHBBHHIIIII", 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, i['delay'], 0))
        else:
            mbid = i['mbid']
            func = i['function']
            addr = i['address']
            ilb.append(struct.pack(">BBHHBBHHIIIII", mbid, func, addr, 0, 0, 0, i['flags'], 0, 0, 0, 0, i['timeout'], dlblen))
            # Precompile the data packet so we only need to copy it
            if func in [W_COIL, W_REGISTER]:  # Write single coil or single register
                # mbid, func, length address and value == 6 bytes
                assert len(i['data']) == 1
                assert len(i['data'][0]) == 2
                dlb.append(struct.pack(">BBBH", 6, mbid, func, addr))
                dlb.append(i['data'][0])
                dlblen += 6 + 1
            elif W_COILS == func:    # Write multiple coils
                nbytes = (len(i['data']) + 7) // 8
                # mbid, func, length address, count and bytecount == 6 + 1 bytes
                d = struct.pack(">BBBHHB", 6 + 1 + nbytes, mbid, func, addr, len(i['data']), nbytes)
                dlblen += 6 + 1 + nbytes + 1
                # Construct a bit string
                val = ''.join(format(bit[0], '01b') for bit in i['data'])
                # Split string in 8-bit sequences
                parts = [val[x:x+8] for x in range(0, len(val), 8)]
                assert nbytes == len(parts)
                # for all bytes, reverse bit string and convert to integer base 2
                for part in parts:
                    d += struct.pack("B", int(part[::-1], 2))
                dlb.append(d)
            elif W_REGISTERS == func:    # Write multiple registers
                d = b''
                for w in i['data']:
                    d += w
                ld = len(d)
                # mbid, func, length address, count and bytecount == 6 + 1 bytes
                d = struct.pack(">BBBHHB", 6 + 1 + ld, mbid, func, addr, ld // 2, ld) + d
                dlblen += 6 + 1 + 1 + ld
                dlb.append(d)
            elif func in [R_COILS, R_INPUTS, R_INPUTREGS, R_REGISTERS]:
                # mbid, func, address and number == 6 bytes
                dlb.append(struct.pack(">BBBHH", 6, mbid, func, addr, i['data'][0]))
                dlblen += 6 + 1
            else:
                perr("Unhandled init function mbid={}, func={}, addr={}".format(mbid, func, addr))
    #
    # typedef struct {
    #   rtapi_u8  mbid;    // Modbus device ID
    #   rtapi_u8  func;    // Function code
    #   rtapi_u16 addr;    // Address
    #   rtapi_u16 pincnt;  // Number of pins
    #   rtapi_u8  htype;   // HAL data type, 0 for init
    #   rtapi_u8  mtype;   // Modbus data type, 0 for init
    #   rtapi_u16 flags;   // Mostly quirks to handle, see MBCCB_CMDF_* defines
    #   rtapi_u16 unused1;
    #   rtapi_u32 unused2[3];
    #   rtapi_u32 timeout; // Response timeout or delay in microseconds
    #   rtapi_u32 dataptr; // Pin names
    # } hm2_modbus_mbccb_cmds_t;
    #
    for i in commands:
        if 'delay' in i:
            cmb.append(struct.pack(">BBHHBBHHIIIII", 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, i['delay'], 0))
        else:
            cmb.append(struct.pack(">BBHHBBHHIIIII", i['mbid'], i['function'], i['address'], i['count'],
                                                     i['htype'], i['mtype'], i['flags'], 0, 0, 0, 0,
                                                     i['timeout'], dlblen))
            # Add pin names to data list
            for p in i['pins']:
                asc = p.encode('ascii')
                d = struct.pack(">B", len(asc)+1) + asc + struct.pack(">B", 0)
                dlblen += 1 + len(asc) + 1
                dlb.append(d)
            npins += len(i['pins'])

    # typedef struct {
    #   rtapi_u8  sig[8];   // Signature and version {'M','e','s','a','M','B','0','1'}
    #   rtapi_u32 baudrate;
    #   rtapi_u8  format;   // Parity and stopbits
    #   rtapi_u8  txdelay;  // Tx t3.5
    #   rtapi_u8  rxdelay;  // Rx t3.5
    #   rtapi_u8  drvdelay; // Delay from output enable to tx start
    #   rtapi_u32 interval; // Update rate in _micro_second interval time (0 = as-fast-as-possible)
    #   rtapi_u32 unused[8];
    #   rtapi_u32 initlen;  // Length of init section
    #   rtapi_u32 cmdslen;  // Length of command section
    #   rtapi_u32 datalen;  // Length of data table
    # } hm2_modbus_mbccb_header_t;
    header = (struct.pack(">8sIBBBBIIIIIIIIIIII",
                        b'MesaMB01',
                        configparams['baudrate'],
                        configparams['parity'] + ((configparams['stopbits'] - 1) << 2) + (configparams['duplex'] << 3),
                        configparams['txdelay'],
                        configparams['rxdelay'],
                        configparams['drivedelay'],
                        configparams['interval'],
                        0, 0, 0, 0, 0, 0, 0, 0,
                        32*len(ilb), 32*len(cmb), dlblen))

    # Generate output
    fsize = len(header)
    try:
        with open(output, "wb") as f:
            f.write(header)
            for b in ilb:
                f.write(b)
                fsize += len(b)
            for b in cmb:
                f.write(b)
                fsize += len(b)
            for b in dlb:
                f.write(b)
                fsize += len(b)
    except OSError as err:
        print("Cannot open '{}' for output: {}".format(output, err.strerror))
        return 1

    if verbose:
        print("Wrote output file '{}':".format(output))
        print("  {:3} inits".format(len(ilb)))
        print("  {:3} commands".format(len(cmb)))
        print("  {:3} pins".format(npins))
        print("  {:3} data fragments".format(len(dlb) - npins))
        print("  total {} bytes".format(fsize))

    # and...done
    return 0

if __name__ == "__main__":
    sys.exit(main())
