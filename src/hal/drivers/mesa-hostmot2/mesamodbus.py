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
# Broadcasts may set the 'quirks="1"' attribute to signal a reply being sent
# from the slave, even though it should keep quiet.
#

import sys
import os
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
                'rxdelay'   : False,
                'txdelay'   : False,
                'drivedelay': '0',
                'interval': '0',
                'timeout'   : '1000000' }

# Allowed values for the parity attribute
PARITIES = {'N': '0', 'O': '1', 'E': '2',
            '0': '0', '1': '1', '2': '2' }

CONFIGLIMITS = {'baudrate'  : [1200, 10000000],
                'parity'    : [0, 2],
                'stopbits'  : [0, 1],
                'rxdelay'   : [15, 255],
                'txdelay'   : [17, 255],
                'drivedelay': [0, 31],
                'interval'  : [0, 1000000000],  # As-fast-as possible to 1000 seconds
                'timeout'   : [50000, 5000000] } # 50 milliseconds to 5 seconds

# Possible values of the 'type' attribute
# [typeId, maxCount, nWords]
PINTYPES =  { 'HAL_BIT'       : [1,   2000, 1], 'HAL_FLOAT'     : [2,    125, 1],
              'HAL_S32'       : [3,    125, 1], 'HAL_U32'       : [4,    125, 1],
              'HAL_FLOAT_ABCD': [0x102, 62, 2], 'HAL_FLOAT_BADC': [0x202, 62, 2],
              'HAL_FLOAT_CDAB': [0x302, 62, 2], 'HAL_FLOAT_DCBA': [0x402, 62, 2],
              'HAL_S32_ABCD'  : [0x103, 62, 2], 'HAL_S32_BADC'  : [0x203, 62, 2],
              'HAL_S32_CDAB'  : [0x303, 62, 2], 'HAL_S32_DCBA'  : [0x403, 62, 2],
              'HAL_U32_ABCD'  : [0x104, 62, 2], 'HAL_U32_BADC'  : [0x204, 62, 2],
              'HAL_U32_CDAB'  : [0x304, 62, 2], 'HAL_U32_DCBA'  : [0x404, 62, 2] }

# Reverse map of pin types
TYPENAMES = { 1    : 'HAL_BIT',        2    : 'HAL_FLOAT',
              3    : 'HAL_S32',        4    : 'HAL_U32',
              0x102: 'HAL_FLOAT_ABCD', 0x202: 'HAL_FLOAT_BADC',
              0x302: 'HAL_FLOAT_CDAB', 0x402: 'HAL_FLOAT_DCBA',
              0x103: 'HAL_S32_ABCD',   0x203: 'HAL_S32_BADC',
              0x303: 'HAL_S32_CDAB',   0x403: 'HAL_S32_DCBA',
              0x104: 'HAL_U32_ABCD',   0x204: 'HAL_U32_BADC',
              0x304: 'HAL_U32_CDAB',   0x404: 'HAL_U32_DCBA' }

R_COILS     =  1
R_INPUTS    =  2
R_REGISTERS =  3
R_INPUTREGS =  4
W_COIL      =  5
W_REGISTER  =  6
W_COILS     = 15
W_REGISTERS = 16

# Map function names/numbers to value
FUNCTIONS = { 'R_COILS': R_COILS, 'R_INPUTS'  : R_INPUTS,   'R_REGISTERS': R_REGISTERS, 'R_INPUTREGS': R_INPUTREGS,
              'W_COIL' : W_COIL,  'W_REGISTER': W_REGISTER, 'W_COILS'    : W_COILS,     'W_REGISTERS': W_REGISTERS,
              '1'      : R_COILS, '2'         : R_INPUTS,   '3'          : R_REGISTERS, '4'          : R_INPUTREGS,
              '5'      : W_COIL,  '6'         : W_REGISTER, '15'         : W_COILS,     '16'         : W_REGISTERS }

FUNCNAMES = { R_COILS: 'R_COILS', R_INPUTS:   'R_INPUTS',   R_REGISTERS: 'R_REGISTERS', R_INPUTREGS: 'R_INPUTREGS',
              W_COIL:  'W_COIL',  W_REGISTER: 'W_REGISTER', W_COILS:     'W_COILS',     W_REGISTERS: 'W_REGISTERS' }

# These functions always use HAL_BIT
BITFUNCTIONS = [R_COILS, R_INPUTS, W_COIL, W_COILS]

# In init-phase or broadcast only write commands are allowed
# [funcId, maxCount]
WRITEFUNCTIONS = { 'W_COIL' : [W_COIL, 1], 'W_REGISTER' : [W_REGISTER, 1], 'W_COILS': [W_COILS, 2000], 'W_REGISTERS': [W_REGISTERS, 125],
                   '5'      : [W_COIL, 1], '6'          : [W_REGISTER, 1], '15'     : [W_COILS, 2000], '16'         : [W_REGISTERS, 125] }

# XXX: keep in sync with hm2_modbus.h
MAXDELAY = 60000000    # Max one minute delay between init commands (in microseconds)
MAXPINNAME = 24        # Max chars for a name

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
    if not k in s:
        return None
    try:
        val = int(s[k], 0)
    except ValueError as err:
        perr("Invalid integer '{}'".format(s))
        return None
    return val

#
# Merge and check comms config from root element attributes
#
def verifyConfigParams(n):
    global configparams
    err = False
    # Merge attributes
    configparams = configparams | n;
    # Fixup parity
    if not configparams['parity'] in PARITIES:
        perr("Attribute 'parity' must be 'E', 'O' or 'N' (or numeric 2, 1 or 0 resp.).")
        err = True
        configparams['parity'] = 'E'
    configparams['parity'] = PARITIES[configparams['parity']]

    if not configparams['rxdelay'] or not configparams['txdelay']:
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
        if baudrate > 144000:
            # with this baudrate, bittimes would be 253
            pwarn("Baudrate > 144000 will make silence timers overflow. Setting rxdelay and txdelay to maximum.")
            pwarn("You must manually set both rxdelay and txdelay attributes to override.")
            if not configparams['rxdelay']:
                configparams['rxdelay'] = '255'
            if not configparams['txdelay']:
                configparams['txdelay'] = '255'
        else:
            bittimes = (1750 * baudrate + 999999) // 1000000
            if not configparams['rxdelay']:
                configparams['rxdelay'] = '37' if baudrate <= 19200 else "{}".format(bittimes - 2)
            if not configparams['txdelay']:
                configparams['txdelay'] = '41' if baudrate <= 19200 else "{}".format(bittimes - 2)

    # convert to int and check against the min/max values
    for k, v in configparams.items():
        # convert to integer
        try:
            p = int(v, 0)
            # Test limits
            if p < CONFIGLIMITS[k][0] or p > CONFIGLIMITS[k][1]:
                perr("Attribute '{}' in <mesamodbus> must be between {} and {} (is set to {})"
                    .format(k, configlimits[k][0], configlimits[k][1], p))
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
        if not 'name' in dev.attrib:
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
        if address < 1 or address > 247:
            perr("Address '{}' out of range [1..247] in {}".format(address, ldl))
            continue

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
            if not 'device' in cmd.attrib:
                perr("Attribute 'device' missing in {}".format(lil))
                continue

            # Device must be known
            device = cmd.attrib['device']
            if not device in devices:
                perr("Device name '{}' not found in devices list {}".format(device, lil))
                continue
            if not 'function' in cmd.attrib:
                perr("Attribute 'function' missing in {}".format(lil))
                continue

            # Marker for broadcasts that get replies
            quirks = 1 if 'broadcast' == device and 'quirks' in cmd.attrib and cmd.attrib['quirks'] == '1' else 0

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
            # Only functions tagged as allowed
            function = cmd.attrib['function']
            if not function in WRITEFUNCTIONS:
                perr("Function '{}' out of range {} in {}".format(function, str(list(WRITEFUNCTIONS.keys())), lil))
                continue
            maxcount = WRITEFUNCTIONS[function][1]
            function = WRITEFUNCTIONS[function][0]

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
            err = False
            for data in cmd:
                ldl = "{}/data[{}]".format(lil, (1 + len(datalist)))
                if data.tag == 'description':
                    continue    # Ignored
                if data.tag != 'data':
                    perr("Expected <data> tag instead of '{}' as child of {}".format(data.tag, lil))
                    err = True
                    break
                value = checkInt(data.attrib, 'value')
                if None == value:
                    perr("Attribute 'value' invalid or missing in {}".format(ldl))
                    err = True
                    break
                if function == W_COILS:  # write multiple coils
                    maxval = 1
                else:
                    maxval = 65535
                if value < 0 or value > maxval:
                    perr("Value '{}' out of range [0..{}] in {}".format(value, maxval, ldl))
                    err = True
                    break
                # Write single coil usually has special value
                # We allow deviation because some devices allow it too
                if function == W_COIL and value != 0 and value != 0xff00:
                    pwarn("Data value '{}' for single coil write is normally expected to be 0 or 0xff00 (65280) in {}".format(maxval, ldl))
                datalist.append(value);
            # end for data in cmd:
            if err:
                continue

            # We cannot write beyond the address region in multi-writes
            if address + maxcount > 65536:
                maxcount = 65536 - address

            if len(datalist) < 1 or len(datalist) > maxcount:
                perr("Data list count '{}' is out of range [1..{}] in {}".format(len(datalist), maxcount, lil))
                continue

            initlist.append({'device': device, 'mbid': devices[device], 'function': function,
                             'address': address, 'data': datalist, 'timeout': timeout, 'quirks': quirks })
            if verbose:
                print("Init {:2}: {} {}({}) addr=0x{:04x} data=".format(len(initlist), device, FUNCNAMES[function], function, address), end='')
                print(','.join(format(d, ('01b' if W_COILS == function else '04x')) for d in datalist))

        # endif
    # end for cmd in init:

    return initlist

#
# Parse the <commands> tag content
#
def handleCommands(commands):
    cmdlist = []
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
            if not 'device' in cmd.attrib:
                perr("Attribute 'device' missing in {}".format(lcl))
                continue
            device = cmd.attrib['device']
            if not device in devices:
                perr("Device name '{}' not found in devices list {}".format(device, lcl))
                continue

            # Marker for broadcasts that get replies
            quirks = 1 if 'broadcast' == device and 'quirks' in cmd.attrib and cmd.attrib['quirks'] == '1' else 0

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
            if not 'function' in cmd.attrib:
                perr("Attribute 'function' missing in {}".format(lcl))
                continue
            function = cmd.attrib['function']
            # check depends on unicast or broadcast
            if 'broadcast' == device:
                if not function in WRITEFUNCTIONS:
                    perr("Function '{}' not available from {} for broadcast in {}".format(function, str(list(WRITEFUNCTIONS.keys())), lcl))
                    continue
                function = WRITEFUNCTIONS[function][0]
            else:
                if not function in FUNCTIONS:
                    perr("Function '{}' not defined in {} in {}".format(function, str(list(FUNCTIONS.keys())), lcl))
                    continue
                function = FUNCTIONS[function]

            if function in BITFUNCTIONS:
                # Coils and inputs are binary and always map to HAL_BIT
                if 'type' in cmd.attrib:
                    pintype = cmd.attrib['type']
                    if not pintype in PINTYPES:
                        perr("Invalid type '{}' must be one of {} in {}".format(pintype, str(list(PINTYPES.keys())), lcl))
                        continue
                    # Compounds need function {R,W}_REGISTERS
                    pt = PINTYPES[pintype][0]
                    if pt >= 0x100 and (function != R_REGISTERS and function != W_REGISTERS):
                        perr("Invalid function '{}' for compound type '{}'. Function must be R_REGISTERS({}) or W_REGISTERS({}) in {}"
                                .format(FUNCNAMES[function], TYPENAMES[pt], R_REGISTERS, W_REGISTERS, lcl))
                        continue
                    pwarn("Attribute 'type' ignored for coil/input functions in {}".format(lcl))
                nwords   = PINTYPES['HAL_BIT'][2]
                maxcount = PINTYPES['HAL_BIT'][1]
                pintype  = PINTYPES['HAL_BIT'][0]
            else:
                # Others must specify a type
                if not 'type' in cmd.attrib:
                    perr("Attribute 'type' missing in {}".format(lcl))
                    continue
                pintype = cmd.attrib['type']
                if not pintype in PINTYPES:
                    perr("Invalid type '{}' must be one of {} in {}".format(pintype, str(list(PINTYPES.keys())), lcl))
                    continue
                nwords   = PINTYPES[pintype][2]
                maxcount = PINTYPES[pintype][1]
                pintype  = PINTYPES[pintype][0]
                # Compounds need function {R,W}_REGISTERS
                if pintype >= 0x100 and (function != R_REGISTERS and function != W_REGISTERS):
                    perr("Invalid function '{}' for compound type '{}'. Function must be R_REGISTERS({}) or W_REGISTERS({}) in {}"
                            .format(FUNCNAMES[function], TYPENAMES[pintype], R_REGISTERS, W_REGISTERS, lcl))
                    continue
                if R_INPUTREGS == function and pintype != HAL_U32:
                    perr("Invalid pintype '{}' for function R_INPUTREGS({}) where only 'HAL_U32' allowed in {}"
                            .format(TYPENAMES[pintype], R_INPUTREGS, lcl))
                    continue

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
                if not 'name' in pin.attrib:
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
                pinlist.append("{}.{}".format(device, pinname))

            if err:
                continue

            # How many pins to read/write
            count = checkInt(cmd.attrib, 'count')
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
                if not 'name' in cmd.attrib:
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
                for i in range(len(pinlist), count):
                    pinlist.append("{}.{}-{:02d}".format(device, pinbase, i))

            cmdlist.append({'device': device, 'mbid': devices[device], 'function': function, 'timeout': timeout,
                            'address': address, 'type': pintype, 'count': count, 'pins': pinlist, 'quirks': quirks })
            if verbose:
                print("Command {:2}: {} {}({}) addr=0x{:04x} {} timeout={}"
                    .format(len(cmdlist), device, FUNCNAMES[function], function, address, TYPENAMES[pintype], timeout))
                io = "in" if FUNCNAMES[function] in WRITEFUNCTIONS else "out"
                for p in range(len(pinlist)):
                    print("  pin {:2} ({}): {}".format(p+1, io, pinlist[p]))
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
    #   rtapi_u8  func;    // Function code, 0 for delay | 0x80 for quirks
    #   rtapi_u16 addr;    // Address, 0 in init
    #   rtapi_u16 pincnt;  // Number of pins, 0 in init
    #   rtapi_u16 type;    // Data type, 0 in init
    #   rtapi_u32 timeout; // Response timeout or delay in microseconds
    #   rtapi_u32 dataptr; // Pin names, packet data for init
    # } hm2_modbus_mbccb_cmds_t;
    #
    for i in initlist:
        if 'delay' in i:
            ilb.append(struct.pack(">BBHHHII", 0, 0, 0, 0, 0, i['delay'], 0))
        else:
            func = i['function'] | (0x80 if i['quirks'] == 1 else 0)
            ilb.append(struct.pack(">BBHHHII", i['mbid'], func, i['address'], 0, 0, i['timeout'], dlblen))
            # Precompile the data packet so we only need to copy it
            if W_COIL == i['function'] or W_REGISTER == i['function']:  # Write single coil or single register
                # mbid, func, length address and value == 6
                dlb.append(struct.pack(">BBBHH", 6, i['mbid'], func, i['address'], i['data'][0]))
                dlblen += 5
            elif W_COILS == i['function']:    # Write multiple coils
                nbytes = (len(i['data']) + 7) // 8
                # mbid, func, length address, count and bytecount == 6 + 1
                d = struct.pack(">BBBHHB", 6 + 1 + nbytes, i['mbid'], func, i['address'], len(i['data']), nbytes)
                dlblen += 6
                # Construct a bit string
                val = ''.join(format(bit, '01b') for bit in i['data'])
                # Split string in 8-bit sequences
                parts = [val[x:x+8] for x in range(0, len(val), 8)]
                assert nbytes == len(parts)
                # for all bytes, reverse bit string and convert to integer base 2
                for part in parts:
                    d += struct.pack("B", int(part[::-1], 2))
                    dlblen += 1
                dlb.append(d)
            elif W_REGISTERS == i['function']:    # Write multiple registers
                # mbid, func, length address, count and bytecount == 6 + 1
                d = struct.pack(">BBBHHB", 6 + 1 + len(i['data'])*2, i['mbid'], func, i['address'], len(i['data']), 2*len(i['data']))
                dlblen += 6
                for w in i['data']:
                    d += struct.pack(">H", w)
                    dlblen += 2
                dlb.append(d)
    #
    # typedef struct {
    #   rtapi_u8  mbid;    // Modbus device ID
    #   rtapi_u8  func;    // Function code
    #   rtapi_u16 addr;    // Address
    #   rtapi_u16 pincnt;  // Number of pins
    #   rtapi_u16 type;    // Data type
    #   rtapi_u32 timeout; // Response timeout or delay in microseconds
    #   rtapi_u32 dataptr; // Pin names
    # } hm2_modbus_mbccb_cmds_t;
    #
    for i in commands:
        if 'delay' in i:
            cmb.append(struct.pack(">BBHHHII", 0, 0, 0, 0, 0, 0, i['delay']))
        else:
            func = i['function'] | (0x80 if i['quirks'] == 1 else 0)
            cmb.append(struct.pack(">BBHHHII", i['mbid'], func, i['address'], i['count'], i['type'], i['timeout'], dlblen))
            # Add pin names to data list
            for p in i['pins']:
                asc = p.encode('ascii')
                d = struct.pack(">B", len(p)+1) + asc + struct.pack(">B", 0)
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
                        configparams['parity'] + (configparams['stopbits'] << 2),
                        configparams['txdelay'],
                        configparams['rxdelay'],
                        configparams['drivedelay'],
                        configparams['interval'],
                        0, 0, 0, 0, 0, 0, 0, 0,
                        16*len(ilb), 16*len(cmb), dlblen))

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
