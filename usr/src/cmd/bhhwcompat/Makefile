#
# This file and its contents are supplied under the terms of the
# Common Development and Distribution License ("CDDL"), version 1.0.
# You may only use this file in accordance with the terms of version
# 1.0 of the CDDL.
#
# A full copy of the text of the CDDL should have accompanied this
# source.  A copy of the CDDL is also available via the Internet at
# http://www.illumos.org/license/CDDL.
#

#
# Copyright 2021 OmniOS Community Edition (OmniOSce) Association.
#

PROG = bhhwcompat
OBJS = bhhwcompat.o
SRCS = $(OBJS:%.o=%.c)

include ../Makefile.cmd
include ../Makefile.ctf
include ../Makefile.cmd.64

CSTD = $(CSTD_GNU99)

CLEANFILES += $(OBJS)

CPPFLAGS += -I../../uts/common -I../../uts/intel
CPPFLAGS += -I../../uts/intel/io/vmm/intel

.KEEP_STATE:

all: $(OBJS) $(PROG)

install: all $(ROOTUSRSBINPROG)

clean:
	$(RM) $(CLEANFILES)

include ../Makefile.targ
