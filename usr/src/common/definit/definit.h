/*
 * This file and its contents are supplied under the terms of the
 * Common Development and Distribution License ("CDDL"), version 1.0.
 * You may only use this file in accordance with the terms of version
 * 1.0 of the CDDL.
 *
 * A full copy of the text of the CDDL should have accompanied this
 * source. A copy of the CDDL is also available via the Internet at
 * http://www.illumos.org/license/CDDL.
 */

/*
 * Copyright 2021 OmniOS Community Edition (OmniOSce) Association.
 */

#ifndef	_DEFINIT_H
#define	_DEFINIT_H

#ifdef	__cplusplus
extern "C" {
#endif

/*
 * Routines for parsing the default init file, /etc/default/init.
 * Used by init, svc.startd and libzonecfg for setting up a default
 * environment.
 */

#define	DEFINIT_DEFAULT_FILE	"/etc/default/init"

int definit_open(const char *, void **);
int definit_close(void *);
const char *definit_token(void *);

#ifdef	__cplusplus
}
#endif

#endif /* !_DEFINIT_H */
