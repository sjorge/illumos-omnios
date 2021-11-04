/*
 * CDDL HEADER START
 *
 * The contents of this file are subject to the terms of the
 * Common Development and Distribution License (the "License").
 * You may not use this file except in compliance with the License.
 *
 * You can obtain a copy of the license at usr/src/OPENSOLARIS.LICENSE
 * or http://www.opensolaris.org/os/licensing.
 * See the License for the specific language governing permissions
 * and limitations under the License.
 *
 * When distributing Covered Code, include this CDDL HEADER in each
 * file and include the License file at usr/src/OPENSOLARIS.LICENSE.
 * If applicable, add the following below this CDDL HEADER, with the
 * fields enclosed by brackets "[]" replaced with your own identifying
 * information: Portions Copyright [yyyy] [name of copyright owner]
 *
 * CDDL HEADER END
 */
/*
 * Copyright 2009 Sun Microsystems, Inc.  All rights reserved.
 * Use is subject to license terms.
 *
 * Copyright 2021 OmniOS Community Edition (OmniOSce) Association.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>

typedef struct definit {
	FILE *di_fp;
	char *di_line;
	size_t di_len;
	char *di_tok;
} definit_t;

int
definit_open(const char *file, void **statep)
{
	FILE *fp;
	definit_t *state;

	if ((fp = fopen(file, "r")) == NULL)
		return (-1);

	if ((state = malloc(sizeof (*state))) == NULL) {
		int _errno = errno;
		(void) fclose(fp);
		errno = _errno;
		return (-1);
	}

	state->di_fp = fp;
	state->di_line = NULL;
	state->di_len = 0;
	state->di_tok = NULL;

	*statep = state;

	return (0);
}

int
definit_close(void *statep)
{
	definit_t *state = statep;

	(void) fclose(state->di_fp);
	free(state->di_line);
	free(state);

	return (0);
}

static char *
definit_nextline(definit_t *state)
{
	ssize_t cnt;

	while ((cnt = getline(&state->di_line, &state->di_len,
	    state->di_fp)) != -1) {
		boolean_t inquotes;
		char *line = state->di_line;
		char *p, *bp;
		size_t wslength;

		/*
		 * Discard newline
		 */
		p = line;
		(void) strsep(&p, "\n");

		/*
		 * Ignore blank or comment lines.
		 */
		if (cnt == 0 || line[0] == '#' || line[0] == '\0' ||
		    (wslength = strspn(line, " \t\n")) == strlen(line) ||
		    strchr(line, '#') == line + wslength) {
			continue;
		}

		/*
		 * First make a pass through the line and remove any
		 * quote characters and change any non-quoted semicolons to
		 * blanks so they will be treated as token separators below.
		 */
		inquotes = B_FALSE;
		for (p = line, bp = NULL; *p != '\0'; p++) {
			switch (*p) {
			case '"':
			case '\'':
				inquotes = !inquotes;
				if (bp == NULL)
					bp = p;
				break;
			case ';':
				if (!inquotes)
					*p = ' ';
				/* FALLTHROUGH */
			default:
				if (bp != NULL)
					*bp++ = *p;
				break;
			}
		}
		if (bp != NULL)
			*bp = '\0';

		/*
		 * Tokens within the line are separated by blanks
		 * and tabs.
		 */
		if ((p = strtok_r(line, " \t", &state->di_tok)) != NULL)
			return (p);
	}

	return (NULL);
}

const char *
definit_token(void *statep)
{
	definit_t *state = statep;
	char *tok;

	for (;;) {
		tok = NULL;

		if (state->di_tok != NULL)
			tok = strtok_r(NULL, " \t", &state->di_tok);

		if (tok == NULL)
			tok = definit_nextline(state);

		if (tok == NULL)
			break;

		if (strchr(tok, '=') != NULL && *tok != '=')
			return (tok);
	}

	return (NULL);
}
