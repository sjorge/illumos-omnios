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
 * Copyright 2015, 2018, 2019, Meisaka Yukara
 * Copyright 2018, 2019 Prominic.NET Inc. All Rights reserved.
 * Copyright 2021 OmniOS Community Edition (OmniOSce) Association.
 */

/*
 * yuka - datalink gateway and CDP service
 */

#include "yuka.h"
#include <unistd.h>
#include <sys/varargs.h>
#include <fcntl.h>
#include <signal.h>
#include <poll.h>
#include <priv.h>
#include <pthread.h>
#include <err.h>
#include <sys/debug.h>

#include <stropts.h>
#include <sys/conf.h>
#include <getopt.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/ethernet.h>
#include <sys/termios.h>
#include <netdb.h>

#include "dlpi.h"
#include "cdp.h"
#include "cmd.h"
#include "ipc.h"

char const *const yuka_pid_path = "/var/run/cdpd.pid";
char const *const devdir = "/dev/";

/* option flags */
boolean_t rflag = B_FALSE;
boolean_t dflag = B_FALSE;
boolean_t vlanflag = B_FALSE;
boolean_t promisc = B_FALSE;
int verbose = 0;

static yuka_session_t **sessions = NULL;
static char hostname[MAXHOSTNAMELEN];
static sigset_t sigwaitset;

yuka_packet_t *
yuka_packet_alloc(void)
{
	yuka_packet_t *p;

	p = malloc(sizeof (yuka_packet_t) + FRAME_SIZE);
	if (p == NULL)
		return (NULL);
	p->buf = (uint8_t *)p + sizeof (yuka_packet_t);
	p->bufsize = FRAME_SIZE;
	return (p);
}

static void
yuka_packet_free(yuka_packet_t *lf)
{
	free(lf);
}

yuka_session_t *
yuka_session_alloc(void)
{
	yuka_session_t *session;

	session = malloc(sizeof (yuka_session_t));

	if (session == NULL)
		return (NULL);

	memset(session, '\0', sizeof (yuka_session_t));

	return (session);
}

static void
yuka_session_free(yuka_session_t *ses)
{
	DlpiCloseSession(ses);
	free(ses->device);
	yuka_packet_free(ses->cdpinfo);
	free(ses);
}

void
add_session_device(yuka_session_t *ses, const char *device)
{
	yuka_packet_t *cdpi = NULL;

	ses->device = strdup(device);
	DlpiOpenSession(ses);

	cdpi = yuka_packet_alloc();
	yuka_cdp_init(ses, cdpi);
	cdp_add_hostid(cdpi, hostname);
	cdp_add_ipaddress(cdpi, (in_addr_t)0);
	cdp_add_portid(cdpi, device);
	cdp_add_capabilities(cdpi, CDPCAP_HOST | CDPCAP_ROUTER | CDPCAP_SWITCH);
	cdp_add_sysname(cdpi);
	yuka_cdp_final(cdpi);

	ses->cdpinfo = cdpi;
}

int
mac_equal(const void *ma, const void *mb)
{
	const uint8_t *a = (uint8_t const *)ma;
	const uint8_t *b = (uint8_t const *)mb;
	return (*((const uint32_t *)a) == *((const uint32_t *)b)) &&
	    (((const uint16_t *)a)[2] == ((const uint16_t *)b)[2]);
}

static void
yuka_packet_reset(yuka_packet_t *lf)
{
	lf->insertpos = 0;
	lf->framelen = 0;
	lf->framepos = 0;
	lf->llcstart = 14;
	lf->llclen = 0;
}

uint16_t
checksum(const void *ptr, int length)
{
	uint32_t sum = 0;
	const uint16_t *w = (const uint16_t *)ptr;
	const uint16_t *en = w + (length >> 1);

	while (w < en)
		sum += ntohs(*w++);
	if (length & 1)
		sum += *w & 0x00ff;

	sum = (sum >> 16) + (sum & 0xFFFF);
	sum += (sum >> 16);
	return (htons(~sum));
}

void
add_long(yuka_packet_t *lfp, uint32_t v)
{
	if (lfp->insertpos + 4 > lfp->bufsize)
		return;
	*((uint32_t *)&lfp->buf[lfp->insertpos]) = htonl(v);
	lfp->insertpos += 4;
	if (lfp->insertpos > lfp->framelen) lfp->framelen = lfp->insertpos;
}

uint32_t
get_long(const uint8_t *rfp)
{
	uint32_t r = (uint32_t)ntohl(*(uint32_t *)(rfp));
	return (r);
}

void
add_ip(yuka_packet_t *lfp, in_addr_t v)
{
	if (lfp->insertpos + 4 > lfp->bufsize)
		return;
	*((in_addr_t *)&lfp->buf[lfp->insertpos]) = v;
	lfp->insertpos += 4;
	if (lfp->insertpos > lfp->framelen)
		lfp->framelen = lfp->insertpos;
}

void
add_short(yuka_packet_t *lfp, uint16_t v)
{
	if (lfp->insertpos + 2 > lfp->bufsize)
		return;
	*((unsigned short *)&lfp->buf[lfp->insertpos]) = htons(v);
	lfp->insertpos += 2;
	if (lfp->insertpos > lfp->framelen)
		lfp->framelen = lfp->insertpos;
}

uint16_t
get_short(const uint8_t *rfp)
{
	uint16_t r = (uint16_t)ntohs(*(uint16_t *)(rfp));
	return (r);
}

uint16_t
get_short_m(const uint8_t **rfp)
{
	uint16_t r = (uint16_t)ntohs(*(uint16_t *)(*rfp));
	(*rfp) += 2;
	return (r);
}

void
add_byte(yuka_packet_t *lfp, uint8_t v)
{
	if (lfp->insertpos + 1 > lfp->bufsize)
		return;
	lfp->buf[lfp->insertpos++] = v;
	if (lfp->insertpos > lfp->framelen)
		lfp->framelen = lfp->insertpos;
}

void
add_string(yuka_packet_t *lfp, const uint8_t *v, int len)
{
	int i;

	if (lfp->insertpos + len > lfp->bufsize)
		return;

	for (i = 0; i < len; i++)
		lfp->buf[lfp->insertpos++] = v[i];

	if (lfp->insertpos > lfp->framelen)
		lfp->framelen = lfp->insertpos;
}

static void *
yuka_xmit(void *arg)
{
	yuka_session_t **sessionlist;
	int i;

	(void) pthread_setname_np(pthread_self(), "xmit");

	sessionlist = (yuka_session_t **)arg;

	for (;;) {
		struct pollfd pollfds[1];

		for (i = 0; sessionlist[i] != NULL; i++) {
			yuka_session_t *s = sessionlist[i];

			if (verbose)
				printf("xmit on %s\n", s->device);

			pollfds[0].fd = s->fd;
			pollfds[0].events = POLLOUT;

			if (poll(pollfds, 1, 1000) <= 0 ||
			    (pollfds[0].revents & POLLOUT) == 0) {
				if (verbose)
					printf("xmit on %s not ready\n",
					    s->device);
				continue;
			}

			if (dflag) {
				printf("SENDING %d bytes\n",
				    s->cdpinfo->framelen);
				xdump(s->cdpinfo->buf, s->cdpinfo->framelen);
				printf("-------------------\n");
			}
			DlpiSnd(s, MAC_CDP,
			    s->cdpinfo->buf, s->cdpinfo->framelen);
		}
		(void) sleep(CDP_XMIT_INTERVAL);
	}

	/* NOTREACHED */
	return (NULL);
}

struct show_cdp_settings {
	int fd;
	int fmt;
	int index;
};

static char *
cdp_capstring(neighbour_t *n)
{
	static char cflags[9];
	char *cfp = cflags;

	if (n->caps & CDPCAP_ROUTER)
		*cfp++ = 'R';
	if (n->caps & CDPCAP_TRBRIDGE)
		*cfp++ = 'T';
	if (n->caps & CDPCAP_SRBRIDGE)
		*cfp++ = 'B';
	if (n->caps & CDPCAP_SWITCH)
		*cfp++ = 'S';
	if (n->caps & CDPCAP_IGMP)
		*cfp++ = 'I';
	if (n->caps & CDPCAP_HOST)
		*cfp++ = 'H';
	if (n->caps & CDPCAP_REPEATER)
		*cfp++ = 'r';
	if (n->caps & CDPCAP_PHONE)
		*cfp++ = 'P';
	*cfp = '\0';

	return (cflags);
}

static void
show_cdp_host_V(neighbour_t *he, void *n)
{
	struct show_cdp_settings *s = (struct show_cdp_settings *)n;
	yuka_session_t const *lses = (yuka_session_t const *)he->localportid;
	const char *plat = he->platform;
	const char *portid = he->portid;
	char buf[0x400];
	char portprefix[5];
	int portw = -13;

	if (s->fmt == YUKA_FMT_TEXT && strncmp(plat, "cisco ", 6) == 0)
		plat += 6;

	/* build the port ID field */
	*portprefix = '\0';
	if (s->fmt == YUKA_FMT_TEXT) {
		if (strncmp(he->portid, "Ethernet", 8) == 0) {
			(void) strcpy(portprefix, "Eth ");
			portid += 8;
			portw = -9;
		} else if (strncmp(he->portid, "FastEthernet", 12) == 0) {
			(void) strcpy(portprefix, "Fas ");
			portid += 12;
			portw = -9;
		} else if (strncmp(he->portid, "GigabitEthernet", 15) == 0) {
			(void) strcpy(portprefix, "Gig ");
			portid += 15;
			portw = -9;
		} else if (strncmp(he->portid, "TenGigabitEthernet", 18) == 0) {
			(void) strcpy(portprefix, "Ten ");
			portid += 18;
			portw = -9;
		}
	}
	const char *fmtstr = "%s%s;%s;%d;%s;%s;%s%*s;%d\n";
	const char *objprefix = "";
	switch (s->fmt) {
	case YUKA_FMT_TEXT:
		fmtstr = strlen(he->devid) > 18 ?
		    "%s%s\n                   "
		    "%-10s %4d  %-8s %-17s %s%*s %3d\n" :
		    "%s%-18s %-10s %4d  %-8s %-17s %s%*s %3d\n";
		break;
	case YUKA_FMT_JSON:
		if (s->index > 0)
			objprefix = ",";
		fmtstr = "%s{\"device\":\"%s\",\"local_port\":\"%s\","
		    "\"holdtime\":%d,\"capability\":\"%s\","
		    "\"platform\":\"%s\",\"remote_port\":\"%s%*s\","
		    "\"native_vlan\":%d}\n";
		break;
	case YUKA_FMT_XML:
		fmtstr = "%1$s <device holdtime=\"%4$d\">\n"
		    "  <name>%2$s</name>\n"
		    "  <local_port>%3$s</local_port>\n"
		    "  <capability>%5$s</capability>\n"
		    "  <platform>%6$s</platform>\n"
		    "  <remote_port>%7$s%9$*8$s</remote_port>\n"
		    "  <native_vlan>%10$d</native_vlan>\n"
		    "</device>\n";
		break;
	}
	int r = snprintf(buf, sizeof (buf), fmtstr,
	    objprefix,
	    he->devid,
	    lses->device,
	    he->holdtime > 0 ? he->holdtime : -he->deadtime,
	    he->holdtime > 0 ? cdp_capstring(he) : "<DEAD>",
	    plat,
	    portprefix,
	    portw, portid,
	    he->vlan);
	(void) write(s->fd, buf, r);
	s->index++;
}

void
yuka_show_cdp_hosts(int outfd, int fflag)
{
	struct show_cdp_settings outset;

	outset.fd = outfd; /* where to output */
	outset.fmt = fflag; /* show formatted */
	outset.index = 0; /* keep track of how many */

	if (verbose > 3)
		printf("show cdp hosts: FD=%d, fmt=%d\n", outfd, fflag);

	if (outset.fmt == YUKA_FMT_TEXT) {
		const char *info =
		    "Device ID          Local Int  Hold  Capable  Platform"
		    "          Port ID      VLAN\n";
		(void) write(outset.fd, info, strlen(info));
	} else if (outset.fmt == YUKA_FMT_JSON) {
		(void) write(outset.fd, "[\n", 2);
	} else if (outset.fmt == YUKA_FMT_XML) {
		(void) write(outset.fd, "<cdpneighbors>\n", 16);
	}

	yuka_cdp_walk(show_cdp_host_V, &outset, B_TRUE);

	if (outset.fmt == YUKA_FMT_JSON) {
		(void) write(outset.fd, "]\n", 2);
	} else if (outset.fmt == YUKA_FMT_XML) {
		(void) write(outset.fd, "</cdpneighbors>\n", 17);
	}
}

static void
show_cdp_host_detail(neighbour_t *n, void *arg)
{
	yuka_session_t const *ses = (yuka_session_t const *)n->localportid;
	FILE *fp = (FILE *)arg;

	(void) fprintf(fp, "--\n");
	(void) fprintf(fp, "---------------------- %s ----------------------\n",
	    n->devid);
	(void) fprintf(fp, "--\n");
	(void) fprintf(fp, "Platform:        %s\n", n->platform);
	(void) fprintf(fp, "Version:\n\n%s\n\n", n->swversion);
	(void) fprintf(fp, "Capabilities:    %s\n", cdp_capstring(n));
	(void) fprintf(fp, "VTP Domain:      %s\n", n->vtpdomain);
	(void) fprintf(fp, "\n");
	(void) fprintf(fp, "Interface:       %s\n", n->portid);
	(void) fprintf(fp, "Local Interface: %s\n", ses->device);
	(void) fprintf(fp, "VLAN:            %u\n", n->vlan);
	(void) fprintf(fp, "Duplex:          %u\n", n->duplex);
	(void) fprintf(fp, "\n");
	(void) fprintf(fp, "CDP Version:     %u\n", n->version);
	(void) fprintf(fp, "Hold Time:       %u\n", n->holdtime);
	(void) fprintf(fp, "Dead Time:       %u\n", n->deadtime);
}

void
yuka_show_detail(int fd)
{
	FILE *fp;

	if ((fp = fdopen(fd, "w")) == NULL) {
		(void) write(fd, "fdopen failed.\n", 15);
		return;
	}

	yuka_cdp_walk(show_cdp_host_detail, (void *)fp, B_TRUE);

	(void) fclose(fp);
}

void
yuka_stats(int fd)
{
	FILE *fp;
	int i;

	if ((fp = fdopen(fd, "w")) == NULL) {
		(void) write(fd, "fdopen failed.\n", 15);
		return;
	}

	(void) fprintf(fp, "%-15s %10s %10s %10s %10s\n",
	    "INTERFACE", "FRAMES OUT", "FRAMES IN", "BYTES OUT", "BYTES IN");
	(void) fprintf(fp, "%-15s %10s %10s %10s %10s\n", "---------",
	    "----------", "----------", "----------", "----------");
	for (i = 0; sessions[i] != NULL; i++) {
		yuka_session_t *s = sessions[i];

		(void) fprintf(fp, "%-15s %10u %10u %10u %10u\n", s->device,
		    s->frames_out, s->frames_in,
		    s->bytes_out, s->bytes_in);
	}

	(void) fclose(fp);
}

static void
update_hold_V(neighbour_t *he, void *n __unused)
{
	if (he->holdtime > 0)
		he->holdtime--;
	else
		he->deadtime++;
}

static void *
yuka_update_hold(void *arg __unused)
{
	(void) pthread_setname_np(pthread_self(), "holdtime");

	for (;;) {
		yuka_cdp_walk(update_hold_V, NULL, B_TRUE);
		(void) sleep(1);
	}
	/* NOTREACHED */
	return (NULL);
}

static void *
yuka_reaper(void *arg __unused)
{
	(void) pthread_setname_np(pthread_self(), "reaper");

	for (;;) {
		yuka_cdp_reap(CDP_DEAD_TIMER);
		(void) sleep(CDP_REAP_INTERVAL);
	}
	/* NOTREACHED */
	return (NULL);
}

void
ascii_out(const unsigned char *p, int n)
{
	int i;

	for (i = (n & 0xf) + 1; i < 16; ++i) {
		if (i == 8)
			(void) putchar(' ');
		printf("   ");
	}
	printf(" ");
	for (i = n & ~0xf; i <= n; ++i) {
		if (!(i & 0x7))
			(void) putchar(' ');
		if (isprint(p[i]))
			(void) putchar(p[i]);
		else
			(void) putchar('.');
	}
	(void) putchar('\n');
}

void
xdump(const unsigned char *p, int l)
{
	int n;

	for (n = 0; n < l; ++n) {
		if (!(n & 0xf))
			printf("    %04x: ", n);
		if (!(n & 0x7))
			(void) putchar(' ');
		printf("%02x ", p[n]);
		if ((n & 0xf) == 0xf)
			ascii_out(p, n);
	}

	if (n & 0xf)
		ascii_out(p, n-1);
}

static void
ether_header_dump(const struct ether_header *p)
{
	int t;

	nbytes_hex("    ether_dhost ",
	    p->ether_dhost.ether_addr_octet, 6, "\n");
	nbytes_hex("    ether_shost ",
	    p->ether_shost.ether_addr_octet, 6, "\n");
	t = ntohs(p->ether_type);
	if (t <= 1500)
		printf("           size 0x%x (%d)\n", t, t);
	else
		printf("     ether_type 0x%x = %s\n", t, get_ether_type(t));
}

const char *
get_llc_sap(uchar_t c)
{
	switch (c & 0xfe) {
	case 0:		return "Null";
	case 2:		return "iLLC Sublayer";
	case 3:		return "gLLC Sublayer";
	case 6:		return "DoD IP";
	case 24:	return "TI";
	case 66:	return "BSTP";
	case 78:	return "EIA-RS 511";
	case 94:	return "ISI IP";
	case 129:	return "BACnet/IP";
	case 152:	return "ARP";
	case 170:	return "SNAP Ext";
	case 224:	return "Netware";
	case 240:	return "NetBIOS";
	case 244:	return "LANMan Ind";
	case 245:	return "LANMan Grp";
	case 248:	return "RPL";
	case 255:	return "Global DSAP";
	default:	return "???";
	}
}

static void
dump_llc_packet(const uchar_t *p, int l)
{
	printf("  -- LLC --\n");
	if (l < 3)
		return;
	printf(" dest-sap: 0x%02x (%s)\n", p[0], get_llc_sap(p[0]));
	printf("  src-sap: 0x%02x (%s)\n", p[1], get_llc_sap(p[1]));
}

static void
dump_ether_packet(const struct ether_header *p, int l)
{
	int t;

	printf("  -- ETHER --\n");

	if (l < sizeof (struct ether_header))
		return;

	ether_header_dump(p);
	t = ntohs(p->ether_type);
	l -= sizeof (struct ether_header);
	if (t == ETHERTYPE_VLAN) {
		const struct ether_vlan_extinfo *vl;

		if (l < sizeof (struct ether_vlan_extinfo))
			return;

		vl = (const struct ether_vlan_extinfo *)(p + 1);
		p = (const struct ether_header *)(vl + 1);

		uint16_t vid = ntohs(vl->ether_tci);
		printf("    vlan: 0x%04x (%d)\n", vid & 0xfff, vid & 0xfff);
		printf("    priority: %d\n", vid >> 12);
		t = ntohs(vl->ether_type);
		if (t <= 1500)
			printf("          size 0x%x (%d)\n", t, t);
		else
			printf("    ether_type 0x%x = %s\n",
			    t, get_ether_type(t));
	}
	if (t < 1501)
		dump_llc_packet((uchar_t *)(p + 1), l);
}

void
nbytes_hex(const char *s1, const uchar_t *b, int n, const char *s2)
{
	int k;

	if (s1)
		printf("%s", s1);
	printf("%02x", b[0]);
	for (k = 1; k < n; ++k)
		printf(":%02x", b[k]);
	if (s2)
		printf("%s", s2);
}

void
Warn(char *str, ...)
{
	va_list ap;
	int e;

	e = errno;

	va_start(ap, str);
	(void) vfprintf(stderr, str, ap);
	va_end(ap);
	fprintf(stderr, " - platform warning\n");

	if (e)
		fprintf(stderr, "system error %d: %s\n", e, strerror(e));
}

void
Error(char *str, ...)
{
	va_list ap;
	int e;

	e = errno;

	va_start(ap, str);
	(void) vfprintf(stderr, str, ap);
	va_end(ap);
	fprintf(stderr, " - platform error, exiting\n");

	if (e)
		fprintf(stderr, "system error %d: %s\n", e, strerror(e));

	exit(1);
}

static void
yuka_parse_frame(yuka_session_t *ses, yuka_packet_t *fr,
    dlsap_addr_t src, dlsap_addr_t dst)
{
	int t;
	const struct ether_header *p =
	    (const struct ether_header *)(fr->buf + fr->framepos);

	if (fr->framelen < sizeof (struct ether_header))
		return;

	if (mac_equal(&p->ether_shost, ses->physaddr))
		return;

	if ((!dflag || verbose < 3) &&
	    !mac_equal(p->ether_dhost.ether_addr_octet, &MAC_CDP))
		return;

	if (dflag) {
		printf("--------------------------------------------\n");
		printf("received:");
		nbytes_hex(" src ", src, 6, NULL);
		nbytes_hex(" dest ", dst, 6, NULL);
		printf(" data-len %d\n", fr->framelen);
		xdump(fr->buf, fr->framelen);
		dump_ether_packet(p, fr->framelen);
	}

	fr->framepos += sizeof (struct ether_header);
	t = ntohs(p->ether_type);

	if (t == ETHERTYPE_VLAN) {
		const struct ether_vlan_extinfo *vl;

		if (fr->framelen - fr->framepos <
		    sizeof (struct ether_vlan_extinfo)) {
			if (verbose > 2)
				printf("VLAN frame length is too short: %d\n",
				    fr->framelen);
			return;
		}

		vl = (const struct ether_vlan_extinfo *)(p + 1);
		fr->llclen -= sizeof (*vl);
		fr->framepos += sizeof (*vl);
		t = ntohs(vl->ether_type);
	}

	if (t < 1501 && mac_equal(p->ether_dhost.ether_addr_octet, &MAC_CDP)) {
		neighbour_t cdphost;

		memset(&cdphost, '\0', sizeof (cdphost));

		if (!yuka_cdp_parse(fr->buf + fr->framepos,
		    fr->framelen - fr->framepos, &cdphost)) {
			cdphost.localportid = ses;
			yuka_cdp_refresh(&cdphost);
		}
	}
}

int
yuka_rcv_data(yuka_session_t *ses)
{
	yuka_packet_t eth;
	dlsap_addr_t src, dst;

	eth.buf = ses->buf;
	eth.bufsize = ses->mtu;

	yuka_packet_reset(&eth);

	if (!DlpiRcv(ses, &eth, src, dst)) {
		if (verbose)
			printf("DlpiRcv failure\n");
		return (0);
	}

	yuka_parse_frame(ses, &eth, src, dst);

	return (dst[0] & 1);
}

void
Usage(const char *prog)
{
	fprintf(stderr,
	    "usage:\n"
	    "# %1$s -r [options] [<device> ...]\n"
	    "        -- run cdp on all interfaces or specific interfaces\n"
	    "$ %1$s -l\n"
	    "        -- list available interfaces\n"
	    " Options:\n"
	    "   -R             same as '-r' but run in the background.\n"
	    "   -p             enable promiscuous mode.\n"
	    "   -v             verbose output. "
	    "(repeatable for more verbosity)\n"
	    "   -V             process VLAN-tagged frames.\n"
	    "   -d             dump frames.\n",
	    prog);

	exit(1);
}

/*
 * Construct the set of signals that we explicitly want to deal with.
 * We block these while we're still single-threaded; this block will
 * be inherited by all the threads we create. When we are ready to
 * start handling signals, we will start the signal handling thread,
 * which will sigwait() this same set of signals, and will thus receive
 * and handle any that are sent to the process.
 */
static void
block_signals(void)
{
	(void) sigemptyset(&sigwaitset);
	(void) sigaddset(&sigwaitset, SIGHUP);
	(void) sigaddset(&sigwaitset, SIGUSR1);
	(void) sigaddset(&sigwaitset, SIGPIPE);
	(void) pthread_sigmask(SIG_BLOCK, &sigwaitset, NULL);
}

static void *
sighandler(void *arg __unused)
{
	int sig;

	(void) pthread_setname_np(pthread_self(), "sigman");

	for (;;) {
		sig = sigwait(&sigwaitset);

		if (verbose)
			printf("signal %s caught", strsignal(sig));

		switch (sig) {
		case SIGHUP:
			rflag = B_FALSE;
			break;
		case SIGUSR1:
			yuka_show_cdp_hosts(1, YUKA_FMT_TEXT);
			break;
		case SIGPIPE:
			break;
		default:
			printf("unexpected signal %s received, ignoring",
			    strsignal(sig));
			break;
		}
	}
	return (NULL);
}

static void
enter_service(void)
{
	char buf[32];
	pid_t pid;
	int fd, n;

	verbose = 0;

	switch (fork()) {
	case -1:
		perror("fork");
		exit(1);
		break;
	case 0:		/* child */
		break;
	default:	/* parent */
		exit(0);
	}

	(void) ioctl(fileno(stdin), TIOCNOTTY);
	(void) freopen("/dev/null", "r", stdin);

	fd = open("/dev/null", O_WRONLY, 0666);
	(void) dup2(fd, fileno(stdout));
	(void) dup2(fd, fileno(stderr));
	(void) close(fd);

	(void) setsid();

	pid = getpid();
	n = snprintf(buf, sizeof (buf), "%d", pid);
	if (n > sizeof (buf))
		n = sizeof (buf);

	(void) unlink(yuka_pid_path);
	fd = open(yuka_pid_path, O_WRONLY | O_CREAT, 0644);
	if (fd >= 0) {
		(void) write(fd, buf, n);
		(void) close(fd);
	}
}

static const struct option lopts[] = {
	{"debug",	no_argument,	NULL, 'd'},
	{"verbose",	no_argument,	NULL, 'v'},
	{"list",	no_argument,	NULL, 'l'},
	{"vlan",	no_argument,	NULL, 'V'},
	{"promisc",	no_argument,	NULL, 'p'}
};

int
main(int argc, char **argv)
{
	struct pollfd *pollfds = NULL;
	pthread_attr_t attr;
	pthread_t tid;
	boolean_t bgflag = B_FALSE;
	int sessioncount, i, x;
	int ret = 0;
	char c;

	setbuf(stdout, NULL);
	setbuf(stderr, NULL);

	while ((c = getopt_long(argc, argv, ":RrdvhlVp", lopts, NULL)) != EOF) {
		switch (c) {
		case 'R':
			bgflag = B_TRUE;
			/* FALLTHROUGH */
		case 'r':
			rflag = B_TRUE;
			break;
		case 'd':
			dflag = B_TRUE;
			break;
		case 'v':
			(void) setpflags(PRIV_DEBUG, 1);
			verbose++;
			break;
		case 'V':
			vlanflag = B_TRUE;
			break;
		case 'p':
			promisc = B_TRUE;
			break;
		case 'l':
			yuka_list_links();
			return (0);
		case '?':
		case 'h':
		default:
			Usage(argv[0]);
		}
	}

	if (!rflag)
		Usage(argv[0]);

	if (gethostname(hostname, MAXHOSTNAMELEN) != 0) {
		perror("gethostname");
		return (0);
	}

	sessioncount = argc - optind;

	if (verbose > 1)
		printf("verbosity: %d\n", verbose);
	if (verbose > 2)
		printf("Sessions: %d\n", sessioncount);

	if (sessioncount > 0) {
		sessions = malloc(
		    sizeof (yuka_session_t *) * (sessioncount + 1));

		if (sessions == NULL)
			err(EXIT_FAILURE, "malloc sessions");

		for (x = 0, i = optind; i < argc; i++, x++) {
			if (verbose > 1)
				printf("Allocating session for %s\n", argv[i]);
			sessions[x] = yuka_session_alloc();
			add_session_device(sessions[x],
			    strncmp(argv[i], devdir, 5) == 0 ? argv[i] + 5 :
			    argv[i]);
		}
	} else {
		stringlist_t *lst = NULL, *lsti;

		yuka_get_links(&lst);

		if (lst == NULL)
			errx(EXIT_FAILURE, "No links found - exiting");

		for (lsti = lst; lsti != NULL; lsti = lsti->next)
			sessioncount++;

		sessions = malloc(
		    sizeof (yuka_session_t *) * (sessioncount + 1));

		if (sessions == NULL)
			err(EXIT_FAILURE, "malloc sessions");

		for (x = 0, lsti = lst; lsti != NULL; lsti = lsti->next, x++) {
			sessions[x] = yuka_session_alloc();
			add_session_device(sessions[x], lsti->str);
		}
		yuka_free_links(lst);
	}

	sessions[sessioncount] = NULL;

	if (bgflag)
		enter_service();

	/* Start threads */

	block_signals();

	if (init_ipc() == 0)
		err(EXIT_FAILURE, "failed to initialise IPC");

	if (pthread_attr_init(&attr) != 0)
		err(EXIT_FAILURE, "pthread_attr_init");

	if (pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED) != 0)
		err(EXIT_FAILURE, "pthread_attr_setdetachstate");

	if (pthread_create(&tid, &attr, yuka_update_hold, NULL) != 0)
		err(EXIT_FAILURE, "pthread_create yuka_update_hold");

	if (pthread_create(&tid, &attr, yuka_reaper, NULL) != 0)
		err(EXIT_FAILURE, "pthread_create yuka_reaper");

	if (pthread_create(&tid, &attr, yuka_xmit, (void *)sessions) != 0)
		err(EXIT_FAILURE, "pthread_create yuka_xmit");

	if (pthread_create(&tid, &attr, sighandler, NULL) != 0)
		err(EXIT_FAILURE, "pthread_create sighandler");

	(void) pthread_attr_destroy(&attr);

	/* Drop privileges now we're up and running */
	priv_set_t *ps, *psinit;

	if ((ps = priv_allocset()) == NULL)
		err(EXIT_FAILURE, "priv_allocset()");
	if ((psinit = priv_allocset()) == NULL)
		err(EXIT_FAILURE, "priv_allocset()");
	if (getppriv(PRIV_EFFECTIVE, psinit) != 0)
		err(EXIT_FAILURE, "failed to fetch current privileges");
	priv_basicset(ps);
	priv_intersect(psinit, ps);
	VERIFY0(priv_delset(ps, PRIV_FILE_LINK_ANY));
	VERIFY0(priv_delset(ps, PRIV_NET_ACCESS));
	VERIFY0(priv_delset(ps, PRIV_PROC_EXEC));
	VERIFY0(priv_delset(ps, PRIV_PROC_FORK));
	VERIFY0(priv_delset(ps, PRIV_PROC_INFO));
	VERIFY0(priv_delset(ps, PRIV_PROC_SECFLAGS));
	VERIFY0(priv_delset(ps, PRIV_PROC_SESSION));
	if (setppriv(PRIV_SET, PRIV_PERMITTED, ps) != 0)
		err(EXIT_FAILURE, "setppriv(PERMITTED)");
	if (setppriv(PRIV_SET, PRIV_LIMIT, ps) != 0)
		err(EXIT_FAILURE, "setppriv(LIMIT)");

	priv_freeset(ps);
	priv_freeset(psinit);

	pollfds = malloc(sizeof (struct pollfd) * sessioncount);
	if (pollfds == NULL)
		err(EXIT_FAILURE, "malloc pollfds");

	for (i = 0; i < sessioncount; i++) {
		pollfds[i].fd = sessions[i]->fd;
		pollfds[i].events = POLLIN;
	}

	int total_frames = 0;

	while (rflag) {
		int n, foflag = 0;

		n = poll(pollfds, sessioncount, -1);

		if (n == 0) {
			/* No data ready */
			continue;
		} else if (n == -1) {
			/*
			 * If the error came from a signal, the signal handler
			 * will set rflag to false.
			 */
			perror("poll");
			continue;
		}

		for (i = 0; i < sessioncount; i++) {
			if (pollfds[i].revents & POLLIN) {
				foflag |= yuka_rcv_data(sessions[i]);
				total_frames++;
			}
		}
		if (verbose > 1)
			printf("\r% 10d recv.", total_frames);
	}

	if (sessions != NULL) {
		for (x = 0; x < sessioncount; x++) {
			DlpiCloseSession(sessions[x]);
			yuka_session_free(sessions[x]);
		}
		free(sessions);
	}

	free(pollfds);

	return (ret);
}
