/*
 * This file and its contents are supplied under the terms of the
 * Common Development and Distribution License ("CDDL"), version 1.0.
 * You may only use this file in accordance with the terms of version
 * 1.0 of the CDDL.
 *
 * A full copy of the text of the CDDL should have accompanied this
 * source.  A copy of the CDDL is also available via the Internet at
 * http://www.illumos.org/license/CDDL.
 */

/*
 * usbser_slcp21: Silicon Laboratories CP2101/CP2102/CP2103/CP2104/CP2105 USB-Serial driver.
 */

#include <sys/types.h>
#include <sys/param.h>
#include <sys/stream.h>
#include <sys/conf.h>
#include <sys/ddi.h>
#include <sys/sunddi.h>

#define	USBDRV_MAJOR_VER	2
#define	USBDRV_MINOR_VER	0

#include <sys/usb/usba.h>

#include <sys/usb/clients/usbser/usbser.h>

/* configuration entry points */
static int	usbser_slcp21_getinfo(dev_info_t *, ddi_info_cmd_t, void *,
		void **);
static int	usbser_slcp21_attach(dev_info_t *, ddi_attach_cmd_t);
static int	usbser_slcp21_detach(dev_info_t *, ddi_detach_cmd_t);
static int	usbser_slcp21_open(queue_t *, dev_t *, int, int, cred_t *);

static void    *usbser_slcp21_statep;

extern ds_ops_t slcp21_ds_ops;	/* DSD operations */

/*
 * STREAMS structures
 */
struct module_info usbser_slcp21_modinfo = {
	0,			/* module id */
	"slcp21",		/* module name */
	USBSER_MIN_PKTSZ,	/* min pkt size */
	USBSER_MAX_PKTSZ,	/* max pkt size */
	USBSER_HIWAT,		/* hi watermark */
	USBSER_LOWAT		/* low watermark */
};

static struct qinit usbser_slcp21_rinit = {
	putq,			/* put procedure */
	usbser_rsrv,		/* service procedure */
	usbser_slcp21_open,	/* called on startup */
	usbser_close,		/* called on finish */
	NULL,			/* for future use */
	&usbser_slcp21_modinfo,	/* module information */
	NULL			/* module statistics */
};

static struct qinit usbser_slcp21_winit = {
	usbser_wput,		/* put procedure */
	usbser_wsrv,		/* service procedure */
	NULL,			/* called on startup */
	NULL,			/* called on finish */
	NULL,			/* for future use */
	&usbser_slcp21_modinfo,	/* module information */
	NULL			/* module statistics */
};

static struct streamtab usbser_slcp21_str_info = {
	&usbser_slcp21_rinit,	/* read init procedure */
	&usbser_slcp21_winit,	/* write init procedure */
};


static struct cb_ops usbser_slcp21_cb_ops = {
	nodev,						/* cb_open */
	nodev,						/* cb_close */
	nodev,						/* cb_strategy */
	nodev,						/* cb_print */
	nodev,						/* cb_dump */
	nodev,						/* cb_read */
	nodev,						/* cb_write */
	nodev,						/* cb_ioctl */
	nodev,						/* cb_devmap */
	nodev,						/* cb_mmap */
	nodev,						/* cb_segmap */
	nochpoll,					/* cb_chpoll */
	ddi_prop_op,					/* cb_prop_op */
	&usbser_slcp21_str_info,			/* cb_stream */
	(int)(D_64BIT | D_NEW | D_MP | D_HOTPLUG)	/* cb_flag */
};

/*
 * auto configuration ops
 */
struct dev_ops usbser_slcp21_ops = {
	DEVO_REV,		/* devo_rev */
	0,			/* devo_refcnt */
	usbser_slcp21_getinfo,	/* devo_getinfo */
	nulldev,		/* devo_identify */
	nulldev,		/* devo_probe */
	usbser_slcp21_attach,	/* devo_attach */
	usbser_slcp21_detach,	/* devo_detach */
	nodev,			/* devo_reset */
	&usbser_slcp21_cb_ops,	/* devo_cb_ops */
	(struct bus_ops *)NULL,	/* devo_bus_ops */
	usbser_power,		/* devo_power */
	ddi_quiesce_not_needed,	/* devo_quiesce */
};

static struct modldrv modldrv = {
	&mod_driverops,		/* type of module - driver */
	"Silicon Laboratories CP210x USB-Serial driver",
	&usbser_slcp21_ops,
};

static struct modlinkage modlinkage = {
	MODREV_1,
	&modldrv,
	0
};

/* entry points */
int
_init(void)
{
	int    error;

	if ((error = mod_install(&modlinkage)) != 0)
		return (error);

	if ((error = ddi_soft_state_init(&usbser_slcp21_statep,
	    usbser_soft_state_size(), 1)) != 0)
		(void) mod_remove(&modlinkage);

	return (error);
}

int
_fini(void)
{
	int error;

	if ((error = mod_remove(&modlinkage)) == 0)
		ddi_soft_state_fini(&usbser_slcp21_statep);

	return (error);
}

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&modlinkage, modinfop));
}

/* slcp21 entry points */
int
usbser_slcp21_getinfo(dev_info_t *dip, ddi_info_cmd_t infocmd, void *arg,
		void **result)
{
	return (usbser_getinfo(dip, infocmd, arg, result,
	    usbser_slcp21_statep));
}

static int
usbser_slcp21_attach(dev_info_t *dip, ddi_attach_cmd_t cmd)
{
	return (usbser_attach(dip, cmd, usbser_slcp21_statep, &slcp21_ds_ops));
}

static int
usbser_slcp21_detach(dev_info_t *dip, ddi_detach_cmd_t cmd)
{
	return (usbser_detach(dip, cmd, usbser_slcp21_statep));
}

static int
usbser_slcp21_open(queue_t *rq, dev_t *dev, int flag, int sflag, cred_t *cr)
{
	return (usbser_open(rq, dev, flag, sflag, cr, usbser_slcp21_statep));
}
