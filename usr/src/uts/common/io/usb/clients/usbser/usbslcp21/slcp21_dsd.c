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
 * DSD code for Silicon Laboratories CP2101/CP2102/CP2103/CP2104/CP2105 USB-Serial adaptors.
 */

#include <sys/types.h>
#include <sys/param.h>
#include <sys/conf.h>
#include <sys/stream.h>
#include <sys/strsun.h>
#include <sys/termio.h>
#include <sys/termiox.h>
#include <sys/ddi.h>
#include <sys/sunddi.h>

#define	USBDRV_MAJOR_VER	2
#define	USBDRV_MINOR_VER	0

#include <sys/usb/usba.h>
#include <sys/usb/usba/usba_types.h>
#include <sys/usb/usba/usba_impl.h>

#include <sys/usb/clients/usbser/usbser_dsdi.h>

#include <sys/usb/usbdevs.h>

#include <sys/cmn_err.h>

/*
 * DSD operations which are filled in ds_ops structure.
 */
static int	slcp21_attach(ds_attach_info_t *);
static void	slcp21_detach(ds_hdl_t);
static int	slcp21_register_cb(ds_hdl_t, uint_t, ds_cb_t *);
static void	slcp21_unregister_cb(ds_hdl_t, uint_t);
static int	slcp21_open_port(ds_hdl_t, uint_t);
static int	slcp21_close_port(ds_hdl_t, uint_t);

/* power management */
static int	slcp21_usb_power(ds_hdl_t, int, int, int *);
static int	slcp21_suspend(ds_hdl_t);
static int	slcp21_resume(ds_hdl_t);

/* hotplug */
static int	slcp21_disconnect(ds_hdl_t);
static int	slcp21_reconnect(ds_hdl_t);

/* standard UART operations */
static int	slcp21_set_port_params(ds_hdl_t, uint_t, ds_port_params_t *);
static int	slcp21_set_modem_ctl(ds_hdl_t, uint_t, int, int);
static int	slcp21_get_modem_ctl(ds_hdl_t, uint_t, int, int *);
static int	slcp21_break_ctl(ds_hdl_t, uint_t, int);
static int	slcp21_loopback(ds_hdl_t, uint_t, int);

/* data xfer */
static int	slcp21_tx(ds_hdl_t, uint_t, mblk_t *);
static mblk_t	*slcp21_rx(ds_hdl_t, uint_t);
static void	slcp21_stop(ds_hdl_t, uint_t, int);
static void	slcp21_start(ds_hdl_t, uint_t, int);
static int	slcp21_fifo_flush(ds_hdl_t, uint_t, int);
static int	slcp21_fifo_drain(ds_hdl_t, uint_t, int);

/*
 * DSD ops structure
 */
ds_ops_t slcp21_ds_ops = {
	DS_OPS_VERSION,
	slcp21_attach,
	slcp21_detach,
	slcp21_register_cb,
	slcp21_unregister_cb,
	slcp21_open_port,
	slcp21_close_port,
	slcp21_usb_power,
	slcp21_suspend,
	slcp21_resume,
	slcp21_disconnect,
	slcp21_reconnect,
	slcp21_set_port_params,
	slcp21_set_modem_ctl,
	slcp21_get_modem_ctl,
	slcp21_break_ctl,
	slcp21_loopback,
	slcp21_tx,
	slcp21_rx,
	slcp21_stop,
	slcp21_start,
	slcp21_fifo_flush,
	slcp21_fifo_drain
};

static int
slcp21_attach(ds_attach_info_t *aip)
{
	cmn_err(CE_WARN, "usbser_slcp21: attach");
	return (USB_FAILURE);
}

static void
slcp21_detach(ds_hdl_t hdl)
{
	cmn_err(CE_WARN, "usbser_slcp21: detach");
	return;
}

static int
slcp21_register_cb(ds_hdl_t hdl, uint_t port_num, ds_cb_t *cb)
{
	cmn_err(CE_WARN, "usbser_slcp21: register_cb");
	return (USB_FAILURE);
}

static void
slcp21_unregister_cb(ds_hdl_t hdl, uint_t port_num)
{
	cmn_err(CE_WARN, "usbser_slcp21: unregister_cb");
	return;
}

static int
slcp21_open_port(ds_hdl_t hdl, uint_t port_num)
{
	cmn_err(CE_WARN, "usbser_slcp21: open_port");
	return (USB_FAILURE);
}

static int
slcp21_close_port(ds_hdl_t hdl, uint_t port_num)
{
	cmn_err(CE_WARN, "usbser_slcp21: close_port");
	return (USB_FAILURE);
}

static int
slcp21_usb_power(ds_hdl_t hdl, int comp, int level, int *new_state)
{
	cmn_err(CE_WARN, "usbser_slcp21: usb_power");
	return (USB_FAILURE);
}

static int
slcp21_suspend(ds_hdl_t hdl)
{
	cmn_err(CE_WARN, "usbser_slcp21: suspend");
	return (USB_FAILURE);
}

static int
slcp21_resume(ds_hdl_t hdl)
{
	cmn_err(CE_WARN, "usbser_slcp21: resume");
	return (USB_FAILURE);
}

static int
slcp21_disconnect(ds_hdl_t hdl)
{
	cmn_err(CE_WARN, "usbser_slcp21: disconnect");
	return (USB_FAILURE);
}

static int
slcp21_reconnect(ds_hdl_t hdl)
{
	cmn_err(CE_WARN, "usbser_slcp21: reconnect");
	return (USB_FAILURE);
}

static int
slcp21_set_port_params(ds_hdl_t hdl, uint_t port_num, ds_port_params_t *tp)
{
	cmn_err(CE_WARN, "usbser_slcp21: set_port_params");
	return (USB_FAILURE);
}

static int
slcp21_set_modem_ctl(ds_hdl_t hdl, uint_t port_num, int mask, int val)
{
	cmn_err(CE_WARN, "usbser_slcp21: set_model_ctl");
	return (USB_FAILURE);
}

static int
slcp21_get_modem_ctl(ds_hdl_t hdl, uint_t port_num, int mask, int *valp)
{
	cmn_err(CE_WARN, "usbser_slcp21: get_model_ctl");
	return (USB_FAILURE);
}

static int
slcp21_break_ctl(ds_hdl_t hdl, uint_t port_num, int ctl)
{
	cmn_err(CE_WARN, "usbser_slcp21: break_ctl");
	return (USB_FAILURE);
}

static int
slcp21_loopback(ds_hdl_t hdl, uint_t port_num, int ctl)
{
	cmn_err(CE_WARN, "usbser_slcp21: loopback");
	return (USB_FAILURE);
}

static int
slcp21_tx(ds_hdl_t hdl, uint_t port_num, mblk_t *mp)
{
	cmn_err(CE_WARN, "usbser_slcp21: tx");
	return (USB_FAILURE);
}

static mblk_t *
slcp21_rx(ds_hdl_t hdl, uint_t port_num)
{
	mblk_t		*mp;

	mp = NULL;

	cmn_err(CE_WARN, "usbser_slcp21: rx");
	return (mp);
}

static void
slcp21_stop(ds_hdl_t hdl, uint_t port_num, int dir)
{
	cmn_err(CE_WARN, "usbser_slcp21: stop");
	return;
}

static void
slcp21_start(ds_hdl_t hdl, uint_t port_num, int dir)
{
	cmn_err(CE_WARN, "usbser_slcp21: start");
	return;
}

static int
slcp21_fifo_flush(ds_hdl_t hdl, uint_t port_num, int dir)
{
	cmn_err(CE_WARN, "usbser_slcp21: fifo_flush");
	return (USB_FAILURE);
}

static int
slcp21_fifo_drain(ds_hdl_t hdl, uint_t port_num, int timeout)
{
	cmn_err(CE_WARN, "usbser_slcp21: fifo_drain");
	return (USB_FAILURE);
}


