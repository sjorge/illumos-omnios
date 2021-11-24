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
 * Copyright 2021 Racktop Systems, Inc.
 */

#include <sys/tsc.h>
#include <sys/types.h>
#include <sys/x86_archext.h>
#include <sys/hyperv.h>
#include <hyperv_reg.h>

static boolean_t
tsc_calibrate_hyperv(uint64_t *freqp)
{
	struct cpuid_regs regs;
	uint64_t freq;

	/*
	 * While hyperv_features should (on Hyper-V systems) contain the
	 * bitfield we need to verify TSC frequency support, we are early
	 * enough in the boot process that it's not yet available, so we
	 * must issue the CPUID instructions ourself.
	 */
	if (get_hwenv() != HW_MICROSOFT) {
		return (B_FALSE);
	}

	/*
	 * Not having a max HV leaf if we're Hyper-V (per the above test)
	 * should not happen. Warn and fallback to other methods if this
	 * happens.
	 */
	bzero(&regs, sizeof (regs));
	regs.cp_eax = CPUID_LEAF_HV_MAXLEAF;
	__cpuid_insn(&regs);
	if (regs.cp_eax < CPUID_LEAF_HV_FEATURES) {
		cmn_err(CE_WARN, "%s: hv max leaf returned 0x%x\n", __func__,
		    regs.cp_eax);
		return (B_FALSE);
	}

	/*
	 * The earliest (pre-2008) versions of Hyper-V do not support
	 * the TSC FREQ MSR. HV_FEATURES will tell us if the MSR is supported
	 * on this Hyper-V host or not.
	 */
	bzero(&regs, sizeof (regs));
	regs.cp_eax = CPUID_LEAF_HV_FEATURES;
	__cpuid_insn(&regs);
	if ((regs.cp_eax & CPUID_HV_MSR_TSC_FREQ) == 0) {
		return (B_FALSE);
	}

	freq = rdmsr(MSR_HV_TSC_FREQUENCY);

	/*
	 * We could panic here since a value of 0 is wrong per Microsoft's own
	 * documentation. But there's little harm in attempting the other
	 * TSC sources -- on Gen1 VMs either the emulated HPET or PIT may
	 * succeed and the worst case is that we panic if those fail.
	 */
	if (freq == 0) {
		cmn_err(CE_WARN, "%s: tsc freq was 0\n", __func__);
		return (B_FALSE);
	}
	*freqp = freq;
	return (B_TRUE);
}

static tsc_calibrate_t tsc_calibration_hyperv = {
	.tscc_source = "Hyper-V",
	.tscc_preference = 100,
	.tscc_calibrate = tsc_calibrate_hyperv,
};
TSC_CALIBRATION_SOURCE(tsc_calibration_hyperv);
