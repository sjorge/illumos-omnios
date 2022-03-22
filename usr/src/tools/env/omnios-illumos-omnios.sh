
######################################################################
# OmniOS-specific overrides

# Enable the non-DEBUG build
NIGHTLY_OPTIONS=${NIGHTLY_OPTIONS/F/}

export ON_CLOSED_BINS=/opt/onbld/closed

# On OmniOS, gcc resides in /opt/gcc-<version> - adjust variables
for name in PRIMARY_CC PRIMARY_CCC SHADOW_CCS SHADOW_CCCS; do
        typeset -n var=$name
        var="`echo $var | sed '
                s^/usr/gcc^/opt/gcc^
                s^/opt/gcc/^/opt/gcc-^
        '`"
done

# OmniOS is built with a gcc-10 primary compiler and gcc-7 secondary. This is
# currently the inverse of illumos-gate. The illumos-gate switch to gcc-10 is
# https://www.illumos.org/issues/14421
export GNUC_ROOT=/opt/gcc-10/
for name in PRIMARY_CC PRIMARY_CCC; do
        typeset -n var=$name
        var="${var//gcc-7/gcc-10}"
        var="${var//gcc7/gcc10}"
done
for name in SHADOW_CCS SHADOW_CCCS; do
        typeset -n var=$name
        var="${var//gcc-10/gcc-7}"
        var="${var//gcc10/gcc7}"
done

ENABLE_SMB_PRINTING='#'

_branch=`git -C $CODEMGR_WS rev-parse --abbrev-ref HEAD`
_hash=`git -C $CODEMGR_WS rev-parse --short HEAD`
export VERSION=`echo omnios-$_branch-$_hash | tr '/' '-'`

export ONNV_BUILDNUM=`grep '^VERSION=r' /etc/os-release | cut -c10-15`
export PKGVERS_BRANCH=$ONNV_BUILDNUM.0

