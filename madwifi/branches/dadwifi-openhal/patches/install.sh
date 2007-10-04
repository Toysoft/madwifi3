#! /bin/sh
#
# Shell script to integrate madwifi sources into a Linux
# source tree so it can be built statically.  Typically this
# is done to simplify debugging with tools like kgdb.
#

set -e

die()
{
	echo "FATAL ERROR: $1" >&2
	exit 1
}

SRC=..
KERNEL_VERSION=`uname -r`

if test -n "$1"; then
	KERNEL_PATH="$1"
else if test -e /lib/modules/${KERNEL_VERSION}/source; then
	KERNEL_PATH="/lib/modules/${KERNEL_VERSION}/source"
else if test -e /lib/modules/${KERNEL_VERSION}/build; then
	KERNEL_PATH="/lib/modules/${KERNEL_VERSION}/build"
else
	die "Cannot guess kernel source location"
fi
fi
fi

test -d ${KERNEL_PATH} || die "No kernel directory ${KERNEL_PATH}"

PATCH()
{
	patch -N $1 < $2
}

#
# Location of various pieces.  These mimic what is in Makefile.inc
# and can be overridden from the environment.
#
SRC_HAL=${HAL:-${SRC}/openhal}
test -d ${SRC_HAL} || die "No openhal directory ${SRC_HAL}"
SRC_ATH=${ATH:-${SRC}/ath}
test -d ${SRC_ATH} || die "No ath directory ${SRC_ATH}"
SRC_COMPAT=${SRC}/include
test -d ${SRC_COMPAT} || die "No compat directory ${SRC_COMPAT}"

WIRELESS=${KERNEL_PATH}/drivers/net/wireless
test -d ${WIRELESS} || die "No wireless directory ${WIRELESS}"

if test -f ${WIRELESS}/Kconfig; then
	kbuild=2.6
	kbuildconf=Kconfig
else
	die "Kernel build system is not supported"
fi

echo "Copying top-level files"
MADWIFI=${WIRELESS}/madwifi
rm -rf ${MADWIFI}
mkdir -p ${MADWIFI}
make -s -C ${SRC} svnversion.h KERNELPATH=${KERNEL_PATH} KERNELCONF=/dev/null ARCH=. TARGET=i386-elf
cp -f ${SRC}/BuildCaps.inc ${SRC}/svnversion.h ${SRC}/release.h ${MADWIFI}


echo "Copying source files"
FILES=`cd ${SRC} && find ath openhal include -name '*.[ch]'`
for f in $FILES; do
	case $f in
		*.mod.c) continue;;
	esac
	mkdir -p `dirname ${MADWIFI}/$f`
	cp -f ${SRC}/$f ${MADWIFI}/$f
done

echo "Copying makefiles"
FILES=`cd ${SRC} && find . -name Makefile.kernel`
for f in $FILES; do
	cp -f ${SRC}/$f `dirname ${MADWIFI}/$f`/Makefile
done


echo "Patching the build system"
cp -f $kbuild/Makefile ${MADWIFI}
if test "$kbuild" = 2.6; then
cp -f $kbuild/Kconfig ${MADWIFI}
sed -i '/madwifi/d;/^endmenu/i\
source "drivers/net/wireless/madwifi/Kconfig"' ${WIRELESS}/Kconfig
sed -i '$a\
obj-$(CONFIG_ATHEROS) += madwifi/
/madwifi/d;' ${WIRELESS}/Makefile
else
cp -f $kbuild/Config.in ${MADWIFI}
sed -i '$a\
source drivers/net/wireless/madwifi/Config.in
/madwifi/d' ${WIRELESS}/Config.in
sed -i '/madwifi/d;/include/i\
subdir-$(CONFIG_ATHEROS) += madwifi\
obj-$(CONFIG_ATHEROS) += madwifi/madwifi.o' ${WIRELESS}/Makefile
fi

DST_DOC=${KERNEL_PATH}/Documentation
if test -f $kbuild/Configure.help.patch; then
	grep -q 'CONFIG_ATHEROS' ${DST_DOC}/Configure.help || \
		PATCH ${DST_DOC}/Configure.help $kbuild/Configure.help.patch
fi


echo "Done"
