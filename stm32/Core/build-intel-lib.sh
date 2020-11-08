#!/bin/sh
if [ -f gcc111libbid.a ]; then exit 0; fi

if [ -z $MK ]; then
  which gmake >/dev/null
  if [ $? -eq 0 ]; then
    MK=gmake
  else
    MK=make
  fi
fi

which arm-none-eabi-gcc >/dev/null
if [ $? -eq 0 ]; then
  CC=arm-none-eabi-gcc
else
  CC=cc
fi

# Hack to support FreeBSD; not 100% sure what this does, but it produces a
# library that passes all tests.

# stm32 is not need
OS_ARG=

# When building for big-endian targets, add "BID_BIG_ENDIAN=true" to the "make"
# command line. The library will work even if you don't, but the state files
# written by Free42 will only be compatible with those written on little-endian
# platforms if you do.
# Note that at the time of writing, I haven't created any successful big-endian
# builds yet. I have tried on Fedora 12 on Qemu emulating 32-bit PowerPC, and
# while the build succeeds and arithmetic and SQRT work, it crashes in the
# transcendentals.

# stm32 is little endian
ENDIAN_ARG=

tar xvfz ../inteldecimal/IntelRDFPMathLib20U1.tar.gz
cd IntelRDFPMathLib20U1
patch -p0 <../intel-lib-linux.patch

# When building for architectures other than x86 or x86_64, I remove the
# section titled "Determine host architecture" in
# IntelRDFPMathLib20U1/LIBRARY/makefile.iml_head, and replace it with a simple
# "_HOST_ARCH := x86" or "_HOST_ARCH := x86_64", depending on whether I'm
# building for a 32-bit or 64-bit platform, respectively. The actual CPU you
# specify seems to matter less than its word size. Thus, setting _HOST_ARCH
# to x86 works when targeting armv7 and ppc, both 32-bit platforms, and setting
# it to x86_64 works when targeting arm64, a 64-bit platform.
# Of course, proceed with caution. Your mileage may vary.

# stm32 is 32 bit
patch -p0 <../intel-lib-unknown-32bit.patch

cd LIBRARY
sed -i 's/\(CC_NAME_LIST  := .*\)/\1 arm-none-eabi-gcc/g;s/\(CC_TYPE_LIST  := .*\)/\1 gcc/g' makefile.iml_head
sed -i "s/\(_CFLAGS_OPT    :=\)/\1-mthumb -march=armv7e-m -mfloat-abi=hard -mfpu=fpv4-sp-d16/g" makefile
$MK $OS_ARG CC=$CC CALL_BY_REF=1 GLOBAL_RND=1 GLOBAL_FLAGS=1 UNCHANGED_BINARY_FLAGS=0 $ENDIAN_ARG
mv libbid.a ../../gcc111libbid.a
cd ../..
( echo '#ifdef FREE42_FPTEST'; echo 'const char *readtest_lines[] = {'; tr -d '\r' < IntelRDFPMathLib20U1/TESTS/readtest.in | sed 's/^\(.*\)$/"\1",/'; echo '0 };'; echo '#endif' ) > readtest_lines.cc
