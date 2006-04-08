/*****************************************************************************
 * Free42 -- an HP-42S calculator simulator
 * Copyright (C) 2004-2006  Thomas Okken
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2,
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *****************************************************************************/

#ifndef FREE42_H
#define FREE42_H 1


#if defined(PALMOS) && !defined(PALMOS_ARM)

#include <PalmOS.h>
#include <Libraries/PalmOSGlue/PalmOSGlue.h>
#ifndef BCD_MATH
extern "C" {
#include "MathLib.h"
}
#endif

/* Segmentation stuff */
#define SHELL1_SECT __attribute__ ((section ("Shell1")))
#define SHELL2_SECT __attribute__ ((section ("Shell2")))
#define FILESYS_SECT __attribute__ ((section ("FileSys")))
#define MAIN_SECT __attribute__ ((section ("Main")))
#define COMMANDS1_SECT __attribute__ ((section ("Commnds1")))
#define COMMANDS2_SECT __attribute__ ((section ("Commnds2")))
#define COMMANDS3_SECT __attribute__ ((section ("Commnds3")))
#define COMMANDS4_SECT __attribute__ ((section ("Commnds4")))
#define COMMANDS5_SECT __attribute__ ((section ("Commnds5")))
#define COMMANDS6_SECT __attribute__ ((section ("Commnds6")))
#define DISPLAY_SECT __attribute__ ((section ("Display")))
#define GLOBALS_SECT __attribute__ ((section ("Globals")))
#define HELPERS_SECT __attribute__ ((section ("Helpers")))
#define KEYDOWN_SECT __attribute__ ((section ("KeyDown")))
#define LINALG1_SECT __attribute__ ((section ("LinAlg1")))
#define LINALG2_SECT __attribute__ ((section ("LinAlg2")))
#define MATH1_SECT __attribute__ ((section ("Math1")))
#define MATH2_SECT __attribute__ ((section ("Math2")))
#define PHLOAT_SECT __attribute__ ((section ("Phloat")))
#define STO_RCL_SECT __attribute__ ((section ("StoRcl")))
#define TABLES_SECT __attribute__ ((section ("Tables")))
#define VARIABLES_SECT __attribute__ ((section ("Variabls")))
#define BCD_SECT __attribute__ ((section ("BcdFloat")))
#define int2 Int16
#define uint2 UInt16
#define int4 Int32
#define uint4 UInt32

#else /* !PALMOS */

#include <math.h>
#define SHELL1_SECT
#define SHELL2_SECT
#define FILESYS_SECT
#define MAIN_SECT
#define COMMANDS1_SECT
#define COMMANDS2_SECT
#define COMMANDS3_SECT
#define COMMANDS4_SECT
#define COMMANDS5_SECT
#define COMMANDS6_SECT
#define DISPLAY_SECT
#define GLOBALS_SECT
#define HELPERS_SECT
#define KEYDOWN_SECT
#define LINALG1_SECT
#define LINALG2_SECT
#define MATH1_SECT
#define MATH2_SECT
#define PHLOAT_SECT
#define STO_RCL_SECT
#define TABLES_SECT
#define VARIABLES_SECT
#define BCD_SECT
#define int2 short
#define uint2 unsigned short
#define int4 int
#define uint4 unsigned int

#endif /* PALMOS */


#if defined(WINDOWS) && !defined(__GNUC__)

#define int8 __int64
#define uint8 unsigned __int64
#define LL(x) x

	/* MSVC++ 6.0 lacks a few math functions that Free42 needs.
	 * I've defined workarounds in mathfudge.c. NOTE: my versions
	 * of isnan(), finite(), and isinf() are a bit lame -- I *think*
	 * they handle infinities properly, but definitely not NaNs
	 * (although NaNs shouldn't be much of a problem because the Free42
	 * code mostly tries to avoid them, rather than detect them after
	 * the fact).
	 */
#ifdef __cplusplus
	extern "C" {
#endif
		int isnan(double x);
		int finite(double x);
		int isinf(double x);
		void sincos(double x, double *sinx, double *cosx);
		double asinh(double x);
		double acosh(double x);
		double atanh(double x);
		double expm1(double x);
		double log1p(double x);
#ifdef _WIN32_WCE
		double hypot(double x, double y);
#endif
#ifdef __cplusplus
	}
#endif
#else

#define int8 long long
#define uint8 unsigned long long
#define LL(x) x##LL

/* NOTE: In my Linux build, all I have to do is DECLARE sincos(); glibc 2.3.3
 * has it (for C99, I suppose) so I don't have to DEFINE it. On other Unixes
 * (e.g. MacOS X), it may not be provided by the standard libraries; in this
 * case, define the NO_SINCOS symbol, here or in the Makefile.
 * For the Palm build, we don't even need the declaration, since sincos() is
 * provided by MathLib.
 */
#ifndef PALMOS
extern "C" void sincos(double x, double *sinx, double *cosx) HELPERS_SECT;
#endif
//#define NO_SINCOS 1

#endif


#if defined(PALMOS) && defined(BCD_MATH)
// Compatibility hacks for the sake of core_phloat and bcdfloat. I don't want
// to depend on MathLib for this stuff, especially since I can make do with
// simple partial implementations.
int isinf(double d) PHLOAT_SECT;
int isnan(double d) PHLOAT_SECT;
double pow(double x, double y) PHLOAT_SECT;
double floor(double x) PHLOAT_SECT;
double log10(double x) PHLOAT_SECT;
#endif


#define uint unsigned int

/* Magic number and version number for the state file.
 * Note: do not confuse the state file version with
 * the application version.
 * The state file version was originally 0; state file
 * versions 1 and 2 were introduced with application version 1.0.13.
 * Version 3: 1.0.16
 * Versions 4 and 5: 1.1
 * Version 6: 1.1.8
 * Version 7: 1.1.12
 * Version 8: 1.1.14
 * Version 9: 1.4
 */
#define FREE42_MAGIC 0x466b3432
#define FREE42_VERSION 9


#endif
