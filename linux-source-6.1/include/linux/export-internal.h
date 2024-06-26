/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Please do not include this explicitly.
 * This is used by C files generated by modpost.
 */

#ifndef __LINUX_EXPORT_INTERNAL_H__
#define __LINUX_EXPORT_INTERNAL_H__

#include <linux/compiler.h>
#include <linux/types.h>

#define SYMBOL_CRC(sym, crc, sec)   \
	asm(".section \"___kcrctab" sec "+" #sym "\",\"a\""	"\n" \
	    ".balign 4"						"\n" \
	    "__crc_" #sym ":"					"\n" \
	    ".long " #crc					"\n" \
	    ".previous"						"\n")

#endif /* __LINUX_EXPORT_INTERNAL_H__ */
