/****************************************************************************
 * boards/tricore/tc4x/triboard_tc4x9_com/scripts/tricore.ld
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#include <nuttx/config.h>

OUTPUT_FORMAT("elf32-tricore")
OUTPUT_ARCH("tricore")
ENTRY(__start)

/* CPU0 only: program flash bank A, the data scratchpad RAM and a window of the
 * DLMU of CPU0.
 * The CSA, the interrupt stack and the idle stack are at the top of the
 * data scratchpad RAM, the heap below them.
 */

MEMORY
{
  pfls (rx!p) : ORIGIN = 0x80000000, LENGTH = 1024K
  dspr (w!xp) : ORIGIN = 0x70000000, LENGTH = 240K
  dlmu (w!xp) : ORIGIN = 0x90020400, LENGTH = 171K
}

REGION_ALIAS("DATA", dlmu)

__CSA_SIZE    = 40K;
__ISTACK_SIZE = CONFIG_ARCH_INTERRUPTSTACK;
__USTACK_SIZE = CONFIG_IDLETHREAD_STACKSIZE;

__CSA0_END    = ORIGIN(dspr) + LENGTH(dspr);
__CSA0        = __CSA0_END - __CSA_SIZE;
__ISTACK0     = __CSA0;
__ISTACK0_END = __ISTACK0 - __ISTACK_SIZE;
__USTACK0     = __ISTACK0_END;
__USTACK0_END = __USTACK0 - __USTACK_SIZE;

/* Boot stack and single boot CSA in the PSPR, which the startup code uses
 * to disable the virtualization before it links the CSAs
 */

__PSRAM_USTACK   = 0x7010eb00;
__PSRAM_BOOT_CSA = 0x7010ec00;

/* The other cores start at the program flash of their own image */

PROVIDE(__start1 = 0x80200000);
PROVIDE(__start2 = 0x80580000);
PROVIDE(__start3 = 0x80680000);
PROVIDE(__start4 = 0x807e0000);
PROVIDE(__start5 = 0x8092c000);

SECTIONS
{
  /* Reset entry, the trap table at +0x100 and the interrupt vector at
   * +0x1fe0, which BIV points at with all priority bits set
   */

  .start ORIGIN(pfls) :
  {
    KEEP(*(.start))
  } > pfls

  .traptab ORIGIN(pfls) + 0x100 :
  {
    __TRAPTAB_CPU0 = .;
    KEEP(*(.traptab))
  } > pfls

  .intvec ORIGIN(pfls) + 0x1fe0 :
  {
    __INTTAB_CPU0 = ORIGIN(pfls);
    KEEP(*(.intvec))
  } > pfls

  .text :
  {
    *(.text .text.*)
    *(.gnu.linkonce.t.*)
    . = ALIGN(8);
  } > pfls

  .rodata :
  {
    *(.rodata .rodata.*)
    *(.zrodata .zrodata.*)
    *(.gnu.linkonce.r.*)
    . = ALIGN(4);
  } > pfls

  /* Small constants, addressed relative to A1 */

  .srodata :
  {
    *(.srodata .srodata.*)
  } > pfls

  __A1_MEM = ADDR(.srodata) + 32K;
  __A8_MEM = __A1_MEM;

  .ctors :
  {
    _sinit = ABSOLUTE(.);
    KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
    KEEP(*(.init_array .ctors))
    _einit = ABSOLUTE(.);
    . = ALIGN(8);
  } > pfls

  .eh_frame :
  {
    KEEP(*(.eh_frame))
  } > pfls

  /* Initialized data, small data (A0) first, copied from flash at startup */

  .data :
  {
    _sdata = ABSOLUTE(.);
    *(.sdata .sdata.*)
    *(.zdata .zdata.*)
    *(.data .data.*)
    *(.gnu.linkonce.d.*)
    . = ALIGN(4);
    _edata = ABSOLUTE(.);
  } > DATA AT > pfls

  _data_flash = LOADADDR(.data);

  .bss (NOLOAD) :
  {
    _sbss = ABSOLUTE(.);
    *(.sbss .sbss.*)
    *(.zbss .zbss.*)
    *(.bss .bss.*)
    *(.gnu.linkonce.b.*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = ABSOLUTE(.);
  } > DATA

  __A0_MEM = _sdata + 32K;
  __A9_MEM = __A0_MEM;

  .heap (NOLOAD) :
  {
    . = ALIGN(8);
    _sheap = ABSOLUTE(.);
  } > dspr

  _eheap = __USTACK0_END;

  .ustack __USTACK0_END (NOLOAD) :
  {
    . += __USTACK_SIZE;
  } > dspr

  .istack __ISTACK0_END (NOLOAD) :
  {
    . += __ISTACK_SIZE;
  } > dspr

  .csa __CSA0 (NOLOAD) :
  {
    . += __CSA_SIZE;
  } > dspr

  /DISCARD/ :
  {
    *(.comment)
  }
}
