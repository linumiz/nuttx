/****************************************************************************
 * boards/tricore/tc3xx/a2g-tc397xa-3v3-tft/scripts/tricore-llvm.ld
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
EXTERN(__start)

__HEAP_SIZE = (256 * 1024);
CONFIG_TRICORE_CSA_COUNT = 512;
CONFIG_TRICORE_INITIAL_STACK_SIZE = 512;
CONFIG_TRICORE_INITIAL_STACK_OFFSET = 0xEB00;

MEMORY
{
#if CPU == 0
  dsram  (rw): ORIGIN = 0x70000000, LENGTH = 240K
  psram  (rw): ORIGIN = 0x70100000, LENGTH = 64K
  dlmu   (rw): ORIGIN = 0x90000000, LENGTH = 64K
  pflash (rx): ORIGIN = 0x80000000, LENGTH = 3M
#endif

#if CPU == 1
  dsram  (rw): ORIGIN = 0x60000000, LENGTH = 240K
  psram  (rw): ORIGIN = 0x60100000, LENGTH = 64K
  dlmu   (rw): ORIGIN = 0x90010000, LENGTH = 64K
  pflash (rx): ORIGIN = 0x80300000, LENGTH = 3M
#endif

#if CPU == 2
  dsram  (rw): ORIGIN = 0x50000000, LENGTH = 96K
  psram  (rw): ORIGIN = 0x50100000, LENGTH = 64K
  dlmu   (rw): ORIGIN = 0x90020000, LENGTH = 64K
  pflash (rx): ORIGIN = 0x80600000, LENGTH = 3M
#endif

#if CPU == 3
  dsram  (rw): ORIGIN = 0x40000000, LENGTH = 96K
  psram  (rw): ORIGIN = 0x40100000, LENGTH = 64K
  dlmu   (rw): ORIGIN = 0x90030000, LENGTH = 64K
  pflash (rx): ORIGIN = 0x80900000, LENGTH = 3M
#endif

#if CPU == 4
  dsram  (rw): ORIGIN = 0x30000000, LENGTH = 96K
  psram  (rw): ORIGIN = 0x30100000, LENGTH = 64K
  dlmu   (rw): ORIGIN = 0x90100000, LENGTH = 64K
  pflash (rx): ORIGIN = 0x80C00000, LENGTH = 3M
#endif

#if CPU == 5
  dsram  (rw): ORIGIN = 0x10000000, LENGTH = 96K
  psram  (rw): ORIGIN = 0x10100000, LENGTH = 64K
  dlmu   (rw): ORIGIN = 0x90110000, LENGTH = 64K
  pflash (rx): ORIGIN = 0x80F00000, LENGTH = 1M
#endif

  lmu    (rw): ORIGIN = 0x90040000, LENGTH = 768K
}

REGION_ALIAS("CODE_MEM", pflash)
REGION_ALIAS("DATA_MEM", dsram)
REGION_ALIAS("BSS_MEM", dsram)
REGION_ALIAS("CSA_MEM", dsram)
REGION_ALIAS("HEAP_MEM", lmu)
REGION_ALIAS("PSRAM_MEM", psram)

SECTIONS
{
  .start :
  {
    KEEP(*(.start))
    . = ALIGN(4);
  } > CODE_MEM

  .traptab :
  {
    KEEP(*(.traptab))
    . = ALIGN(256);
  } > CODE_MEM

  .ivtab :
  {
    KEEP(*(.ivtab))
    . = ALIGN(4);
  } > CODE_MEM

  .text :
  {
    KEEP(*(.text))
    . = ALIGN(2);
  } > CODE_MEM

  .data : ALIGN(4)
  {
    _sdata = ABSOLUTE(.);
    *(.data .data.*)
    *(.gnu.linkonce.d.*)
    CONSTRUCTORS
    . = ALIGN(4);
    _edata = ABSOLUTE(.);
  } > DATA_MEM AT> CODE_MEM
  _data_flash = LOADADDR(.data);

  .psram_base :
  {
    . += CONFIG_TRICORE_INITIAL_STACK_OFFSET;
     __PSRAM_USTACK = . ;
    . += CONFIG_TRICORE_INITIAL_STACK_SIZE;
     __PSRAM_BOOT_CSA = . ;
  } > PSRAM_MEM

  .bss :
  {
    _sbss = ABSOLUTE(.);
    *(.bss .bss.*)
    *(.gnu.linkonce.b.*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = ABSOLUTE(.);
  } > BSS_MEM

  .ustack ALIGN(8) (NOLOAD) :
  {
    . +=  CONFIG_IDLETHREAD_STACKSIZE ;
    __USTACK =  . ;
  } > BSS_MEM

#if defined(CONFIG_ARCH_INTERRUPTSTACK)
  .istack ALIGN(8) (NOLOAD) :
  {
    . += CONFIG_ARCH_INTERRUPTSTACK;
    __istack =  . ;
  } > BSS_MEM
#endif

  .csa (NOLOAD) : ALIGN(64)
  {
    __CSA_BEGIN = . ;
    . +=  CONFIG_TRICORE_CSA_COUNT * 64 ;
    __CSA_END = .;
  } > CSA_MEM
  __CSA_SIZE = __CSA_END - __CSA_BEGIN;

  .heap (NOLOAD) : ALIGN(8)
  {
    _sheap =  . ;
    . +=  __HEAP_SIZE;
    _eheap =  . ;
  } > HEAP_MEM
  _heap_size = _sheap - _eheap;
}
