Binary Files
===============

:date: 2017-06-29
:summary: Working with binary files on Linux

Determine ARM Type
--------------------

Determine type of binary, in this case, ARMv6:

.. code-block:: bash

  pi@starfire ~ $ readelf -A /usr/local/lib/python2.7/dist-packages/cv2.so 
  Attribute Section: aeabi
  File Attributes
    Tag_CPU_name: "6"
    Tag_CPU_arch: v6
    Tag_ARM_ISA_use: Yes
    Tag_THUMB_ISA_use: Thumb-1
    Tag_FP_arch: VFPv2
    Tag_ABI_PCS_wchar_t: 4
    Tag_ABI_FP_denormal: Needed
    Tag_ABI_FP_exceptions: Needed
    Tag_ABI_FP_number_model: IEEE 754
    Tag_ABI_align_needed: 8-byte
    Tag_ABI_align_preserved: 8-byte, except leaf SP
    Tag_ABI_enum_size: int
    Tag_ABI_HardFP_use: SP and DP
    Tag_ABI_VFP_args: VFP registers
    Tag_CPU_unaligned_access: v6
    Tag_ABI_FP_16bit_format: IEEE 754

Determine needed Libraries
-----------------------------

.. code-block:: bash

    pi@fry opencv3 $ readelf -d /usr/lib/liblapack.so | grep NEEDED
     0x00000001 (NEEDED)                     Shared library: [libblas.so.3]
     0x00000001 (NEEDED)                     Shared library: [libatlas.so.3]
     0x00000001 (NEEDED)                     Shared library: [libgfortran.so.3]
     0x00000001 (NEEDED)                     Shared library: [libgcc_s.so.1]
     0x00000001 (NEEDED)                     Shared library: [libpthread.so.0]
     0x00000001 (NEEDED)                     Shared library: [libm.so.6]
     0x00000001 (NEEDED)                     Shared library: [libc.so.6]
     0x00000001 (NEEDED)                     Shared library: [ld-linux-armhf.so.3]
 

.. code-block:: bash

    pi@fry ~ $ objdump -x /usr/lib/liblapack.so

    /usr/lib/liblapack.so:     file format elf32-littlearm
    /usr/lib/liblapack.so
    architecture: arm, flags 0x00000150:
    HAS_SYMS, DYNAMIC, D_PAGED
    start address 0x0001ea88

    Program Header:
        LOAD off    0x00000000 vaddr 0x00000000 paddr 0x00000000 align 2**16
             filesz 0x004b6188 memsz 0x004b6188 flags r-x
        LOAD off    0x004b7000 vaddr 0x004c7000 paddr 0x004c7000 align 2**16
             filesz 0x00002e38 memsz 0x00002ef0 flags rw-
     DYNAMIC off    0x004b7000 vaddr 0x004c7000 paddr 0x004c7000 align 2**2
             filesz 0x000000f0 memsz 0x000000f0 flags rw-
       STACK off    0x00000000 vaddr 0x00000000 paddr 0x00000000 align 2**4
             filesz 0x00000000 memsz 0x00000000 flags rw-

    Dynamic Section:
      NEEDED               libblas.so.3
      NEEDED               libatlas.so.3
      NEEDED               libgfortran.so.3
      NEEDED               libgcc_s.so.1
      NEEDED               libpthread.so.0
      NEEDED               libm.so.6
      NEEDED               libc.so.6
      NEEDED               ld-linux-armhf.so.3
      SONAME               liblapack.so.3
      HASH                 0x000000b4
      STRTAB               0x0000e77c
      SYMTAB               0x0000488c
      STRSZ                0x00005c43
      SYMENT               0x00000010
      PLTGOT               0x004c70f0
      PLTRELSZ             0x00003a68
      PLTREL               0x00000011
      JMPREL               0x00015870
      REL                  0x00015860
      RELSZ                0x00000010
      RELENT               0x00000008
      VERNEED              0x000157a0
      VERNEEDNUM           0x00000005
      VERSYM               0x000143c0

    Version References:
      required from ld-linux-armhf.so.3:
        0x0d696914 0x00 08 GLIBC_2.4
      required from libgcc_s.so.1:
        0x0b792655 0x00 07 GCC_3.5
        0x09275a60 0x00 06 GCC_4.0.0
      required from libc.so.6:
        0x0d696914 0x00 04 GLIBC_2.4
      required from libm.so.6:
        0x0d696914 0x00 03 GLIBC_2.4
      required from libgfortran.so.3:
        0x02f96580 0x00 05 GFORTRAN_1.0
        0x02f96584 0x00 02 GFORTRAN_1.4
    private flags = 5000402: [Version5 EABI] [hard-float ABI] [has entry point]

    Sections:
    Idx Name          Size      VMA       LMA       File off  Algn
      0 .hash         000047d8  000000b4  000000b4  000000b4  2**2
                      CONTENTS, ALLOC, LOAD, READONLY, DATA
      1 .dynsym       00009ef0  0000488c  0000488c  0000488c  2**2
                      CONTENTS, ALLOC, LOAD, READONLY, DATA
      2 .dynstr       00005c43  0000e77c  0000e77c  0000e77c  2**0
                      CONTENTS, ALLOC, LOAD, READONLY, DATA
      3 .gnu.version  000013de  000143c0  000143c0  000143c0  2**1
                      CONTENTS, ALLOC, LOAD, READONLY, DATA
      4 .gnu.version_r 000000c0  000157a0  000157a0  000157a0  2**2
                      CONTENTS, ALLOC, LOAD, READONLY, DATA
      5 .rel.dyn      00000010  00015860  00015860  00015860  2**2
                      CONTENTS, ALLOC, LOAD, READONLY, DATA
      6 .rel.plt      00003a68  00015870  00015870  00015870  2**2
                      CONTENTS, ALLOC, LOAD, READONLY, DATA
      7 .plt          000057b0  000192d8  000192d8  000192d8  2**2
                      CONTENTS, ALLOC, LOAD, READONLY, CODE
      8 .text         004829b8  0001ea88  0001ea88  0001ea88  2**3
                      CONTENTS, ALLOC, LOAD, READONLY, CODE
      9 .rodata       00014d48  004a1440  004a1440  004a1440  2**3
                      CONTENTS, ALLOC, LOAD, READONLY, DATA
     10 .dynamic      000000f0  004c7000  004c7000  004b7000  2**2
                      CONTENTS, ALLOC, LOAD, DATA
     11 .got          00001d48  004c70f0  004c70f0  004b70f0  2**2
                      CONTENTS, ALLOC, LOAD, DATA
     12 .data         00001000  004c8e38  004c8e38  004b8e38  2**2
                      CONTENTS, ALLOC, LOAD, DATA
     13 .bss          000000b8  004c9e38  004c9e38  004b9e38  2**3
                      ALLOC
     14 .ARM.attributes 00000031  00000000  00000000  004b9e38  2**0
                      CONTENTS, READONLY
    SYMBOL TABLE:
    no symbols

