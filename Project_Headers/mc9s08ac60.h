/* Based on CPU DB MC9S08AC60_64, version 3.00.040 (RegistersPrg V2.32) */
/*
** ###################################################################
**     Filename  : mc9s08ac60.h
**     Processor : MC9S08AC60CPU
**     FileFormat: V2.32
**     DataSheet : MC9S08AC60 Rev. 2 3/2008
**     Compiler  : CodeWarrior compiler
**     Date/Time : 5.10.2010, 13:47
**     Abstract  :
**         This header implements the mapping of I/O devices.
**
**     Copyright : 1997 - 2010 Freescale Semiconductor, Inc. All Rights Reserved.
**     
**     http      : www.freescale.com
**     mail      : support@freescale.com
**
**     CPU Registers Revisions:
**      - 18.07.2008, V3.00.0:
**              - Added CRC module at 0x0056.
**              -   REASON: Changes in Data Sheet (from Rev. 1 Draft E 5/2007 to Rev.2 3/2008).
**      - 2.11.2009, V3.00.1:
**              - Added NVICGTRM, NVICGTRM1 registers.
**              -   REASON: Bug-fix (#7770 in Issue Manager).
**
**     File-Format-Revisions:
**      - 19.07.2007, V2.18 :
**               - Improved number of blanked lines inside register structures
**      - 06.08.2007, V2.19 :
**               - CPUDB revisions generated ahead of the file-format revisions.
**      - 11.09.2007, V2.20 :
**               - Added comment about initialization of unbonded pins.
**      - 02.01.2008, V2.21 :
**               - Changes have not affected this file (because they are related to another family)
**      - 13.02.2008, V2.22 :
**               - Changes have not affected this file (because they are related to another family)
**      - 20.02.2008, V2.23 :
**               - Changes have not affected this file (because they are related to another family)
**      - 03.07.2008, V2.24 :
**               - Added support for bits with name starting with number (like "1HZ")
**      - 28.11.2008, V2.25 :
**               - StandBy RAM array declaration for ANSI-C added
**      - 1.12.2008, V2.26 :
**               - Duplication of bit (or bit-group) name with register name is not marked as a problem, if register is internal only and it is not displayed in I/O map.
**      - 17.3.2009, V2.27 :
**               - Merged bit-group is not generated, if the name matches with another bit name in the register
**      - 6.4.2009, V2.28 :
**               - Fixed generation of merged bits for bit-groups with a digit at the end, if group-name is defined in CPUDB
**      - 3.8.2009, V2.29 :
**               - If there is just one bits group matching register name, single bits are not generated
**      - 10.9.2009, V2.30 :
**               - Fixed generation of registers arrays.
**      - 15.10.2009, V2.31 :
**               - HCS08 family: Bits and bit-groups are published for 16-bit registers: 8-bit overlay registers are required.
**      - 18.05.2010, V2.32 :
**               - MISRA compliance: U/UL suffixes added to all numbers (_MASK,_BITNUM and addresses)
**
**     Not all general-purpose I/O pins are available on all packages or on all mask sets of a specific
**     derivative device. To avoid extra current drain from floating input pins, the user’s reset
**     initialization routine in the application program must either enable on-chip pull-up devices
**     or change the direction of unconnected pins to outputs so the pins do not float.
** ###################################################################
*/

#ifndef _MC9S08AC60_H
#define _MC9S08AC60_H

/*lint -save  -e950 -esym(960,18.4) -e46 -esym(961,19.7) Disable MISRA rule (1.1,18.4,6.4,19.7) checking. */
/* Types definition */
typedef unsigned char byte;
typedef unsigned int word;
typedef unsigned long dword;
typedef unsigned long dlong[2];

/* Watchdog reset macro */
#ifndef __RESET_WATCHDOG
#ifdef _lint
  #define __RESET_WATCHDOG()  /* empty */
#else
  #define __RESET_WATCHDOG() {asm sta SRS;}
#endif
#endif /* __RESET_WATCHDOG */

#define REG_BASE 0x0000                /* Base address for the I/O register block */


#pragma MESSAGE DISABLE C1106 /* WARNING C1106: Non-standard bitfield type */

/**************** interrupt vector numbers ****************/
#define VectorNumber_Vtpm3ovf           28U
#define VectorNumber_Vtpm3ch1           27U
#define VectorNumber_Vtpm3ch0           26U
#define VectorNumber_Vrti               25U
#define VectorNumber_Viic1              24U
#define VectorNumber_Vadc1              23U
#define VectorNumber_Vkeyboard1         22U
#define VectorNumber_Vsci2tx            21U
#define VectorNumber_Vsci2rx            20U
#define VectorNumber_Vsci2err           19U
#define VectorNumber_Vsci1tx            18U
#define VectorNumber_Vsci1rx            17U
#define VectorNumber_Vsci1err           16U
#define VectorNumber_Vspi1              15U
#define VectorNumber_Vtpm2ovf           14U
#define VectorNumber_Vtpm2ch1           13U
#define VectorNumber_Vtpm2ch0           12U
#define VectorNumber_Vtpm1ovf           11U
#define VectorNumber_Vtpm1ch5           10U
#define VectorNumber_Vtpm1ch4           9U
#define VectorNumber_Vtpm1ch3           8U
#define VectorNumber_Vtpm1ch2           7U
#define VectorNumber_Vtpm1ch1           6U
#define VectorNumber_Vtpm1ch0           5U
#define VectorNumber_Vicg               4U
#define VectorNumber_Vlvd               3U
#define VectorNumber_Virq               2U
#define VectorNumber_Vswi               1U
#define VectorNumber_Vreset             0U

/**************** interrupt vector table ****************/
#define Vtpm3ovf                        0xFFC6U
#define Vtpm3ch1                        0xFFC8U
#define Vtpm3ch0                        0xFFCAU
#define Vrti                            0xFFCCU
#define Viic1                           0xFFCEU
#define Vadc1                           0xFFD0U
#define Vkeyboard1                      0xFFD2U
#define Vsci2tx                         0xFFD4U
#define Vsci2rx                         0xFFD6U
#define Vsci2err                        0xFFD8U
#define Vsci1tx                         0xFFDAU
#define Vsci1rx                         0xFFDCU
#define Vsci1err                        0xFFDEU
#define Vspi1                           0xFFE0U
#define Vtpm2ovf                        0xFFE2U
#define Vtpm2ch1                        0xFFE4U
#define Vtpm2ch0                        0xFFE6U
#define Vtpm1ovf                        0xFFE8U
#define Vtpm1ch5                        0xFFEAU
#define Vtpm1ch4                        0xFFECU
#define Vtpm1ch3                        0xFFEEU
#define Vtpm1ch2                        0xFFF0U
#define Vtpm1ch1                        0xFFF2U
#define Vtpm1ch0                        0xFFF4U
#define Vicg                            0xFFF6U
#define Vlvd                            0xFFF8U
#define Virq                            0xFFFAU
#define Vswi                            0xFFFCU
#define Vreset                          0xFFFEU

/**************** registers I/O map ****************/

/*** PTAD - Port A Data Register; 0x00000000 ***/
typedef union {
  byte Byte;
  struct {
    byte PTAD0       :1;                                       /* Port A Data Register Bit 0 */
    byte PTAD1       :1;                                       /* Port A Data Register Bit 1 */
    byte PTAD2       :1;                                       /* Port A Data Register Bit 2 */
    byte PTAD3       :1;                                       /* Port A Data Register Bit 3 */
    byte PTAD4       :1;                                       /* Port A Data Register Bit 4 */
    byte PTAD5       :1;                                       /* Port A Data Register Bit 5 */
    byte PTAD6       :1;                                       /* Port A Data Register Bit 6 */
    byte PTAD7       :1;                                       /* Port A Data Register Bit 7 */
  } Bits;
} PTADSTR;
extern volatile PTADSTR _PTAD @0x00000000;
#define PTAD                            _PTAD.Byte
#define PTAD_PTAD0                      _PTAD.Bits.PTAD0
#define PTAD_PTAD1                      _PTAD.Bits.PTAD1
#define PTAD_PTAD2                      _PTAD.Bits.PTAD2
#define PTAD_PTAD3                      _PTAD.Bits.PTAD3
#define PTAD_PTAD4                      _PTAD.Bits.PTAD4
#define PTAD_PTAD5                      _PTAD.Bits.PTAD5
#define PTAD_PTAD6                      _PTAD.Bits.PTAD6
#define PTAD_PTAD7                      _PTAD.Bits.PTAD7

#define PTAD_PTAD0_MASK                 1U
#define PTAD_PTAD1_MASK                 2U
#define PTAD_PTAD2_MASK                 4U
#define PTAD_PTAD3_MASK                 8U
#define PTAD_PTAD4_MASK                 16U
#define PTAD_PTAD5_MASK                 32U
#define PTAD_PTAD6_MASK                 64U
#define PTAD_PTAD7_MASK                 128U


/*** PTADD - Port A Data Direction Register; 0x00000001 ***/
typedef union {
  byte Byte;
  struct {
    byte PTADD0      :1;                                       /* Data Direction for Port A Bit 0 */
    byte PTADD1      :1;                                       /* Data Direction for Port A Bit 1 */
    byte PTADD2      :1;                                       /* Data Direction for Port A Bit 2 */
    byte PTADD3      :1;                                       /* Data Direction for Port A Bit 3 */
    byte PTADD4      :1;                                       /* Data Direction for Port A Bit 4 */
    byte PTADD5      :1;                                       /* Data Direction for Port A Bit 5 */
    byte PTADD6      :1;                                       /* Data Direction for Port A Bit 6 */
    byte PTADD7      :1;                                       /* Data Direction for Port A Bit 7 */
  } Bits;
} PTADDSTR;
extern volatile PTADDSTR _PTADD @0x00000001;
#define PTADD                           _PTADD.Byte
#define PTADD_PTADD0                    _PTADD.Bits.PTADD0
#define PTADD_PTADD1                    _PTADD.Bits.PTADD1
#define PTADD_PTADD2                    _PTADD.Bits.PTADD2
#define PTADD_PTADD3                    _PTADD.Bits.PTADD3
#define PTADD_PTADD4                    _PTADD.Bits.PTADD4
#define PTADD_PTADD5                    _PTADD.Bits.PTADD5
#define PTADD_PTADD6                    _PTADD.Bits.PTADD6
#define PTADD_PTADD7                    _PTADD.Bits.PTADD7

#define PTADD_PTADD0_MASK               1U
#define PTADD_PTADD1_MASK               2U
#define PTADD_PTADD2_MASK               4U
#define PTADD_PTADD3_MASK               8U
#define PTADD_PTADD4_MASK               16U
#define PTADD_PTADD5_MASK               32U
#define PTADD_PTADD6_MASK               64U
#define PTADD_PTADD7_MASK               128U


/*** PTBD - Port B Data Register; 0x00000002 ***/
typedef union {
  byte Byte;
  struct {
    byte PTBD0       :1;                                       /* Port B Data Register Bit 0 */
    byte PTBD1       :1;                                       /* Port B Data Register Bit 1 */
    byte PTBD2       :1;                                       /* Port B Data Register Bit 2 */
    byte PTBD3       :1;                                       /* Port B Data Register Bit 3 */
    byte PTBD4       :1;                                       /* Port B Data Register Bit 4 */
    byte PTBD5       :1;                                       /* Port B Data Register Bit 5 */
    byte PTBD6       :1;                                       /* Port B Data Register Bit 6 */
    byte PTBD7       :1;                                       /* Port B Data Register Bit 7 */
  } Bits;
} PTBDSTR;
extern volatile PTBDSTR _PTBD @0x00000002;
#define PTBD                            _PTBD.Byte
#define PTBD_PTBD0                      _PTBD.Bits.PTBD0
#define PTBD_PTBD1                      _PTBD.Bits.PTBD1
#define PTBD_PTBD2                      _PTBD.Bits.PTBD2
#define PTBD_PTBD3                      _PTBD.Bits.PTBD3
#define PTBD_PTBD4                      _PTBD.Bits.PTBD4
#define PTBD_PTBD5                      _PTBD.Bits.PTBD5
#define PTBD_PTBD6                      _PTBD.Bits.PTBD6
#define PTBD_PTBD7                      _PTBD.Bits.PTBD7

#define PTBD_PTBD0_MASK                 1U
#define PTBD_PTBD1_MASK                 2U
#define PTBD_PTBD2_MASK                 4U
#define PTBD_PTBD3_MASK                 8U
#define PTBD_PTBD4_MASK                 16U
#define PTBD_PTBD5_MASK                 32U
#define PTBD_PTBD6_MASK                 64U
#define PTBD_PTBD7_MASK                 128U


/*** PTBDD - Port B Data Direction Register; 0x00000003 ***/
typedef union {
  byte Byte;
  struct {
    byte PTBDD0      :1;                                       /* Data Direction for Port B Bit 0 */
    byte PTBDD1      :1;                                       /* Data Direction for Port B Bit 1 */
    byte PTBDD2      :1;                                       /* Data Direction for Port B Bit 2 */
    byte PTBDD3      :1;                                       /* Data Direction for Port B Bit 3 */
    byte PTBDD4      :1;                                       /* Data Direction for Port B Bit 4 */
    byte PTBDD5      :1;                                       /* Data Direction for Port B Bit 5 */
    byte PTBDD6      :1;                                       /* Data Direction for Port B Bit 6 */
    byte PTBDD7      :1;                                       /* Data Direction for Port B Bit 7 */
  } Bits;
} PTBDDSTR;
extern volatile PTBDDSTR _PTBDD @0x00000003;
#define PTBDD                           _PTBDD.Byte
#define PTBDD_PTBDD0                    _PTBDD.Bits.PTBDD0
#define PTBDD_PTBDD1                    _PTBDD.Bits.PTBDD1
#define PTBDD_PTBDD2                    _PTBDD.Bits.PTBDD2
#define PTBDD_PTBDD3                    _PTBDD.Bits.PTBDD3
#define PTBDD_PTBDD4                    _PTBDD.Bits.PTBDD4
#define PTBDD_PTBDD5                    _PTBDD.Bits.PTBDD5
#define PTBDD_PTBDD6                    _PTBDD.Bits.PTBDD6
#define PTBDD_PTBDD7                    _PTBDD.Bits.PTBDD7

#define PTBDD_PTBDD0_MASK               1U
#define PTBDD_PTBDD1_MASK               2U
#define PTBDD_PTBDD2_MASK               4U
#define PTBDD_PTBDD3_MASK               8U
#define PTBDD_PTBDD4_MASK               16U
#define PTBDD_PTBDD5_MASK               32U
#define PTBDD_PTBDD6_MASK               64U
#define PTBDD_PTBDD7_MASK               128U


/*** PTCD - Port C Data Register; 0x00000004 ***/
typedef union {
  byte Byte;
  struct {
    byte PTCD0       :1;                                       /* Port C Data Register Bit 0 */
    byte PTCD1       :1;                                       /* Port C Data Register Bit 1 */
    byte PTCD2       :1;                                       /* Port C Data Register Bit 2 */
    byte PTCD3       :1;                                       /* Port C Data Register Bit 3 */
    byte PTCD4       :1;                                       /* Port C Data Register Bit 4 */
    byte PTCD5       :1;                                       /* Port C Data Register Bit 5 */
    byte PTCD6       :1;                                       /* Port C Data Register Bit 6 */
    byte             :1; 
  } Bits;
  struct {
    byte grpPTCD :7;
    byte         :1;
  } MergedBits;
} PTCDSTR;
extern volatile PTCDSTR _PTCD @0x00000004;
#define PTCD                            _PTCD.Byte
#define PTCD_PTCD0                      _PTCD.Bits.PTCD0
#define PTCD_PTCD1                      _PTCD.Bits.PTCD1
#define PTCD_PTCD2                      _PTCD.Bits.PTCD2
#define PTCD_PTCD3                      _PTCD.Bits.PTCD3
#define PTCD_PTCD4                      _PTCD.Bits.PTCD4
#define PTCD_PTCD5                      _PTCD.Bits.PTCD5
#define PTCD_PTCD6                      _PTCD.Bits.PTCD6
#define PTCD_PTCD                       _PTCD.MergedBits.grpPTCD

#define PTCD_PTCD0_MASK                 1U
#define PTCD_PTCD1_MASK                 2U
#define PTCD_PTCD2_MASK                 4U
#define PTCD_PTCD3_MASK                 8U
#define PTCD_PTCD4_MASK                 16U
#define PTCD_PTCD5_MASK                 32U
#define PTCD_PTCD6_MASK                 64U
#define PTCD_PTCD_MASK                  127U
#define PTCD_PTCD_BITNUM                0U


/*** PTCDD - Port C Data Direction Register; 0x00000005 ***/
typedef union {
  byte Byte;
  struct {
    byte PTCDD0      :1;                                       /* Data Direction for Port C Bit 0 */
    byte PTCDD1      :1;                                       /* Data Direction for Port C Bit 1 */
    byte PTCDD2      :1;                                       /* Data Direction for Port C Bit 2 */
    byte PTCDD3      :1;                                       /* Data Direction for Port C Bit 3 */
    byte PTCDD4      :1;                                       /* Data Direction for Port C Bit 4 */
    byte PTCDD5      :1;                                       /* Data Direction for Port C Bit 5 */
    byte PTCDD6      :1;                                       /* Data Direction for Port C Bit 6 */
    byte             :1; 
  } Bits;
  struct {
    byte grpPTCDD :7;
    byte         :1;
  } MergedBits;
} PTCDDSTR;
extern volatile PTCDDSTR _PTCDD @0x00000005;
#define PTCDD                           _PTCDD.Byte
#define PTCDD_PTCDD0                    _PTCDD.Bits.PTCDD0
#define PTCDD_PTCDD1                    _PTCDD.Bits.PTCDD1
#define PTCDD_PTCDD2                    _PTCDD.Bits.PTCDD2
#define PTCDD_PTCDD3                    _PTCDD.Bits.PTCDD3
#define PTCDD_PTCDD4                    _PTCDD.Bits.PTCDD4
#define PTCDD_PTCDD5                    _PTCDD.Bits.PTCDD5
#define PTCDD_PTCDD6                    _PTCDD.Bits.PTCDD6
#define PTCDD_PTCDD                     _PTCDD.MergedBits.grpPTCDD

#define PTCDD_PTCDD0_MASK               1U
#define PTCDD_PTCDD1_MASK               2U
#define PTCDD_PTCDD2_MASK               4U
#define PTCDD_PTCDD3_MASK               8U
#define PTCDD_PTCDD4_MASK               16U
#define PTCDD_PTCDD5_MASK               32U
#define PTCDD_PTCDD6_MASK               64U
#define PTCDD_PTCDD_MASK                127U
#define PTCDD_PTCDD_BITNUM              0U


/*** PTDD - Port D Data Register; 0x00000006 ***/
typedef union {
  byte Byte;
  struct {
    byte PTDD0       :1;                                       /* Port D Data Register Bit 0 */
    byte PTDD1       :1;                                       /* Port D Data Register Bit 1 */
    byte PTDD2       :1;                                       /* Port D Data Register Bit 2 */
    byte PTDD3       :1;                                       /* Port D Data Register Bit 3 */
    byte PTDD4       :1;                                       /* Port D Data Register Bit 4 */
    byte PTDD5       :1;                                       /* Port D Data Register Bit 5 */
    byte PTDD6       :1;                                       /* Port D Data Register Bit 6 */
    byte PTDD7       :1;                                       /* Port D Data Register Bit 7 */
  } Bits;
} PTDDSTR;
extern volatile PTDDSTR _PTDD @0x00000006;
#define PTDD                            _PTDD.Byte
#define PTDD_PTDD0                      _PTDD.Bits.PTDD0
#define PTDD_PTDD1                      _PTDD.Bits.PTDD1
#define PTDD_PTDD2                      _PTDD.Bits.PTDD2
#define PTDD_PTDD3                      _PTDD.Bits.PTDD3
#define PTDD_PTDD4                      _PTDD.Bits.PTDD4
#define PTDD_PTDD5                      _PTDD.Bits.PTDD5
#define PTDD_PTDD6                      _PTDD.Bits.PTDD6
#define PTDD_PTDD7                      _PTDD.Bits.PTDD7

#define PTDD_PTDD0_MASK                 1U
#define PTDD_PTDD1_MASK                 2U
#define PTDD_PTDD2_MASK                 4U
#define PTDD_PTDD3_MASK                 8U
#define PTDD_PTDD4_MASK                 16U
#define PTDD_PTDD5_MASK                 32U
#define PTDD_PTDD6_MASK                 64U
#define PTDD_PTDD7_MASK                 128U


/*** PTDDD - Port D Data Direction Register; 0x00000007 ***/
typedef union {
  byte Byte;
  struct {
    byte PTDDD0      :1;                                       /* Data Direction for Port D Bit 0 */
    byte PTDDD1      :1;                                       /* Data Direction for Port D Bit 1 */
    byte PTDDD2      :1;                                       /* Data Direction for Port D Bit 2 */
    byte PTDDD3      :1;                                       /* Data Direction for Port D Bit 3 */
    byte PTDDD4      :1;                                       /* Data Direction for Port D Bit 4 */
    byte PTDDD5      :1;                                       /* Data Direction for Port D Bit 5 */
    byte PTDDD6      :1;                                       /* Data Direction for Port D Bit 6 */
    byte PTDDD7      :1;                                       /* Data Direction for Port D Bit 7 */
  } Bits;
} PTDDDSTR;
extern volatile PTDDDSTR _PTDDD @0x00000007;
#define PTDDD                           _PTDDD.Byte
#define PTDDD_PTDDD0                    _PTDDD.Bits.PTDDD0
#define PTDDD_PTDDD1                    _PTDDD.Bits.PTDDD1
#define PTDDD_PTDDD2                    _PTDDD.Bits.PTDDD2
#define PTDDD_PTDDD3                    _PTDDD.Bits.PTDDD3
#define PTDDD_PTDDD4                    _PTDDD.Bits.PTDDD4
#define PTDDD_PTDDD5                    _PTDDD.Bits.PTDDD5
#define PTDDD_PTDDD6                    _PTDDD.Bits.PTDDD6
#define PTDDD_PTDDD7                    _PTDDD.Bits.PTDDD7

#define PTDDD_PTDDD0_MASK               1U
#define PTDDD_PTDDD1_MASK               2U
#define PTDDD_PTDDD2_MASK               4U
#define PTDDD_PTDDD3_MASK               8U
#define PTDDD_PTDDD4_MASK               16U
#define PTDDD_PTDDD5_MASK               32U
#define PTDDD_PTDDD6_MASK               64U
#define PTDDD_PTDDD7_MASK               128U


/*** PTED - Port E Data Register; 0x00000008 ***/
typedef union {
  byte Byte;
  struct {
    byte PTED0       :1;                                       /* Port E Data Register Bit 0 */
    byte PTED1       :1;                                       /* Port E Data Register Bit 1 */
    byte PTED2       :1;                                       /* Port E Data Register Bit 2 */
    byte PTED3       :1;                                       /* Port E Data Register Bit 3 */
    byte PTED4       :1;                                       /* Port E Data Register Bit 4 */
    byte PTED5       :1;                                       /* Port E Data Register Bit 5 */
    byte PTED6       :1;                                       /* Port E Data Register Bit 6 */
    byte PTED7       :1;                                       /* Port E Data Register Bit 7 */
  } Bits;
} PTEDSTR;
extern volatile PTEDSTR _PTED @0x00000008;
#define PTED                            _PTED.Byte
#define PTED_PTED0                      _PTED.Bits.PTED0
#define PTED_PTED1                      _PTED.Bits.PTED1
#define PTED_PTED2                      _PTED.Bits.PTED2
#define PTED_PTED3                      _PTED.Bits.PTED3
#define PTED_PTED4                      _PTED.Bits.PTED4
#define PTED_PTED5                      _PTED.Bits.PTED5
#define PTED_PTED6                      _PTED.Bits.PTED6
#define PTED_PTED7                      _PTED.Bits.PTED7

#define PTED_PTED0_MASK                 1U
#define PTED_PTED1_MASK                 2U
#define PTED_PTED2_MASK                 4U
#define PTED_PTED3_MASK                 8U
#define PTED_PTED4_MASK                 16U
#define PTED_PTED5_MASK                 32U
#define PTED_PTED6_MASK                 64U
#define PTED_PTED7_MASK                 128U


/*** PTEDD - Port E Data Direction Register; 0x00000009 ***/
typedef union {
  byte Byte;
  struct {
    byte PTEDD0      :1;                                       /* Data Direction for Port E Bit 0 */
    byte PTEDD1      :1;                                       /* Data Direction for Port E Bit 1 */
    byte PTEDD2      :1;                                       /* Data Direction for Port E Bit 2 */
    byte PTEDD3      :1;                                       /* Data Direction for Port E Bit 3 */
    byte PTEDD4      :1;                                       /* Data Direction for Port E Bit 4 */
    byte PTEDD5      :1;                                       /* Data Direction for Port E Bit 5 */
    byte PTEDD6      :1;                                       /* Data Direction for Port E Bit 6 */
    byte PTEDD7      :1;                                       /* Data Direction for Port E Bit 7 */
  } Bits;
} PTEDDSTR;
extern volatile PTEDDSTR _PTEDD @0x00000009;
#define PTEDD                           _PTEDD.Byte
#define PTEDD_PTEDD0                    _PTEDD.Bits.PTEDD0
#define PTEDD_PTEDD1                    _PTEDD.Bits.PTEDD1
#define PTEDD_PTEDD2                    _PTEDD.Bits.PTEDD2
#define PTEDD_PTEDD3                    _PTEDD.Bits.PTEDD3
#define PTEDD_PTEDD4                    _PTEDD.Bits.PTEDD4
#define PTEDD_PTEDD5                    _PTEDD.Bits.PTEDD5
#define PTEDD_PTEDD6                    _PTEDD.Bits.PTEDD6
#define PTEDD_PTEDD7                    _PTEDD.Bits.PTEDD7

#define PTEDD_PTEDD0_MASK               1U
#define PTEDD_PTEDD1_MASK               2U
#define PTEDD_PTEDD2_MASK               4U
#define PTEDD_PTEDD3_MASK               8U
#define PTEDD_PTEDD4_MASK               16U
#define PTEDD_PTEDD5_MASK               32U
#define PTEDD_PTEDD6_MASK               64U
#define PTEDD_PTEDD7_MASK               128U


/*** PTFD - Port F Data Register; 0x0000000A ***/
typedef union {
  byte Byte;
  struct {
    byte PTFD0       :1;                                       /* Port F Data Register Bit 0 */
    byte PTFD1       :1;                                       /* Port F Data Register Bit 1 */
    byte PTFD2       :1;                                       /* Port F Data Register Bit 2 */
    byte PTFD3       :1;                                       /* Port F Data Register Bit 3 */
    byte PTFD4       :1;                                       /* Port F Data Register Bit 4 */
    byte PTFD5       :1;                                       /* Port F Data Register Bit 5 */
    byte PTFD6       :1;                                       /* Port F Data Register Bit 6 */
    byte PTFD7       :1;                                       /* Port F Data Register Bit 7 */
  } Bits;
} PTFDSTR;
extern volatile PTFDSTR _PTFD @0x0000000A;
#define PTFD                            _PTFD.Byte
#define PTFD_PTFD0                      _PTFD.Bits.PTFD0
#define PTFD_PTFD1                      _PTFD.Bits.PTFD1
#define PTFD_PTFD2                      _PTFD.Bits.PTFD2
#define PTFD_PTFD3                      _PTFD.Bits.PTFD3
#define PTFD_PTFD4                      _PTFD.Bits.PTFD4
#define PTFD_PTFD5                      _PTFD.Bits.PTFD5
#define PTFD_PTFD6                      _PTFD.Bits.PTFD6
#define PTFD_PTFD7                      _PTFD.Bits.PTFD7

#define PTFD_PTFD0_MASK                 1U
#define PTFD_PTFD1_MASK                 2U
#define PTFD_PTFD2_MASK                 4U
#define PTFD_PTFD3_MASK                 8U
#define PTFD_PTFD4_MASK                 16U
#define PTFD_PTFD5_MASK                 32U
#define PTFD_PTFD6_MASK                 64U
#define PTFD_PTFD7_MASK                 128U


/*** PTFDD - Port F Data Direction Register; 0x0000000B ***/
typedef union {
  byte Byte;
  struct {
    byte PTFDD0      :1;                                       /* Data Direction for Port F Bit 0 */
    byte PTFDD1      :1;                                       /* Data Direction for Port F Bit 1 */
    byte PTFDD2      :1;                                       /* Data Direction for Port F Bit 2 */
    byte PTFDD3      :1;                                       /* Data Direction for Port F Bit 3 */
    byte PTFDD4      :1;                                       /* Data Direction for Port F Bit 4 */
    byte PTFDD5      :1;                                       /* Data Direction for Port F Bit 5 */
    byte PTFDD6      :1;                                       /* Data Direction for Port F Bit 6 */
    byte PTFDD7      :1;                                       /* Data Direction for Port F Bit 7 */
  } Bits;
} PTFDDSTR;
extern volatile PTFDDSTR _PTFDD @0x0000000B;
#define PTFDD                           _PTFDD.Byte
#define PTFDD_PTFDD0                    _PTFDD.Bits.PTFDD0
#define PTFDD_PTFDD1                    _PTFDD.Bits.PTFDD1
#define PTFDD_PTFDD2                    _PTFDD.Bits.PTFDD2
#define PTFDD_PTFDD3                    _PTFDD.Bits.PTFDD3
#define PTFDD_PTFDD4                    _PTFDD.Bits.PTFDD4
#define PTFDD_PTFDD5                    _PTFDD.Bits.PTFDD5
#define PTFDD_PTFDD6                    _PTFDD.Bits.PTFDD6
#define PTFDD_PTFDD7                    _PTFDD.Bits.PTFDD7

#define PTFDD_PTFDD0_MASK               1U
#define PTFDD_PTFDD1_MASK               2U
#define PTFDD_PTFDD2_MASK               4U
#define PTFDD_PTFDD3_MASK               8U
#define PTFDD_PTFDD4_MASK               16U
#define PTFDD_PTFDD5_MASK               32U
#define PTFDD_PTFDD6_MASK               64U
#define PTFDD_PTFDD7_MASK               128U


/*** PTGD - Port G Data Register; 0x0000000C ***/
typedef union {
  byte Byte;
  struct {
    byte PTGD0       :1;                                       /* Port G Data Register Bit 0 */
    byte PTGD1       :1;                                       /* Port G Data Register Bit 1 */
    byte PTGD2       :1;                                       /* Port G Data Register Bit 2 */
    byte PTGD3       :1;                                       /* Port G Data Register Bit 3 */
    byte PTGD4       :1;                                       /* Port G Data Register Bit 4 */
    byte PTGD5       :1;                                       /* Port G Data Register Bit 5 */
    byte PTGD6       :1;                                       /* Port G Data Register Bit 6 */
    byte             :1; 
  } Bits;
  struct {
    byte grpPTGD :7;
    byte         :1;
  } MergedBits;
} PTGDSTR;
extern volatile PTGDSTR _PTGD @0x0000000C;
#define PTGD                            _PTGD.Byte
#define PTGD_PTGD0                      _PTGD.Bits.PTGD0
#define PTGD_PTGD1                      _PTGD.Bits.PTGD1
#define PTGD_PTGD2                      _PTGD.Bits.PTGD2
#define PTGD_PTGD3                      _PTGD.Bits.PTGD3
#define PTGD_PTGD4                      _PTGD.Bits.PTGD4
#define PTGD_PTGD5                      _PTGD.Bits.PTGD5
#define PTGD_PTGD6                      _PTGD.Bits.PTGD6
#define PTGD_PTGD                       _PTGD.MergedBits.grpPTGD

#define PTGD_PTGD0_MASK                 1U
#define PTGD_PTGD1_MASK                 2U
#define PTGD_PTGD2_MASK                 4U
#define PTGD_PTGD3_MASK                 8U
#define PTGD_PTGD4_MASK                 16U
#define PTGD_PTGD5_MASK                 32U
#define PTGD_PTGD6_MASK                 64U
#define PTGD_PTGD_MASK                  127U
#define PTGD_PTGD_BITNUM                0U


/*** PTGDD - Port G Data Direction Register; 0x0000000D ***/
typedef union {
  byte Byte;
  struct {
    byte PTGDD0      :1;                                       /* Data Direction for Port G Bit 0 */
    byte PTGDD1      :1;                                       /* Data Direction for Port G Bit 1 */
    byte PTGDD2      :1;                                       /* Data Direction for Port G Bit 2 */
    byte PTGDD3      :1;                                       /* Data Direction for Port G Bit 3 */
    byte PTGDD4      :1;                                       /* Data Direction for Port G Bit 4 */
    byte PTGDD5      :1;                                       /* Data Direction for Port G Bit 5 */
    byte PTGDD6      :1;                                       /* Data Direction for Port G Bit 6 */
    byte             :1; 
  } Bits;
  struct {
    byte grpPTGDD :7;
    byte         :1;
  } MergedBits;
} PTGDDSTR;
extern volatile PTGDDSTR _PTGDD @0x0000000D;
#define PTGDD                           _PTGDD.Byte
#define PTGDD_PTGDD0                    _PTGDD.Bits.PTGDD0
#define PTGDD_PTGDD1                    _PTGDD.Bits.PTGDD1
#define PTGDD_PTGDD2                    _PTGDD.Bits.PTGDD2
#define PTGDD_PTGDD3                    _PTGDD.Bits.PTGDD3
#define PTGDD_PTGDD4                    _PTGDD.Bits.PTGDD4
#define PTGDD_PTGDD5                    _PTGDD.Bits.PTGDD5
#define PTGDD_PTGDD6                    _PTGDD.Bits.PTGDD6
#define PTGDD_PTGDD                     _PTGDD.MergedBits.grpPTGDD

#define PTGDD_PTGDD0_MASK               1U
#define PTGDD_PTGDD1_MASK               2U
#define PTGDD_PTGDD2_MASK               4U
#define PTGDD_PTGDD3_MASK               8U
#define PTGDD_PTGDD4_MASK               16U
#define PTGDD_PTGDD5_MASK               32U
#define PTGDD_PTGDD6_MASK               64U
#define PTGDD_PTGDD_MASK                127U
#define PTGDD_PTGDD_BITNUM              0U


/*** ADC1SC1 - Status and Control Register 1; 0x00000010 ***/
typedef union {
  byte Byte;
  struct {
    byte ADCH0       :1;                                       /* Input Channel Select Bit 0 */
    byte ADCH1       :1;                                       /* Input Channel Select Bit 1 */
    byte ADCH2       :1;                                       /* Input Channel Select Bit 2 */
    byte ADCH3       :1;                                       /* Input Channel Select Bit 3 */
    byte ADCH4       :1;                                       /* Input Channel Select Bit 4 */
    byte ADCO        :1;                                       /* Continuous Conversion Enable - ADCO is used to enable continuous conversions */
    byte AIEN        :1;                                       /* Interrupt Enable - AIEN is used to enable conversion complete interrupts. When COCO becomes set while AIEN is high, an interrupt is asserted */
    byte COCO        :1;                                       /* Conversion Complete Flag */
  } Bits;
  struct {
    byte grpADCH :5;
    byte         :1;
    byte         :1;
    byte         :1;
  } MergedBits;
} ADC1SC1STR;
extern volatile ADC1SC1STR _ADC1SC1 @0x00000010;
#define ADC1SC1                         _ADC1SC1.Byte
#define ADC1SC1_ADCH0                   _ADC1SC1.Bits.ADCH0
#define ADC1SC1_ADCH1                   _ADC1SC1.Bits.ADCH1
#define ADC1SC1_ADCH2                   _ADC1SC1.Bits.ADCH2
#define ADC1SC1_ADCH3                   _ADC1SC1.Bits.ADCH3
#define ADC1SC1_ADCH4                   _ADC1SC1.Bits.ADCH4
#define ADC1SC1_ADCO                    _ADC1SC1.Bits.ADCO
#define ADC1SC1_AIEN                    _ADC1SC1.Bits.AIEN
#define ADC1SC1_COCO                    _ADC1SC1.Bits.COCO
#define ADC1SC1_ADCH                    _ADC1SC1.MergedBits.grpADCH

#define ADC1SC1_ADCH0_MASK              1U
#define ADC1SC1_ADCH1_MASK              2U
#define ADC1SC1_ADCH2_MASK              4U
#define ADC1SC1_ADCH3_MASK              8U
#define ADC1SC1_ADCH4_MASK              16U
#define ADC1SC1_ADCO_MASK               32U
#define ADC1SC1_AIEN_MASK               64U
#define ADC1SC1_COCO_MASK               128U
#define ADC1SC1_ADCH_MASK               31U
#define ADC1SC1_ADCH_BITNUM             0U


/*** ADC1SC2 - Status and Control Register 2; 0x00000011 ***/
typedef union {
  byte Byte;
  struct {
    byte             :1; 
    byte             :1; 
    byte             :1; 
    byte             :1; 
    byte ACFGT       :1;                                       /* Compare Function Greater Than Enable */
    byte ACFE        :1;                                       /* Compare Function Enable - ACFE is used to enable the compare function */
    byte ADTRG       :1;                                       /* Conversion Trigger Select-ADTRG is used to select the type of trigger to be used for initiating a conversion */
    byte ADACT       :1;                                       /* Conversion Active - ADACT indicates that a conversion is in progress. ADACT is set when a conversion is initiated and cleared when a conversion is completed or aborted */
  } Bits;
} ADC1SC2STR;
extern volatile ADC1SC2STR _ADC1SC2 @0x00000011;
#define ADC1SC2                         _ADC1SC2.Byte
#define ADC1SC2_ACFGT                   _ADC1SC2.Bits.ACFGT
#define ADC1SC2_ACFE                    _ADC1SC2.Bits.ACFE
#define ADC1SC2_ADTRG                   _ADC1SC2.Bits.ADTRG
#define ADC1SC2_ADACT                   _ADC1SC2.Bits.ADACT

#define ADC1SC2_ACFGT_MASK              16U
#define ADC1SC2_ACFE_MASK               32U
#define ADC1SC2_ADTRG_MASK              64U
#define ADC1SC2_ADACT_MASK              128U


/*** ADC1R - Data Result Register; 0x00000012 ***/
typedef union {
  word Word;
   /* Overlapped registers: */
  struct {
    /*** ADC1RH - Data Result High Register; 0x00000012 ***/
    union {
      byte Byte;
      struct {
        byte ADR8        :1;                                       /* ADC Result Data Bit 8 */
        byte ADR9        :1;                                       /* ADC Result Data Bit 9 */
        byte             :1; 
        byte             :1; 
        byte             :1; 
        byte             :1; 
        byte             :1; 
        byte             :1; 
      } Bits;
      struct {
        byte grpADR_8 :2;
        byte     :1;
        byte     :1;
        byte     :1;
        byte     :1;
        byte     :1;
        byte     :1;
      } MergedBits;
    } ADC1RHSTR;
    #define ADC1RH                      _ADC1R.Overlap_STR.ADC1RHSTR.Byte
    #define ADC1RH_ADR8                 _ADC1R.Overlap_STR.ADC1RHSTR.Bits.ADR8
    #define ADC1RH_ADR9                 _ADC1R.Overlap_STR.ADC1RHSTR.Bits.ADR9
    #define ADC1RH_ADR_8                _ADC1R.Overlap_STR.ADC1RHSTR.MergedBits.grpADR_8
    #define ADC1RH_ADR                  ADC1RH_ADR_8
    
    #define ADC1RH_ADR8_MASK            1U
    #define ADC1RH_ADR9_MASK            2U
    #define ADC1RH_ADR_8_MASK           3U
    #define ADC1RH_ADR_8_BITNUM         0U
    

    /*** ADC1RL - Data Result Low Register; 0x00000013 ***/
    union {
      byte Byte;
      struct {
        byte ADR0        :1;                                       /* ADC Result Data Bit 0 */
        byte ADR1        :1;                                       /* ADC Result Data Bit 1 */
        byte ADR2        :1;                                       /* ADC Result Data Bit 2 */
        byte ADR3        :1;                                       /* ADC Result Data Bit 3 */
        byte ADR4        :1;                                       /* ADC Result Data Bit 4 */
        byte ADR5        :1;                                       /* ADC Result Data Bit 5 */
        byte ADR6        :1;                                       /* ADC Result Data Bit 6 */
        byte ADR7        :1;                                       /* ADC Result Data Bit 7 */
      } Bits;
    } ADC1RLSTR;
    #define ADC1RL                      _ADC1R.Overlap_STR.ADC1RLSTR.Byte
    #define ADC1RL_ADR0                 _ADC1R.Overlap_STR.ADC1RLSTR.Bits.ADR0
    #define ADC1RL_ADR1                 _ADC1R.Overlap_STR.ADC1RLSTR.Bits.ADR1
    #define ADC1RL_ADR2                 _ADC1R.Overlap_STR.ADC1RLSTR.Bits.ADR2
    #define ADC1RL_ADR3                 _ADC1R.Overlap_STR.ADC1RLSTR.Bits.ADR3
    #define ADC1RL_ADR4                 _ADC1R.Overlap_STR.ADC1RLSTR.Bits.ADR4
    #define ADC1RL_ADR5                 _ADC1R.Overlap_STR.ADC1RLSTR.Bits.ADR5
    #define ADC1RL_ADR6                 _ADC1R.Overlap_STR.ADC1RLSTR.Bits.ADR6
    #define ADC1RL_ADR7                 _ADC1R.Overlap_STR.ADC1RLSTR.Bits.ADR7
    
    #define ADC1RL_ADR0_MASK            1U
    #define ADC1RL_ADR1_MASK            2U
    #define ADC1RL_ADR2_MASK            4U
    #define ADC1RL_ADR3_MASK            8U
    #define ADC1RL_ADR4_MASK            16U
    #define ADC1RL_ADR5_MASK            32U
    #define ADC1RL_ADR6_MASK            64U
    #define ADC1RL_ADR7_MASK            128U
    
  } Overlap_STR;

} ADC1RSTR;
extern volatile ADC1RSTR _ADC1R @0x00000012;
#define ADC1R                           _ADC1R.Word


/*** ADC1CV - Compare Value Register; 0x00000014 ***/
typedef union {
  word Word;
   /* Overlapped registers: */
  struct {
    /*** ADC1CVH - Compare Value High Register; 0x00000014 ***/
    union {
      byte Byte;
      struct {
        byte ADCV8       :1;                                       /* Compare Function Value 8 */
        byte ADCV9       :1;                                       /* Compare Function Value 9 */
        byte             :1; 
        byte             :1; 
        byte             :1; 
        byte             :1; 
        byte             :1; 
        byte             :1; 
      } Bits;
      struct {
        byte grpADCV_8 :2;
        byte     :1;
        byte     :1;
        byte     :1;
        byte     :1;
        byte     :1;
        byte     :1;
      } MergedBits;
    } ADC1CVHSTR;
    #define ADC1CVH                     _ADC1CV.Overlap_STR.ADC1CVHSTR.Byte
    #define ADC1CVH_ADCV8               _ADC1CV.Overlap_STR.ADC1CVHSTR.Bits.ADCV8
    #define ADC1CVH_ADCV9               _ADC1CV.Overlap_STR.ADC1CVHSTR.Bits.ADCV9
    #define ADC1CVH_ADCV_8              _ADC1CV.Overlap_STR.ADC1CVHSTR.MergedBits.grpADCV_8
    #define ADC1CVH_ADCV                ADC1CVH_ADCV_8
    
    #define ADC1CVH_ADCV8_MASK          1U
    #define ADC1CVH_ADCV9_MASK          2U
    #define ADC1CVH_ADCV_8_MASK         3U
    #define ADC1CVH_ADCV_8_BITNUM       0U
    

    /*** ADC1CVL - Compare Value Low Register; 0x00000015 ***/
    union {
      byte Byte;
      struct {
        byte ADCV0       :1;                                       /* Compare Function Value 0 */
        byte ADCV1       :1;                                       /* Compare Function Value 1 */
        byte ADCV2       :1;                                       /* Compare Function Value 2 */
        byte ADCV3       :1;                                       /* Compare Function Value 3 */
        byte ADCV4       :1;                                       /* Compare Function Value 4 */
        byte ADCV5       :1;                                       /* Compare Function Value 5 */
        byte ADCV6       :1;                                       /* Compare Function Value 6 */
        byte ADCV7       :1;                                       /* Compare Function Value 7 */
      } Bits;
    } ADC1CVLSTR;
    #define ADC1CVL                     _ADC1CV.Overlap_STR.ADC1CVLSTR.Byte
    #define ADC1CVL_ADCV0               _ADC1CV.Overlap_STR.ADC1CVLSTR.Bits.ADCV0
    #define ADC1CVL_ADCV1               _ADC1CV.Overlap_STR.ADC1CVLSTR.Bits.ADCV1
    #define ADC1CVL_ADCV2               _ADC1CV.Overlap_STR.ADC1CVLSTR.Bits.ADCV2
    #define ADC1CVL_ADCV3               _ADC1CV.Overlap_STR.ADC1CVLSTR.Bits.ADCV3
    #define ADC1CVL_ADCV4               _ADC1CV.Overlap_STR.ADC1CVLSTR.Bits.ADCV4
    #define ADC1CVL_ADCV5               _ADC1CV.Overlap_STR.ADC1CVLSTR.Bits.ADCV5
    #define ADC1CVL_ADCV6               _ADC1CV.Overlap_STR.ADC1CVLSTR.Bits.ADCV6
    #define ADC1CVL_ADCV7               _ADC1CV.Overlap_STR.ADC1CVLSTR.Bits.ADCV7
    
    #define ADC1CVL_ADCV0_MASK          1U
    #define ADC1CVL_ADCV1_MASK          2U
    #define ADC1CVL_ADCV2_MASK          4U
    #define ADC1CVL_ADCV3_MASK          8U
    #define ADC1CVL_ADCV4_MASK          16U
    #define ADC1CVL_ADCV5_MASK          32U
    #define ADC1CVL_ADCV6_MASK          64U
    #define ADC1CVL_ADCV7_MASK          128U
    
  } Overlap_STR;

} ADC1CVSTR;
extern volatile ADC1CVSTR _ADC1CV @0x00000014;
#define ADC1CV                          _ADC1CV.Word


/*** ADC1CFG - Configuration Register; 0x00000016 ***/
typedef union {
  byte Byte;
  struct {
    byte ADICLK0     :1;                                       /* Input Clock Select Bit 0 */
    byte ADICLK1     :1;                                       /* Input Clock Select Bit 1 */
    byte MODE0       :1;                                       /* Conversion Mode Selection Bit 0 */
    byte MODE1       :1;                                       /* Conversion Mode Selection Bit 1 */
    byte ADLSMP      :1;                                       /* Long Sample Time Configuration */
    byte ADIV0       :1;                                       /* Clock Divide Select Bit 0 */
    byte ADIV1       :1;                                       /* Clock Divide Select Bit 1 */
    byte ADLPC       :1;                                       /* Low Power Configuration */
  } Bits;
  struct {
    byte grpADICLK :2;
    byte grpMODE :2;
    byte         :1;
    byte grpADIV :2;
    byte         :1;
  } MergedBits;
} ADC1CFGSTR;
extern volatile ADC1CFGSTR _ADC1CFG @0x00000016;
#define ADC1CFG                         _ADC1CFG.Byte
#define ADC1CFG_ADICLK0                 _ADC1CFG.Bits.ADICLK0
#define ADC1CFG_ADICLK1                 _ADC1CFG.Bits.ADICLK1
#define ADC1CFG_MODE0                   _ADC1CFG.Bits.MODE0
#define ADC1CFG_MODE1                   _ADC1CFG.Bits.MODE1
#define ADC1CFG_ADLSMP                  _ADC1CFG.Bits.ADLSMP
#define ADC1CFG_ADIV0                   _ADC1CFG.Bits.ADIV0
#define ADC1CFG_ADIV1                   _ADC1CFG.Bits.ADIV1
#define ADC1CFG_ADLPC                   _ADC1CFG.Bits.ADLPC
#define ADC1CFG_ADICLK                  _ADC1CFG.MergedBits.grpADICLK
#define ADC1CFG_MODE                    _ADC1CFG.MergedBits.grpMODE
#define ADC1CFG_ADIV                    _ADC1CFG.MergedBits.grpADIV

#define ADC1CFG_ADICLK0_MASK            1U
#define ADC1CFG_ADICLK1_MASK            2U
#define ADC1CFG_MODE0_MASK              4U
#define ADC1CFG_MODE1_MASK              8U
#define ADC1CFG_ADLSMP_MASK             16U
#define ADC1CFG_ADIV0_MASK              32U
#define ADC1CFG_ADIV1_MASK              64U
#define ADC1CFG_ADLPC_MASK              128U
#define ADC1CFG_ADICLK_MASK             3U
#define ADC1CFG_ADICLK_BITNUM           0U
#define ADC1CFG_MODE_MASK               12U
#define ADC1CFG_MODE_BITNUM             2U
#define ADC1CFG_ADIV_MASK               96U
#define ADC1CFG_ADIV_BITNUM             5U


/*** APCTL1 - Pin Control 1 Register; 0x00000017 ***/
typedef union {
  byte Byte;
  struct {
    byte ADPC0       :1;                                       /* ADC Pin Control 0 - ADPC0 is used to control the pin associated with channel AD0 */
    byte ADPC1       :1;                                       /* ADC Pin Control 1 - ADPC1 is used to control the pin associated with channel AD1 */
    byte ADPC2       :1;                                       /* ADC Pin Control 2 - ADPC2 is used to control the pin associated with channel AD2 */
    byte ADPC3       :1;                                       /* ADC Pin Control 3 - ADPC3 is used to control the pin associated with channel AD3 */
    byte ADPC4       :1;                                       /* ADC Pin Control 4 - ADPC4 is used to control the pin associated with channel AD4 */
    byte ADPC5       :1;                                       /* ADC Pin Control 5 - ADPC5 is used to control the pin associated with channel AD5 */
    byte ADPC6       :1;                                       /* ADC Pin Control 6 - ADPC6 is used to control the pin associated with channel AD6 */
    byte ADPC7       :1;                                       /* ADC Pin Control 7 - ADPC7 is used to control the pin associated with channel AD7 */
  } Bits;
} APCTL1STR;
extern volatile APCTL1STR _APCTL1 @0x00000017;
#define APCTL1                          _APCTL1.Byte
#define APCTL1_ADPC0                    _APCTL1.Bits.ADPC0
#define APCTL1_ADPC1                    _APCTL1.Bits.ADPC1
#define APCTL1_ADPC2                    _APCTL1.Bits.ADPC2
#define APCTL1_ADPC3                    _APCTL1.Bits.ADPC3
#define APCTL1_ADPC4                    _APCTL1.Bits.ADPC4
#define APCTL1_ADPC5                    _APCTL1.Bits.ADPC5
#define APCTL1_ADPC6                    _APCTL1.Bits.ADPC6
#define APCTL1_ADPC7                    _APCTL1.Bits.ADPC7

#define APCTL1_ADPC0_MASK               1U
#define APCTL1_ADPC1_MASK               2U
#define APCTL1_ADPC2_MASK               4U
#define APCTL1_ADPC3_MASK               8U
#define APCTL1_ADPC4_MASK               16U
#define APCTL1_ADPC5_MASK               32U
#define APCTL1_ADPC6_MASK               64U
#define APCTL1_ADPC7_MASK               128U


/*** APCTL2 - Pin Control 2 Register; 0x00000018 ***/
typedef union {
  byte Byte;
  struct {
    byte ADPC8       :1;                                       /* ADC Pin Control 8 - ADPC8 is used to control the pin associated with channel AD8 */
    byte ADPC9       :1;                                       /* ADC Pin Control 9 - ADPC9 is used to control the pin associated with channel AD9 */
    byte ADPC10      :1;                                       /* ADC Pin Control 10 - ADPC10 is used to control the pin associated with channel AD10 */
    byte ADPC11      :1;                                       /* ADC Pin Control 11 - ADPC11 is used to control the pin associated with channel AD11 */
    byte ADPC12      :1;                                       /* ADC Pin Control 12 - ADPC12 is used to control the pin associated with channel AD12 */
    byte ADPC13      :1;                                       /* ADC Pin Control 13 - ADPC13 is used to control the pin associated with channel AD13 */
    byte ADPC14      :1;                                       /* ADC Pin Control 14 - ADPC14 is used to control the pin associated with channel AD14 */
    byte ADPC15      :1;                                       /* ADC Pin Control 15 - ADPC15 is used to control the pin associated with channel AD15 */
  } Bits;
} APCTL2STR;
extern volatile APCTL2STR _APCTL2 @0x00000018;
#define APCTL2                          _APCTL2.Byte
#define APCTL2_ADPC8                    _APCTL2.Bits.ADPC8
#define APCTL2_ADPC9                    _APCTL2.Bits.ADPC9
#define APCTL2_ADPC10                   _APCTL2.Bits.ADPC10
#define APCTL2_ADPC11                   _APCTL2.Bits.ADPC11
#define APCTL2_ADPC12                   _APCTL2.Bits.ADPC12
#define APCTL2_ADPC13                   _APCTL2.Bits.ADPC13
#define APCTL2_ADPC14                   _APCTL2.Bits.ADPC14
#define APCTL2_ADPC15                   _APCTL2.Bits.ADPC15

#define APCTL2_ADPC8_MASK               1U
#define APCTL2_ADPC9_MASK               2U
#define APCTL2_ADPC10_MASK              4U
#define APCTL2_ADPC11_MASK              8U
#define APCTL2_ADPC12_MASK              16U
#define APCTL2_ADPC13_MASK              32U
#define APCTL2_ADPC14_MASK              64U
#define APCTL2_ADPC15_MASK              128U


/*** IRQSC - Interrupt request status and control register; 0x0000001C ***/
typedef union {
  byte Byte;
  struct {
    byte IRQMOD      :1;                                       /* IRQ Detection Mode */
    byte IRQIE       :1;                                       /* IRQ Interrupt Enable */
    byte IRQACK      :1;                                       /* IRQ Acknowledge */
    byte IRQF        :1;                                       /* IRQ Flag */
    byte IRQPE       :1;                                       /* IRQ Pin Enable */
    byte IRQEDG      :1;                                       /* IRQ Edge Select */
    byte IRQPDD      :1;                                       /* IRQ Pull Device Disable */
    byte             :1; 
  } Bits;
} IRQSCSTR;
extern volatile IRQSCSTR _IRQSC @0x0000001C;
#define IRQSC                           _IRQSC.Byte
#define IRQSC_IRQMOD                    _IRQSC.Bits.IRQMOD
#define IRQSC_IRQIE                     _IRQSC.Bits.IRQIE
#define IRQSC_IRQACK                    _IRQSC.Bits.IRQACK
#define IRQSC_IRQF                      _IRQSC.Bits.IRQF
#define IRQSC_IRQPE                     _IRQSC.Bits.IRQPE
#define IRQSC_IRQEDG                    _IRQSC.Bits.IRQEDG
#define IRQSC_IRQPDD                    _IRQSC.Bits.IRQPDD

#define IRQSC_IRQMOD_MASK               1U
#define IRQSC_IRQIE_MASK                2U
#define IRQSC_IRQACK_MASK               4U
#define IRQSC_IRQF_MASK                 8U
#define IRQSC_IRQPE_MASK                16U
#define IRQSC_IRQEDG_MASK               32U
#define IRQSC_IRQPDD_MASK               64U


/*** KBISC - KBI Status and Control Register; 0x0000001E ***/
typedef union {
  byte Byte;
  struct {
    byte KBIMOD      :1;                                       /* Keyboard Detection Mode */
    byte KBIE        :1;                                       /* Keyboard Interrupt Enable */
    byte KBACK       :1;                                       /* Keyboard Interrupt Acknowledge */
    byte KBF         :1;                                       /* Keyboard Interrupt Flag */
    byte KBEDG4      :1;                                       /* Keyboard Edge Select for KBI Port Bit 4 */
    byte KBEDG5      :1;                                       /* Keyboard Edge Select for KBI Port Bit 5 */
    byte KBEDG6      :1;                                       /* Keyboard Edge Select for KBI Port Bit 6 */
    byte KBEDG7      :1;                                       /* Keyboard Edge Select for KBI Port Bit 7 */
  } Bits;
  struct {
    byte         :1;
    byte         :1;
    byte         :1;
    byte         :1;
    byte grpKBEDG_4 :4;
  } MergedBits;
} KBISCSTR;
extern volatile KBISCSTR _KBISC @0x0000001E;
#define KBISC                           _KBISC.Byte
#define KBISC_KBIMOD                    _KBISC.Bits.KBIMOD
#define KBISC_KBIE                      _KBISC.Bits.KBIE
#define KBISC_KBACK                     _KBISC.Bits.KBACK
#define KBISC_KBF                       _KBISC.Bits.KBF
#define KBISC_KBEDG4                    _KBISC.Bits.KBEDG4
#define KBISC_KBEDG5                    _KBISC.Bits.KBEDG5
#define KBISC_KBEDG6                    _KBISC.Bits.KBEDG6
#define KBISC_KBEDG7                    _KBISC.Bits.KBEDG7
#define KBISC_KBEDG_4                   _KBISC.MergedBits.grpKBEDG_4
#define KBISC_KBEDG                     KBISC_KBEDG_4

#define KBISC_KBIMOD_MASK               1U
#define KBISC_KBIE_MASK                 2U
#define KBISC_KBACK_MASK                4U
#define KBISC_KBF_MASK                  8U
#define KBISC_KBEDG4_MASK               16U
#define KBISC_KBEDG5_MASK               32U
#define KBISC_KBEDG6_MASK               64U
#define KBISC_KBEDG7_MASK               128U
#define KBISC_KBEDG_4_MASK              240U
#define KBISC_KBEDG_4_BITNUM            4U


/*** KBIPE - KBI Pin Enable Register; 0x0000001F ***/
typedef union {
  byte Byte;
  struct {
    byte KBIPE0      :1;                                       /* KBI Pin Enable for KBI Port Bit 0 */
    byte KBIPE1      :1;                                       /* KBI Pin Enable for KBI Port Bit 1 */
    byte KBIPE2      :1;                                       /* KBI Pin Enable for KBI Port Bit 2 */
    byte KBIPE3      :1;                                       /* KBI Pin Enable for KBI Port Bit 3 */
    byte KBIPE4      :1;                                       /* KBI Pin Enable for KBI Port Bit 4 */
    byte KBIPE5      :1;                                       /* KBI Pin Enable for KBI Port Bit 5 */
    byte KBIPE6      :1;                                       /* KBI Pin Enable for KBI Port Bit 6 */
    byte KBIPE7      :1;                                       /* KBI Pin Enable for KBI Port Bit 7 */
  } Bits;
} KBIPESTR;
extern volatile KBIPESTR _KBIPE @0x0000001F;
#define KBIPE                           _KBIPE.Byte
#define KBIPE_KBIPE0                    _KBIPE.Bits.KBIPE0
#define KBIPE_KBIPE1                    _KBIPE.Bits.KBIPE1
#define KBIPE_KBIPE2                    _KBIPE.Bits.KBIPE2
#define KBIPE_KBIPE3                    _KBIPE.Bits.KBIPE3
#define KBIPE_KBIPE4                    _KBIPE.Bits.KBIPE4
#define KBIPE_KBIPE5                    _KBIPE.Bits.KBIPE5
#define KBIPE_KBIPE6                    _KBIPE.Bits.KBIPE6
#define KBIPE_KBIPE7                    _KBIPE.Bits.KBIPE7

#define KBIPE_KBIPE0_MASK               1U
#define KBIPE_KBIPE1_MASK               2U
#define KBIPE_KBIPE2_MASK               4U
#define KBIPE_KBIPE3_MASK               8U
#define KBIPE_KBIPE4_MASK               16U
#define KBIPE_KBIPE5_MASK               32U
#define KBIPE_KBIPE6_MASK               64U
#define KBIPE_KBIPE7_MASK               128U


/*** TPM1SC - TPM1 Status and Control Register; 0x00000020 ***/
typedef union {
  byte Byte;
  struct {
    byte PS0         :1;                                       /* Prescale Divisor Select Bit 0 */
    byte PS1         :1;                                       /* Prescale Divisor Select Bit 1 */
    byte PS2         :1;                                       /* Prescale Divisor Select Bit 2 */
    byte CLKSA       :1;                                       /* Clock Source Select A */
    byte CLKSB       :1;                                       /* Clock Source Select B */
    byte CPWMS       :1;                                       /* Center-Aligned PWM Select */
    byte TOIE        :1;                                       /* Timer Overflow Interrupt Enable */
    byte TOF         :1;                                       /* Timer Overflow Flag */
  } Bits;
  struct {
    byte grpPS   :3;
    byte grpCLKSx :2;
    byte         :1;
    byte         :1;
    byte         :1;
  } MergedBits;
} TPM1SCSTR;
extern volatile TPM1SCSTR _TPM1SC @0x00000020;
#define TPM1SC                          _TPM1SC.Byte
#define TPM1SC_PS0                      _TPM1SC.Bits.PS0
#define TPM1SC_PS1                      _TPM1SC.Bits.PS1
#define TPM1SC_PS2                      _TPM1SC.Bits.PS2
#define TPM1SC_CLKSA                    _TPM1SC.Bits.CLKSA
#define TPM1SC_CLKSB                    _TPM1SC.Bits.CLKSB
#define TPM1SC_CPWMS                    _TPM1SC.Bits.CPWMS
#define TPM1SC_TOIE                     _TPM1SC.Bits.TOIE
#define TPM1SC_TOF                      _TPM1SC.Bits.TOF
#define TPM1SC_PS                       _TPM1SC.MergedBits.grpPS
#define TPM1SC_CLKSx                    _TPM1SC.MergedBits.grpCLKSx

#define TPM1SC_PS0_MASK                 1U
#define TPM1SC_PS1_MASK                 2U
#define TPM1SC_PS2_MASK                 4U
#define TPM1SC_CLKSA_MASK               8U
#define TPM1SC_CLKSB_MASK               16U
#define TPM1SC_CPWMS_MASK               32U
#define TPM1SC_TOIE_MASK                64U
#define TPM1SC_TOF_MASK                 128U
#define TPM1SC_PS_MASK                  7U
#define TPM1SC_PS_BITNUM                0U
#define TPM1SC_CLKSx_MASK               24U
#define TPM1SC_CLKSx_BITNUM             3U


/*** TPM1CNT - TPM1 Timer Counter Register; 0x00000021 ***/
typedef union {
  word Word;
   /* Overlapped registers: */
  struct {
    /*** TPM1CNTH - TPM1 Timer Counter Register High; 0x00000021 ***/
    union {
      byte Byte;
    } TPM1CNTHSTR;
    #define TPM1CNTH                    _TPM1CNT.Overlap_STR.TPM1CNTHSTR.Byte
    

    /*** TPM1CNTL - TPM1 Timer Counter Register Low; 0x00000022 ***/
    union {
      byte Byte;
    } TPM1CNTLSTR;
    #define TPM1CNTL                    _TPM1CNT.Overlap_STR.TPM1CNTLSTR.Byte
    
  } Overlap_STR;

} TPM1CNTSTR;
extern volatile TPM1CNTSTR _TPM1CNT @0x00000021;
#define TPM1CNT                         _TPM1CNT.Word


/*** TPM1MOD - TPM1 Timer Counter Modulo Register; 0x00000023 ***/
typedef union {
  word Word;
   /* Overlapped registers: */
  struct {
    /*** TPM1MODH - TPM1 Timer Counter Modulo Register High; 0x00000023 ***/
    union {
      byte Byte;
    } TPM1MODHSTR;
    #define TPM1MODH                    _TPM1MOD.Overlap_STR.TPM1MODHSTR.Byte
    

    /*** TPM1MODL - TPM1 Timer Counter Modulo Register Low; 0x00000024 ***/
    union {
      byte Byte;
    } TPM1MODLSTR;
    #define TPM1MODL                    _TPM1MOD.Overlap_STR.TPM1MODLSTR.Byte
    
  } Overlap_STR;

} TPM1MODSTR;
extern volatile TPM1MODSTR _TPM1MOD @0x00000023;
#define TPM1MOD                         _TPM1MOD.Word


/*** TPM1C0SC - TPM1 Timer Channel 0 Status and Control Register; 0x00000025 ***/
typedef union {
  byte Byte;
  struct {
    byte             :1; 
    byte             :1; 
    byte ELS0A       :1;                                       /* Edge/Level Select Bit A */
    byte ELS0B       :1;                                       /* Edge/Level Select Bit B */
    byte MS0A        :1;                                       /* Mode Select A for TPM Channel 0 */
    byte MS0B        :1;                                       /* Mode Select B for TPM Channel 0 */
    byte CH0IE       :1;                                       /* Channel 0 Interrupt Enable */
    byte CH0F        :1;                                       /* Channel 0 Flag */
  } Bits;
  struct {
    byte         :1;
    byte         :1;
    byte grpELS0x :2;
    byte grpMS0x :2;
    byte         :1;
    byte         :1;
  } MergedBits;
} TPM1C0SCSTR;
extern volatile TPM1C0SCSTR _TPM1C0SC @0x00000025;
#define TPM1C0SC                        _TPM1C0SC.Byte
#define TPM1C0SC_ELS0A                  _TPM1C0SC.Bits.ELS0A
#define TPM1C0SC_ELS0B                  _TPM1C0SC.Bits.ELS0B
#define TPM1C0SC_MS0A                   _TPM1C0SC.Bits.MS0A
#define TPM1C0SC_MS0B                   _TPM1C0SC.Bits.MS0B
#define TPM1C0SC_CH0IE                  _TPM1C0SC.Bits.CH0IE
#define TPM1C0SC_CH0F                   _TPM1C0SC.Bits.CH0F
#define TPM1C0SC_ELS0x                  _TPM1C0SC.MergedBits.grpELS0x
#define TPM1C0SC_MS0x                   _TPM1C0SC.MergedBits.grpMS0x

#define TPM1C0SC_ELS0A_MASK             4U
#define TPM1C0SC_ELS0B_MASK             8U
#define TPM1C0SC_MS0A_MASK              16U
#define TPM1C0SC_MS0B_MASK              32U
#define TPM1C0SC_CH0IE_MASK             64U
#define TPM1C0SC_CH0F_MASK              128U
#define TPM1C0SC_ELS0x_MASK             12U
#define TPM1C0SC_ELS0x_BITNUM           2U
#define TPM1C0SC_MS0x_MASK              48U
#define TPM1C0SC_MS0x_BITNUM            4U


/*** TPM1C0V - TPM1 Timer Channel 0 Value Register; 0x00000026 ***/
typedef union {
  word Word;
   /* Overlapped registers: */
  struct {
    /*** TPM1C0VH - TPM1 Timer Channel 0 Value Register High; 0x00000026 ***/
    union {
      byte Byte;
    } TPM1C0VHSTR;
    #define TPM1C0VH                    _TPM1C0V.Overlap_STR.TPM1C0VHSTR.Byte
    

    /*** TPM1C0VL - TPM1 Timer Channel 0 Value Register Low; 0x00000027 ***/
    union {
      byte Byte;
    } TPM1C0VLSTR;
    #define TPM1C0VL                    _TPM1C0V.Overlap_STR.TPM1C0VLSTR.Byte
    
  } Overlap_STR;

} TPM1C0VSTR;
extern volatile TPM1C0VSTR _TPM1C0V @0x00000026;
#define TPM1C0V                         _TPM1C0V.Word


/*** TPM1C1SC - TPM1 Timer Channel 1 Status and Control Register; 0x00000028 ***/
typedef union {
  byte Byte;
  struct {
    byte             :1; 
    byte             :1; 
    byte ELS1A       :1;                                       /* Edge/Level Select Bit A */
    byte ELS1B       :1;                                       /* Edge/Level Select Bit B */
    byte MS1A        :1;                                       /* Mode Select A for TPM Channel 1 */
    byte MS1B        :1;                                       /* Mode Select B for TPM Channel 1 */
    byte CH1IE       :1;                                       /* Channel 1 Interrupt Enable */
    byte CH1F        :1;                                       /* Channel 1 Flag */
  } Bits;
  struct {
    byte         :1;
    byte         :1;
    byte grpELS1x :2;
    byte grpMS1x :2;
    byte         :1;
    byte         :1;
  } MergedBits;
} TPM1C1SCSTR;
extern volatile TPM1C1SCSTR _TPM1C1SC @0x00000028;
#define TPM1C1SC                        _TPM1C1SC.Byte
#define TPM1C1SC_ELS1A                  _TPM1C1SC.Bits.ELS1A
#define TPM1C1SC_ELS1B                  _TPM1C1SC.Bits.ELS1B
#define TPM1C1SC_MS1A                   _TPM1C1SC.Bits.MS1A
#define TPM1C1SC_MS1B                   _TPM1C1SC.Bits.MS1B
#define TPM1C1SC_CH1IE                  _TPM1C1SC.Bits.CH1IE
#define TPM1C1SC_CH1F                   _TPM1C1SC.Bits.CH1F
#define TPM1C1SC_ELS1x                  _TPM1C1SC.MergedBits.grpELS1x
#define TPM1C1SC_MS1x                   _TPM1C1SC.MergedBits.grpMS1x

#define TPM1C1SC_ELS1A_MASK             4U
#define TPM1C1SC_ELS1B_MASK             8U
#define TPM1C1SC_MS1A_MASK              16U
#define TPM1C1SC_MS1B_MASK              32U
#define TPM1C1SC_CH1IE_MASK             64U
#define TPM1C1SC_CH1F_MASK              128U
#define TPM1C1SC_ELS1x_MASK             12U
#define TPM1C1SC_ELS1x_BITNUM           2U
#define TPM1C1SC_MS1x_MASK              48U
#define TPM1C1SC_MS1x_BITNUM            4U


/*** TPM1C1V - TPM1 Timer Channel 1 Value Register; 0x00000029 ***/
typedef union {
  word Word;
   /* Overlapped registers: */
  struct {
    /*** TPM1C1VH - TPM1 Timer Channel 1 Value Register High; 0x00000029 ***/
    union {
      byte Byte;
    } TPM1C1VHSTR;
    #define TPM1C1VH                    _TPM1C1V.Overlap_STR.TPM1C1VHSTR.Byte
    

    /*** TPM1C1VL - TPM1 Timer Channel 1 Value Register Low; 0x0000002A ***/
    union {
      byte Byte;
    } TPM1C1VLSTR;
    #define TPM1C1VL                    _TPM1C1V.Overlap_STR.TPM1C1VLSTR.Byte
    
  } Overlap_STR;

} TPM1C1VSTR;
extern volatile TPM1C1VSTR _TPM1C1V @0x00000029;
#define TPM1C1V                         _TPM1C1V.Word


/*** TPM1C2SC - TPM1 Timer Channel 2 Status and Control Register; 0x0000002B ***/
typedef union {
  byte Byte;
  struct {
    byte             :1; 
    byte             :1; 
    byte ELS2A       :1;                                       /* Edge/Level Select Bit A */
    byte ELS2B       :1;                                       /* Edge/Level Select Bit B */
    byte MS2A        :1;                                       /* Mode Select A for TPM Channel 2 */
    byte MS2B        :1;                                       /* Mode Select B for TPM Channel 2 */
    byte CH2IE       :1;                                       /* Channel 2 Interrupt Enable */
    byte CH2F        :1;                                       /* Channel 2 Flag */
  } Bits;
  struct {
    byte         :1;
    byte         :1;
    byte grpELS2x :2;
    byte grpMS2x :2;
    byte         :1;
    byte         :1;
  } MergedBits;
} TPM1C2SCSTR;
extern volatile TPM1C2SCSTR _TPM1C2SC @0x0000002B;
#define TPM1C2SC                        _TPM1C2SC.Byte
#define TPM1C2SC_ELS2A                  _TPM1C2SC.Bits.ELS2A
#define TPM1C2SC_ELS2B                  _TPM1C2SC.Bits.ELS2B
#define TPM1C2SC_MS2A                   _TPM1C2SC.Bits.MS2A
#define TPM1C2SC_MS2B                   _TPM1C2SC.Bits.MS2B
#define TPM1C2SC_CH2IE                  _TPM1C2SC.Bits.CH2IE
#define TPM1C2SC_CH2F                   _TPM1C2SC.Bits.CH2F
#define TPM1C2SC_ELS2x                  _TPM1C2SC.MergedBits.grpELS2x
#define TPM1C2SC_MS2x                   _TPM1C2SC.MergedBits.grpMS2x

#define TPM1C2SC_ELS2A_MASK             4U
#define TPM1C2SC_ELS2B_MASK             8U
#define TPM1C2SC_MS2A_MASK              16U
#define TPM1C2SC_MS2B_MASK              32U
#define TPM1C2SC_CH2IE_MASK             64U
#define TPM1C2SC_CH2F_MASK              128U
#define TPM1C2SC_ELS2x_MASK             12U
#define TPM1C2SC_ELS2x_BITNUM           2U
#define TPM1C2SC_MS2x_MASK              48U
#define TPM1C2SC_MS2x_BITNUM            4U


/*** TPM1C2V - TPM1 Timer Channel 2 Value Register; 0x0000002C ***/
typedef union {
  word Word;
   /* Overlapped registers: */
  struct {
    /*** TPM1C2VH - TPM1 Timer Channel 2 Value Register High; 0x0000002C ***/
    union {
      byte Byte;
    } TPM1C2VHSTR;
    #define TPM1C2VH                    _TPM1C2V.Overlap_STR.TPM1C2VHSTR.Byte
    

    /*** TPM1C2VL - TPM1 Timer Channel 2 Value Register Low; 0x0000002D ***/
    union {
      byte Byte;
    } TPM1C2VLSTR;
    #define TPM1C2VL                    _TPM1C2V.Overlap_STR.TPM1C2VLSTR.Byte
    
  } Overlap_STR;

} TPM1C2VSTR;
extern volatile TPM1C2VSTR _TPM1C2V @0x0000002C;
#define TPM1C2V                         _TPM1C2V.Word


/*** TPM1C3SC - TPM1 Timer Channel 3 Status and Control Register; 0x0000002E ***/
typedef union {
  byte Byte;
  struct {
    byte             :1; 
    byte             :1; 
    byte ELS3A       :1;                                       /* Edge/Level Select Bit A */
    byte ELS3B       :1;                                       /* Edge/Level Select Bit B */
    byte MS3A        :1;                                       /* Mode Select A for TPM Channel 3 */
    byte MS3B        :1;                                       /* Mode Select B for TPM Channel 3 */
    byte CH3IE       :1;                                       /* Channel 3 Interrupt Enable */
    byte CH3F        :1;                                       /* Channel 3 Flag */
  } Bits;
  struct {
    byte         :1;
    byte         :1;
    byte grpELS3x :2;
    byte grpMS3x :2;
    byte         :1;
    byte         :1;
  } MergedBits;
} TPM1C3SCSTR;
extern volatile TPM1C3SCSTR _TPM1C3SC @0x0000002E;
#define TPM1C3SC                        _TPM1C3SC.Byte
#define TPM1C3SC_ELS3A                  _TPM1C3SC.Bits.ELS3A
#define TPM1C3SC_ELS3B                  _TPM1C3SC.Bits.ELS3B
#define TPM1C3SC_MS3A                   _TPM1C3SC.Bits.MS3A
#define TPM1C3SC_MS3B                   _TPM1C3SC.Bits.MS3B
#define TPM1C3SC_CH3IE                  _TPM1C3SC.Bits.CH3IE
#define TPM1C3SC_CH3F                   _TPM1C3SC.Bits.CH3F
#define TPM1C3SC_ELS3x                  _TPM1C3SC.MergedBits.grpELS3x
#define TPM1C3SC_MS3x                   _TPM1C3SC.MergedBits.grpMS3x

#define TPM1C3SC_ELS3A_MASK             4U
#define TPM1C3SC_ELS3B_MASK             8U
#define TPM1C3SC_MS3A_MASK              16U
#define TPM1C3SC_MS3B_MASK              32U
#define TPM1C3SC_CH3IE_MASK             64U
#define TPM1C3SC_CH3F_MASK              128U
#define TPM1C3SC_ELS3x_MASK             12U
#define TPM1C3SC_ELS3x_BITNUM           2U
#define TPM1C3SC_MS3x_MASK              48U
#define TPM1C3SC_MS3x_BITNUM            4U


/*** TPM1C3V - TPM1 Timer Channel 3 Value Register; 0x0000002F ***/
typedef union {
  word Word;
   /* Overlapped registers: */
  struct {
    /*** TPM1C3VH - TPM1 Timer Channel 3 Value Register High; 0x0000002F ***/
    union {
      byte Byte;
    } TPM1C3VHSTR;
    #define TPM1C3VH                    _TPM1C3V.Overlap_STR.TPM1C3VHSTR.Byte
    

    /*** TPM1C3VL - TPM1 Timer Channel 3 Value Register Low; 0x00000030 ***/
    union {
      byte Byte;
    } TPM1C3VLSTR;
    #define TPM1C3VL                    _TPM1C3V.Overlap_STR.TPM1C3VLSTR.Byte
    
  } Overlap_STR;

} TPM1C3VSTR;
extern volatile TPM1C3VSTR _TPM1C3V @0x0000002F;
#define TPM1C3V                         _TPM1C3V.Word


/*** TPM1C4SC - TPM1 Timer Channel 4 Status and Control Register; 0x00000031 ***/
typedef union {
  byte Byte;
  struct {
    byte             :1; 
    byte             :1; 
    byte ELS4A       :1;                                       /* Edge/Level Select Bit A */
    byte ELS4B       :1;                                       /* Edge/Level Select Bit B */
    byte MS4A        :1;                                       /* Mode Select A for TPM Channel 4 */
    byte MS4B        :1;                                       /* Mode Select B for TPM Channel 4 */
    byte CH4IE       :1;                                       /* Channel 4 Interrupt Enable */
    byte CH4F        :1;                                       /* Channel 4 Flag */
  } Bits;
  struct {
    byte         :1;
    byte         :1;
    byte grpELS4x :2;
    byte grpMS4x :2;
    byte         :1;
    byte         :1;
  } MergedBits;
} TPM1C4SCSTR;
extern volatile TPM1C4SCSTR _TPM1C4SC @0x00000031;
#define TPM1C4SC                        _TPM1C4SC.Byte
#define TPM1C4SC_ELS4A                  _TPM1C4SC.Bits.ELS4A
#define TPM1C4SC_ELS4B                  _TPM1C4SC.Bits.ELS4B
#define TPM1C4SC_MS4A                   _TPM1C4SC.Bits.MS4A
#define TPM1C4SC_MS4B                   _TPM1C4SC.Bits.MS4B
#define TPM1C4SC_CH4IE                  _TPM1C4SC.Bits.CH4IE
#define TPM1C4SC_CH4F                   _TPM1C4SC.Bits.CH4F
#define TPM1C4SC_ELS4x                  _TPM1C4SC.MergedBits.grpELS4x
#define TPM1C4SC_MS4x                   _TPM1C4SC.MergedBits.grpMS4x

#define TPM1C4SC_ELS4A_MASK             4U
#define TPM1C4SC_ELS4B_MASK             8U
#define TPM1C4SC_MS4A_MASK              16U
#define TPM1C4SC_MS4B_MASK              32U
#define TPM1C4SC_CH4IE_MASK             64U
#define TPM1C4SC_CH4F_MASK              128U
#define TPM1C4SC_ELS4x_MASK             12U
#define TPM1C4SC_ELS4x_BITNUM           2U
#define TPM1C4SC_MS4x_MASK              48U
#define TPM1C4SC_MS4x_BITNUM            4U


/*** TPM1C4V - TPM1 Timer Channel 4 Value Register; 0x00000032 ***/
typedef union {
  word Word;
   /* Overlapped registers: */
  struct {
    /*** TPM1C4VH - TPM1 Timer Channel 4 Value Register High; 0x00000032 ***/
    union {
      byte Byte;
    } TPM1C4VHSTR;
    #define TPM1C4VH                    _TPM1C4V.Overlap_STR.TPM1C4VHSTR.Byte
    

    /*** TPM1C4VL - TPM1 Timer Channel 4 Value Register Low; 0x00000033 ***/
    union {
      byte Byte;
    } TPM1C4VLSTR;
    #define TPM1C4VL                    _TPM1C4V.Overlap_STR.TPM1C4VLSTR.Byte
    
  } Overlap_STR;

} TPM1C4VSTR;
extern volatile TPM1C4VSTR _TPM1C4V @0x00000032;
#define TPM1C4V                         _TPM1C4V.Word


/*** TPM1C5SC - TPM1 Timer Channel 5 Status and Control Register; 0x00000034 ***/
typedef union {
  byte Byte;
  struct {
    byte             :1; 
    byte             :1; 
    byte ELS5A       :1;                                       /* Edge/Level Select Bit A */
    byte ELS5B       :1;                                       /* Edge/Level Select Bit B */
    byte MS5A        :1;                                       /* Mode Select A for TPM Channel 5 */
    byte MS5B        :1;                                       /* Mode Select B for TPM Channel 5 */
    byte CH5IE       :1;                                       /* Channel 5 Interrupt Enable */
    byte CH5F        :1;                                       /* Channel 5 Flag */
  } Bits;
  struct {
    byte         :1;
    byte         :1;
    byte grpELS5x :2;
    byte grpMS5x :2;
    byte         :1;
    byte         :1;
  } MergedBits;
} TPM1C5SCSTR;
extern volatile TPM1C5SCSTR _TPM1C5SC @0x00000034;
#define TPM1C5SC                        _TPM1C5SC.Byte
#define TPM1C5SC_ELS5A                  _TPM1C5SC.Bits.ELS5A
#define TPM1C5SC_ELS5B                  _TPM1C5SC.Bits.ELS5B
#define TPM1C5SC_MS5A                   _TPM1C5SC.Bits.MS5A
#define TPM1C5SC_MS5B                   _TPM1C5SC.Bits.MS5B
#define TPM1C5SC_CH5IE                  _TPM1C5SC.Bits.CH5IE
#define TPM1C5SC_CH5F                   _TPM1C5SC.Bits.CH5F
#define TPM1C5SC_ELS5x                  _TPM1C5SC.MergedBits.grpELS5x
#define TPM1C5SC_MS5x                   _TPM1C5SC.MergedBits.grpMS5x

#define TPM1C5SC_ELS5A_MASK             4U
#define TPM1C5SC_ELS5B_MASK             8U
#define TPM1C5SC_MS5A_MASK              16U
#define TPM1C5SC_MS5B_MASK              32U
#define TPM1C5SC_CH5IE_MASK             64U
#define TPM1C5SC_CH5F_MASK              128U
#define TPM1C5SC_ELS5x_MASK             12U
#define TPM1C5SC_ELS5x_BITNUM           2U
#define TPM1C5SC_MS5x_MASK              48U
#define TPM1C5SC_MS5x_BITNUM            4U


/*** TPM1C5V - TPM1 Timer Channel 5 Value Register; 0x00000035 ***/
typedef union {
  word Word;
   /* Overlapped registers: */
  struct {
    /*** TPM1C5VH - TPM1 Timer Channel 5 Value Register High; 0x00000035 ***/
    union {
      byte Byte;
    } TPM1C5VHSTR;
    #define TPM1C5VH                    _TPM1C5V.Overlap_STR.TPM1C5VHSTR.Byte
    

    /*** TPM1C5VL - TPM1 Timer Channel 5 Value Register Low; 0x00000036 ***/
    union {
      byte Byte;
    } TPM1C5VLSTR;
    #define TPM1C5VL                    _TPM1C5V.Overlap_STR.TPM1C5VLSTR.Byte
    
  } Overlap_STR;

} TPM1C5VSTR;
extern volatile TPM1C5VSTR _TPM1C5V @0x00000035;
#define TPM1C5V                         _TPM1C5V.Word


/*** SCI1BD - SCI1 Baud Rate Register; 0x00000038 ***/
typedef union {
  word Word;
   /* Overlapped registers: */
  struct {
    /*** SCI1BDH - SCI1 Baud Rate Register High; 0x00000038 ***/
    union {
      byte Byte;
      struct {
        byte SBR8        :1;                                       /* Baud Rate Modulo Divisor Bit 8 */
        byte SBR9        :1;                                       /* Baud Rate Modulo Divisor Bit 9 */
        byte SBR10       :1;                                       /* Baud Rate Modulo Divisor Bit 10 */
        byte SBR11       :1;                                       /* Baud Rate Modulo Divisor Bit 11 */
        byte SBR12       :1;                                       /* Baud Rate Modulo Divisor Bit 12 */
        byte             :1; 
        byte RXEDGIE     :1;                                       /* RxD Input Active Edge Interrupt Enable (for RXEDGIF) */
        byte LBKDIE      :1;                                       /* LIN Break Detect Interrupt Enable (for LBKDIF) */
      } Bits;
      struct {
        byte grpSBR_8 :5;
        byte     :1;
        byte     :1;
        byte     :1;
      } MergedBits;
    } SCI1BDHSTR;
    #define SCI1BDH                     _SCI1BD.Overlap_STR.SCI1BDHSTR.Byte
    #define SCI1BDH_SBR8                _SCI1BD.Overlap_STR.SCI1BDHSTR.Bits.SBR8
    #define SCI1BDH_SBR9                _SCI1BD.Overlap_STR.SCI1BDHSTR.Bits.SBR9
    #define SCI1BDH_SBR10               _SCI1BD.Overlap_STR.SCI1BDHSTR.Bits.SBR10
    #define SCI1BDH_SBR11               _SCI1BD.Overlap_STR.SCI1BDHSTR.Bits.SBR11
    #define SCI1BDH_SBR12               _SCI1BD.Overlap_STR.SCI1BDHSTR.Bits.SBR12
    #define SCI1BDH_RXEDGIE             _SCI1BD.Overlap_STR.SCI1BDHSTR.Bits.RXEDGIE
    #define SCI1BDH_LBKDIE              _SCI1BD.Overlap_STR.SCI1BDHSTR.Bits.LBKDIE
    #define SCI1BDH_SBR_8               _SCI1BD.Overlap_STR.SCI1BDHSTR.MergedBits.grpSBR_8
    #define SCI1BDH_SBR                 SCI1BDH_SBR_8
    
    #define SCI1BDH_SBR8_MASK           1U
    #define SCI1BDH_SBR9_MASK           2U
    #define SCI1BDH_SBR10_MASK          4U
    #define SCI1BDH_SBR11_MASK          8U
    #define SCI1BDH_SBR12_MASK          16U
    #define SCI1BDH_RXEDGIE_MASK        64U
    #define SCI1BDH_LBKDIE_MASK         128U
    #define SCI1BDH_SBR_8_MASK          31U
    #define SCI1BDH_SBR_8_BITNUM        0U
    

    /*** SCI1BDL - SCI1 Baud Rate Register Low; 0x00000039 ***/
    union {
      byte Byte;
      struct {
        byte SBR0        :1;                                       /* Baud Rate Modulo Divisor Bit 0 */
        byte SBR1        :1;                                       /* Baud Rate Modulo Divisor Bit 1 */
        byte SBR2        :1;                                       /* Baud Rate Modulo Divisor Bit 2 */
        byte SBR3        :1;                                       /* Baud Rate Modulo Divisor Bit 3 */
        byte SBR4        :1;                                       /* Baud Rate Modulo Divisor Bit 4 */
        byte SBR5        :1;                                       /* Baud Rate Modulo Divisor Bit 5 */
        byte SBR6        :1;                                       /* Baud Rate Modulo Divisor Bit 6 */
        byte SBR7        :1;                                       /* Baud Rate Modulo Divisor Bit 7 */
      } Bits;
    } SCI1BDLSTR;
    #define SCI1BDL                     _SCI1BD.Overlap_STR.SCI1BDLSTR.Byte
    #define SCI1BDL_SBR0                _SCI1BD.Overlap_STR.SCI1BDLSTR.Bits.SBR0
    #define SCI1BDL_SBR1                _SCI1BD.Overlap_STR.SCI1BDLSTR.Bits.SBR1
    #define SCI1BDL_SBR2                _SCI1BD.Overlap_STR.SCI1BDLSTR.Bits.SBR2
    #define SCI1BDL_SBR3                _SCI1BD.Overlap_STR.SCI1BDLSTR.Bits.SBR3
    #define SCI1BDL_SBR4                _SCI1BD.Overlap_STR.SCI1BDLSTR.Bits.SBR4
    #define SCI1BDL_SBR5                _SCI1BD.Overlap_STR.SCI1BDLSTR.Bits.SBR5
    #define SCI1BDL_SBR6                _SCI1BD.Overlap_STR.SCI1BDLSTR.Bits.SBR6
    #define SCI1BDL_SBR7                _SCI1BD.Overlap_STR.SCI1BDLSTR.Bits.SBR7
    
    #define SCI1BDL_SBR0_MASK           1U
    #define SCI1BDL_SBR1_MASK           2U
    #define SCI1BDL_SBR2_MASK           4U
    #define SCI1BDL_SBR3_MASK           8U
    #define SCI1BDL_SBR4_MASK           16U
    #define SCI1BDL_SBR5_MASK           32U
    #define SCI1BDL_SBR6_MASK           64U
    #define SCI1BDL_SBR7_MASK           128U
    
  } Overlap_STR;

} SCI1BDSTR;
extern volatile SCI1BDSTR _SCI1BD @0x00000038;
#define SCI1BD                          _SCI1BD.Word


/*** SCI1C1 - SCI1 Control Register 1; 0x0000003A ***/
typedef union {
  byte Byte;
  struct {
    byte PT          :1;                                       /* Parity Type */
    byte PE          :1;                                       /* Parity Enable */
    byte ILT         :1;                                       /* Idle Line Type Select */
    byte WAKE        :1;                                       /* Receiver Wakeup Method Select */
    byte M           :1;                                       /* 9-Bit or 8-Bit Mode Select */
    byte RSRC        :1;                                       /* Receiver Source Select */
    byte SCISWAI     :1;                                       /* SCI Stops in Wait Mode */
    byte LOOPS       :1;                                       /* Loop Mode Select */
  } Bits;
} SCI1C1STR;
extern volatile SCI1C1STR _SCI1C1 @0x0000003A;
#define SCI1C1                          _SCI1C1.Byte
#define SCI1C1_PT                       _SCI1C1.Bits.PT
#define SCI1C1_PE                       _SCI1C1.Bits.PE
#define SCI1C1_ILT                      _SCI1C1.Bits.ILT
#define SCI1C1_WAKE                     _SCI1C1.Bits.WAKE
#define SCI1C1_M                        _SCI1C1.Bits.M
#define SCI1C1_RSRC                     _SCI1C1.Bits.RSRC
#define SCI1C1_SCISWAI                  _SCI1C1.Bits.SCISWAI
#define SCI1C1_LOOPS                    _SCI1C1.Bits.LOOPS

#define SCI1C1_PT_MASK                  1U
#define SCI1C1_PE_MASK                  2U
#define SCI1C1_ILT_MASK                 4U
#define SCI1C1_WAKE_MASK                8U
#define SCI1C1_M_MASK                   16U
#define SCI1C1_RSRC_MASK                32U
#define SCI1C1_SCISWAI_MASK             64U
#define SCI1C1_LOOPS_MASK               128U


/*** SCI1C2 - SCI1 Control Register 2; 0x0000003B ***/
typedef union {
  byte Byte;
  struct {
    byte SBK         :1;                                       /* Send Break */
    byte RWU         :1;                                       /* Receiver Wakeup Control */
    byte RE          :1;                                       /* Receiver Enable */
    byte TE          :1;                                       /* Transmitter Enable */
    byte ILIE        :1;                                       /* Idle Line Interrupt Enable (for IDLE) */
    byte RIE         :1;                                       /* Receiver Interrupt Enable (for RDRF) */
    byte TCIE        :1;                                       /* Transmission Complete Interrupt Enable (for TC) */
    byte TIE         :1;                                       /* Transmit Interrupt Enable (for TDRE) */
  } Bits;
} SCI1C2STR;
extern volatile SCI1C2STR _SCI1C2 @0x0000003B;
#define SCI1C2                          _SCI1C2.Byte
#define SCI1C2_SBK                      _SCI1C2.Bits.SBK
#define SCI1C2_RWU                      _SCI1C2.Bits.RWU
#define SCI1C2_RE                       _SCI1C2.Bits.RE
#define SCI1C2_TE                       _SCI1C2.Bits.TE
#define SCI1C2_ILIE                     _SCI1C2.Bits.ILIE
#define SCI1C2_RIE                      _SCI1C2.Bits.RIE
#define SCI1C2_TCIE                     _SCI1C2.Bits.TCIE
#define SCI1C2_TIE                      _SCI1C2.Bits.TIE

#define SCI1C2_SBK_MASK                 1U
#define SCI1C2_RWU_MASK                 2U
#define SCI1C2_RE_MASK                  4U
#define SCI1C2_TE_MASK                  8U
#define SCI1C2_ILIE_MASK                16U
#define SCI1C2_RIE_MASK                 32U
#define SCI1C2_TCIE_MASK                64U
#define SCI1C2_TIE_MASK                 128U


/*** SCI1S1 - SCI1 Status Register 1; 0x0000003C ***/
typedef union {
  byte Byte;
  struct {
    byte PF          :1;                                       /* Parity Error Flag */
    byte FE          :1;                                       /* Framing Error Flag */
    byte NF          :1;                                       /* Noise Flag */
    byte OR          :1;                                       /* Receiver Overrun Flag */
    byte IDLE        :1;                                       /* Idle Line Flag */
    byte RDRF        :1;                                       /* Receive Data Register Full Flag */
    byte TC          :1;                                       /* Transmission Complete Flag */
    byte TDRE        :1;                                       /* Transmit Data Register Empty Flag */
  } Bits;
} SCI1S1STR;
extern volatile SCI1S1STR _SCI1S1 @0x0000003C;
#define SCI1S1                          _SCI1S1.Byte
#define SCI1S1_PF                       _SCI1S1.Bits.PF
#define SCI1S1_FE                       _SCI1S1.Bits.FE
#define SCI1S1_NF                       _SCI1S1.Bits.NF
#define SCI1S1_OR                       _SCI1S1.Bits.OR
#define SCI1S1_IDLE                     _SCI1S1.Bits.IDLE
#define SCI1S1_RDRF                     _SCI1S1.Bits.RDRF
#define SCI1S1_TC                       _SCI1S1.Bits.TC
#define SCI1S1_TDRE                     _SCI1S1.Bits.TDRE

#define SCI1S1_PF_MASK                  1U
#define SCI1S1_FE_MASK                  2U
#define SCI1S1_NF_MASK                  4U
#define SCI1S1_OR_MASK                  8U
#define SCI1S1_IDLE_MASK                16U
#define SCI1S1_RDRF_MASK                32U
#define SCI1S1_TC_MASK                  64U
#define SCI1S1_TDRE_MASK                128U


/*** SCI1S2 - SCI1 Status Register 2; 0x0000003D ***/
typedef union {
  byte Byte;
  struct {
    byte RAF         :1;                                       /* Receiver Active Flag */
    byte LBKDE       :1;                                       /* LIN Break Detection Enable */
    byte BRK13       :1;                                       /* Break Character Generation Length */
    byte RWUID       :1;                                       /* Receive Wake Up Idle Detect */
    byte RXINV       :1;                                       /* Receive Data Inversion */
    byte             :1; 
    byte RXEDGIF     :1;                                       /* RxD Pin Active Edge Interrupt Flag */
    byte LBKDIF      :1;                                       /* LIN Break Detect Interrupt Flag */
  } Bits;
} SCI1S2STR;
extern volatile SCI1S2STR _SCI1S2 @0x0000003D;
#define SCI1S2                          _SCI1S2.Byte
#define SCI1S2_RAF                      _SCI1S2.Bits.RAF
#define SCI1S2_LBKDE                    _SCI1S2.Bits.LBKDE
#define SCI1S2_BRK13                    _SCI1S2.Bits.BRK13
#define SCI1S2_RWUID                    _SCI1S2.Bits.RWUID
#define SCI1S2_RXINV                    _SCI1S2.Bits.RXINV
#define SCI1S2_RXEDGIF                  _SCI1S2.Bits.RXEDGIF
#define SCI1S2_LBKDIF                   _SCI1S2.Bits.LBKDIF

#define SCI1S2_RAF_MASK                 1U
#define SCI1S2_LBKDE_MASK               2U
#define SCI1S2_BRK13_MASK               4U
#define SCI1S2_RWUID_MASK               8U
#define SCI1S2_RXINV_MASK               16U
#define SCI1S2_RXEDGIF_MASK             64U
#define SCI1S2_LBKDIF_MASK              128U


/*** SCI1C3 - SCI1 Control Register 3; 0x0000003E ***/
typedef union {
  byte Byte;
  struct {
    byte PEIE        :1;                                       /* Parity Error Interrupt Enable */
    byte FEIE        :1;                                       /* Framing Error Interrupt Enable */
    byte NEIE        :1;                                       /* Noise Error Interrupt Enable */
    byte ORIE        :1;                                       /* Overrun Interrupt Enable */
    byte TXINV       :1;                                       /* Transmit Data Inversion */
    byte TXDIR       :1;                                       /* TxD Pin Direction in Single-Wire Mode */
    byte T8          :1;                                       /* Ninth Data Bit for Transmitter */
    byte R8          :1;                                       /* Ninth Data Bit for Receiver */
  } Bits;
} SCI1C3STR;
extern volatile SCI1C3STR _SCI1C3 @0x0000003E;
#define SCI1C3                          _SCI1C3.Byte
#define SCI1C3_PEIE                     _SCI1C3.Bits.PEIE
#define SCI1C3_FEIE                     _SCI1C3.Bits.FEIE
#define SCI1C3_NEIE                     _SCI1C3.Bits.NEIE
#define SCI1C3_ORIE                     _SCI1C3.Bits.ORIE
#define SCI1C3_TXINV                    _SCI1C3.Bits.TXINV
#define SCI1C3_TXDIR                    _SCI1C3.Bits.TXDIR
#define SCI1C3_T8                       _SCI1C3.Bits.T8
#define SCI1C3_R8                       _SCI1C3.Bits.R8

#define SCI1C3_PEIE_MASK                1U
#define SCI1C3_FEIE_MASK                2U
#define SCI1C3_NEIE_MASK                4U
#define SCI1C3_ORIE_MASK                8U
#define SCI1C3_TXINV_MASK               16U
#define SCI1C3_TXDIR_MASK               32U
#define SCI1C3_T8_MASK                  64U
#define SCI1C3_R8_MASK                  128U


/*** SCI1D - SCI1 Data Register; 0x0000003F ***/
typedef union {
  byte Byte;
  struct {
    byte R0_T0       :1;                                       /* Receive/Transmit Data Bit 0 */
    byte R1_T1       :1;                                       /* Receive/Transmit Data Bit 1 */
    byte R2_T2       :1;                                       /* Receive/Transmit Data Bit 2 */
    byte R3_T3       :1;                                       /* Receive/Transmit Data Bit 3 */
    byte R4_T4       :1;                                       /* Receive/Transmit Data Bit 4 */
    byte R5_T5       :1;                                       /* Receive/Transmit Data Bit 5 */
    byte R6_T6       :1;                                       /* Receive/Transmit Data Bit 6 */
    byte R7_T7       :1;                                       /* Receive/Transmit Data Bit 7 */
  } Bits;
} SCI1DSTR;
extern volatile SCI1DSTR _SCI1D @0x0000003F;
#define SCI1D                           _SCI1D.Byte
#define SCI1D_R0_T0                     _SCI1D.Bits.R0_T0
#define SCI1D_R1_T1                     _SCI1D.Bits.R1_T1
#define SCI1D_R2_T2                     _SCI1D.Bits.R2_T2
#define SCI1D_R3_T3                     _SCI1D.Bits.R3_T3
#define SCI1D_R4_T4                     _SCI1D.Bits.R4_T4
#define SCI1D_R5_T5                     _SCI1D.Bits.R5_T5
#define SCI1D_R6_T6                     _SCI1D.Bits.R6_T6
#define SCI1D_R7_T7                     _SCI1D.Bits.R7_T7

#define SCI1D_R0_T0_MASK                1U
#define SCI1D_R1_T1_MASK                2U
#define SCI1D_R2_T2_MASK                4U
#define SCI1D_R3_T3_MASK                8U
#define SCI1D_R4_T4_MASK                16U
#define SCI1D_R5_T5_MASK                32U
#define SCI1D_R6_T6_MASK                64U
#define SCI1D_R7_T7_MASK                128U


/*** SCI2BD - SCI2 Baud Rate Register; 0x00000040 ***/
typedef union {
  word Word;
   /* Overlapped registers: */
  struct {
    /*** SCI2BDH - SCI2 Baud Rate Register High; 0x00000040 ***/
    union {
      byte Byte;
      struct {
        byte SBR8        :1;                                       /* Baud Rate Modulo Divisor Bit 8 */
        byte SBR9        :1;                                       /* Baud Rate Modulo Divisor Bit 9 */
        byte SBR10       :1;                                       /* Baud Rate Modulo Divisor Bit 10 */
        byte SBR11       :1;                                       /* Baud Rate Modulo Divisor Bit 11 */
        byte SBR12       :1;                                       /* Baud Rate Modulo Divisor Bit 12 */
        byte             :1; 
        byte RXEDGIE     :1;                                       /* RxD Input Active Edge Interrupt Enable (for RXEDGIF) */
        byte LBKDIE      :1;                                       /* LIN Break Detect Interrupt Enable (for LBKDIF) */
      } Bits;
      struct {
        byte grpSBR_8 :5;
        byte     :1;
        byte     :1;
        byte     :1;
      } MergedBits;
    } SCI2BDHSTR;
    #define SCI2BDH                     _SCI2BD.Overlap_STR.SCI2BDHSTR.Byte
    #define SCI2BDH_SBR8                _SCI2BD.Overlap_STR.SCI2BDHSTR.Bits.SBR8
    #define SCI2BDH_SBR9                _SCI2BD.Overlap_STR.SCI2BDHSTR.Bits.SBR9
    #define SCI2BDH_SBR10               _SCI2BD.Overlap_STR.SCI2BDHSTR.Bits.SBR10
    #define SCI2BDH_SBR11               _SCI2BD.Overlap_STR.SCI2BDHSTR.Bits.SBR11
    #define SCI2BDH_SBR12               _SCI2BD.Overlap_STR.SCI2BDHSTR.Bits.SBR12
    #define SCI2BDH_RXEDGIE             _SCI2BD.Overlap_STR.SCI2BDHSTR.Bits.RXEDGIE
    #define SCI2BDH_LBKDIE              _SCI2BD.Overlap_STR.SCI2BDHSTR.Bits.LBKDIE
    #define SCI2BDH_SBR_8               _SCI2BD.Overlap_STR.SCI2BDHSTR.MergedBits.grpSBR_8
    #define SCI2BDH_SBR                 SCI2BDH_SBR_8
    
    #define SCI2BDH_SBR8_MASK           1U
    #define SCI2BDH_SBR9_MASK           2U
    #define SCI2BDH_SBR10_MASK          4U
    #define SCI2BDH_SBR11_MASK          8U
    #define SCI2BDH_SBR12_MASK          16U
    #define SCI2BDH_RXEDGIE_MASK        64U
    #define SCI2BDH_LBKDIE_MASK         128U
    #define SCI2BDH_SBR_8_MASK          31U
    #define SCI2BDH_SBR_8_BITNUM        0U
    

    /*** SCI2BDL - SCI2 Baud Rate Register Low; 0x00000041 ***/
    union {
      byte Byte;
      struct {
        byte SBR0        :1;                                       /* Baud Rate Modulo Divisor Bit 0 */
        byte SBR1        :1;                                       /* Baud Rate Modulo Divisor Bit 1 */
        byte SBR2        :1;                                       /* Baud Rate Modulo Divisor Bit 2 */
        byte SBR3        :1;                                       /* Baud Rate Modulo Divisor Bit 3 */
        byte SBR4        :1;                                       /* Baud Rate Modulo Divisor Bit 4 */
        byte SBR5        :1;                                       /* Baud Rate Modulo Divisor Bit 5 */
        byte SBR6        :1;                                       /* Baud Rate Modulo Divisor Bit 6 */
        byte SBR7        :1;                                       /* Baud Rate Modulo Divisor Bit 7 */
      } Bits;
    } SCI2BDLSTR;
    #define SCI2BDL                     _SCI2BD.Overlap_STR.SCI2BDLSTR.Byte
    #define SCI2BDL_SBR0                _SCI2BD.Overlap_STR.SCI2BDLSTR.Bits.SBR0
    #define SCI2BDL_SBR1                _SCI2BD.Overlap_STR.SCI2BDLSTR.Bits.SBR1
    #define SCI2BDL_SBR2                _SCI2BD.Overlap_STR.SCI2BDLSTR.Bits.SBR2
    #define SCI2BDL_SBR3                _SCI2BD.Overlap_STR.SCI2BDLSTR.Bits.SBR3
    #define SCI2BDL_SBR4                _SCI2BD.Overlap_STR.SCI2BDLSTR.Bits.SBR4
    #define SCI2BDL_SBR5                _SCI2BD.Overlap_STR.SCI2BDLSTR.Bits.SBR5
    #define SCI2BDL_SBR6                _SCI2BD.Overlap_STR.SCI2BDLSTR.Bits.SBR6
    #define SCI2BDL_SBR7                _SCI2BD.Overlap_STR.SCI2BDLSTR.Bits.SBR7
    
    #define SCI2BDL_SBR0_MASK           1U
    #define SCI2BDL_SBR1_MASK           2U
    #define SCI2BDL_SBR2_MASK           4U
    #define SCI2BDL_SBR3_MASK           8U
    #define SCI2BDL_SBR4_MASK           16U
    #define SCI2BDL_SBR5_MASK           32U
    #define SCI2BDL_SBR6_MASK           64U
    #define SCI2BDL_SBR7_MASK           128U
    
  } Overlap_STR;

} SCI2BDSTR;
extern volatile SCI2BDSTR _SCI2BD @0x00000040;
#define SCI2BD                          _SCI2BD.Word


/*** SCI2C1 - SCI2 Control Register 1; 0x00000042 ***/
typedef union {
  byte Byte;
  struct {
    byte PT          :1;                                       /* Parity Type */
    byte PE          :1;                                       /* Parity Enable */
    byte ILT         :1;                                       /* Idle Line Type Select */
    byte WAKE        :1;                                       /* Receiver Wakeup Method Select */
    byte M           :1;                                       /* 9-Bit or 8-Bit Mode Select */
    byte RSRC        :1;                                       /* Receiver Source Select */
    byte SCISWAI     :1;                                       /* SCI Stops in Wait Mode */
    byte LOOPS       :1;                                       /* Loop Mode Select */
  } Bits;
} SCI2C1STR;
extern volatile SCI2C1STR _SCI2C1 @0x00000042;
#define SCI2C1                          _SCI2C1.Byte
#define SCI2C1_PT                       _SCI2C1.Bits.PT
#define SCI2C1_PE                       _SCI2C1.Bits.PE
#define SCI2C1_ILT                      _SCI2C1.Bits.ILT
#define SCI2C1_WAKE                     _SCI2C1.Bits.WAKE
#define SCI2C1_M                        _SCI2C1.Bits.M
#define SCI2C1_RSRC                     _SCI2C1.Bits.RSRC
#define SCI2C1_SCISWAI                  _SCI2C1.Bits.SCISWAI
#define SCI2C1_LOOPS                    _SCI2C1.Bits.LOOPS

#define SCI2C1_PT_MASK                  1U
#define SCI2C1_PE_MASK                  2U
#define SCI2C1_ILT_MASK                 4U
#define SCI2C1_WAKE_MASK                8U
#define SCI2C1_M_MASK                   16U
#define SCI2C1_RSRC_MASK                32U
#define SCI2C1_SCISWAI_MASK             64U
#define SCI2C1_LOOPS_MASK               128U


/*** SCI2C2 - SCI2 Control Register 2; 0x00000043 ***/
typedef union {
  byte Byte;
  struct {
    byte SBK         :1;                                       /* Send Break */
    byte RWU         :1;                                       /* Receiver Wakeup Control */
    byte RE          :1;                                       /* Receiver Enable */
    byte TE          :1;                                       /* Transmitter Enable */
    byte ILIE        :1;                                       /* Idle Line Interrupt Enable (for IDLE) */
    byte RIE         :1;                                       /* Receiver Interrupt Enable (for RDRF) */
    byte TCIE        :1;                                       /* Transmission Complete Interrupt Enable (for TC) */
    byte TIE         :1;                                       /* Transmit Interrupt Enable (for TDRE) */
  } Bits;
} SCI2C2STR;
extern volatile SCI2C2STR _SCI2C2 @0x00000043;
#define SCI2C2                          _SCI2C2.Byte
#define SCI2C2_SBK                      _SCI2C2.Bits.SBK
#define SCI2C2_RWU                      _SCI2C2.Bits.RWU
#define SCI2C2_RE                       _SCI2C2.Bits.RE
#define SCI2C2_TE                       _SCI2C2.Bits.TE
#define SCI2C2_ILIE                     _SCI2C2.Bits.ILIE
#define SCI2C2_RIE                      _SCI2C2.Bits.RIE
#define SCI2C2_TCIE                     _SCI2C2.Bits.TCIE
#define SCI2C2_TIE                      _SCI2C2.Bits.TIE

#define SCI2C2_SBK_MASK                 1U
#define SCI2C2_RWU_MASK                 2U
#define SCI2C2_RE_MASK                  4U
#define SCI2C2_TE_MASK                  8U
#define SCI2C2_ILIE_MASK                16U
#define SCI2C2_RIE_MASK                 32U
#define SCI2C2_TCIE_MASK                64U
#define SCI2C2_TIE_MASK                 128U


/*** SCI2S1 - SCI2 Status Register 1; 0x00000044 ***/
typedef union {
  byte Byte;
  struct {
    byte PF          :1;                                       /* Parity Error Flag */
    byte FE          :1;                                       /* Framing Error Flag */
    byte NF          :1;                                       /* Noise Flag */
    byte OR          :1;                                       /* Receiver Overrun Flag */
    byte IDLE        :1;                                       /* Idle Line Flag */
    byte RDRF        :1;                                       /* Receive Data Register Full Flag */
    byte TC          :1;                                       /* Transmission Complete Flag */
    byte TDRE        :1;                                       /* Transmit Data Register Empty Flag */
  } Bits;
} SCI2S1STR;
extern volatile SCI2S1STR _SCI2S1 @0x00000044;
#define SCI2S1                          _SCI2S1.Byte
#define SCI2S1_PF                       _SCI2S1.Bits.PF
#define SCI2S1_FE                       _SCI2S1.Bits.FE
#define SCI2S1_NF                       _SCI2S1.Bits.NF
#define SCI2S1_OR                       _SCI2S1.Bits.OR
#define SCI2S1_IDLE                     _SCI2S1.Bits.IDLE
#define SCI2S1_RDRF                     _SCI2S1.Bits.RDRF
#define SCI2S1_TC                       _SCI2S1.Bits.TC
#define SCI2S1_TDRE                     _SCI2S1.Bits.TDRE

#define SCI2S1_PF_MASK                  1U
#define SCI2S1_FE_MASK                  2U
#define SCI2S1_NF_MASK                  4U
#define SCI2S1_OR_MASK                  8U
#define SCI2S1_IDLE_MASK                16U
#define SCI2S1_RDRF_MASK                32U
#define SCI2S1_TC_MASK                  64U
#define SCI2S1_TDRE_MASK                128U


/*** SCI2S2 - SCI2 Status Register 2; 0x00000045 ***/
typedef union {
  byte Byte;
  struct {
    byte RAF         :1;                                       /* Receiver Active Flag */
    byte LBKDE       :1;                                       /* LIN Break Detection Enable */
    byte BRK13       :1;                                       /* Break Character Generation Length */
    byte RWUID       :1;                                       /* Receive Wake Up Idle Detect */
    byte RXINV       :1;                                       /* Receive Data Inversion */
    byte             :1; 
    byte RXEDGIF     :1;                                       /* RxD Pin Active Edge Interrupt Flag */
    byte LBKDIF      :1;                                       /* LIN Break Detect Interrupt Flag */
  } Bits;
} SCI2S2STR;
extern volatile SCI2S2STR _SCI2S2 @0x00000045;
#define SCI2S2                          _SCI2S2.Byte
#define SCI2S2_RAF                      _SCI2S2.Bits.RAF
#define SCI2S2_LBKDE                    _SCI2S2.Bits.LBKDE
#define SCI2S2_BRK13                    _SCI2S2.Bits.BRK13
#define SCI2S2_RWUID                    _SCI2S2.Bits.RWUID
#define SCI2S2_RXINV                    _SCI2S2.Bits.RXINV
#define SCI2S2_RXEDGIF                  _SCI2S2.Bits.RXEDGIF
#define SCI2S2_LBKDIF                   _SCI2S2.Bits.LBKDIF

#define SCI2S2_RAF_MASK                 1U
#define SCI2S2_LBKDE_MASK               2U
#define SCI2S2_BRK13_MASK               4U
#define SCI2S2_RWUID_MASK               8U
#define SCI2S2_RXINV_MASK               16U
#define SCI2S2_RXEDGIF_MASK             64U
#define SCI2S2_LBKDIF_MASK              128U


/*** SCI2C3 - SCI2 Control Register 3; 0x00000046 ***/
typedef union {
  byte Byte;
  struct {
    byte PEIE        :1;                                       /* Parity Error Interrupt Enable */
    byte FEIE        :1;                                       /* Framing Error Interrupt Enable */
    byte NEIE        :1;                                       /* Noise Error Interrupt Enable */
    byte ORIE        :1;                                       /* Overrun Interrupt Enable */
    byte TXINV       :1;                                       /* Transmit Data Inversion */
    byte TXDIR       :1;                                       /* TxD Pin Direction in Single-Wire Mode */
    byte T8          :1;                                       /* Ninth Data Bit for Transmitter */
    byte R8          :1;                                       /* Ninth Data Bit for Receiver */
  } Bits;
} SCI2C3STR;
extern volatile SCI2C3STR _SCI2C3 @0x00000046;
#define SCI2C3                          _SCI2C3.Byte
#define SCI2C3_PEIE                     _SCI2C3.Bits.PEIE
#define SCI2C3_FEIE                     _SCI2C3.Bits.FEIE
#define SCI2C3_NEIE                     _SCI2C3.Bits.NEIE
#define SCI2C3_ORIE                     _SCI2C3.Bits.ORIE
#define SCI2C3_TXINV                    _SCI2C3.Bits.TXINV
#define SCI2C3_TXDIR                    _SCI2C3.Bits.TXDIR
#define SCI2C3_T8                       _SCI2C3.Bits.T8
#define SCI2C3_R8                       _SCI2C3.Bits.R8

#define SCI2C3_PEIE_MASK                1U
#define SCI2C3_FEIE_MASK                2U
#define SCI2C3_NEIE_MASK                4U
#define SCI2C3_ORIE_MASK                8U
#define SCI2C3_TXINV_MASK               16U
#define SCI2C3_TXDIR_MASK               32U
#define SCI2C3_T8_MASK                  64U
#define SCI2C3_R8_MASK                  128U


/*** SCI2D - SCI2 Data Register; 0x00000047 ***/
typedef union {
  byte Byte;
  struct {
    byte R0_T0       :1;                                       /* Receive/Transmit Data Bit 0 */
    byte R1_T1       :1;                                       /* Receive/Transmit Data Bit 1 */
    byte R2_T2       :1;                                       /* Receive/Transmit Data Bit 2 */
    byte R3_T3       :1;                                       /* Receive/Transmit Data Bit 3 */
    byte R4_T4       :1;                                       /* Receive/Transmit Data Bit 4 */
    byte R5_T5       :1;                                       /* Receive/Transmit Data Bit 5 */
    byte R6_T6       :1;                                       /* Receive/Transmit Data Bit 6 */
    byte R7_T7       :1;                                       /* Receive/Transmit Data Bit 7 */
  } Bits;
} SCI2DSTR;
extern volatile SCI2DSTR _SCI2D @0x00000047;
#define SCI2D                           _SCI2D.Byte
#define SCI2D_R0_T0                     _SCI2D.Bits.R0_T0
#define SCI2D_R1_T1                     _SCI2D.Bits.R1_T1
#define SCI2D_R2_T2                     _SCI2D.Bits.R2_T2
#define SCI2D_R3_T3                     _SCI2D.Bits.R3_T3
#define SCI2D_R4_T4                     _SCI2D.Bits.R4_T4
#define SCI2D_R5_T5                     _SCI2D.Bits.R5_T5
#define SCI2D_R6_T6                     _SCI2D.Bits.R6_T6
#define SCI2D_R7_T7                     _SCI2D.Bits.R7_T7

#define SCI2D_R0_T0_MASK                1U
#define SCI2D_R1_T1_MASK                2U
#define SCI2D_R2_T2_MASK                4U
#define SCI2D_R3_T3_MASK                8U
#define SCI2D_R4_T4_MASK                16U
#define SCI2D_R5_T5_MASK                32U
#define SCI2D_R6_T6_MASK                64U
#define SCI2D_R7_T7_MASK                128U


/*** ICGC1 - ICG Control Register 1; 0x00000048 ***/
typedef union {
  byte Byte;
  struct {
    byte             :1; 
    byte LOCD        :1;                                       /* Loss of Clock Disable */
    byte OSCSTEN     :1;                                       /* Enable Oscillator in Off Mode */
    byte CLKS0       :1;                                       /* Clock Mode Select, bit 0 */
    byte CLKS1       :1;                                       /* Clock Mode Select, bit 1 */
    byte REFS        :1;                                       /* External Reference Select */
    byte RANGE       :1;                                       /* Frequency Range Select */
    byte HGO         :1;                                       /* High Gain Oscillator Select */
  } Bits;
  struct {
    byte         :1;
    byte         :1;
    byte         :1;
    byte grpCLKS :2;
    byte         :1;
    byte         :1;
    byte         :1;
  } MergedBits;
} ICGC1STR;
extern volatile ICGC1STR _ICGC1 @0x00000048;
#define ICGC1                           _ICGC1.Byte
#define ICGC1_LOCD                      _ICGC1.Bits.LOCD
#define ICGC1_OSCSTEN                   _ICGC1.Bits.OSCSTEN
#define ICGC1_CLKS0                     _ICGC1.Bits.CLKS0
#define ICGC1_CLKS1                     _ICGC1.Bits.CLKS1
#define ICGC1_REFS                      _ICGC1.Bits.REFS
#define ICGC1_RANGE                     _ICGC1.Bits.RANGE
#define ICGC1_HGO                       _ICGC1.Bits.HGO
#define ICGC1_CLKS                      _ICGC1.MergedBits.grpCLKS

#define ICGC1_LOCD_MASK                 2U
#define ICGC1_OSCSTEN_MASK              4U
#define ICGC1_CLKS0_MASK                8U
#define ICGC1_CLKS1_MASK                16U
#define ICGC1_REFS_MASK                 32U
#define ICGC1_RANGE_MASK                64U
#define ICGC1_HGO_MASK                  128U
#define ICGC1_CLKS_MASK                 24U
#define ICGC1_CLKS_BITNUM               3U


/*** ICGC2 - ICG Control Register 2; 0x00000049 ***/
typedef union {
  byte Byte;
  struct {
    byte RFD0        :1;                                       /* Reduced Frequency Divider, bit 0 */
    byte RFD1        :1;                                       /* Reduced Frequency Divider, bit 1 */
    byte RFD2        :1;                                       /* Reduced Frequency Divider, bit 2 */
    byte LOCRE       :1;                                       /* Loss of Clock Reset Enable */
    byte MFD0        :1;                                       /* Multiplication Factor, bit 0 */
    byte MFD1        :1;                                       /* Multiplication Factor, bit 1 */
    byte MFD2        :1;                                       /* Multiplication Factor, bit 2 */
    byte LOLRE       :1;                                       /* Loss of Lock Reset Enable */
  } Bits;
  struct {
    byte grpRFD  :3;
    byte         :1;
    byte grpMFD  :3;
    byte         :1;
  } MergedBits;
} ICGC2STR;
extern volatile ICGC2STR _ICGC2 @0x00000049;
#define ICGC2                           _ICGC2.Byte
#define ICGC2_RFD0                      _ICGC2.Bits.RFD0
#define ICGC2_RFD1                      _ICGC2.Bits.RFD1
#define ICGC2_RFD2                      _ICGC2.Bits.RFD2
#define ICGC2_LOCRE                     _ICGC2.Bits.LOCRE
#define ICGC2_MFD0                      _ICGC2.Bits.MFD0
#define ICGC2_MFD1                      _ICGC2.Bits.MFD1
#define ICGC2_MFD2                      _ICGC2.Bits.MFD2
#define ICGC2_LOLRE                     _ICGC2.Bits.LOLRE
#define ICGC2_RFD                       _ICGC2.MergedBits.grpRFD
#define ICGC2_MFD                       _ICGC2.MergedBits.grpMFD

#define ICGC2_RFD0_MASK                 1U
#define ICGC2_RFD1_MASK                 2U
#define ICGC2_RFD2_MASK                 4U
#define ICGC2_LOCRE_MASK                8U
#define ICGC2_MFD0_MASK                 16U
#define ICGC2_MFD1_MASK                 32U
#define ICGC2_MFD2_MASK                 64U
#define ICGC2_LOLRE_MASK                128U
#define ICGC2_RFD_MASK                  7U
#define ICGC2_RFD_BITNUM                0U
#define ICGC2_MFD_MASK                  112U
#define ICGC2_MFD_BITNUM                4U


/*** ICGS1 - ICG Status Register 1; 0x0000004A ***/
typedef union {
  byte Byte;
  struct {
    byte ICGIF       :1;                                       /* ICG Interrupt Flag */
    byte ERCS        :1;                                       /* External Reference Clock Status */
    byte LOCS        :1;                                       /* Loss Of Clock Status */
    byte LOCK        :1;                                       /* FLL Lock Status */
    byte LOLS        :1;                                       /* FLL Loss of Lock Status */
    byte REFST       :1;                                       /* Reference Clock Status */
    byte CLKST0      :1;                                       /* Clock Mode Status, bit 0 */
    byte CLKST1      :1;                                       /* Clock Mode Status, bit 1 */
  } Bits;
  struct {
    byte         :1;
    byte         :1;
    byte         :1;
    byte         :1;
    byte         :1;
    byte         :1;
    byte grpCLKST :2;
  } MergedBits;
} ICGS1STR;
extern volatile ICGS1STR _ICGS1 @0x0000004A;
#define ICGS1                           _ICGS1.Byte
#define ICGS1_ICGIF                     _ICGS1.Bits.ICGIF
#define ICGS1_ERCS                      _ICGS1.Bits.ERCS
#define ICGS1_LOCS                      _ICGS1.Bits.LOCS
#define ICGS1_LOCK                      _ICGS1.Bits.LOCK
#define ICGS1_LOLS                      _ICGS1.Bits.LOLS
#define ICGS1_REFST                     _ICGS1.Bits.REFST
#define ICGS1_CLKST0                    _ICGS1.Bits.CLKST0
#define ICGS1_CLKST1                    _ICGS1.Bits.CLKST1
#define ICGS1_CLKST                     _ICGS1.MergedBits.grpCLKST

#define ICGS1_ICGIF_MASK                1U
#define ICGS1_ERCS_MASK                 2U
#define ICGS1_LOCS_MASK                 4U
#define ICGS1_LOCK_MASK                 8U
#define ICGS1_LOLS_MASK                 16U
#define ICGS1_REFST_MASK                32U
#define ICGS1_CLKST0_MASK               64U
#define ICGS1_CLKST1_MASK               128U
#define ICGS1_CLKST_MASK                192U
#define ICGS1_CLKST_BITNUM              6U


/*** ICGS2 - ICG Status Register 2; 0x0000004B ***/
typedef union {
  byte Byte;
  struct {
    byte DCOS        :1;                                       /* DCO Clock Stable */
    byte             :1; 
    byte             :1; 
    byte             :1; 
    byte             :1; 
    byte             :1; 
    byte             :1; 
    byte             :1; 
  } Bits;
} ICGS2STR;
extern volatile ICGS2STR _ICGS2 @0x0000004B;
#define ICGS2                           _ICGS2.Byte
#define ICGS2_DCOS                      _ICGS2.Bits.DCOS

#define ICGS2_DCOS_MASK                 1U


/*** ICGFLT - ICG Filter Register; 0x0000004C ***/
typedef union {
  word Word;
   /* Overlapped registers: */
  struct {
    /*** ICGFLTU - ICG Upper Filter Register; 0x0000004C ***/
    union {
      byte Byte;
      struct {
        byte FLT8        :1;                                       /* Filter Value, bit 8 */
        byte FLT9        :1;                                       /* Filter Value, bit 9 */
        byte FLT10       :1;                                       /* Filter Value, bit 10 */
        byte FLT11       :1;                                       /* Filter Value, bit 11 */
        byte             :1; 
        byte             :1; 
        byte             :1; 
        byte             :1; 
      } Bits;
      struct {
        byte grpFLT_8 :4;
        byte     :1;
        byte     :1;
        byte     :1;
        byte     :1;
      } MergedBits;
    } ICGFLTUSTR;
    #define ICGFLTU                     _ICGFLT.Overlap_STR.ICGFLTUSTR.Byte
    #define ICGFLTU_FLT8                _ICGFLT.Overlap_STR.ICGFLTUSTR.Bits.FLT8
    #define ICGFLTU_FLT9                _ICGFLT.Overlap_STR.ICGFLTUSTR.Bits.FLT9
    #define ICGFLTU_FLT10               _ICGFLT.Overlap_STR.ICGFLTUSTR.Bits.FLT10
    #define ICGFLTU_FLT11               _ICGFLT.Overlap_STR.ICGFLTUSTR.Bits.FLT11
    #define ICGFLTU_FLT_8               _ICGFLT.Overlap_STR.ICGFLTUSTR.MergedBits.grpFLT_8
    #define ICGFLTU_FLT                 ICGFLTU_FLT_8
    
    #define ICGFLTU_FLT8_MASK           1U
    #define ICGFLTU_FLT9_MASK           2U
    #define ICGFLTU_FLT10_MASK          4U
    #define ICGFLTU_FLT11_MASK          8U
    #define ICGFLTU_FLT_8_MASK          15U
    #define ICGFLTU_FLT_8_BITNUM        0U
    

    /*** ICGFLTL - ICG Lower Filter Register; 0x0000004D ***/
    union {
      byte Byte;
      struct {
        byte FLT0        :1;                                       /* Filter Value, bit 0 */
        byte FLT1        :1;                                       /* Filter Value, bit 1 */
        byte FLT2        :1;                                       /* Filter Value, bit 2 */
        byte FLT3        :1;                                       /* Filter Value, bit 3 */
        byte FLT4        :1;                                       /* Filter Value, bit 4 */
        byte FLT5        :1;                                       /* Filter Value, bit 5 */
        byte FLT6        :1;                                       /* Filter Value, bit 6 */
        byte FLT7        :1;                                       /* Filter Value, bit 7 */
      } Bits;
    } ICGFLTLSTR;
    #define ICGFLTL                     _ICGFLT.Overlap_STR.ICGFLTLSTR.Byte
    #define ICGFLTL_FLT0                _ICGFLT.Overlap_STR.ICGFLTLSTR.Bits.FLT0
    #define ICGFLTL_FLT1                _ICGFLT.Overlap_STR.ICGFLTLSTR.Bits.FLT1
    #define ICGFLTL_FLT2                _ICGFLT.Overlap_STR.ICGFLTLSTR.Bits.FLT2
    #define ICGFLTL_FLT3                _ICGFLT.Overlap_STR.ICGFLTLSTR.Bits.FLT3
    #define ICGFLTL_FLT4                _ICGFLT.Overlap_STR.ICGFLTLSTR.Bits.FLT4
    #define ICGFLTL_FLT5                _ICGFLT.Overlap_STR.ICGFLTLSTR.Bits.FLT5
    #define ICGFLTL_FLT6                _ICGFLT.Overlap_STR.ICGFLTLSTR.Bits.FLT6
    #define ICGFLTL_FLT7                _ICGFLT.Overlap_STR.ICGFLTLSTR.Bits.FLT7
    
    #define ICGFLTL_FLT0_MASK           1U
    #define ICGFLTL_FLT1_MASK           2U
    #define ICGFLTL_FLT2_MASK           4U
    #define ICGFLTL_FLT3_MASK           8U
    #define ICGFLTL_FLT4_MASK           16U
    #define ICGFLTL_FLT5_MASK           32U
    #define ICGFLTL_FLT6_MASK           64U
    #define ICGFLTL_FLT7_MASK           128U
    
  } Overlap_STR;

} ICGFLTSTR;
extern volatile ICGFLTSTR _ICGFLT @0x0000004C;
#define ICGFLT                          _ICGFLT.Word


/*** ICGTRM - ICG Trim Register; 0x0000004E ***/
typedef union {
  byte Byte;
  struct {
    byte TRIM0       :1;                                       /* ICG Trim Setting, bit 0 */
    byte TRIM1       :1;                                       /* ICG Trim Setting, bit 1 */
    byte TRIM2       :1;                                       /* ICG Trim Setting, bit 2 */
    byte TRIM3       :1;                                       /* ICG Trim Setting, bit 3 */
    byte TRIM4       :1;                                       /* ICG Trim Setting, bit 4 */
    byte TRIM5       :1;                                       /* ICG Trim Setting, bit 5 */
    byte TRIM6       :1;                                       /* ICG Trim Setting, bit 6 */
    byte TRIM7       :1;                                       /* ICG Trim Setting, bit 7 */
  } Bits;
} ICGTRMSTR;
extern volatile ICGTRMSTR _ICGTRM @0x0000004E;
#define ICGTRM                          _ICGTRM.Byte
#define ICGTRM_TRIM0                    _ICGTRM.Bits.TRIM0
#define ICGTRM_TRIM1                    _ICGTRM.Bits.TRIM1
#define ICGTRM_TRIM2                    _ICGTRM.Bits.TRIM2
#define ICGTRM_TRIM3                    _ICGTRM.Bits.TRIM3
#define ICGTRM_TRIM4                    _ICGTRM.Bits.TRIM4
#define ICGTRM_TRIM5                    _ICGTRM.Bits.TRIM5
#define ICGTRM_TRIM6                    _ICGTRM.Bits.TRIM6
#define ICGTRM_TRIM7                    _ICGTRM.Bits.TRIM7

#define ICGTRM_TRIM0_MASK               1U
#define ICGTRM_TRIM1_MASK               2U
#define ICGTRM_TRIM2_MASK               4U
#define ICGTRM_TRIM3_MASK               8U
#define ICGTRM_TRIM4_MASK               16U
#define ICGTRM_TRIM5_MASK               32U
#define ICGTRM_TRIM6_MASK               64U
#define ICGTRM_TRIM7_MASK               128U


/*** SPI1C1 - SPI1 Control Register 1; 0x00000050 ***/
typedef union {
  byte Byte;
  struct {
    byte LSBFE       :1;                                       /* LSB First (Shifter Direction) */
    byte SSOE        :1;                                       /* Slave Select Output Enable */
    byte CPHA        :1;                                       /* Clock Phase */
    byte CPOL        :1;                                       /* Clock Polarity */
    byte MSTR        :1;                                       /* Master/Slave Mode Select */
    byte SPTIE       :1;                                       /* SPI Transmit Interrupt Enable */
    byte SPE         :1;                                       /* SPI System Enable */
    byte SPIE        :1;                                       /* SPI Interrupt Enable (for SPRF and MODF) */
  } Bits;
} SPI1C1STR;
extern volatile SPI1C1STR _SPI1C1 @0x00000050;
#define SPI1C1                          _SPI1C1.Byte
#define SPI1C1_LSBFE                    _SPI1C1.Bits.LSBFE
#define SPI1C1_SSOE                     _SPI1C1.Bits.SSOE
#define SPI1C1_CPHA                     _SPI1C1.Bits.CPHA
#define SPI1C1_CPOL                     _SPI1C1.Bits.CPOL
#define SPI1C1_MSTR                     _SPI1C1.Bits.MSTR
#define SPI1C1_SPTIE                    _SPI1C1.Bits.SPTIE
#define SPI1C1_SPE                      _SPI1C1.Bits.SPE
#define SPI1C1_SPIE                     _SPI1C1.Bits.SPIE

#define SPI1C1_LSBFE_MASK               1U
#define SPI1C1_SSOE_MASK                2U
#define SPI1C1_CPHA_MASK                4U
#define SPI1C1_CPOL_MASK                8U
#define SPI1C1_MSTR_MASK                16U
#define SPI1C1_SPTIE_MASK               32U
#define SPI1C1_SPE_MASK                 64U
#define SPI1C1_SPIE_MASK                128U


/*** SPI1C2 - SPI1 Control Register 2; 0x00000051 ***/
typedef union {
  byte Byte;
  struct {
    byte SPC0        :1;                                       /* SPI Pin Control 0 */
    byte SPISWAI     :1;                                       /* SPI Stop in Wait Mode */
    byte             :1; 
    byte BIDIROE     :1;                                       /* Bidirectional Mode Output Enable */
    byte MODFEN      :1;                                       /* Master Mode-Fault Function Enable */
    byte             :1; 
    byte             :1; 
    byte             :1; 
  } Bits;
} SPI1C2STR;
extern volatile SPI1C2STR _SPI1C2 @0x00000051;
#define SPI1C2                          _SPI1C2.Byte
#define SPI1C2_SPC0                     _SPI1C2.Bits.SPC0
#define SPI1C2_SPISWAI                  _SPI1C2.Bits.SPISWAI
#define SPI1C2_BIDIROE                  _SPI1C2.Bits.BIDIROE
#define SPI1C2_MODFEN                   _SPI1C2.Bits.MODFEN

#define SPI1C2_SPC0_MASK                1U
#define SPI1C2_SPISWAI_MASK             2U
#define SPI1C2_BIDIROE_MASK             8U
#define SPI1C2_MODFEN_MASK              16U


/*** SPI1BR - SPI1 Baud Rate Register; 0x00000052 ***/
typedef union {
  byte Byte;
  struct {
    byte SPR0        :1;                                       /* SPI Baud Rate Divisor Bit 0 */
    byte SPR1        :1;                                       /* SPI Baud Rate Divisor Bit 1 */
    byte SPR2        :1;                                       /* SPI Baud Rate Divisor Bit 2 */
    byte             :1; 
    byte SPPR0       :1;                                       /* SPI Baud Rate Prescale Divisor Bit 0 */
    byte SPPR1       :1;                                       /* SPI Baud Rate Prescale Divisor Bit 1 */
    byte SPPR2       :1;                                       /* SPI Baud Rate Prescale Divisor Bit 2 */
    byte             :1; 
  } Bits;
  struct {
    byte grpSPR  :3;
    byte         :1;
    byte grpSPPR :3;
    byte         :1;
  } MergedBits;
} SPI1BRSTR;
extern volatile SPI1BRSTR _SPI1BR @0x00000052;
#define SPI1BR                          _SPI1BR.Byte
#define SPI1BR_SPR0                     _SPI1BR.Bits.SPR0
#define SPI1BR_SPR1                     _SPI1BR.Bits.SPR1
#define SPI1BR_SPR2                     _SPI1BR.Bits.SPR2
#define SPI1BR_SPPR0                    _SPI1BR.Bits.SPPR0
#define SPI1BR_SPPR1                    _SPI1BR.Bits.SPPR1
#define SPI1BR_SPPR2                    _SPI1BR.Bits.SPPR2
#define SPI1BR_SPR                      _SPI1BR.MergedBits.grpSPR
#define SPI1BR_SPPR                     _SPI1BR.MergedBits.grpSPPR

#define SPI1BR_SPR0_MASK                1U
#define SPI1BR_SPR1_MASK                2U
#define SPI1BR_SPR2_MASK                4U
#define SPI1BR_SPPR0_MASK               16U
#define SPI1BR_SPPR1_MASK               32U
#define SPI1BR_SPPR2_MASK               64U
#define SPI1BR_SPR_MASK                 7U
#define SPI1BR_SPR_BITNUM               0U
#define SPI1BR_SPPR_MASK                112U
#define SPI1BR_SPPR_BITNUM              4U


/*** SPI1S - SPI1 Status Register; 0x00000053 ***/
typedef union {
  byte Byte;
  struct {
    byte             :1; 
    byte             :1; 
    byte             :1; 
    byte             :1; 
    byte MODF        :1;                                       /* Master Mode Fault Flag */
    byte SPTEF       :1;                                       /* SPI Transmit Buffer Empty Flag */
    byte             :1; 
    byte SPRF        :1;                                       /* SPI Read Buffer Full Flag */
  } Bits;
} SPI1SSTR;
extern volatile SPI1SSTR _SPI1S @0x00000053;
#define SPI1S                           _SPI1S.Byte
#define SPI1S_MODF                      _SPI1S.Bits.MODF
#define SPI1S_SPTEF                     _SPI1S.Bits.SPTEF
#define SPI1S_SPRF                      _SPI1S.Bits.SPRF

#define SPI1S_MODF_MASK                 16U
#define SPI1S_SPTEF_MASK                32U
#define SPI1S_SPRF_MASK                 128U


/*** SPI1D - SPI1 Data Register; 0x00000055 ***/
typedef union {
  byte Byte;
} SPI1DSTR;
extern volatile SPI1DSTR _SPI1D @0x00000055;
#define SPI1D                           _SPI1D.Byte


/*** CRCH - CRC High Register; 0x00000056 ***/
typedef union {
  byte Byte;
  struct {
    byte BIT8        :1;                                       /* High byte of CRC register ,bit 8 */
    byte BIT9        :1;                                       /* High byte of CRC register ,bit 9 */
    byte BIT10       :1;                                       /* High byte of CRC register ,bit 10 */
    byte BIT11       :1;                                       /* High byte of CRC register ,bit 11 */
    byte BIT12       :1;                                       /* High byte of CRC register ,bit 12 */
    byte BIT13       :1;                                       /* High byte of CRC register ,bit 13 */
    byte BIT14       :1;                                       /* High byte of CRC register ,bit 14 */
    byte BIT15       :1;                                       /* High byte of CRC register ,bit 15 */
  } Bits;
} CRCHSTR;
extern volatile CRCHSTR _CRCH @0x00000056;
#define CRCH                            _CRCH.Byte
#define CRCH_BIT8                       _CRCH.Bits.BIT8
#define CRCH_BIT9                       _CRCH.Bits.BIT9
#define CRCH_BIT10                      _CRCH.Bits.BIT10
#define CRCH_BIT11                      _CRCH.Bits.BIT11
#define CRCH_BIT12                      _CRCH.Bits.BIT12
#define CRCH_BIT13                      _CRCH.Bits.BIT13
#define CRCH_BIT14                      _CRCH.Bits.BIT14
#define CRCH_BIT15                      _CRCH.Bits.BIT15

#define CRCH_BIT8_MASK                  1U
#define CRCH_BIT9_MASK                  2U
#define CRCH_BIT10_MASK                 4U
#define CRCH_BIT11_MASK                 8U
#define CRCH_BIT12_MASK                 16U
#define CRCH_BIT13_MASK                 32U
#define CRCH_BIT14_MASK                 64U
#define CRCH_BIT15_MASK                 128U


/*** CRCL - CRC Low Register; 0x00000057 ***/
typedef union {
  byte Byte;
  struct {
    byte BIT0        :1;                                       /* Low byte of CRC register, bit 0 */
    byte BIT1        :1;                                       /* Low byte of CRC register, bit 1 */
    byte BIT2        :1;                                       /* Low byte of CRC register, bit 2 */
    byte BIT3        :1;                                       /* Low byte of CRC register, bit 3 */
    byte BIT4        :1;                                       /* Low byte of CRC register, bit 4 */
    byte BIT5        :1;                                       /* Low byte of CRC register, bit 5 */
    byte BIT6        :1;                                       /* Low byte of CRC register, bit 6 */
    byte BIT7        :1;                                       /* Low byte of CRC register, bit 7 */
  } Bits;
} CRCLSTR;
extern volatile CRCLSTR _CRCL @0x00000057;
#define CRCL                            _CRCL.Byte
#define CRCL_BIT0                       _CRCL.Bits.BIT0
#define CRCL_BIT1                       _CRCL.Bits.BIT1
#define CRCL_BIT2                       _CRCL.Bits.BIT2
#define CRCL_BIT3                       _CRCL.Bits.BIT3
#define CRCL_BIT4                       _CRCL.Bits.BIT4
#define CRCL_BIT5                       _CRCL.Bits.BIT5
#define CRCL_BIT6                       _CRCL.Bits.BIT6
#define CRCL_BIT7                       _CRCL.Bits.BIT7

#define CRCL_BIT0_MASK                  1U
#define CRCL_BIT1_MASK                  2U
#define CRCL_BIT2_MASK                  4U
#define CRCL_BIT3_MASK                  8U
#define CRCL_BIT4_MASK                  16U
#define CRCL_BIT5_MASK                  32U
#define CRCL_BIT6_MASK                  64U
#define CRCL_BIT7_MASK                  128U


/*** IIC1A - IIC Address Register; 0x00000058 ***/
typedef union {
  byte Byte;
  struct {
    byte             :1; 
    byte AD1         :1;                                       /* Slave Address Bit 1 */
    byte AD2         :1;                                       /* Slave Address Bit 2 */
    byte AD3         :1;                                       /* Slave Address Bit 3 */
    byte AD4         :1;                                       /* Slave Address Bit 4 */
    byte AD5         :1;                                       /* Slave Address Bit 5 */
    byte AD6         :1;                                       /* Slave Address Bit 6 */
    byte AD7         :1;                                       /* Slave Address Bit 7 */
  } Bits;
  struct {
    byte         :1;
    byte grpAD_1 :7;
  } MergedBits;
} IIC1ASTR;
extern volatile IIC1ASTR _IIC1A @0x00000058;
#define IIC1A                           _IIC1A.Byte
#define IIC1A_AD1                       _IIC1A.Bits.AD1
#define IIC1A_AD2                       _IIC1A.Bits.AD2
#define IIC1A_AD3                       _IIC1A.Bits.AD3
#define IIC1A_AD4                       _IIC1A.Bits.AD4
#define IIC1A_AD5                       _IIC1A.Bits.AD5
#define IIC1A_AD6                       _IIC1A.Bits.AD6
#define IIC1A_AD7                       _IIC1A.Bits.AD7
#define IIC1A_AD_1                      _IIC1A.MergedBits.grpAD_1
#define IIC1A_AD                        IIC1A_AD_1

#define IIC1A_AD1_MASK                  2U
#define IIC1A_AD2_MASK                  4U
#define IIC1A_AD3_MASK                  8U
#define IIC1A_AD4_MASK                  16U
#define IIC1A_AD5_MASK                  32U
#define IIC1A_AD6_MASK                  64U
#define IIC1A_AD7_MASK                  128U
#define IIC1A_AD_1_MASK                 254U
#define IIC1A_AD_1_BITNUM               1U


/*** IIC1F - IIC Frequency Divider Register; 0x00000059 ***/
typedef union {
  byte Byte;
  struct {
    byte ICR0        :1;                                       /* IIC Clock Rate Bit 0 */
    byte ICR1        :1;                                       /* IIC Clock Rate Bit 1 */
    byte ICR2        :1;                                       /* IIC Clock Rate Bit 2 */
    byte ICR3        :1;                                       /* IIC Clock Rate Bit 3 */
    byte ICR4        :1;                                       /* IIC Clock Rate Bit 4 */
    byte ICR5        :1;                                       /* IIC Clock Rate Bit 5 */
    byte MULT0       :1;                                       /* Multiplier Factor Bit 0 */
    byte MULT1       :1;                                       /* Multiplier Factor Bit 1 */
  } Bits;
  struct {
    byte grpICR  :6;
    byte grpMULT :2;
  } MergedBits;
} IIC1FSTR;
extern volatile IIC1FSTR _IIC1F @0x00000059;
#define IIC1F                           _IIC1F.Byte
#define IIC1F_ICR0                      _IIC1F.Bits.ICR0
#define IIC1F_ICR1                      _IIC1F.Bits.ICR1
#define IIC1F_ICR2                      _IIC1F.Bits.ICR2
#define IIC1F_ICR3                      _IIC1F.Bits.ICR3
#define IIC1F_ICR4                      _IIC1F.Bits.ICR4
#define IIC1F_ICR5                      _IIC1F.Bits.ICR5
#define IIC1F_MULT0                     _IIC1F.Bits.MULT0
#define IIC1F_MULT1                     _IIC1F.Bits.MULT1
#define IIC1F_ICR                       _IIC1F.MergedBits.grpICR
#define IIC1F_MULT                      _IIC1F.MergedBits.grpMULT

#define IIC1F_ICR0_MASK                 1U
#define IIC1F_ICR1_MASK                 2U
#define IIC1F_ICR2_MASK                 4U
#define IIC1F_ICR3_MASK                 8U
#define IIC1F_ICR4_MASK                 16U
#define IIC1F_ICR5_MASK                 32U
#define IIC1F_MULT0_MASK                64U
#define IIC1F_MULT1_MASK                128U
#define IIC1F_ICR_MASK                  63U
#define IIC1F_ICR_BITNUM                0U
#define IIC1F_MULT_MASK                 192U
#define IIC1F_MULT_BITNUM               6U


/*** IIC1C1 - IIC Control Register 1; 0x0000005A ***/
typedef union {
  byte Byte;
  union { /* Several registers at the same address */
    /*** IIC1C1 - IIC Control Register 1; Several registers at the same address ***/
    union {
      struct {
        byte             :1; 
        byte             :1; 
        byte RSTA        :1;                                       /* Repeat START */
        byte TXAK        :1;                                       /* Transmit Acknowledge Enable */
        byte TX          :1;                                       /* Transmit Mode Select */
        byte MST         :1;                                       /* Master Mode Select */
        byte IICIE       :1;                                       /* IIC Interrupt Enable */
        byte IICEN       :1;                                       /* IIC Enable */
      } Bits;
    } IIC1C1STR;
    #define IIC1C1                      _IIC1C1.Byte
    #define IIC1C1_RSTA                 _IIC1C1.SameAddr_STR.IIC1C1STR.Bits.RSTA
    #define IIC1C1_TXAK                 _IIC1C1.SameAddr_STR.IIC1C1STR.Bits.TXAK
    #define IIC1C1_TX                   _IIC1C1.SameAddr_STR.IIC1C1STR.Bits.TX
    #define IIC1C1_MST                  _IIC1C1.SameAddr_STR.IIC1C1STR.Bits.MST
    #define IIC1C1_IICIE                _IIC1C1.SameAddr_STR.IIC1C1STR.Bits.IICIE
    #define IIC1C1_IICEN                _IIC1C1.SameAddr_STR.IIC1C1STR.Bits.IICEN
    
    #define IIC1C1_RSTA_MASK            4U
    #define IIC1C1_TXAK_MASK            8U
    #define IIC1C1_TX_MASK              16U
    #define IIC1C1_MST_MASK             32U
    #define IIC1C1_IICIE_MASK           64U
    #define IIC1C1_IICEN_MASK           128U
    
    /*** IIC1C - IIC Control Register; Several registers at the same address ***/
    union {
      struct {
        byte             :1; 
        byte             :1; 
        byte RSTA        :1;                                       /* Repeat START */
        byte TXAK        :1;                                       /* Transmit Acknowledge Enable */
        byte TX          :1;                                       /* Transmit Mode Select */
        byte MST         :1;                                       /* Master Mode Select */
        byte IICIE       :1;                                       /* IIC Interrupt Enable */
        byte IICEN       :1;                                       /* IIC Enable */
      } Bits;
    } IIC1CSTR;
    #define IIC1C                       _IIC1C1.Byte
    #define IIC1C_RSTA                  _IIC1C1.SameAddr_STR.IIC1CSTR.Bits.RSTA
    #define IIC1C_TXAK                  _IIC1C1.SameAddr_STR.IIC1CSTR.Bits.TXAK
    #define IIC1C_TX                    _IIC1C1.SameAddr_STR.IIC1CSTR.Bits.TX
    #define IIC1C_MST                   _IIC1C1.SameAddr_STR.IIC1CSTR.Bits.MST
    #define IIC1C_IICIE                 _IIC1C1.SameAddr_STR.IIC1CSTR.Bits.IICIE
    #define IIC1C_IICEN                 _IIC1C1.SameAddr_STR.IIC1CSTR.Bits.IICEN
    
    #define IIC1C_RSTA_MASK             4U
    #define IIC1C_TXAK_MASK             8U
    #define IIC1C_TX_MASK               16U
    #define IIC1C_MST_MASK              32U
    #define IIC1C_IICIE_MASK            64U
    #define IIC1C_IICEN_MASK            128U
    
  } SameAddr_STR; /*Several registers at the same address */

} IIC1C1STR;
extern volatile IIC1C1STR _IIC1C1 @0x0000005A;


/*** IIC1S - IIC Status Register; 0x0000005B ***/
typedef union {
  byte Byte;
  struct {
    byte RXAK        :1;                                       /* Receive Acknowledge */
    byte IICIF       :1;                                       /* IIC Interrupt Flag */
    byte SRW         :1;                                       /* Slave Read/Write */
    byte             :1; 
    byte ARBL        :1;                                       /* Arbitration Lost */
    byte BUSY        :1;                                       /* Bus Busy */
    byte IAAS        :1;                                       /* Addressed as a Slave */
    byte TCF         :1;                                       /* Transfer Complete Flag */
  } Bits;
} IIC1SSTR;
extern volatile IIC1SSTR _IIC1S @0x0000005B;
#define IIC1S                           _IIC1S.Byte
#define IIC1S_RXAK                      _IIC1S.Bits.RXAK
#define IIC1S_IICIF                     _IIC1S.Bits.IICIF
#define IIC1S_SRW                       _IIC1S.Bits.SRW
#define IIC1S_ARBL                      _IIC1S.Bits.ARBL
#define IIC1S_BUSY                      _IIC1S.Bits.BUSY
#define IIC1S_IAAS                      _IIC1S.Bits.IAAS
#define IIC1S_TCF                       _IIC1S.Bits.TCF

#define IIC1S_RXAK_MASK                 1U
#define IIC1S_IICIF_MASK                2U
#define IIC1S_SRW_MASK                  4U
#define IIC1S_ARBL_MASK                 16U
#define IIC1S_BUSY_MASK                 32U
#define IIC1S_IAAS_MASK                 64U
#define IIC1S_TCF_MASK                  128U


/*** IIC1D - IIC Data I/O Register; 0x0000005C ***/
typedef union {
  byte Byte;
  struct {
    byte DATA0       :1;                                       /* IIC Data Bit 0 */
    byte DATA1       :1;                                       /* IIC Data Bit 1 */
    byte DATA2       :1;                                       /* IIC Data Bit 2 */
    byte DATA3       :1;                                       /* IIC Data Bit 3 */
    byte DATA4       :1;                                       /* IIC Data Bit 4 */
    byte DATA5       :1;                                       /* IIC Data Bit 5 */
    byte DATA6       :1;                                       /* IIC Data Bit 6 */
    byte DATA7       :1;                                       /* IIC Data Bit 7 */
  } Bits;
} IIC1DSTR;
extern volatile IIC1DSTR _IIC1D @0x0000005C;
#define IIC1D                           _IIC1D.Byte
#define IIC1D_DATA0                     _IIC1D.Bits.DATA0
#define IIC1D_DATA1                     _IIC1D.Bits.DATA1
#define IIC1D_DATA2                     _IIC1D.Bits.DATA2
#define IIC1D_DATA3                     _IIC1D.Bits.DATA3
#define IIC1D_DATA4                     _IIC1D.Bits.DATA4
#define IIC1D_DATA5                     _IIC1D.Bits.DATA5
#define IIC1D_DATA6                     _IIC1D.Bits.DATA6
#define IIC1D_DATA7                     _IIC1D.Bits.DATA7

#define IIC1D_DATA0_MASK                1U
#define IIC1D_DATA1_MASK                2U
#define IIC1D_DATA2_MASK                4U
#define IIC1D_DATA3_MASK                8U
#define IIC1D_DATA4_MASK                16U
#define IIC1D_DATA5_MASK                32U
#define IIC1D_DATA6_MASK                64U
#define IIC1D_DATA7_MASK                128U


/*** IIC1C2 - IIC Control Register 2; 0x0000005D ***/
typedef union {
  byte Byte;
  struct {
    byte AD8         :1;                                       /* Slave Address Bit 8 */
    byte AD9         :1;                                       /* Slave Address Bit 9 */
    byte AD10        :1;                                       /* Slave Address Bit 10 */
    byte             :1; 
    byte             :1; 
    byte             :1; 
    byte ADEXT       :1;                                       /* Address Extension */
    byte GCAEN       :1;                                       /* General Call Address Enable */
  } Bits;
  struct {
    byte grpAD_8 :3;
    byte         :1;
    byte         :1;
    byte         :1;
    byte         :1;
    byte         :1;
  } MergedBits;
} IIC1C2STR;
extern volatile IIC1C2STR _IIC1C2 @0x0000005D;
#define IIC1C2                          _IIC1C2.Byte
#define IIC1C2_AD8                      _IIC1C2.Bits.AD8
#define IIC1C2_AD9                      _IIC1C2.Bits.AD9
#define IIC1C2_AD10                     _IIC1C2.Bits.AD10
#define IIC1C2_ADEXT                    _IIC1C2.Bits.ADEXT
#define IIC1C2_GCAEN                    _IIC1C2.Bits.GCAEN
#define IIC1C2_AD_8                     _IIC1C2.MergedBits.grpAD_8
#define IIC1C2_AD                       IIC1C2_AD_8

#define IIC1C2_AD8_MASK                 1U
#define IIC1C2_AD9_MASK                 2U
#define IIC1C2_AD10_MASK                4U
#define IIC1C2_ADEXT_MASK               64U
#define IIC1C2_GCAEN_MASK               128U
#define IIC1C2_AD_8_MASK                7U
#define IIC1C2_AD_8_BITNUM              0U


/*** TPM2SC - TPM2 Status and Control Register; 0x00000060 ***/
typedef union {
  byte Byte;
  struct {
    byte PS0         :1;                                       /* Prescale Divisor Select Bit 0 */
    byte PS1         :1;                                       /* Prescale Divisor Select Bit 1 */
    byte PS2         :1;                                       /* Prescale Divisor Select Bit 2 */
    byte CLKSA       :1;                                       /* Clock Source Select A */
    byte CLKSB       :1;                                       /* Clock Source Select B */
    byte CPWMS       :1;                                       /* Center-Aligned PWM Select */
    byte TOIE        :1;                                       /* Timer Overflow Interrupt Enable */
    byte TOF         :1;                                       /* Timer Overflow Flag */
  } Bits;
  struct {
    byte grpPS   :3;
    byte grpCLKSx :2;
    byte         :1;
    byte         :1;
    byte         :1;
  } MergedBits;
} TPM2SCSTR;
extern volatile TPM2SCSTR _TPM2SC @0x00000060;
#define TPM2SC                          _TPM2SC.Byte
#define TPM2SC_PS0                      _TPM2SC.Bits.PS0
#define TPM2SC_PS1                      _TPM2SC.Bits.PS1
#define TPM2SC_PS2                      _TPM2SC.Bits.PS2
#define TPM2SC_CLKSA                    _TPM2SC.Bits.CLKSA
#define TPM2SC_CLKSB                    _TPM2SC.Bits.CLKSB
#define TPM2SC_CPWMS                    _TPM2SC.Bits.CPWMS
#define TPM2SC_TOIE                     _TPM2SC.Bits.TOIE
#define TPM2SC_TOF                      _TPM2SC.Bits.TOF
#define TPM2SC_PS                       _TPM2SC.MergedBits.grpPS
#define TPM2SC_CLKSx                    _TPM2SC.MergedBits.grpCLKSx

#define TPM2SC_PS0_MASK                 1U
#define TPM2SC_PS1_MASK                 2U
#define TPM2SC_PS2_MASK                 4U
#define TPM2SC_CLKSA_MASK               8U
#define TPM2SC_CLKSB_MASK               16U
#define TPM2SC_CPWMS_MASK               32U
#define TPM2SC_TOIE_MASK                64U
#define TPM2SC_TOF_MASK                 128U
#define TPM2SC_PS_MASK                  7U
#define TPM2SC_PS_BITNUM                0U
#define TPM2SC_CLKSx_MASK               24U
#define TPM2SC_CLKSx_BITNUM             3U


/*** TPM2CNT - TPM2 Timer Counter Register; 0x00000061 ***/
typedef union {
  word Word;
   /* Overlapped registers: */
  struct {
    /*** TPM2CNTH - TPM2 Timer Counter Register High; 0x00000061 ***/
    union {
      byte Byte;
    } TPM2CNTHSTR;
    #define TPM2CNTH                    _TPM2CNT.Overlap_STR.TPM2CNTHSTR.Byte
    

    /*** TPM2CNTL - TPM2 Timer Counter Register Low; 0x00000062 ***/
    union {
      byte Byte;
    } TPM2CNTLSTR;
    #define TPM2CNTL                    _TPM2CNT.Overlap_STR.TPM2CNTLSTR.Byte
    
  } Overlap_STR;

} TPM2CNTSTR;
extern volatile TPM2CNTSTR _TPM2CNT @0x00000061;
#define TPM2CNT                         _TPM2CNT.Word


/*** TPM2MOD - TPM2 Timer Counter Modulo Register; 0x00000063 ***/
typedef union {
  word Word;
   /* Overlapped registers: */
  struct {
    /*** TPM2MODH - TPM2 Timer Counter Modulo Register High; 0x00000063 ***/
    union {
      byte Byte;
    } TPM2MODHSTR;
    #define TPM2MODH                    _TPM2MOD.Overlap_STR.TPM2MODHSTR.Byte
    

    /*** TPM2MODL - TPM2 Timer Counter Modulo Register Low; 0x00000064 ***/
    union {
      byte Byte;
    } TPM2MODLSTR;
    #define TPM2MODL                    _TPM2MOD.Overlap_STR.TPM2MODLSTR.Byte
    
  } Overlap_STR;

} TPM2MODSTR;
extern volatile TPM2MODSTR _TPM2MOD @0x00000063;
#define TPM2MOD                         _TPM2MOD.Word


/*** TPM2C0SC - TPM2 Timer Channel 0 Status and Control Register; 0x00000065 ***/
typedef union {
  byte Byte;
  struct {
    byte             :1; 
    byte             :1; 
    byte ELS0A       :1;                                       /* Edge/Level Select Bit A */
    byte ELS0B       :1;                                       /* Edge/Level Select Bit B */
    byte MS0A        :1;                                       /* Mode Select A for TPM Channel 0 */
    byte MS0B        :1;                                       /* Mode Select B for TPM Channel 0 */
    byte CH0IE       :1;                                       /* Channel 0 Interrupt Enable */
    byte CH0F        :1;                                       /* Channel 0 Flag */
  } Bits;
  struct {
    byte         :1;
    byte         :1;
    byte grpELS0x :2;
    byte grpMS0x :2;
    byte         :1;
    byte         :1;
  } MergedBits;
} TPM2C0SCSTR;
extern volatile TPM2C0SCSTR _TPM2C0SC @0x00000065;
#define TPM2C0SC                        _TPM2C0SC.Byte
#define TPM2C0SC_ELS0A                  _TPM2C0SC.Bits.ELS0A
#define TPM2C0SC_ELS0B                  _TPM2C0SC.Bits.ELS0B
#define TPM2C0SC_MS0A                   _TPM2C0SC.Bits.MS0A
#define TPM2C0SC_MS0B                   _TPM2C0SC.Bits.MS0B
#define TPM2C0SC_CH0IE                  _TPM2C0SC.Bits.CH0IE
#define TPM2C0SC_CH0F                   _TPM2C0SC.Bits.CH0F
#define TPM2C0SC_ELS0x                  _TPM2C0SC.MergedBits.grpELS0x
#define TPM2C0SC_MS0x                   _TPM2C0SC.MergedBits.grpMS0x

#define TPM2C0SC_ELS0A_MASK             4U
#define TPM2C0SC_ELS0B_MASK             8U
#define TPM2C0SC_MS0A_MASK              16U
#define TPM2C0SC_MS0B_MASK              32U
#define TPM2C0SC_CH0IE_MASK             64U
#define TPM2C0SC_CH0F_MASK              128U
#define TPM2C0SC_ELS0x_MASK             12U
#define TPM2C0SC_ELS0x_BITNUM           2U
#define TPM2C0SC_MS0x_MASK              48U
#define TPM2C0SC_MS0x_BITNUM            4U


/*** TPM2C0V - TPM2 Timer Channel 0 Value Register; 0x00000066 ***/
typedef union {
  word Word;
   /* Overlapped registers: */
  struct {
    /*** TPM2C0VH - TPM2 Timer Channel 0 Value Register High; 0x00000066 ***/
    union {
      byte Byte;
    } TPM2C0VHSTR;
    #define TPM2C0VH                    _TPM2C0V.Overlap_STR.TPM2C0VHSTR.Byte
    

    /*** TPM2C0VL - TPM2 Timer Channel 0 Value Register Low; 0x00000067 ***/
    union {
      byte Byte;
    } TPM2C0VLSTR;
    #define TPM2C0VL                    _TPM2C0V.Overlap_STR.TPM2C0VLSTR.Byte
    
  } Overlap_STR;

} TPM2C0VSTR;
extern volatile TPM2C0VSTR _TPM2C0V @0x00000066;
#define TPM2C0V                         _TPM2C0V.Word


/*** TPM2C1SC - TPM2 Timer Channel 1 Status and Control Register; 0x00000068 ***/
typedef union {
  byte Byte;
  struct {
    byte             :1; 
    byte             :1; 
    byte ELS1A       :1;                                       /* Edge/Level Select Bit A */
    byte ELS1B       :1;                                       /* Edge/Level Select Bit B */
    byte MS1A        :1;                                       /* Mode Select A for TPM Channel 1 */
    byte MS1B        :1;                                       /* Mode Select B for TPM Channel 1 */
    byte CH1IE       :1;                                       /* Channel 1 Interrupt Enable */
    byte CH1F        :1;                                       /* Channel 1 Flag */
  } Bits;
  struct {
    byte         :1;
    byte         :1;
    byte grpELS1x :2;
    byte grpMS1x :2;
    byte         :1;
    byte         :1;
  } MergedBits;
} TPM2C1SCSTR;
extern volatile TPM2C1SCSTR _TPM2C1SC @0x00000068;
#define TPM2C1SC                        _TPM2C1SC.Byte
#define TPM2C1SC_ELS1A                  _TPM2C1SC.Bits.ELS1A
#define TPM2C1SC_ELS1B                  _TPM2C1SC.Bits.ELS1B
#define TPM2C1SC_MS1A                   _TPM2C1SC.Bits.MS1A
#define TPM2C1SC_MS1B                   _TPM2C1SC.Bits.MS1B
#define TPM2C1SC_CH1IE                  _TPM2C1SC.Bits.CH1IE
#define TPM2C1SC_CH1F                   _TPM2C1SC.Bits.CH1F
#define TPM2C1SC_ELS1x                  _TPM2C1SC.MergedBits.grpELS1x
#define TPM2C1SC_MS1x                   _TPM2C1SC.MergedBits.grpMS1x

#define TPM2C1SC_ELS1A_MASK             4U
#define TPM2C1SC_ELS1B_MASK             8U
#define TPM2C1SC_MS1A_MASK              16U
#define TPM2C1SC_MS1B_MASK              32U
#define TPM2C1SC_CH1IE_MASK             64U
#define TPM2C1SC_CH1F_MASK              128U
#define TPM2C1SC_ELS1x_MASK             12U
#define TPM2C1SC_ELS1x_BITNUM           2U
#define TPM2C1SC_MS1x_MASK              48U
#define TPM2C1SC_MS1x_BITNUM            4U


/*** TPM2C1V - TPM2 Timer Channel 1 Value Register; 0x00000069 ***/
typedef union {
  word Word;
   /* Overlapped registers: */
  struct {
    /*** TPM2C1VH - TPM2 Timer Channel 1 Value Register High; 0x00000069 ***/
    union {
      byte Byte;
    } TPM2C1VHSTR;
    #define TPM2C1VH                    _TPM2C1V.Overlap_STR.TPM2C1VHSTR.Byte
    

    /*** TPM2C1VL - TPM2 Timer Channel 1 Value Register Low; 0x0000006A ***/
    union {
      byte Byte;
    } TPM2C1VLSTR;
    #define TPM2C1VL                    _TPM2C1V.Overlap_STR.TPM2C1VLSTR.Byte
    
  } Overlap_STR;

} TPM2C1VSTR;
extern volatile TPM2C1VSTR _TPM2C1V @0x00000069;
#define TPM2C1V                         _TPM2C1V.Word


/*** SRS - System Reset Status Register; 0x00001800 ***/
typedef union {
  byte Byte;
  struct {
    byte             :1; 
    byte LVD         :1;                                       /* Low Voltage Detect */
    byte ICG         :1;                                       /* Internal Clock Generation Module Reset */
    byte             :1; 
    byte ILOP        :1;                                       /* Illegal Opcode */
    byte COP         :1;                                       /* Computer Operating Properly (COP) Watchdog */
    byte PIN         :1;                                       /* External Reset Pin */
    byte POR         :1;                                       /* Power-On Reset */
  } Bits;
} SRSSTR;
extern volatile SRSSTR _SRS @0x00001800;
#define SRS                             _SRS.Byte
#define SRS_LVD                         _SRS.Bits.LVD
#define SRS_ICG                         _SRS.Bits.ICG
#define SRS_ILOP                        _SRS.Bits.ILOP
#define SRS_COP                         _SRS.Bits.COP
#define SRS_PIN                         _SRS.Bits.PIN
#define SRS_POR                         _SRS.Bits.POR

#define SRS_LVD_MASK                    2U
#define SRS_ICG_MASK                    4U
#define SRS_ILOP_MASK                   16U
#define SRS_COP_MASK                    32U
#define SRS_PIN_MASK                    64U
#define SRS_POR_MASK                    128U


/*** SBDFR - System Background Debug Force Reset Register; 0x00001801 ***/
typedef union {
  byte Byte;
  struct {
    byte BDFR        :1;                                       /* Background Debug Force Reset */
    byte             :1; 
    byte             :1; 
    byte             :1; 
    byte             :1; 
    byte             :1; 
    byte             :1; 
    byte             :1; 
  } Bits;
} SBDFRSTR;
extern volatile SBDFRSTR _SBDFR @0x00001801;
#define SBDFR                           _SBDFR.Byte
#define SBDFR_BDFR                      _SBDFR.Bits.BDFR

#define SBDFR_BDFR_MASK                 1U


/*** SOPT - System Options Register; 0x00001802 ***/
typedef union {
  byte Byte;
  struct {
    byte             :1; 
    byte             :1; 
    byte             :1; 
    byte             :1; 
    byte             :1; 
    byte STOPE       :1;                                       /* Stop Mode Enable */
    byte COPT        :1;                                       /* COP Watchdog Timeout */
    byte COPE        :1;                                       /* COP Watchdog Enable */
  } Bits;
} SOPTSTR;
extern volatile SOPTSTR _SOPT @0x00001802;
#define SOPT                            _SOPT.Byte
#define SOPT_STOPE                      _SOPT.Bits.STOPE
#define SOPT_COPT                       _SOPT.Bits.COPT
#define SOPT_COPE                       _SOPT.Bits.COPE

#define SOPT_STOPE_MASK                 32U
#define SOPT_COPT_MASK                  64U
#define SOPT_COPE_MASK                  128U


/*** SMCLK - System MCLK Control Register; 0x00001803 ***/
typedef union {
  byte Byte;
  struct {
    byte MCSEL0      :1;                                       /* MCLK Divide Select, bit 0 */
    byte MCSEL1      :1;                                       /* MCLK Divide Select, bit 1 */
    byte MCSEL2      :1;                                       /* MCLK Divide Select, bit 2 */
    byte             :1; 
    byte MPE         :1;                                       /* MCLK Pin Enable */
    byte             :1; 
    byte             :1; 
    byte             :1; 
  } Bits;
  struct {
    byte grpMCSEL :3;
    byte         :1;
    byte         :1;
    byte         :1;
    byte         :1;
    byte         :1;
  } MergedBits;
} SMCLKSTR;
extern volatile SMCLKSTR _SMCLK @0x00001803;
#define SMCLK                           _SMCLK.Byte
#define SMCLK_MCSEL0                    _SMCLK.Bits.MCSEL0
#define SMCLK_MCSEL1                    _SMCLK.Bits.MCSEL1
#define SMCLK_MCSEL2                    _SMCLK.Bits.MCSEL2
#define SMCLK_MPE                       _SMCLK.Bits.MPE
#define SMCLK_MCSEL                     _SMCLK.MergedBits.grpMCSEL

#define SMCLK_MCSEL0_MASK               1U
#define SMCLK_MCSEL1_MASK               2U
#define SMCLK_MCSEL2_MASK               4U
#define SMCLK_MPE_MASK                  16U
#define SMCLK_MCSEL_MASK                7U
#define SMCLK_MCSEL_BITNUM              0U


/*** SDID - System Device Identification Register; 0x00001806 ***/
typedef union {
  word Word;
   /* Overlapped registers: */
  struct {
    /*** SDIDH - System Device Identification Register High; 0x00001806 ***/
    union {
      byte Byte;
      struct {
        byte ID8         :1;                                       /* Part Identification Number, bit 8 */
        byte ID9         :1;                                       /* Part Identification Number, bit 9 */
        byte ID10        :1;                                       /* Part Identification Number, bit 10 */
        byte ID11        :1;                                       /* Part Identification Number, bit 11 */
        byte             :1; 
        byte             :1; 
        byte             :1; 
        byte             :1; 
      } Bits;
      struct {
        byte grpID_8 :4;
        byte     :1;
        byte     :1;
        byte     :1;
        byte     :1;
      } MergedBits;
    } SDIDHSTR;
    #define SDIDH                       _SDID.Overlap_STR.SDIDHSTR.Byte
    #define SDIDH_ID8                   _SDID.Overlap_STR.SDIDHSTR.Bits.ID8
    #define SDIDH_ID9                   _SDID.Overlap_STR.SDIDHSTR.Bits.ID9
    #define SDIDH_ID10                  _SDID.Overlap_STR.SDIDHSTR.Bits.ID10
    #define SDIDH_ID11                  _SDID.Overlap_STR.SDIDHSTR.Bits.ID11
    #define SDIDH_ID_8                  _SDID.Overlap_STR.SDIDHSTR.MergedBits.grpID_8
    #define SDIDH_ID                    SDIDH_ID_8
    
    #define SDIDH_ID8_MASK              1U
    #define SDIDH_ID9_MASK              2U
    #define SDIDH_ID10_MASK             4U
    #define SDIDH_ID11_MASK             8U
    #define SDIDH_ID_8_MASK             15U
    #define SDIDH_ID_8_BITNUM           0U
    

    /*** SDIDL - System Device Identification Register Low; 0x00001807 ***/
    union {
      byte Byte;
      struct {
        byte ID0         :1;                                       /* Part Identification Number, bit 0 */
        byte ID1         :1;                                       /* Part Identification Number, bit 1 */
        byte ID2         :1;                                       /* Part Identification Number, bit 2 */
        byte ID3         :1;                                       /* Part Identification Number, bit 3 */
        byte ID4         :1;                                       /* Part Identification Number, bit 4 */
        byte ID5         :1;                                       /* Part Identification Number, bit 5 */
        byte ID6         :1;                                       /* Part Identification Number, bit 6 */
        byte ID7         :1;                                       /* Part Identification Number, bit 7 */
      } Bits;
    } SDIDLSTR;
    #define SDIDL                       _SDID.Overlap_STR.SDIDLSTR.Byte
    #define SDIDL_ID0                   _SDID.Overlap_STR.SDIDLSTR.Bits.ID0
    #define SDIDL_ID1                   _SDID.Overlap_STR.SDIDLSTR.Bits.ID1
    #define SDIDL_ID2                   _SDID.Overlap_STR.SDIDLSTR.Bits.ID2
    #define SDIDL_ID3                   _SDID.Overlap_STR.SDIDLSTR.Bits.ID3
    #define SDIDL_ID4                   _SDID.Overlap_STR.SDIDLSTR.Bits.ID4
    #define SDIDL_ID5                   _SDID.Overlap_STR.SDIDLSTR.Bits.ID5
    #define SDIDL_ID6                   _SDID.Overlap_STR.SDIDLSTR.Bits.ID6
    #define SDIDL_ID7                   _SDID.Overlap_STR.SDIDLSTR.Bits.ID7
    
    #define SDIDL_ID0_MASK              1U
    #define SDIDL_ID1_MASK              2U
    #define SDIDL_ID2_MASK              4U
    #define SDIDL_ID3_MASK              8U
    #define SDIDL_ID4_MASK              16U
    #define SDIDL_ID5_MASK              32U
    #define SDIDL_ID6_MASK              64U
    #define SDIDL_ID7_MASK              128U
    
  } Overlap_STR;

} SDIDSTR;
extern volatile SDIDSTR _SDID @0x00001806;
#define SDID                            _SDID.Word


/*** SRTISC - System Real-Time Interrupt Status and Control Register; 0x00001808 ***/
typedef union {
  byte Byte;
  struct {
    byte RTIS0       :1;                                       /* Real-Time Interrupt Delay Selects, bit 0 */
    byte RTIS1       :1;                                       /* Real-Time Interrupt Delay Selects, bit 1 */
    byte RTIS2       :1;                                       /* Real-Time Interrupt Delay Selects, bit 2 */
    byte             :1; 
    byte RTIE        :1;                                       /* Real-Time Interrupt Enable */
    byte RTICLKS     :1;                                       /* Real-Time Interrupt Clock Select */
    byte RTIACK      :1;                                       /* Real-Time Interrupt Acknowledge */
    byte RTIF        :1;                                       /* Real-Time Interrupt Flag */
  } Bits;
  struct {
    byte grpRTIS :3;
    byte         :1;
    byte         :1;
    byte         :1;
    byte         :1;
    byte         :1;
  } MergedBits;
} SRTISCSTR;
extern volatile SRTISCSTR _SRTISC @0x00001808;
#define SRTISC                          _SRTISC.Byte
#define SRTISC_RTIS0                    _SRTISC.Bits.RTIS0
#define SRTISC_RTIS1                    _SRTISC.Bits.RTIS1
#define SRTISC_RTIS2                    _SRTISC.Bits.RTIS2
#define SRTISC_RTIE                     _SRTISC.Bits.RTIE
#define SRTISC_RTICLKS                  _SRTISC.Bits.RTICLKS
#define SRTISC_RTIACK                   _SRTISC.Bits.RTIACK
#define SRTISC_RTIF                     _SRTISC.Bits.RTIF
#define SRTISC_RTIS                     _SRTISC.MergedBits.grpRTIS

#define SRTISC_RTIS0_MASK               1U
#define SRTISC_RTIS1_MASK               2U
#define SRTISC_RTIS2_MASK               4U
#define SRTISC_RTIE_MASK                16U
#define SRTISC_RTICLKS_MASK             32U
#define SRTISC_RTIACK_MASK              64U
#define SRTISC_RTIF_MASK                128U
#define SRTISC_RTIS_MASK                7U
#define SRTISC_RTIS_BITNUM              0U


/*** SPMSC1 - System Power Management Status and Control 1 Register; 0x00001809 ***/
typedef union {
  byte Byte;
  struct {
    byte BGBE        :1;                                       /* Bandgap Buffer Enable */
    byte             :1; 
    byte LVDE        :1;                                       /* Low-Voltage Detect Enable */
    byte LVDSE       :1;                                       /* Low-Voltage Detect Stop Enable */
    byte LVDRE       :1;                                       /* Low-Voltage Detect Reset Enable */
    byte LVDIE       :1;                                       /* Low-Voltage Detect Interrupt Enable */
    byte LVDACK      :1;                                       /* Low-Voltage Detect Acknowledge */
    byte LVDF        :1;                                       /* Low-Voltage Detect Flag */
  } Bits;
} SPMSC1STR;
extern volatile SPMSC1STR _SPMSC1 @0x00001809;
#define SPMSC1                          _SPMSC1.Byte
#define SPMSC1_BGBE                     _SPMSC1.Bits.BGBE
#define SPMSC1_LVDE                     _SPMSC1.Bits.LVDE
#define SPMSC1_LVDSE                    _SPMSC1.Bits.LVDSE
#define SPMSC1_LVDRE                    _SPMSC1.Bits.LVDRE
#define SPMSC1_LVDIE                    _SPMSC1.Bits.LVDIE
#define SPMSC1_LVDACK                   _SPMSC1.Bits.LVDACK
#define SPMSC1_LVDF                     _SPMSC1.Bits.LVDF

#define SPMSC1_BGBE_MASK                1U
#define SPMSC1_LVDE_MASK                4U
#define SPMSC1_LVDSE_MASK               8U
#define SPMSC1_LVDRE_MASK               16U
#define SPMSC1_LVDIE_MASK               32U
#define SPMSC1_LVDACK_MASK              64U
#define SPMSC1_LVDF_MASK                128U


/*** SPMSC2 - System Power Management Status and Control 2 Register; 0x0000180A ***/
typedef union {
  byte Byte;
  struct {
    byte PPDC        :1;                                       /* Partial Power Down Control */
    byte             :1; 
    byte PPDACK      :1;                                       /* Partial Power Down Acknowledge */
    byte PPDF        :1;                                       /* Partial Power Down Flag */
    byte LVWV        :1;                                       /* Low-Voltage Warning Voltage Select */
    byte LVDV        :1;                                       /* Low-Voltage Detect Voltage Select */
    byte LVWACK      :1;                                       /* Low-Voltage Warning Acknowledge */
    byte LVWF        :1;                                       /* Low-Voltage Warning Flag */
  } Bits;
} SPMSC2STR;
extern volatile SPMSC2STR _SPMSC2 @0x0000180A;
#define SPMSC2                          _SPMSC2.Byte
#define SPMSC2_PPDC                     _SPMSC2.Bits.PPDC
#define SPMSC2_PPDACK                   _SPMSC2.Bits.PPDACK
#define SPMSC2_PPDF                     _SPMSC2.Bits.PPDF
#define SPMSC2_LVWV                     _SPMSC2.Bits.LVWV
#define SPMSC2_LVDV                     _SPMSC2.Bits.LVDV
#define SPMSC2_LVWACK                   _SPMSC2.Bits.LVWACK
#define SPMSC2_LVWF                     _SPMSC2.Bits.LVWF

#define SPMSC2_PPDC_MASK                1U
#define SPMSC2_PPDACK_MASK              4U
#define SPMSC2_PPDF_MASK                8U
#define SPMSC2_LVWV_MASK                16U
#define SPMSC2_LVDV_MASK                32U
#define SPMSC2_LVWACK_MASK              64U
#define SPMSC2_LVWF_MASK                128U


/*** SOPT2 - System Options Register 2; 0x0000180C ***/
typedef union {
  byte Byte;
  struct {
    byte             :1; 
    byte             :1; 
    byte             :1; 
    byte TPMCCFG     :1;                                       /* TPM Clock Confguration */
    byte             :1; 
    byte             :1; 
    byte             :1; 
    byte COPCLKS     :1;                                       /* COP Watchdog Clock Select */
  } Bits;
} SOPT2STR;
extern volatile SOPT2STR _SOPT2 @0x0000180C;
#define SOPT2                           _SOPT2.Byte
#define SOPT2_TPMCCFG                   _SOPT2.Bits.TPMCCFG
#define SOPT2_COPCLKS                   _SOPT2.Bits.COPCLKS

#define SOPT2_TPMCCFG_MASK              8U
#define SOPT2_COPCLKS_MASK              128U


/*** DBGCA - Debug Comparator A Register; 0x00001810 ***/
typedef union {
  word Word;
   /* Overlapped registers: */
  struct {
    /*** DBGCAH - Debug Comparator A High Register; 0x00001810 ***/
    union {
      byte Byte;
      struct {
        byte Bit8        :1;                                       /* Comparator A High Compare Bit 8 */
        byte Bit9        :1;                                       /* Comparator A High Compare Bit 9 */
        byte Bit10       :1;                                       /* Comparator A High Compare Bit 10 */
        byte Bit11       :1;                                       /* Comparator A High Compare Bit 11 */
        byte Bit12       :1;                                       /* Comparator A High Compare Bit 12 */
        byte Bit13       :1;                                       /* Comparator A High Compare Bit 13 */
        byte Bit14       :1;                                       /* Comparator A High Compare Bit 14 */
        byte Bit15       :1;                                       /* Comparator A High Compare Bit 15 */
      } Bits;
    } DBGCAHSTR;
    #define DBGCAH                      _DBGCA.Overlap_STR.DBGCAHSTR.Byte
    #define DBGCAH_Bit8                 _DBGCA.Overlap_STR.DBGCAHSTR.Bits.Bit8
    #define DBGCAH_Bit9                 _DBGCA.Overlap_STR.DBGCAHSTR.Bits.Bit9
    #define DBGCAH_Bit10                _DBGCA.Overlap_STR.DBGCAHSTR.Bits.Bit10
    #define DBGCAH_Bit11                _DBGCA.Overlap_STR.DBGCAHSTR.Bits.Bit11
    #define DBGCAH_Bit12                _DBGCA.Overlap_STR.DBGCAHSTR.Bits.Bit12
    #define DBGCAH_Bit13                _DBGCA.Overlap_STR.DBGCAHSTR.Bits.Bit13
    #define DBGCAH_Bit14                _DBGCA.Overlap_STR.DBGCAHSTR.Bits.Bit14
    #define DBGCAH_Bit15                _DBGCA.Overlap_STR.DBGCAHSTR.Bits.Bit15
    
    #define DBGCAH_Bit8_MASK            1U
    #define DBGCAH_Bit9_MASK            2U
    #define DBGCAH_Bit10_MASK           4U
    #define DBGCAH_Bit11_MASK           8U
    #define DBGCAH_Bit12_MASK           16U
    #define DBGCAH_Bit13_MASK           32U
    #define DBGCAH_Bit14_MASK           64U
    #define DBGCAH_Bit15_MASK           128U
    

    /*** DBGCAL - Debug Comparator A Low Register; 0x00001811 ***/
    union {
      byte Byte;
      struct {
        byte Bit0        :1;                                       /* Comparator A Low Compare Bit 0 */
        byte Bit1        :1;                                       /* Comparator A Low Compare Bit 1 */
        byte Bit2        :1;                                       /* Comparator A Low Compare Bit 2 */
        byte Bit3        :1;                                       /* Comparator A Low Compare Bit 3 */
        byte Bit4        :1;                                       /* Comparator A Low Compare Bit 4 */
        byte Bit5        :1;                                       /* Comparator A Low Compare Bit 5 */
        byte Bit6        :1;                                       /* Comparator A Low Compare Bit 6 */
        byte Bit7        :1;                                       /* Comparator A Low Compare Bit 7 */
      } Bits;
    } DBGCALSTR;
    #define DBGCAL                      _DBGCA.Overlap_STR.DBGCALSTR.Byte
    #define DBGCAL_Bit0                 _DBGCA.Overlap_STR.DBGCALSTR.Bits.Bit0
    #define DBGCAL_Bit1                 _DBGCA.Overlap_STR.DBGCALSTR.Bits.Bit1
    #define DBGCAL_Bit2                 _DBGCA.Overlap_STR.DBGCALSTR.Bits.Bit2
    #define DBGCAL_Bit3                 _DBGCA.Overlap_STR.DBGCALSTR.Bits.Bit3
    #define DBGCAL_Bit4                 _DBGCA.Overlap_STR.DBGCALSTR.Bits.Bit4
    #define DBGCAL_Bit5                 _DBGCA.Overlap_STR.DBGCALSTR.Bits.Bit5
    #define DBGCAL_Bit6                 _DBGCA.Overlap_STR.DBGCALSTR.Bits.Bit6
    #define DBGCAL_Bit7                 _DBGCA.Overlap_STR.DBGCALSTR.Bits.Bit7
    
    #define DBGCAL_Bit0_MASK            1U
    #define DBGCAL_Bit1_MASK            2U
    #define DBGCAL_Bit2_MASK            4U
    #define DBGCAL_Bit3_MASK            8U
    #define DBGCAL_Bit4_MASK            16U
    #define DBGCAL_Bit5_MASK            32U
    #define DBGCAL_Bit6_MASK            64U
    #define DBGCAL_Bit7_MASK            128U
    
  } Overlap_STR;

} DBGCASTR;
extern volatile DBGCASTR _DBGCA @0x00001810;
#define DBGCA                           _DBGCA.Word


/*** DBGCB - Debug Comparator B Register; 0x00001812 ***/
typedef union {
  word Word;
   /* Overlapped registers: */
  struct {
    /*** DBGCBH - Debug Comparator B High Register; 0x00001812 ***/
    union {
      byte Byte;
      struct {
        byte Bit8        :1;                                       /* Comparator B High Compare Bit 8 */
        byte Bit9        :1;                                       /* Comparator B High Compare Bit 9 */
        byte Bit10       :1;                                       /* Comparator B High Compare Bit 10 */
        byte Bit11       :1;                                       /* Comparator B High Compare Bit 11 */
        byte Bit12       :1;                                       /* Comparator B High Compare Bit 12 */
        byte Bit13       :1;                                       /* Comparator B High Compare Bit 13 */
        byte Bit14       :1;                                       /* Comparator B High Compare Bit 14 */
        byte Bit15       :1;                                       /* Comparator B High Compare Bit 15 */
      } Bits;
    } DBGCBHSTR;
    #define DBGCBH                      _DBGCB.Overlap_STR.DBGCBHSTR.Byte
    #define DBGCBH_Bit8                 _DBGCB.Overlap_STR.DBGCBHSTR.Bits.Bit8
    #define DBGCBH_Bit9                 _DBGCB.Overlap_STR.DBGCBHSTR.Bits.Bit9
    #define DBGCBH_Bit10                _DBGCB.Overlap_STR.DBGCBHSTR.Bits.Bit10
    #define DBGCBH_Bit11                _DBGCB.Overlap_STR.DBGCBHSTR.Bits.Bit11
    #define DBGCBH_Bit12                _DBGCB.Overlap_STR.DBGCBHSTR.Bits.Bit12
    #define DBGCBH_Bit13                _DBGCB.Overlap_STR.DBGCBHSTR.Bits.Bit13
    #define DBGCBH_Bit14                _DBGCB.Overlap_STR.DBGCBHSTR.Bits.Bit14
    #define DBGCBH_Bit15                _DBGCB.Overlap_STR.DBGCBHSTR.Bits.Bit15
    
    #define DBGCBH_Bit8_MASK            1U
    #define DBGCBH_Bit9_MASK            2U
    #define DBGCBH_Bit10_MASK           4U
    #define DBGCBH_Bit11_MASK           8U
    #define DBGCBH_Bit12_MASK           16U
    #define DBGCBH_Bit13_MASK           32U
    #define DBGCBH_Bit14_MASK           64U
    #define DBGCBH_Bit15_MASK           128U
    

    /*** DBGCBL - Debug Comparator B Low Register; 0x00001813 ***/
    union {
      byte Byte;
      struct {
        byte Bit0        :1;                                       /* Comparator B Low Compare Bit 0 */
        byte Bit1        :1;                                       /* Comparator B Low Compare Bit 1 */
        byte Bit2        :1;                                       /* Comparator B Low Compare Bit 2 */
        byte Bit3        :1;                                       /* Comparator B Low Compare Bit 3 */
        byte Bit4        :1;                                       /* Comparator B Low Compare Bit 4 */
        byte Bit5        :1;                                       /* Comparator B Low Compare Bit 5 */
        byte Bit6        :1;                                       /* Comparator B Low Compare Bit 6 */
        byte Bit7        :1;                                       /* Comparator B Low Compare Bit 7 */
      } Bits;
    } DBGCBLSTR;
    #define DBGCBL                      _DBGCB.Overlap_STR.DBGCBLSTR.Byte
    #define DBGCBL_Bit0                 _DBGCB.Overlap_STR.DBGCBLSTR.Bits.Bit0
    #define DBGCBL_Bit1                 _DBGCB.Overlap_STR.DBGCBLSTR.Bits.Bit1
    #define DBGCBL_Bit2                 _DBGCB.Overlap_STR.DBGCBLSTR.Bits.Bit2
    #define DBGCBL_Bit3                 _DBGCB.Overlap_STR.DBGCBLSTR.Bits.Bit3
    #define DBGCBL_Bit4                 _DBGCB.Overlap_STR.DBGCBLSTR.Bits.Bit4
    #define DBGCBL_Bit5                 _DBGCB.Overlap_STR.DBGCBLSTR.Bits.Bit5
    #define DBGCBL_Bit6                 _DBGCB.Overlap_STR.DBGCBLSTR.Bits.Bit6
    #define DBGCBL_Bit7                 _DBGCB.Overlap_STR.DBGCBLSTR.Bits.Bit7
    
    #define DBGCBL_Bit0_MASK            1U
    #define DBGCBL_Bit1_MASK            2U
    #define DBGCBL_Bit2_MASK            4U
    #define DBGCBL_Bit3_MASK            8U
    #define DBGCBL_Bit4_MASK            16U
    #define DBGCBL_Bit5_MASK            32U
    #define DBGCBL_Bit6_MASK            64U
    #define DBGCBL_Bit7_MASK            128U
    
  } Overlap_STR;

} DBGCBSTR;
extern volatile DBGCBSTR _DBGCB @0x00001812;
#define DBGCB                           _DBGCB.Word


/*** DBGF - Debug FIFO Register; 0x00001814 ***/
typedef union {
  word Word;
   /* Overlapped registers: */
  struct {
    /*** DBGFH - Debug FIFO High Register; 0x00001814 ***/
    union {
      byte Byte;
      struct {
        byte Bit8        :1;                                       /* FIFO High Data Bit 8 */
        byte Bit9        :1;                                       /* FIFO High Data Bit 9 */
        byte Bit10       :1;                                       /* FIFO High Data Bit 10 */
        byte Bit11       :1;                                       /* FIFO High Data Bit 11 */
        byte Bit12       :1;                                       /* FIFO High Data Bit 12 */
        byte Bit13       :1;                                       /* FIFO High Data Bit 13 */
        byte Bit14       :1;                                       /* FIFO High Data Bit 14 */
        byte Bit15       :1;                                       /* FIFO High Data Bit 15 */
      } Bits;
    } DBGFHSTR;
    #define DBGFH                       _DBGF.Overlap_STR.DBGFHSTR.Byte
    #define DBGFH_Bit8                  _DBGF.Overlap_STR.DBGFHSTR.Bits.Bit8
    #define DBGFH_Bit9                  _DBGF.Overlap_STR.DBGFHSTR.Bits.Bit9
    #define DBGFH_Bit10                 _DBGF.Overlap_STR.DBGFHSTR.Bits.Bit10
    #define DBGFH_Bit11                 _DBGF.Overlap_STR.DBGFHSTR.Bits.Bit11
    #define DBGFH_Bit12                 _DBGF.Overlap_STR.DBGFHSTR.Bits.Bit12
    #define DBGFH_Bit13                 _DBGF.Overlap_STR.DBGFHSTR.Bits.Bit13
    #define DBGFH_Bit14                 _DBGF.Overlap_STR.DBGFHSTR.Bits.Bit14
    #define DBGFH_Bit15                 _DBGF.Overlap_STR.DBGFHSTR.Bits.Bit15
    
    #define DBGFH_Bit8_MASK             1U
    #define DBGFH_Bit9_MASK             2U
    #define DBGFH_Bit10_MASK            4U
    #define DBGFH_Bit11_MASK            8U
    #define DBGFH_Bit12_MASK            16U
    #define DBGFH_Bit13_MASK            32U
    #define DBGFH_Bit14_MASK            64U
    #define DBGFH_Bit15_MASK            128U
    

    /*** DBGFL - Debug FIFO Low Register; 0x00001815 ***/
    union {
      byte Byte;
      struct {
        byte Bit0        :1;                                       /* FIFO Low Data Bit 0 */
        byte Bit1        :1;                                       /* FIFO Low Data Bit 1 */
        byte Bit2        :1;                                       /* FIFO Low Data Bit 2 */
        byte Bit3        :1;                                       /* FIFO Low Data Bit 3 */
        byte Bit4        :1;                                       /* FIFO Low Data Bit 4 */
        byte Bit5        :1;                                       /* FIFO Low Data Bit 5 */
        byte Bit6        :1;                                       /* FIFO Low Data Bit 6 */
        byte Bit7        :1;                                       /* FIFO Low Data Bit 7 */
      } Bits;
    } DBGFLSTR;
    #define DBGFL                       _DBGF.Overlap_STR.DBGFLSTR.Byte
    #define DBGFL_Bit0                  _DBGF.Overlap_STR.DBGFLSTR.Bits.Bit0
    #define DBGFL_Bit1                  _DBGF.Overlap_STR.DBGFLSTR.Bits.Bit1
    #define DBGFL_Bit2                  _DBGF.Overlap_STR.DBGFLSTR.Bits.Bit2
    #define DBGFL_Bit3                  _DBGF.Overlap_STR.DBGFLSTR.Bits.Bit3
    #define DBGFL_Bit4                  _DBGF.Overlap_STR.DBGFLSTR.Bits.Bit4
    #define DBGFL_Bit5                  _DBGF.Overlap_STR.DBGFLSTR.Bits.Bit5
    #define DBGFL_Bit6                  _DBGF.Overlap_STR.DBGFLSTR.Bits.Bit6
    #define DBGFL_Bit7                  _DBGF.Overlap_STR.DBGFLSTR.Bits.Bit7
    
    #define DBGFL_Bit0_MASK             1U
    #define DBGFL_Bit1_MASK             2U
    #define DBGFL_Bit2_MASK             4U
    #define DBGFL_Bit3_MASK             8U
    #define DBGFL_Bit4_MASK             16U
    #define DBGFL_Bit5_MASK             32U
    #define DBGFL_Bit6_MASK             64U
    #define DBGFL_Bit7_MASK             128U
    
  } Overlap_STR;

} DBGFSTR;
extern volatile DBGFSTR _DBGF @0x00001814;
#define DBGF                            _DBGF.Word


/*** DBGC - Debug Control Register; 0x00001816 ***/
typedef union {
  byte Byte;
  struct {
    byte RWBEN       :1;                                       /* Enable R/W for Comparator B */
    byte RWB         :1;                                       /* R/W Comparison Value for Comparator B */
    byte RWAEN       :1;                                       /* Enable R/W for Comparator A */
    byte RWA         :1;                                       /* R/W Comparison Value for Comparator A */
    byte BRKEN       :1;                                       /* Break Enable */
    byte TAG         :1;                                       /* Tag/Force Select */
    byte ARM         :1;                                       /* Arm Control */
    byte DBGEN       :1;                                       /* Debug Module Enable */
  } Bits;
} DBGCSTR;
extern volatile DBGCSTR _DBGC @0x00001816;
#define DBGC                            _DBGC.Byte
#define DBGC_RWBEN                      _DBGC.Bits.RWBEN
#define DBGC_RWB                        _DBGC.Bits.RWB
#define DBGC_RWAEN                      _DBGC.Bits.RWAEN
#define DBGC_RWA                        _DBGC.Bits.RWA
#define DBGC_BRKEN                      _DBGC.Bits.BRKEN
#define DBGC_TAG                        _DBGC.Bits.TAG
#define DBGC_ARM                        _DBGC.Bits.ARM
#define DBGC_DBGEN                      _DBGC.Bits.DBGEN

#define DBGC_RWBEN_MASK                 1U
#define DBGC_RWB_MASK                   2U
#define DBGC_RWAEN_MASK                 4U
#define DBGC_RWA_MASK                   8U
#define DBGC_BRKEN_MASK                 16U
#define DBGC_TAG_MASK                   32U
#define DBGC_ARM_MASK                   64U
#define DBGC_DBGEN_MASK                 128U


/*** DBGT - Debug Trigger Register; 0x00001817 ***/
typedef union {
  byte Byte;
  struct {
    byte TRG0        :1;                                       /* Select Trigger Mode Bit 0 */
    byte TRG1        :1;                                       /* Select Trigger Mode Bit 1 */
    byte TRG2        :1;                                       /* Select Trigger Mode Bit 2 */
    byte TRG3        :1;                                       /* Select Trigger Mode Bit 3 */
    byte             :1; 
    byte             :1; 
    byte BEGIN       :1;                                       /* Begin/End Trigger Select */
    byte TRGSEL      :1;                                       /* Trigger Type */
  } Bits;
  struct {
    byte grpTRG  :4;
    byte         :1;
    byte         :1;
    byte         :1;
    byte         :1;
  } MergedBits;
} DBGTSTR;
extern volatile DBGTSTR _DBGT @0x00001817;
#define DBGT                            _DBGT.Byte
#define DBGT_TRG0                       _DBGT.Bits.TRG0
#define DBGT_TRG1                       _DBGT.Bits.TRG1
#define DBGT_TRG2                       _DBGT.Bits.TRG2
#define DBGT_TRG3                       _DBGT.Bits.TRG3
#define DBGT_BEGIN                      _DBGT.Bits.BEGIN
#define DBGT_TRGSEL                     _DBGT.Bits.TRGSEL
#define DBGT_TRG                        _DBGT.MergedBits.grpTRG

#define DBGT_TRG0_MASK                  1U
#define DBGT_TRG1_MASK                  2U
#define DBGT_TRG2_MASK                  4U
#define DBGT_TRG3_MASK                  8U
#define DBGT_BEGIN_MASK                 64U
#define DBGT_TRGSEL_MASK                128U
#define DBGT_TRG_MASK                   15U
#define DBGT_TRG_BITNUM                 0U


/*** DBGS - Debug Status Register; 0x00001818 ***/
typedef union {
  byte Byte;
  struct {
    byte CNT0        :1;                                       /* FIFO Valid Count Bit 0 */
    byte CNT1        :1;                                       /* FIFO Valid Count Bit 1 */
    byte CNT2        :1;                                       /* FIFO Valid Count Bit 2 */
    byte CNT3        :1;                                       /* FIFO Valid Count Bit 3 */
    byte             :1; 
    byte ARMF        :1;                                       /* Arm Flag */
    byte BF          :1;                                       /* Trigger Match B Flag */
    byte AF          :1;                                       /* Trigger Match A Flag */
  } Bits;
  struct {
    byte grpCNT  :4;
    byte         :1;
    byte         :1;
    byte         :1;
    byte         :1;
  } MergedBits;
} DBGSSTR;
extern volatile DBGSSTR _DBGS @0x00001818;
#define DBGS                            _DBGS.Byte
#define DBGS_CNT0                       _DBGS.Bits.CNT0
#define DBGS_CNT1                       _DBGS.Bits.CNT1
#define DBGS_CNT2                       _DBGS.Bits.CNT2
#define DBGS_CNT3                       _DBGS.Bits.CNT3
#define DBGS_ARMF                       _DBGS.Bits.ARMF
#define DBGS_BF                         _DBGS.Bits.BF
#define DBGS_AF                         _DBGS.Bits.AF
#define DBGS_CNT                        _DBGS.MergedBits.grpCNT

#define DBGS_CNT0_MASK                  1U
#define DBGS_CNT1_MASK                  2U
#define DBGS_CNT2_MASK                  4U
#define DBGS_CNT3_MASK                  8U
#define DBGS_ARMF_MASK                  32U
#define DBGS_BF_MASK                    64U
#define DBGS_AF_MASK                    128U
#define DBGS_CNT_MASK                   15U
#define DBGS_CNT_BITNUM                 0U


/*** FCDIV - FLASH Clock Divider Register; 0x00001820 ***/
typedef union {
  byte Byte;
  struct {
    byte DIV0        :1;                                       /* Divisor for FLASH Clock Divider, bit 0 */
    byte DIV1        :1;                                       /* Divisor for FLASH Clock Divider, bit 1 */
    byte DIV2        :1;                                       /* Divisor for FLASH Clock Divider, bit 2 */
    byte DIV3        :1;                                       /* Divisor for FLASH Clock Divider, bit 3 */
    byte DIV4        :1;                                       /* Divisor for FLASH Clock Divider, bit 4 */
    byte DIV5        :1;                                       /* Divisor for FLASH Clock Divider, bit 5 */
    byte PRDIV8      :1;                                       /* Prescale (Divide) FLASH Clock by 8 */
    byte DIVLD       :1;                                       /* Divisor Loaded Status Flag */
  } Bits;
  struct {
    byte grpDIV  :6;
    byte grpPRDIV_8 :1;
    byte         :1;
  } MergedBits;
} FCDIVSTR;
extern volatile FCDIVSTR _FCDIV @0x00001820;
#define FCDIV                           _FCDIV.Byte
#define FCDIV_DIV0                      _FCDIV.Bits.DIV0
#define FCDIV_DIV1                      _FCDIV.Bits.DIV1
#define FCDIV_DIV2                      _FCDIV.Bits.DIV2
#define FCDIV_DIV3                      _FCDIV.Bits.DIV3
#define FCDIV_DIV4                      _FCDIV.Bits.DIV4
#define FCDIV_DIV5                      _FCDIV.Bits.DIV5
#define FCDIV_PRDIV8                    _FCDIV.Bits.PRDIV8
#define FCDIV_DIVLD                     _FCDIV.Bits.DIVLD
#define FCDIV_DIV                       _FCDIV.MergedBits.grpDIV

#define FCDIV_DIV0_MASK                 1U
#define FCDIV_DIV1_MASK                 2U
#define FCDIV_DIV2_MASK                 4U
#define FCDIV_DIV3_MASK                 8U
#define FCDIV_DIV4_MASK                 16U
#define FCDIV_DIV5_MASK                 32U
#define FCDIV_PRDIV8_MASK               64U
#define FCDIV_DIVLD_MASK                128U
#define FCDIV_DIV_MASK                  63U
#define FCDIV_DIV_BITNUM                0U


/*** FOPT - FLASH Options Register; 0x00001821 ***/
typedef union {
  byte Byte;
  struct {
    byte SEC00       :1;                                       /* Security State Code, bit 0 */
    byte SEC01       :1;                                       /* Security State Code, bit 1 */
    byte             :1; 
    byte             :1; 
    byte             :1; 
    byte             :1; 
    byte FNORED      :1;                                       /* Vector Redirection Disable */
    byte KEYEN       :1;                                       /* Backdoor Key Mechanism Enable */
  } Bits;
  struct {
    byte grpSEC0 :2;
    byte         :1;
    byte         :1;
    byte         :1;
    byte         :1;
    byte         :1;
    byte         :1;
  } MergedBits;
} FOPTSTR;
extern volatile FOPTSTR _FOPT @0x00001821;
#define FOPT                            _FOPT.Byte
#define FOPT_SEC00                      _FOPT.Bits.SEC00
#define FOPT_SEC01                      _FOPT.Bits.SEC01
#define FOPT_FNORED                     _FOPT.Bits.FNORED
#define FOPT_KEYEN                      _FOPT.Bits.KEYEN
#define FOPT_SEC0                       _FOPT.MergedBits.grpSEC0

#define FOPT_SEC00_MASK                 1U
#define FOPT_SEC01_MASK                 2U
#define FOPT_FNORED_MASK                64U
#define FOPT_KEYEN_MASK                 128U
#define FOPT_SEC0_MASK                  3U
#define FOPT_SEC0_BITNUM                0U


/*** FCNFG - FLASH Configuration Register; 0x00001823 ***/
typedef union {
  byte Byte;
  struct {
    byte             :1; 
    byte             :1; 
    byte             :1; 
    byte             :1; 
    byte             :1; 
    byte KEYACC      :1;                                       /* Enable Writing of Access Key */
    byte             :1; 
    byte             :1; 
  } Bits;
} FCNFGSTR;
extern volatile FCNFGSTR _FCNFG @0x00001823;
#define FCNFG                           _FCNFG.Byte
#define FCNFG_KEYACC                    _FCNFG.Bits.KEYACC

#define FCNFG_KEYACC_MASK               32U


/*** FPROT - FLASH Protection Register; 0x00001824 ***/
typedef union {
  byte Byte;
  struct {
    byte FPDIS       :1;                                       /* FLASH Protection Disable */
    byte FPS1        :1;                                       /* FLASH Protect Select Bit 1 */
    byte FPS2        :1;                                       /* FLASH Protect Select Bit 2 */
    byte FPS3        :1;                                       /* FLASH Protect Select Bit 3 */
    byte FPS4        :1;                                       /* FLASH Protect Select Bit 4 */
    byte FPS5        :1;                                       /* FLASH Protect Select Bit 5 */
    byte FPS6        :1;                                       /* FLASH Protect Select Bit 6 */
    byte FPS7        :1;                                       /* FLASH Protect Select Bit 7 */
  } Bits;
  struct {
    byte         :1;
    byte grpFPS_1 :7;
  } MergedBits;
} FPROTSTR;
extern volatile FPROTSTR _FPROT @0x00001824;
#define FPROT                           _FPROT.Byte
#define FPROT_FPDIS                     _FPROT.Bits.FPDIS
#define FPROT_FPS1                      _FPROT.Bits.FPS1
#define FPROT_FPS2                      _FPROT.Bits.FPS2
#define FPROT_FPS3                      _FPROT.Bits.FPS3
#define FPROT_FPS4                      _FPROT.Bits.FPS4
#define FPROT_FPS5                      _FPROT.Bits.FPS5
#define FPROT_FPS6                      _FPROT.Bits.FPS6
#define FPROT_FPS7                      _FPROT.Bits.FPS7
#define FPROT_FPS_1                     _FPROT.MergedBits.grpFPS_1
#define FPROT_FPS                       FPROT_FPS_1

#define FPROT_FPDIS_MASK                1U
#define FPROT_FPS1_MASK                 2U
#define FPROT_FPS2_MASK                 4U
#define FPROT_FPS3_MASK                 8U
#define FPROT_FPS4_MASK                 16U
#define FPROT_FPS5_MASK                 32U
#define FPROT_FPS6_MASK                 64U
#define FPROT_FPS7_MASK                 128U
#define FPROT_FPS_1_MASK                254U
#define FPROT_FPS_1_BITNUM              1U


/*** FSTAT - Flash Status Register; 0x00001825 ***/
typedef union {
  byte Byte;
  struct {
    byte             :1; 
    byte             :1; 
    byte FBLANK      :1;                                       /* FLASH Flag Indicating the Erase Verify Operation Status */
    byte             :1; 
    byte FACCERR     :1;                                       /* FLASH Access Error Flag */
    byte FPVIOL      :1;                                       /* FLASH Protection Violation Flag */
    byte FCCF        :1;                                       /* FLASH Command Complete Interrupt Flag */
    byte FCBEF       :1;                                       /* FLASH Command Buffer Empty Flag */
  } Bits;
} FSTATSTR;
extern volatile FSTATSTR _FSTAT @0x00001825;
#define FSTAT                           _FSTAT.Byte
#define FSTAT_FBLANK                    _FSTAT.Bits.FBLANK
#define FSTAT_FACCERR                   _FSTAT.Bits.FACCERR
#define FSTAT_FPVIOL                    _FSTAT.Bits.FPVIOL
#define FSTAT_FCCF                      _FSTAT.Bits.FCCF
#define FSTAT_FCBEF                     _FSTAT.Bits.FCBEF

#define FSTAT_FBLANK_MASK               4U
#define FSTAT_FACCERR_MASK              16U
#define FSTAT_FPVIOL_MASK               32U
#define FSTAT_FCCF_MASK                 64U
#define FSTAT_FCBEF_MASK                128U


/*** FCMD - FLASH Command Register; 0x00001826 ***/
typedef union {
  byte Byte;
  struct {
    byte FCMD0       :1;                                       /* FLASH Command Bit 0 */
    byte FCMD1       :1;                                       /* FLASH Command Bit 1 */
    byte FCMD2       :1;                                       /* FLASH Command Bit 2 */
    byte FCMD3       :1;                                       /* FLASH Command Bit 3 */
    byte FCMD4       :1;                                       /* FLASH Command Bit 4 */
    byte FCMD5       :1;                                       /* FLASH Command Bit 5 */
    byte FCMD6       :1;                                       /* FLASH Command Bit 6 */
    byte FCMD7       :1;                                       /* FLASH Command Bit 7 */
  } Bits;
} FCMDSTR;
extern volatile FCMDSTR _FCMD @0x00001826;
#define FCMD                            _FCMD.Byte
#define FCMD_FCMD0                      _FCMD.Bits.FCMD0
#define FCMD_FCMD1                      _FCMD.Bits.FCMD1
#define FCMD_FCMD2                      _FCMD.Bits.FCMD2
#define FCMD_FCMD3                      _FCMD.Bits.FCMD3
#define FCMD_FCMD4                      _FCMD.Bits.FCMD4
#define FCMD_FCMD5                      _FCMD.Bits.FCMD5
#define FCMD_FCMD6                      _FCMD.Bits.FCMD6
#define FCMD_FCMD7                      _FCMD.Bits.FCMD7

#define FCMD_FCMD0_MASK                 1U
#define FCMD_FCMD1_MASK                 2U
#define FCMD_FCMD2_MASK                 4U
#define FCMD_FCMD3_MASK                 8U
#define FCMD_FCMD4_MASK                 16U
#define FCMD_FCMD5_MASK                 32U
#define FCMD_FCMD6_MASK                 64U
#define FCMD_FCMD7_MASK                 128U


/*** TPM3SC - TPM3 Status and Control Register; 0x00001830 ***/
typedef union {
  byte Byte;
  struct {
    byte PS0         :1;                                       /* Prescale Divisor Select Bit 0 */
    byte PS1         :1;                                       /* Prescale Divisor Select Bit 1 */
    byte PS2         :1;                                       /* Prescale Divisor Select Bit 2 */
    byte CLKSA       :1;                                       /* Clock Source Select A */
    byte CLKSB       :1;                                       /* Clock Source Select B */
    byte CPWMS       :1;                                       /* Center-Aligned PWM Select */
    byte TOIE        :1;                                       /* Timer Overflow Interrupt Enable */
    byte TOF         :1;                                       /* Timer Overflow Flag */
  } Bits;
  struct {
    byte grpPS   :3;
    byte grpCLKSx :2;
    byte         :1;
    byte         :1;
    byte         :1;
  } MergedBits;
} TPM3SCSTR;
extern volatile TPM3SCSTR _TPM3SC @0x00001830;
#define TPM3SC                          _TPM3SC.Byte
#define TPM3SC_PS0                      _TPM3SC.Bits.PS0
#define TPM3SC_PS1                      _TPM3SC.Bits.PS1
#define TPM3SC_PS2                      _TPM3SC.Bits.PS2
#define TPM3SC_CLKSA                    _TPM3SC.Bits.CLKSA
#define TPM3SC_CLKSB                    _TPM3SC.Bits.CLKSB
#define TPM3SC_CPWMS                    _TPM3SC.Bits.CPWMS
#define TPM3SC_TOIE                     _TPM3SC.Bits.TOIE
#define TPM3SC_TOF                      _TPM3SC.Bits.TOF
#define TPM3SC_PS                       _TPM3SC.MergedBits.grpPS
#define TPM3SC_CLKSx                    _TPM3SC.MergedBits.grpCLKSx

#define TPM3SC_PS0_MASK                 1U
#define TPM3SC_PS1_MASK                 2U
#define TPM3SC_PS2_MASK                 4U
#define TPM3SC_CLKSA_MASK               8U
#define TPM3SC_CLKSB_MASK               16U
#define TPM3SC_CPWMS_MASK               32U
#define TPM3SC_TOIE_MASK                64U
#define TPM3SC_TOF_MASK                 128U
#define TPM3SC_PS_MASK                  7U
#define TPM3SC_PS_BITNUM                0U
#define TPM3SC_CLKSx_MASK               24U
#define TPM3SC_CLKSx_BITNUM             3U


/*** TPM3CNT - TPM3 Timer Counter Register; 0x00001831 ***/
typedef union {
  word Word;
   /* Overlapped registers: */
  struct {
    /*** TPM3CNTH - TPM3 Timer Counter Register High; 0x00001831 ***/
    union {
      byte Byte;
    } TPM3CNTHSTR;
    #define TPM3CNTH                    _TPM3CNT.Overlap_STR.TPM3CNTHSTR.Byte
    

    /*** TPM3CNTL - TPM3 Timer Counter Register Low; 0x00001832 ***/
    union {
      byte Byte;
    } TPM3CNTLSTR;
    #define TPM3CNTL                    _TPM3CNT.Overlap_STR.TPM3CNTLSTR.Byte
    
  } Overlap_STR;

} TPM3CNTSTR;
extern volatile TPM3CNTSTR _TPM3CNT @0x00001831;
#define TPM3CNT                         _TPM3CNT.Word


/*** TPM3MOD - TPM3 Timer Counter Modulo Register; 0x00001833 ***/
typedef union {
  word Word;
   /* Overlapped registers: */
  struct {
    /*** TPM3MODH - TPM3 Timer Counter Modulo Register High; 0x00001833 ***/
    union {
      byte Byte;
    } TPM3MODHSTR;
    #define TPM3MODH                    _TPM3MOD.Overlap_STR.TPM3MODHSTR.Byte
    

    /*** TPM3MODL - TPM3 Timer Counter Modulo Register Low; 0x00001834 ***/
    union {
      byte Byte;
    } TPM3MODLSTR;
    #define TPM3MODL                    _TPM3MOD.Overlap_STR.TPM3MODLSTR.Byte
    
  } Overlap_STR;

} TPM3MODSTR;
extern volatile TPM3MODSTR _TPM3MOD @0x00001833;
#define TPM3MOD                         _TPM3MOD.Word


/*** TPM3C0SC - TPM3 Timer Channel 0 Status and Control Register; 0x00001835 ***/
typedef union {
  byte Byte;
  struct {
    byte             :1; 
    byte             :1; 
    byte ELS0A       :1;                                       /* Edge/Level Select Bit A */
    byte ELS0B       :1;                                       /* Edge/Level Select Bit B */
    byte MS0A        :1;                                       /* Mode Select A for TPM Channel 0 */
    byte MS0B        :1;                                       /* Mode Select B for TPM Channel 0 */
    byte CH0IE       :1;                                       /* Channel 0 Interrupt Enable */
    byte CH0F        :1;                                       /* Channel 0 Flag */
  } Bits;
  struct {
    byte         :1;
    byte         :1;
    byte grpELS0x :2;
    byte grpMS0x :2;
    byte         :1;
    byte         :1;
  } MergedBits;
} TPM3C0SCSTR;
extern volatile TPM3C0SCSTR _TPM3C0SC @0x00001835;
#define TPM3C0SC                        _TPM3C0SC.Byte
#define TPM3C0SC_ELS0A                  _TPM3C0SC.Bits.ELS0A
#define TPM3C0SC_ELS0B                  _TPM3C0SC.Bits.ELS0B
#define TPM3C0SC_MS0A                   _TPM3C0SC.Bits.MS0A
#define TPM3C0SC_MS0B                   _TPM3C0SC.Bits.MS0B
#define TPM3C0SC_CH0IE                  _TPM3C0SC.Bits.CH0IE
#define TPM3C0SC_CH0F                   _TPM3C0SC.Bits.CH0F
#define TPM3C0SC_ELS0x                  _TPM3C0SC.MergedBits.grpELS0x
#define TPM3C0SC_MS0x                   _TPM3C0SC.MergedBits.grpMS0x

#define TPM3C0SC_ELS0A_MASK             4U
#define TPM3C0SC_ELS0B_MASK             8U
#define TPM3C0SC_MS0A_MASK              16U
#define TPM3C0SC_MS0B_MASK              32U
#define TPM3C0SC_CH0IE_MASK             64U
#define TPM3C0SC_CH0F_MASK              128U
#define TPM3C0SC_ELS0x_MASK             12U
#define TPM3C0SC_ELS0x_BITNUM           2U
#define TPM3C0SC_MS0x_MASK              48U
#define TPM3C0SC_MS0x_BITNUM            4U


/*** TPM3C0V - TPM3 Timer Channel 0 Value Register; 0x00001836 ***/
typedef union {
  word Word;
   /* Overlapped registers: */
  struct {
    /*** TPM3C0VH - TPM3 Timer Channel 0 Value Register High; 0x00001836 ***/
    union {
      byte Byte;
    } TPM3C0VHSTR;
    #define TPM3C0VH                    _TPM3C0V.Overlap_STR.TPM3C0VHSTR.Byte
    

    /*** TPM3C0VL - TPM3 Timer Channel 0 Value Register Low; 0x00001837 ***/
    union {
      byte Byte;
    } TPM3C0VLSTR;
    #define TPM3C0VL                    _TPM3C0V.Overlap_STR.TPM3C0VLSTR.Byte
    
  } Overlap_STR;

} TPM3C0VSTR;
extern volatile TPM3C0VSTR _TPM3C0V @0x00001836;
#define TPM3C0V                         _TPM3C0V.Word


/*** TPM3C1SC - TPM3 Timer Channel 1 Status and Control Register; 0x00001838 ***/
typedef union {
  byte Byte;
  struct {
    byte             :1; 
    byte             :1; 
    byte ELS1A       :1;                                       /* Edge/Level Select Bit A */
    byte ELS1B       :1;                                       /* Edge/Level Select Bit B */
    byte MS1A        :1;                                       /* Mode Select A for TPM Channel 1 */
    byte MS1B        :1;                                       /* Mode Select B for TPM Channel 1 */
    byte CH1IE       :1;                                       /* Channel 1 Interrupt Enable */
    byte CH1F        :1;                                       /* Channel 1 Flag */
  } Bits;
  struct {
    byte         :1;
    byte         :1;
    byte grpELS1x :2;
    byte grpMS1x :2;
    byte         :1;
    byte         :1;
  } MergedBits;
} TPM3C1SCSTR;
extern volatile TPM3C1SCSTR _TPM3C1SC @0x00001838;
#define TPM3C1SC                        _TPM3C1SC.Byte
#define TPM3C1SC_ELS1A                  _TPM3C1SC.Bits.ELS1A
#define TPM3C1SC_ELS1B                  _TPM3C1SC.Bits.ELS1B
#define TPM3C1SC_MS1A                   _TPM3C1SC.Bits.MS1A
#define TPM3C1SC_MS1B                   _TPM3C1SC.Bits.MS1B
#define TPM3C1SC_CH1IE                  _TPM3C1SC.Bits.CH1IE
#define TPM3C1SC_CH1F                   _TPM3C1SC.Bits.CH1F
#define TPM3C1SC_ELS1x                  _TPM3C1SC.MergedBits.grpELS1x
#define TPM3C1SC_MS1x                   _TPM3C1SC.MergedBits.grpMS1x

#define TPM3C1SC_ELS1A_MASK             4U
#define TPM3C1SC_ELS1B_MASK             8U
#define TPM3C1SC_MS1A_MASK              16U
#define TPM3C1SC_MS1B_MASK              32U
#define TPM3C1SC_CH1IE_MASK             64U
#define TPM3C1SC_CH1F_MASK              128U
#define TPM3C1SC_ELS1x_MASK             12U
#define TPM3C1SC_ELS1x_BITNUM           2U
#define TPM3C1SC_MS1x_MASK              48U
#define TPM3C1SC_MS1x_BITNUM            4U


/*** TPM3C1V - TPM3 Timer Channel 1 Value Register; 0x00001839 ***/
typedef union {
  word Word;
   /* Overlapped registers: */
  struct {
    /*** TPM3C1VH - TPM3 Timer Channel 1 Value Register High; 0x00001839 ***/
    union {
      byte Byte;
    } TPM3C1VHSTR;
    #define TPM3C1VH                    _TPM3C1V.Overlap_STR.TPM3C1VHSTR.Byte
    

    /*** TPM3C1VL - TPM3 Timer Channel 1 Value Register Low; 0x0000183A ***/
    union {
      byte Byte;
    } TPM3C1VLSTR;
    #define TPM3C1VL                    _TPM3C1V.Overlap_STR.TPM3C1VLSTR.Byte
    
  } Overlap_STR;

} TPM3C1VSTR;
extern volatile TPM3C1VSTR _TPM3C1V @0x00001839;
#define TPM3C1V                         _TPM3C1V.Word


/*** PTAPE - Port A Pull Enable Register; 0x00001840 ***/
typedef union {
  byte Byte;
  struct {
    byte PTAPE0      :1;                                       /* Internal Pull Enable for Port A Bit 0 */
    byte PTAPE1      :1;                                       /* Internal Pull Enable for Port A Bit 1 */
    byte PTAPE2      :1;                                       /* Internal Pull Enable for Port A Bit 2 */
    byte PTAPE3      :1;                                       /* Internal Pull Enable for Port A Bit 3 */
    byte PTAPE4      :1;                                       /* Internal Pull Enable for Port A Bit 4 */
    byte PTAPE5      :1;                                       /* Internal Pull Enable for Port A Bit 5 */
    byte PTAPE6      :1;                                       /* Internal Pull Enable for Port A Bit 6 */
    byte PTAPE7      :1;                                       /* Internal Pull Enable for Port A Bit 7 */
  } Bits;
} PTAPESTR;
extern volatile PTAPESTR _PTAPE @0x00001840;
#define PTAPE                           _PTAPE.Byte
#define PTAPE_PTAPE0                    _PTAPE.Bits.PTAPE0
#define PTAPE_PTAPE1                    _PTAPE.Bits.PTAPE1
#define PTAPE_PTAPE2                    _PTAPE.Bits.PTAPE2
#define PTAPE_PTAPE3                    _PTAPE.Bits.PTAPE3
#define PTAPE_PTAPE4                    _PTAPE.Bits.PTAPE4
#define PTAPE_PTAPE5                    _PTAPE.Bits.PTAPE5
#define PTAPE_PTAPE6                    _PTAPE.Bits.PTAPE6
#define PTAPE_PTAPE7                    _PTAPE.Bits.PTAPE7

#define PTAPE_PTAPE0_MASK               1U
#define PTAPE_PTAPE1_MASK               2U
#define PTAPE_PTAPE2_MASK               4U
#define PTAPE_PTAPE3_MASK               8U
#define PTAPE_PTAPE4_MASK               16U
#define PTAPE_PTAPE5_MASK               32U
#define PTAPE_PTAPE6_MASK               64U
#define PTAPE_PTAPE7_MASK               128U


/*** PTASE - Port A Slew Rate Enable Register; 0x00001841 ***/
typedef union {
  byte Byte;
  struct {
    byte PTASE0      :1;                                       /* Output Slew Rate Enable for Port A Bit 0 */
    byte PTASE1      :1;                                       /* Output Slew Rate Enable for Port A Bit 1 */
    byte PTASE2      :1;                                       /* Output Slew Rate Enable for Port A Bit 2 */
    byte PTASE3      :1;                                       /* Output Slew Rate Enable for Port A Bit 3 */
    byte PTASE4      :1;                                       /* Output Slew Rate Enable for Port A Bit 4 */
    byte PTASE5      :1;                                       /* Output Slew Rate Enable for Port A Bit 5 */
    byte PTASE6      :1;                                       /* Output Slew Rate Enable for Port A Bit 6 */
    byte PTASE7      :1;                                       /* Output Slew Rate Enable for Port A Bit 7 */
  } Bits;
} PTASESTR;
extern volatile PTASESTR _PTASE @0x00001841;
#define PTASE                           _PTASE.Byte
#define PTASE_PTASE0                    _PTASE.Bits.PTASE0
#define PTASE_PTASE1                    _PTASE.Bits.PTASE1
#define PTASE_PTASE2                    _PTASE.Bits.PTASE2
#define PTASE_PTASE3                    _PTASE.Bits.PTASE3
#define PTASE_PTASE4                    _PTASE.Bits.PTASE4
#define PTASE_PTASE5                    _PTASE.Bits.PTASE5
#define PTASE_PTASE6                    _PTASE.Bits.PTASE6
#define PTASE_PTASE7                    _PTASE.Bits.PTASE7

#define PTASE_PTASE0_MASK               1U
#define PTASE_PTASE1_MASK               2U
#define PTASE_PTASE2_MASK               4U
#define PTASE_PTASE3_MASK               8U
#define PTASE_PTASE4_MASK               16U
#define PTASE_PTASE5_MASK               32U
#define PTASE_PTASE6_MASK               64U
#define PTASE_PTASE7_MASK               128U


/*** PTADS - Port A Drive Strength Selection Register; 0x00001842 ***/
typedef union {
  byte Byte;
  struct {
    byte PTADS0      :1;                                       /* Output Drive Strength Selection for Port A Bit 0 */
    byte PTADS1      :1;                                       /* Output Drive Strength Selection for Port A Bit 1 */
    byte PTADS2      :1;                                       /* Output Drive Strength Selection for Port A Bit 2 */
    byte PTADS3      :1;                                       /* Output Drive Strength Selection for Port A Bit 3 */
    byte PTADS4      :1;                                       /* Output Drive Strength Selection for Port A Bit 4 */
    byte PTADS5      :1;                                       /* Output Drive Strength Selection for Port A Bit 5 */
    byte PTADS6      :1;                                       /* Output Drive Strength Selection for Port A Bit 6 */
    byte PTADS7      :1;                                       /* Output Drive Strength Selection for Port A Bit 7 */
  } Bits;
} PTADSSTR;
extern volatile PTADSSTR _PTADS @0x00001842;
#define PTADS                           _PTADS.Byte
#define PTADS_PTADS0                    _PTADS.Bits.PTADS0
#define PTADS_PTADS1                    _PTADS.Bits.PTADS1
#define PTADS_PTADS2                    _PTADS.Bits.PTADS2
#define PTADS_PTADS3                    _PTADS.Bits.PTADS3
#define PTADS_PTADS4                    _PTADS.Bits.PTADS4
#define PTADS_PTADS5                    _PTADS.Bits.PTADS5
#define PTADS_PTADS6                    _PTADS.Bits.PTADS6
#define PTADS_PTADS7                    _PTADS.Bits.PTADS7

#define PTADS_PTADS0_MASK               1U
#define PTADS_PTADS1_MASK               2U
#define PTADS_PTADS2_MASK               4U
#define PTADS_PTADS3_MASK               8U
#define PTADS_PTADS4_MASK               16U
#define PTADS_PTADS5_MASK               32U
#define PTADS_PTADS6_MASK               64U
#define PTADS_PTADS7_MASK               128U


/*** PTBPE - Port B Pull Enable Register; 0x00001844 ***/
typedef union {
  byte Byte;
  struct {
    byte PTBPE0      :1;                                       /* Internal Pull Enable for Port B Bit 0 */
    byte PTBPE1      :1;                                       /* Internal Pull Enable for Port B Bit 1 */
    byte PTBPE2      :1;                                       /* Internal Pull Enable for Port B Bit 2 */
    byte PTBPE3      :1;                                       /* Internal Pull Enable for Port B Bit 3 */
    byte PTBPE4      :1;                                       /* Internal Pull Enable for Port B Bit 4 */
    byte PTBPE5      :1;                                       /* Internal Pull Enable for Port B Bit 5 */
    byte PTBPE6      :1;                                       /* Internal Pull Enable for Port B Bit 6 */
    byte PTBPE7      :1;                                       /* Internal Pull Enable for Port B Bit 7 */
  } Bits;
} PTBPESTR;
extern volatile PTBPESTR _PTBPE @0x00001844;
#define PTBPE                           _PTBPE.Byte
#define PTBPE_PTBPE0                    _PTBPE.Bits.PTBPE0
#define PTBPE_PTBPE1                    _PTBPE.Bits.PTBPE1
#define PTBPE_PTBPE2                    _PTBPE.Bits.PTBPE2
#define PTBPE_PTBPE3                    _PTBPE.Bits.PTBPE3
#define PTBPE_PTBPE4                    _PTBPE.Bits.PTBPE4
#define PTBPE_PTBPE5                    _PTBPE.Bits.PTBPE5
#define PTBPE_PTBPE6                    _PTBPE.Bits.PTBPE6
#define PTBPE_PTBPE7                    _PTBPE.Bits.PTBPE7

#define PTBPE_PTBPE0_MASK               1U
#define PTBPE_PTBPE1_MASK               2U
#define PTBPE_PTBPE2_MASK               4U
#define PTBPE_PTBPE3_MASK               8U
#define PTBPE_PTBPE4_MASK               16U
#define PTBPE_PTBPE5_MASK               32U
#define PTBPE_PTBPE6_MASK               64U
#define PTBPE_PTBPE7_MASK               128U


/*** PTBSE - Port B Slew Rate Enable Register; 0x00001845 ***/
typedef union {
  byte Byte;
  struct {
    byte PTBSE0      :1;                                       /* Output Slew Rate Enable for Port B Bit 0 */
    byte PTBSE1      :1;                                       /* Output Slew Rate Enable for Port B Bit 1 */
    byte PTBSE2      :1;                                       /* Output Slew Rate Enable for Port B Bit 2 */
    byte PTBSE3      :1;                                       /* Output Slew Rate Enable for Port B Bit 3 */
    byte PTBSE4      :1;                                       /* Output Slew Rate Enable for Port B Bit 4 */
    byte PTBSE5      :1;                                       /* Output Slew Rate Enable for Port B Bit 5 */
    byte PTBSE6      :1;                                       /* Output Slew Rate Enable for Port B Bit 6 */
    byte PTBSE7      :1;                                       /* Output Slew Rate Enable for Port B Bit 7 */
  } Bits;
} PTBSESTR;
extern volatile PTBSESTR _PTBSE @0x00001845;
#define PTBSE                           _PTBSE.Byte
#define PTBSE_PTBSE0                    _PTBSE.Bits.PTBSE0
#define PTBSE_PTBSE1                    _PTBSE.Bits.PTBSE1
#define PTBSE_PTBSE2                    _PTBSE.Bits.PTBSE2
#define PTBSE_PTBSE3                    _PTBSE.Bits.PTBSE3
#define PTBSE_PTBSE4                    _PTBSE.Bits.PTBSE4
#define PTBSE_PTBSE5                    _PTBSE.Bits.PTBSE5
#define PTBSE_PTBSE6                    _PTBSE.Bits.PTBSE6
#define PTBSE_PTBSE7                    _PTBSE.Bits.PTBSE7

#define PTBSE_PTBSE0_MASK               1U
#define PTBSE_PTBSE1_MASK               2U
#define PTBSE_PTBSE2_MASK               4U
#define PTBSE_PTBSE3_MASK               8U
#define PTBSE_PTBSE4_MASK               16U
#define PTBSE_PTBSE5_MASK               32U
#define PTBSE_PTBSE6_MASK               64U
#define PTBSE_PTBSE7_MASK               128U


/*** PTBDS - Port B Drive Strength Selection Register; 0x00001846 ***/
typedef union {
  byte Byte;
  struct {
    byte PTBDS0      :1;                                       /* Output Drive Strength Selection for Port B Bit 0 */
    byte PTBDS1      :1;                                       /* Output Drive Strength Selection for Port B Bit 1 */
    byte PTBDS2      :1;                                       /* Output Drive Strength Selection for Port B Bit 2 */
    byte PTBDS3      :1;                                       /* Output Drive Strength Selection for Port B Bit 3 */
    byte PTBDS4      :1;                                       /* Output Drive Strength Selection for Port B Bit 4 */
    byte PTBDS5      :1;                                       /* Output Drive Strength Selection for Port B Bit 5 */
    byte PTBDS6      :1;                                       /* Output Drive Strength Selection for Port B Bit 6 */
    byte PTBDS7      :1;                                       /* Output Drive Strength Selection for Port B Bit 7 */
  } Bits;
} PTBDSSTR;
extern volatile PTBDSSTR _PTBDS @0x00001846;
#define PTBDS                           _PTBDS.Byte
#define PTBDS_PTBDS0                    _PTBDS.Bits.PTBDS0
#define PTBDS_PTBDS1                    _PTBDS.Bits.PTBDS1
#define PTBDS_PTBDS2                    _PTBDS.Bits.PTBDS2
#define PTBDS_PTBDS3                    _PTBDS.Bits.PTBDS3
#define PTBDS_PTBDS4                    _PTBDS.Bits.PTBDS4
#define PTBDS_PTBDS5                    _PTBDS.Bits.PTBDS5
#define PTBDS_PTBDS6                    _PTBDS.Bits.PTBDS6
#define PTBDS_PTBDS7                    _PTBDS.Bits.PTBDS7

#define PTBDS_PTBDS0_MASK               1U
#define PTBDS_PTBDS1_MASK               2U
#define PTBDS_PTBDS2_MASK               4U
#define PTBDS_PTBDS3_MASK               8U
#define PTBDS_PTBDS4_MASK               16U
#define PTBDS_PTBDS5_MASK               32U
#define PTBDS_PTBDS6_MASK               64U
#define PTBDS_PTBDS7_MASK               128U


/*** PTCPE - Port C Pull Enable Register; 0x00001848 ***/
typedef union {
  byte Byte;
  struct {
    byte PTCPE0      :1;                                       /* Internal Pull Enable for Port C Bit 0 */
    byte PTCPE1      :1;                                       /* Internal Pull Enable for Port C Bit 1 */
    byte PTCPE2      :1;                                       /* Internal Pull Enable for Port C Bit 2 */
    byte PTCPE3      :1;                                       /* Internal Pull Enable for Port C Bit 3 */
    byte PTCPE4      :1;                                       /* Internal Pull Enable for Port C Bit 4 */
    byte PTCPE5      :1;                                       /* Internal Pull Enable for Port C Bit 5 */
    byte PTCPE6      :1;                                       /* Internal Pull Enable for Port C Bit 6 */
    byte             :1; 
  } Bits;
  struct {
    byte grpPTCPE :7;
    byte         :1;
  } MergedBits;
} PTCPESTR;
extern volatile PTCPESTR _PTCPE @0x00001848;
#define PTCPE                           _PTCPE.Byte
#define PTCPE_PTCPE0                    _PTCPE.Bits.PTCPE0
#define PTCPE_PTCPE1                    _PTCPE.Bits.PTCPE1
#define PTCPE_PTCPE2                    _PTCPE.Bits.PTCPE2
#define PTCPE_PTCPE3                    _PTCPE.Bits.PTCPE3
#define PTCPE_PTCPE4                    _PTCPE.Bits.PTCPE4
#define PTCPE_PTCPE5                    _PTCPE.Bits.PTCPE5
#define PTCPE_PTCPE6                    _PTCPE.Bits.PTCPE6
#define PTCPE_PTCPE                     _PTCPE.MergedBits.grpPTCPE

#define PTCPE_PTCPE0_MASK               1U
#define PTCPE_PTCPE1_MASK               2U
#define PTCPE_PTCPE2_MASK               4U
#define PTCPE_PTCPE3_MASK               8U
#define PTCPE_PTCPE4_MASK               16U
#define PTCPE_PTCPE5_MASK               32U
#define PTCPE_PTCPE6_MASK               64U
#define PTCPE_PTCPE_MASK                127U
#define PTCPE_PTCPE_BITNUM              0U


/*** PTCSE - Port C Slew Rate Enable Register; 0x00001849 ***/
typedef union {
  byte Byte;
  struct {
    byte PTCSE0      :1;                                       /* Output Slew Rate Enable for Port C Bit 0 */
    byte PTCSE1      :1;                                       /* Output Slew Rate Enable for Port C Bit 1 */
    byte PTCSE2      :1;                                       /* Output Slew Rate Enable for Port C Bit 2 */
    byte PTCSE3      :1;                                       /* Output Slew Rate Enable for Port C Bit 3 */
    byte PTCSE4      :1;                                       /* Output Slew Rate Enable for Port C Bit 4 */
    byte PTCSE5      :1;                                       /* Output Slew Rate Enable for Port C Bit 5 */
    byte PTCSE6      :1;                                       /* Output Slew Rate Enable for Port C Bit 6 */
    byte             :1; 
  } Bits;
  struct {
    byte grpPTCSE :7;
    byte         :1;
  } MergedBits;
} PTCSESTR;
extern volatile PTCSESTR _PTCSE @0x00001849;
#define PTCSE                           _PTCSE.Byte
#define PTCSE_PTCSE0                    _PTCSE.Bits.PTCSE0
#define PTCSE_PTCSE1                    _PTCSE.Bits.PTCSE1
#define PTCSE_PTCSE2                    _PTCSE.Bits.PTCSE2
#define PTCSE_PTCSE3                    _PTCSE.Bits.PTCSE3
#define PTCSE_PTCSE4                    _PTCSE.Bits.PTCSE4
#define PTCSE_PTCSE5                    _PTCSE.Bits.PTCSE5
#define PTCSE_PTCSE6                    _PTCSE.Bits.PTCSE6
#define PTCSE_PTCSE                     _PTCSE.MergedBits.grpPTCSE

#define PTCSE_PTCSE0_MASK               1U
#define PTCSE_PTCSE1_MASK               2U
#define PTCSE_PTCSE2_MASK               4U
#define PTCSE_PTCSE3_MASK               8U
#define PTCSE_PTCSE4_MASK               16U
#define PTCSE_PTCSE5_MASK               32U
#define PTCSE_PTCSE6_MASK               64U
#define PTCSE_PTCSE_MASK                127U
#define PTCSE_PTCSE_BITNUM              0U


/*** PTCDS - Port C Drive Strength Selection Register; 0x0000184A ***/
typedef union {
  byte Byte;
  struct {
    byte PTCDS0      :1;                                       /* Output Drive Strength Selection for Port C Bit 0 */
    byte PTCDS1      :1;                                       /* Output Drive Strength Selection for Port C Bit 1 */
    byte PTCDS2      :1;                                       /* Output Drive Strength Selection for Port C Bit 2 */
    byte PTCDS3      :1;                                       /* Output Drive Strength Selection for Port C Bit 3 */
    byte PTCDS4      :1;                                       /* Output Drive Strength Selection for Port C Bit 4 */
    byte PTCDS5      :1;                                       /* Output Drive Strength Selection for Port C Bit 5 */
    byte PTCDS6      :1;                                       /* Output Drive Strength Selection for Port C Bit 6 */
    byte             :1; 
  } Bits;
  struct {
    byte grpPTCDS :7;
    byte         :1;
  } MergedBits;
} PTCDSSTR;
extern volatile PTCDSSTR _PTCDS @0x0000184A;
#define PTCDS                           _PTCDS.Byte
#define PTCDS_PTCDS0                    _PTCDS.Bits.PTCDS0
#define PTCDS_PTCDS1                    _PTCDS.Bits.PTCDS1
#define PTCDS_PTCDS2                    _PTCDS.Bits.PTCDS2
#define PTCDS_PTCDS3                    _PTCDS.Bits.PTCDS3
#define PTCDS_PTCDS4                    _PTCDS.Bits.PTCDS4
#define PTCDS_PTCDS5                    _PTCDS.Bits.PTCDS5
#define PTCDS_PTCDS6                    _PTCDS.Bits.PTCDS6
#define PTCDS_PTCDS                     _PTCDS.MergedBits.grpPTCDS

#define PTCDS_PTCDS0_MASK               1U
#define PTCDS_PTCDS1_MASK               2U
#define PTCDS_PTCDS2_MASK               4U
#define PTCDS_PTCDS3_MASK               8U
#define PTCDS_PTCDS4_MASK               16U
#define PTCDS_PTCDS5_MASK               32U
#define PTCDS_PTCDS6_MASK               64U
#define PTCDS_PTCDS_MASK                127U
#define PTCDS_PTCDS_BITNUM              0U


/*** PTDPE - Port D Pull Enable Register; 0x0000184C ***/
typedef union {
  byte Byte;
  struct {
    byte PTDPE0      :1;                                       /* Internal Pull Enable for Port D Bit 0 */
    byte PTDPE1      :1;                                       /* Internal Pull Enable for Port D Bit 1 */
    byte PTDPE2      :1;                                       /* Internal Pull Enable for Port D Bit 2 */
    byte PTDPE3      :1;                                       /* Internal Pull Enable for Port D Bit 3 */
    byte PTDPE4      :1;                                       /* Internal Pull Enable for Port D Bit 4 */
    byte PTDPE5      :1;                                       /* Internal Pull Enable for Port D Bit 5 */
    byte PTDPE6      :1;                                       /* Internal Pull Enable for Port D Bit 6 */
    byte PTDPE7      :1;                                       /* Internal Pull Enable for Port D Bit 7 */
  } Bits;
} PTDPESTR;
extern volatile PTDPESTR _PTDPE @0x0000184C;
#define PTDPE                           _PTDPE.Byte
#define PTDPE_PTDPE0                    _PTDPE.Bits.PTDPE0
#define PTDPE_PTDPE1                    _PTDPE.Bits.PTDPE1
#define PTDPE_PTDPE2                    _PTDPE.Bits.PTDPE2
#define PTDPE_PTDPE3                    _PTDPE.Bits.PTDPE3
#define PTDPE_PTDPE4                    _PTDPE.Bits.PTDPE4
#define PTDPE_PTDPE5                    _PTDPE.Bits.PTDPE5
#define PTDPE_PTDPE6                    _PTDPE.Bits.PTDPE6
#define PTDPE_PTDPE7                    _PTDPE.Bits.PTDPE7

#define PTDPE_PTDPE0_MASK               1U
#define PTDPE_PTDPE1_MASK               2U
#define PTDPE_PTDPE2_MASK               4U
#define PTDPE_PTDPE3_MASK               8U
#define PTDPE_PTDPE4_MASK               16U
#define PTDPE_PTDPE5_MASK               32U
#define PTDPE_PTDPE6_MASK               64U
#define PTDPE_PTDPE7_MASK               128U


/*** PTDSE - Port D Slew Rate Enable Register; 0x0000184D ***/
typedef union {
  byte Byte;
  struct {
    byte PTDSE0      :1;                                       /* Output Slew Rate Enable for Port D Bit 0 */
    byte PTDSE1      :1;                                       /* Output Slew Rate Enable for Port D Bit 1 */
    byte PTDSE2      :1;                                       /* Output Slew Rate Enable for Port D Bit 2 */
    byte PTDSE3      :1;                                       /* Output Slew Rate Enable for Port D Bit 3 */
    byte PTDSE4      :1;                                       /* Output Slew Rate Enable for Port D Bit 4 */
    byte PTDSE5      :1;                                       /* Output Slew Rate Enable for Port D Bit 5 */
    byte PTDSE6      :1;                                       /* Output Slew Rate Enable for Port D Bit 6 */
    byte PTDSE7      :1;                                       /* Output Slew Rate Enable for Port D Bit 7 */
  } Bits;
} PTDSESTR;
extern volatile PTDSESTR _PTDSE @0x0000184D;
#define PTDSE                           _PTDSE.Byte
#define PTDSE_PTDSE0                    _PTDSE.Bits.PTDSE0
#define PTDSE_PTDSE1                    _PTDSE.Bits.PTDSE1
#define PTDSE_PTDSE2                    _PTDSE.Bits.PTDSE2
#define PTDSE_PTDSE3                    _PTDSE.Bits.PTDSE3
#define PTDSE_PTDSE4                    _PTDSE.Bits.PTDSE4
#define PTDSE_PTDSE5                    _PTDSE.Bits.PTDSE5
#define PTDSE_PTDSE6                    _PTDSE.Bits.PTDSE6
#define PTDSE_PTDSE7                    _PTDSE.Bits.PTDSE7

#define PTDSE_PTDSE0_MASK               1U
#define PTDSE_PTDSE1_MASK               2U
#define PTDSE_PTDSE2_MASK               4U
#define PTDSE_PTDSE3_MASK               8U
#define PTDSE_PTDSE4_MASK               16U
#define PTDSE_PTDSE5_MASK               32U
#define PTDSE_PTDSE6_MASK               64U
#define PTDSE_PTDSE7_MASK               128U


/*** PTDDS - Port D Drive Strength Selection Register; 0x0000184E ***/
typedef union {
  byte Byte;
  struct {
    byte PTDDS0      :1;                                       /* Output Drive Strength Selection for Port D Bit 0 */
    byte PTDDS1      :1;                                       /* Output Drive Strength Selection for Port D Bit 1 */
    byte PTDDS2      :1;                                       /* Output Drive Strength Selection for Port D Bit 2 */
    byte PTDDS3      :1;                                       /* Output Drive Strength Selection for Port D Bit 3 */
    byte PTDDS4      :1;                                       /* Output Drive Strength Selection for Port D Bit 4 */
    byte PTDDS5      :1;                                       /* Output Drive Strength Selection for Port D Bit 5 */
    byte PTDDS6      :1;                                       /* Output Drive Strength Selection for Port D Bit 6 */
    byte PTDDS7      :1;                                       /* Output Drive Strength Selection for Port D Bit 7 */
  } Bits;
} PTDDSSTR;
extern volatile PTDDSSTR _PTDDS @0x0000184E;
#define PTDDS                           _PTDDS.Byte
#define PTDDS_PTDDS0                    _PTDDS.Bits.PTDDS0
#define PTDDS_PTDDS1                    _PTDDS.Bits.PTDDS1
#define PTDDS_PTDDS2                    _PTDDS.Bits.PTDDS2
#define PTDDS_PTDDS3                    _PTDDS.Bits.PTDDS3
#define PTDDS_PTDDS4                    _PTDDS.Bits.PTDDS4
#define PTDDS_PTDDS5                    _PTDDS.Bits.PTDDS5
#define PTDDS_PTDDS6                    _PTDDS.Bits.PTDDS6
#define PTDDS_PTDDS7                    _PTDDS.Bits.PTDDS7

#define PTDDS_PTDDS0_MASK               1U
#define PTDDS_PTDDS1_MASK               2U
#define PTDDS_PTDDS2_MASK               4U
#define PTDDS_PTDDS3_MASK               8U
#define PTDDS_PTDDS4_MASK               16U
#define PTDDS_PTDDS5_MASK               32U
#define PTDDS_PTDDS6_MASK               64U
#define PTDDS_PTDDS7_MASK               128U


/*** PTEPE - Port E Pull Enable Register; 0x00001850 ***/
typedef union {
  byte Byte;
  struct {
    byte PTEPE0      :1;                                       /* Internal Pull Enable for Port E Bit 0 */
    byte PTEPE1      :1;                                       /* Internal Pull Enable for Port E Bit 1 */
    byte PTEPE2      :1;                                       /* Internal Pull Enable for Port E Bit 2 */
    byte PTEPE3      :1;                                       /* Internal Pull Enable for Port E Bit 3 */
    byte PTEPE4      :1;                                       /* Internal Pull Enable for Port E Bit 4 */
    byte PTEPE5      :1;                                       /* Internal Pull Enable for Port E Bit 5 */
    byte PTEPE6      :1;                                       /* Internal Pull Enable for Port E Bit 6 */
    byte PTEPE7      :1;                                       /* Internal Pull Enable for Port E Bit 7 */
  } Bits;
} PTEPESTR;
extern volatile PTEPESTR _PTEPE @0x00001850;
#define PTEPE                           _PTEPE.Byte
#define PTEPE_PTEPE0                    _PTEPE.Bits.PTEPE0
#define PTEPE_PTEPE1                    _PTEPE.Bits.PTEPE1
#define PTEPE_PTEPE2                    _PTEPE.Bits.PTEPE2
#define PTEPE_PTEPE3                    _PTEPE.Bits.PTEPE3
#define PTEPE_PTEPE4                    _PTEPE.Bits.PTEPE4
#define PTEPE_PTEPE5                    _PTEPE.Bits.PTEPE5
#define PTEPE_PTEPE6                    _PTEPE.Bits.PTEPE6
#define PTEPE_PTEPE7                    _PTEPE.Bits.PTEPE7

#define PTEPE_PTEPE0_MASK               1U
#define PTEPE_PTEPE1_MASK               2U
#define PTEPE_PTEPE2_MASK               4U
#define PTEPE_PTEPE3_MASK               8U
#define PTEPE_PTEPE4_MASK               16U
#define PTEPE_PTEPE5_MASK               32U
#define PTEPE_PTEPE6_MASK               64U
#define PTEPE_PTEPE7_MASK               128U


/*** PTESE - Port E Slew Rate Enable Register; 0x00001851 ***/
typedef union {
  byte Byte;
  struct {
    byte PTESE0      :1;                                       /* Output Slew Rate Enable for Port E Bit 0 */
    byte PTESE1      :1;                                       /* Output Slew Rate Enable for Port E Bit 1 */
    byte PTESE2      :1;                                       /* Output Slew Rate Enable for Port E Bit 2 */
    byte PTESE3      :1;                                       /* Output Slew Rate Enable for Port E Bit 3 */
    byte PTESE4      :1;                                       /* Output Slew Rate Enable for Port E Bit 4 */
    byte PTESE5      :1;                                       /* Output Slew Rate Enable for Port E Bit 5 */
    byte PTESE6      :1;                                       /* Output Slew Rate Enable for Port E Bit 6 */
    byte PTESE7      :1;                                       /* Output Slew Rate Enable for Port E Bit 7 */
  } Bits;
} PTESESTR;
extern volatile PTESESTR _PTESE @0x00001851;
#define PTESE                           _PTESE.Byte
#define PTESE_PTESE0                    _PTESE.Bits.PTESE0
#define PTESE_PTESE1                    _PTESE.Bits.PTESE1
#define PTESE_PTESE2                    _PTESE.Bits.PTESE2
#define PTESE_PTESE3                    _PTESE.Bits.PTESE3
#define PTESE_PTESE4                    _PTESE.Bits.PTESE4
#define PTESE_PTESE5                    _PTESE.Bits.PTESE5
#define PTESE_PTESE6                    _PTESE.Bits.PTESE6
#define PTESE_PTESE7                    _PTESE.Bits.PTESE7

#define PTESE_PTESE0_MASK               1U
#define PTESE_PTESE1_MASK               2U
#define PTESE_PTESE2_MASK               4U
#define PTESE_PTESE3_MASK               8U
#define PTESE_PTESE4_MASK               16U
#define PTESE_PTESE5_MASK               32U
#define PTESE_PTESE6_MASK               64U
#define PTESE_PTESE7_MASK               128U


/*** PTEDS - Port E Drive Strength Selection Register; 0x00001852 ***/
typedef union {
  byte Byte;
  struct {
    byte PTEDS0      :1;                                       /* Output Drive Strength Selection for Port E Bit 0 */
    byte PTEDS1      :1;                                       /* Output Drive Strength Selection for Port E Bit 1 */
    byte PTEDS2      :1;                                       /* Output Drive Strength Selection for Port E Bit 2 */
    byte PTEDS3      :1;                                       /* Output Drive Strength Selection for Port E Bit 3 */
    byte PTEDS4      :1;                                       /* Output Drive Strength Selection for Port E Bit 4 */
    byte PTEDS5      :1;                                       /* Output Drive Strength Selection for Port E Bit 5 */
    byte PTEDS6      :1;                                       /* Output Drive Strength Selection for Port E Bit 6 */
    byte PTEDS7      :1;                                       /* Output Drive Strength Selection for Port E Bit 7 */
  } Bits;
} PTEDSSTR;
extern volatile PTEDSSTR _PTEDS @0x00001852;
#define PTEDS                           _PTEDS.Byte
#define PTEDS_PTEDS0                    _PTEDS.Bits.PTEDS0
#define PTEDS_PTEDS1                    _PTEDS.Bits.PTEDS1
#define PTEDS_PTEDS2                    _PTEDS.Bits.PTEDS2
#define PTEDS_PTEDS3                    _PTEDS.Bits.PTEDS3
#define PTEDS_PTEDS4                    _PTEDS.Bits.PTEDS4
#define PTEDS_PTEDS5                    _PTEDS.Bits.PTEDS5
#define PTEDS_PTEDS6                    _PTEDS.Bits.PTEDS6
#define PTEDS_PTEDS7                    _PTEDS.Bits.PTEDS7

#define PTEDS_PTEDS0_MASK               1U
#define PTEDS_PTEDS1_MASK               2U
#define PTEDS_PTEDS2_MASK               4U
#define PTEDS_PTEDS3_MASK               8U
#define PTEDS_PTEDS4_MASK               16U
#define PTEDS_PTEDS5_MASK               32U
#define PTEDS_PTEDS6_MASK               64U
#define PTEDS_PTEDS7_MASK               128U


/*** PTFPE - Port F Pull Enable Register; 0x00001854 ***/
typedef union {
  byte Byte;
  struct {
    byte PTFPE0      :1;                                       /* Internal Pull Enable for Port F Bit 0 */
    byte PTFPE1      :1;                                       /* Internal Pull Enable for Port F Bit 1 */
    byte PTFPE2      :1;                                       /* Internal Pull Enable for Port F Bit 2 */
    byte PTFPE3      :1;                                       /* Internal Pull Enable for Port F Bit 3 */
    byte PTFPE4      :1;                                       /* Internal Pull Enable for Port F Bit 4 */
    byte PTFPE5      :1;                                       /* Internal Pull Enable for Port F Bit 5 */
    byte PTFPE6      :1;                                       /* Internal Pull Enable for Port F Bit 6 */
    byte PTFPE7      :1;                                       /* Internal Pull Enable for Port F Bit 7 */
  } Bits;
} PTFPESTR;
extern volatile PTFPESTR _PTFPE @0x00001854;
#define PTFPE                           _PTFPE.Byte
#define PTFPE_PTFPE0                    _PTFPE.Bits.PTFPE0
#define PTFPE_PTFPE1                    _PTFPE.Bits.PTFPE1
#define PTFPE_PTFPE2                    _PTFPE.Bits.PTFPE2
#define PTFPE_PTFPE3                    _PTFPE.Bits.PTFPE3
#define PTFPE_PTFPE4                    _PTFPE.Bits.PTFPE4
#define PTFPE_PTFPE5                    _PTFPE.Bits.PTFPE5
#define PTFPE_PTFPE6                    _PTFPE.Bits.PTFPE6
#define PTFPE_PTFPE7                    _PTFPE.Bits.PTFPE7

#define PTFPE_PTFPE0_MASK               1U
#define PTFPE_PTFPE1_MASK               2U
#define PTFPE_PTFPE2_MASK               4U
#define PTFPE_PTFPE3_MASK               8U
#define PTFPE_PTFPE4_MASK               16U
#define PTFPE_PTFPE5_MASK               32U
#define PTFPE_PTFPE6_MASK               64U
#define PTFPE_PTFPE7_MASK               128U


/*** PTFSE - Port F Slew Rate Enable Register; 0x00001855 ***/
typedef union {
  byte Byte;
  struct {
    byte PTFSE0      :1;                                       /* Output Slew Rate Enable for Port F Bit 0 */
    byte PTFSE1      :1;                                       /* Output Slew Rate Enable for Port F Bit 1 */
    byte PTFSE2      :1;                                       /* Output Slew Rate Enable for Port F Bit 2 */
    byte PTFSE3      :1;                                       /* Output Slew Rate Enable for Port F Bit 3 */
    byte PTFSE4      :1;                                       /* Output Slew Rate Enable for Port F Bit 4 */
    byte PTFSE5      :1;                                       /* Output Slew Rate Enable for Port F Bit 5 */
    byte PTFSE6      :1;                                       /* Output Slew Rate Enable for Port F Bit 6 */
    byte PTFSE7      :1;                                       /* Output Slew Rate Enable for Port F Bit 7 */
  } Bits;
} PTFSESTR;
extern volatile PTFSESTR _PTFSE @0x00001855;
#define PTFSE                           _PTFSE.Byte
#define PTFSE_PTFSE0                    _PTFSE.Bits.PTFSE0
#define PTFSE_PTFSE1                    _PTFSE.Bits.PTFSE1
#define PTFSE_PTFSE2                    _PTFSE.Bits.PTFSE2
#define PTFSE_PTFSE3                    _PTFSE.Bits.PTFSE3
#define PTFSE_PTFSE4                    _PTFSE.Bits.PTFSE4
#define PTFSE_PTFSE5                    _PTFSE.Bits.PTFSE5
#define PTFSE_PTFSE6                    _PTFSE.Bits.PTFSE6
#define PTFSE_PTFSE7                    _PTFSE.Bits.PTFSE7

#define PTFSE_PTFSE0_MASK               1U
#define PTFSE_PTFSE1_MASK               2U
#define PTFSE_PTFSE2_MASK               4U
#define PTFSE_PTFSE3_MASK               8U
#define PTFSE_PTFSE4_MASK               16U
#define PTFSE_PTFSE5_MASK               32U
#define PTFSE_PTFSE6_MASK               64U
#define PTFSE_PTFSE7_MASK               128U


/*** PTFDS - Port F Drive Strength Selection Register; 0x00001856 ***/
typedef union {
  byte Byte;
  struct {
    byte PTFDS0      :1;                                       /* Output Drive Strength Selection for Port F Bit 0 */
    byte PTFDS1      :1;                                       /* Output Drive Strength Selection for Port F Bit 1 */
    byte PTFDS2      :1;                                       /* Output Drive Strength Selection for Port F Bit 2 */
    byte PTFDS3      :1;                                       /* Output Drive Strength Selection for Port F Bit 3 */
    byte PTFDS4      :1;                                       /* Output Drive Strength Selection for Port F Bit 4 */
    byte PTFDS5      :1;                                       /* Output Drive Strength Selection for Port F Bit 5 */
    byte PTFDS6      :1;                                       /* Output Drive Strength Selection for Port F Bit 6 */
    byte PTFDS7      :1;                                       /* Output Drive Strength Selection for Port F Bit 7 */
  } Bits;
} PTFDSSTR;
extern volatile PTFDSSTR _PTFDS @0x00001856;
#define PTFDS                           _PTFDS.Byte
#define PTFDS_PTFDS0                    _PTFDS.Bits.PTFDS0
#define PTFDS_PTFDS1                    _PTFDS.Bits.PTFDS1
#define PTFDS_PTFDS2                    _PTFDS.Bits.PTFDS2
#define PTFDS_PTFDS3                    _PTFDS.Bits.PTFDS3
#define PTFDS_PTFDS4                    _PTFDS.Bits.PTFDS4
#define PTFDS_PTFDS5                    _PTFDS.Bits.PTFDS5
#define PTFDS_PTFDS6                    _PTFDS.Bits.PTFDS6
#define PTFDS_PTFDS7                    _PTFDS.Bits.PTFDS7

#define PTFDS_PTFDS0_MASK               1U
#define PTFDS_PTFDS1_MASK               2U
#define PTFDS_PTFDS2_MASK               4U
#define PTFDS_PTFDS3_MASK               8U
#define PTFDS_PTFDS4_MASK               16U
#define PTFDS_PTFDS5_MASK               32U
#define PTFDS_PTFDS6_MASK               64U
#define PTFDS_PTFDS7_MASK               128U


/*** PTGPE - Port G Pull Enable Register; 0x00001858 ***/
typedef union {
  byte Byte;
  struct {
    byte PTGPE0      :1;                                       /* Internal Pull Enable for Port G Bit 0 */
    byte PTGPE1      :1;                                       /* Internal Pull Enable for Port G Bit 1 */
    byte PTGPE2      :1;                                       /* Internal Pull Enable for Port G Bit 2 */
    byte PTGPE3      :1;                                       /* Internal Pull Enable for Port G Bit 3 */
    byte PTGPE4      :1;                                       /* Internal Pull Enable for Port G Bit 4 */
    byte PTGPE5      :1;                                       /* Internal Pull Enable for Port G Bit 5 */
    byte PTGPE6      :1;                                       /* Internal Pull Enable for Port G Bit 6 */
    byte             :1; 
  } Bits;
  struct {
    byte grpPTGPE :7;
    byte         :1;
  } MergedBits;
} PTGPESTR;
extern volatile PTGPESTR _PTGPE @0x00001858;
#define PTGPE                           _PTGPE.Byte
#define PTGPE_PTGPE0                    _PTGPE.Bits.PTGPE0
#define PTGPE_PTGPE1                    _PTGPE.Bits.PTGPE1
#define PTGPE_PTGPE2                    _PTGPE.Bits.PTGPE2
#define PTGPE_PTGPE3                    _PTGPE.Bits.PTGPE3
#define PTGPE_PTGPE4                    _PTGPE.Bits.PTGPE4
#define PTGPE_PTGPE5                    _PTGPE.Bits.PTGPE5
#define PTGPE_PTGPE6                    _PTGPE.Bits.PTGPE6
#define PTGPE_PTGPE                     _PTGPE.MergedBits.grpPTGPE

#define PTGPE_PTGPE0_MASK               1U
#define PTGPE_PTGPE1_MASK               2U
#define PTGPE_PTGPE2_MASK               4U
#define PTGPE_PTGPE3_MASK               8U
#define PTGPE_PTGPE4_MASK               16U
#define PTGPE_PTGPE5_MASK               32U
#define PTGPE_PTGPE6_MASK               64U
#define PTGPE_PTGPE_MASK                127U
#define PTGPE_PTGPE_BITNUM              0U


/*** PTGSE - Port G Slew Rate Enable Register; 0x00001859 ***/
typedef union {
  byte Byte;
  struct {
    byte PTGSE0      :1;                                       /* Output Slew Rate Enable for Port G Bit 0 */
    byte PTGSE1      :1;                                       /* Output Slew Rate Enable for Port G Bit 1 */
    byte PTGSE2      :1;                                       /* Output Slew Rate Enable for Port G Bit 2 */
    byte PTGSE3      :1;                                       /* Output Slew Rate Enable for Port G Bit 3 */
    byte PTGSE4      :1;                                       /* Output Slew Rate Enable for Port G Bit 4 */
    byte PTGSE5      :1;                                       /* Output Slew Rate Enable for Port G Bit 5 */
    byte PTGSE6      :1;                                       /* Output Slew Rate Enable for Port G Bit 6 */
    byte             :1; 
  } Bits;
  struct {
    byte grpPTGSE :7;
    byte         :1;
  } MergedBits;
} PTGSESTR;
extern volatile PTGSESTR _PTGSE @0x00001859;
#define PTGSE                           _PTGSE.Byte
#define PTGSE_PTGSE0                    _PTGSE.Bits.PTGSE0
#define PTGSE_PTGSE1                    _PTGSE.Bits.PTGSE1
#define PTGSE_PTGSE2                    _PTGSE.Bits.PTGSE2
#define PTGSE_PTGSE3                    _PTGSE.Bits.PTGSE3
#define PTGSE_PTGSE4                    _PTGSE.Bits.PTGSE4
#define PTGSE_PTGSE5                    _PTGSE.Bits.PTGSE5
#define PTGSE_PTGSE6                    _PTGSE.Bits.PTGSE6
#define PTGSE_PTGSE                     _PTGSE.MergedBits.grpPTGSE

#define PTGSE_PTGSE0_MASK               1U
#define PTGSE_PTGSE1_MASK               2U
#define PTGSE_PTGSE2_MASK               4U
#define PTGSE_PTGSE3_MASK               8U
#define PTGSE_PTGSE4_MASK               16U
#define PTGSE_PTGSE5_MASK               32U
#define PTGSE_PTGSE6_MASK               64U
#define PTGSE_PTGSE_MASK                127U
#define PTGSE_PTGSE_BITNUM              0U


/*** PTGDS - Port G Drive Strength Selection Register; 0x0000185A ***/
typedef union {
  byte Byte;
  struct {
    byte PTGDS0      :1;                                       /* Output Drive Strength Selection for Port G Bit 0 */
    byte PTGDS1      :1;                                       /* Output Drive Strength Selection for Port G Bit 1 */
    byte PTGDS2      :1;                                       /* Output Drive Strength Selection for Port G Bit 2 */
    byte PTGDS3      :1;                                       /* Output Drive Strength Selection for Port G Bit 3 */
    byte PTGDS4      :1;                                       /* Output Drive Strength Selection for Port G Bit 4 */
    byte PTGDS5      :1;                                       /* Output Drive Strength Selection for Port G Bit 5 */
    byte PTGDS6      :1;                                       /* Output Drive Strength Selection for Port G Bit 6 */
    byte             :1; 
  } Bits;
  struct {
    byte grpPTGDS :7;
    byte         :1;
  } MergedBits;
} PTGDSSTR;
extern volatile PTGDSSTR _PTGDS @0x0000185A;
#define PTGDS                           _PTGDS.Byte
#define PTGDS_PTGDS0                    _PTGDS.Bits.PTGDS0
#define PTGDS_PTGDS1                    _PTGDS.Bits.PTGDS1
#define PTGDS_PTGDS2                    _PTGDS.Bits.PTGDS2
#define PTGDS_PTGDS3                    _PTGDS.Bits.PTGDS3
#define PTGDS_PTGDS4                    _PTGDS.Bits.PTGDS4
#define PTGDS_PTGDS5                    _PTGDS.Bits.PTGDS5
#define PTGDS_PTGDS6                    _PTGDS.Bits.PTGDS6
#define PTGDS_PTGDS                     _PTGDS.MergedBits.grpPTGDS

#define PTGDS_PTGDS0_MASK               1U
#define PTGDS_PTGDS1_MASK               2U
#define PTGDS_PTGDS2_MASK               4U
#define PTGDS_PTGDS3_MASK               8U
#define PTGDS_PTGDS4_MASK               16U
#define PTGDS_PTGDS5_MASK               32U
#define PTGDS_PTGDS6_MASK               64U
#define PTGDS_PTGDS_MASK                127U
#define PTGDS_PTGDS_BITNUM              0U


/*** NVBACKKEY0 - Backdoor Comparison Key 0; 0x0000FFB0 ***/
typedef union {
  byte Byte;
  struct {
    byte KEY0        :1;                                       /* Backdoor Comparison Key 0 Bits, bit 0 */
    byte KEY1        :1;                                       /* Backdoor Comparison Key 0 Bits, bit 1 */
    byte KEY2        :1;                                       /* Backdoor Comparison Key 0 Bits, bit 2 */
    byte KEY3        :1;                                       /* Backdoor Comparison Key 0 Bits, bit 3 */
    byte KEY4        :1;                                       /* Backdoor Comparison Key 0 Bits, bit 4 */
    byte KEY5        :1;                                       /* Backdoor Comparison Key 0 Bits, bit 5 */
    byte KEY6        :1;                                       /* Backdoor Comparison Key 0 Bits, bit 6 */
    byte KEY7        :1;                                       /* Backdoor Comparison Key 0 Bits, bit 7 */
  } Bits;
} NVBACKKEY0STR;
/* Tip for register initialization in the user code:  const byte NVBACKKEY0_INIT @0x0000FFB0 = <NVBACKKEY0_INITVAL>; */
#define _NVBACKKEY0 (*(const NVBACKKEY0STR * __far)0x0000FFB0)
#define NVBACKKEY0                      _NVBACKKEY0.Byte
#define NVBACKKEY0_KEY0                 _NVBACKKEY0.Bits.KEY0
#define NVBACKKEY0_KEY1                 _NVBACKKEY0.Bits.KEY1
#define NVBACKKEY0_KEY2                 _NVBACKKEY0.Bits.KEY2
#define NVBACKKEY0_KEY3                 _NVBACKKEY0.Bits.KEY3
#define NVBACKKEY0_KEY4                 _NVBACKKEY0.Bits.KEY4
#define NVBACKKEY0_KEY5                 _NVBACKKEY0.Bits.KEY5
#define NVBACKKEY0_KEY6                 _NVBACKKEY0.Bits.KEY6
#define NVBACKKEY0_KEY7                 _NVBACKKEY0.Bits.KEY7
/* NVBACKKEY_ARR: Access 8 NVBACKKEYx registers in an array */
#define NVBACKKEY_ARR                   ((volatile byte * __far) &NVBACKKEY0)

#define NVBACKKEY0_KEY0_MASK            1U
#define NVBACKKEY0_KEY1_MASK            2U
#define NVBACKKEY0_KEY2_MASK            4U
#define NVBACKKEY0_KEY3_MASK            8U
#define NVBACKKEY0_KEY4_MASK            16U
#define NVBACKKEY0_KEY5_MASK            32U
#define NVBACKKEY0_KEY6_MASK            64U
#define NVBACKKEY0_KEY7_MASK            128U


/*** NVBACKKEY1 - Backdoor Comparison Key 1; 0x0000FFB1 ***/
typedef union {
  byte Byte;
  struct {
    byte KEY0        :1;                                       /* Backdoor Comparison Key 1 Bits, bit 0 */
    byte KEY1        :1;                                       /* Backdoor Comparison Key 1 Bits, bit 1 */
    byte KEY2        :1;                                       /* Backdoor Comparison Key 1 Bits, bit 2 */
    byte KEY3        :1;                                       /* Backdoor Comparison Key 1 Bits, bit 3 */
    byte KEY4        :1;                                       /* Backdoor Comparison Key 1 Bits, bit 4 */
    byte KEY5        :1;                                       /* Backdoor Comparison Key 1 Bits, bit 5 */
    byte KEY6        :1;                                       /* Backdoor Comparison Key 1 Bits, bit 6 */
    byte KEY7        :1;                                       /* Backdoor Comparison Key 1 Bits, bit 7 */
  } Bits;
} NVBACKKEY1STR;
/* Tip for register initialization in the user code:  const byte NVBACKKEY1_INIT @0x0000FFB1 = <NVBACKKEY1_INITVAL>; */
#define _NVBACKKEY1 (*(const NVBACKKEY1STR * __far)0x0000FFB1)
#define NVBACKKEY1                      _NVBACKKEY1.Byte
#define NVBACKKEY1_KEY0                 _NVBACKKEY1.Bits.KEY0
#define NVBACKKEY1_KEY1                 _NVBACKKEY1.Bits.KEY1
#define NVBACKKEY1_KEY2                 _NVBACKKEY1.Bits.KEY2
#define NVBACKKEY1_KEY3                 _NVBACKKEY1.Bits.KEY3
#define NVBACKKEY1_KEY4                 _NVBACKKEY1.Bits.KEY4
#define NVBACKKEY1_KEY5                 _NVBACKKEY1.Bits.KEY5
#define NVBACKKEY1_KEY6                 _NVBACKKEY1.Bits.KEY6
#define NVBACKKEY1_KEY7                 _NVBACKKEY1.Bits.KEY7

#define NVBACKKEY1_KEY0_MASK            1U
#define NVBACKKEY1_KEY1_MASK            2U
#define NVBACKKEY1_KEY2_MASK            4U
#define NVBACKKEY1_KEY3_MASK            8U
#define NVBACKKEY1_KEY4_MASK            16U
#define NVBACKKEY1_KEY5_MASK            32U
#define NVBACKKEY1_KEY6_MASK            64U
#define NVBACKKEY1_KEY7_MASK            128U


/*** NVBACKKEY2 - Backdoor Comparison Key 2; 0x0000FFB2 ***/
typedef union {
  byte Byte;
  struct {
    byte KEY0        :1;                                       /* Backdoor Comparison Key 2 Bits, bit 0 */
    byte KEY1        :1;                                       /* Backdoor Comparison Key 2 Bits, bit 1 */
    byte KEY2        :1;                                       /* Backdoor Comparison Key 2 Bits, bit 2 */
    byte KEY3        :1;                                       /* Backdoor Comparison Key 2 Bits, bit 3 */
    byte KEY4        :1;                                       /* Backdoor Comparison Key 2 Bits, bit 4 */
    byte KEY5        :1;                                       /* Backdoor Comparison Key 2 Bits, bit 5 */
    byte KEY6        :1;                                       /* Backdoor Comparison Key 2 Bits, bit 6 */
    byte KEY7        :1;                                       /* Backdoor Comparison Key 2 Bits, bit 7 */
  } Bits;
} NVBACKKEY2STR;
/* Tip for register initialization in the user code:  const byte NVBACKKEY2_INIT @0x0000FFB2 = <NVBACKKEY2_INITVAL>; */
#define _NVBACKKEY2 (*(const NVBACKKEY2STR * __far)0x0000FFB2)
#define NVBACKKEY2                      _NVBACKKEY2.Byte
#define NVBACKKEY2_KEY0                 _NVBACKKEY2.Bits.KEY0
#define NVBACKKEY2_KEY1                 _NVBACKKEY2.Bits.KEY1
#define NVBACKKEY2_KEY2                 _NVBACKKEY2.Bits.KEY2
#define NVBACKKEY2_KEY3                 _NVBACKKEY2.Bits.KEY3
#define NVBACKKEY2_KEY4                 _NVBACKKEY2.Bits.KEY4
#define NVBACKKEY2_KEY5                 _NVBACKKEY2.Bits.KEY5
#define NVBACKKEY2_KEY6                 _NVBACKKEY2.Bits.KEY6
#define NVBACKKEY2_KEY7                 _NVBACKKEY2.Bits.KEY7

#define NVBACKKEY2_KEY0_MASK            1U
#define NVBACKKEY2_KEY1_MASK            2U
#define NVBACKKEY2_KEY2_MASK            4U
#define NVBACKKEY2_KEY3_MASK            8U
#define NVBACKKEY2_KEY4_MASK            16U
#define NVBACKKEY2_KEY5_MASK            32U
#define NVBACKKEY2_KEY6_MASK            64U
#define NVBACKKEY2_KEY7_MASK            128U


/*** NVBACKKEY3 - Backdoor Comparison Key 3; 0x0000FFB3 ***/
typedef union {
  byte Byte;
  struct {
    byte KEY0        :1;                                       /* Backdoor Comparison Key 3 Bits, bit 0 */
    byte KEY1        :1;                                       /* Backdoor Comparison Key 3 Bits, bit 1 */
    byte KEY2        :1;                                       /* Backdoor Comparison Key 3 Bits, bit 2 */
    byte KEY3        :1;                                       /* Backdoor Comparison Key 3 Bits, bit 3 */
    byte KEY4        :1;                                       /* Backdoor Comparison Key 3 Bits, bit 4 */
    byte KEY5        :1;                                       /* Backdoor Comparison Key 3 Bits, bit 5 */
    byte KEY6        :1;                                       /* Backdoor Comparison Key 3 Bits, bit 6 */
    byte KEY7        :1;                                       /* Backdoor Comparison Key 3 Bits, bit 7 */
  } Bits;
} NVBACKKEY3STR;
/* Tip for register initialization in the user code:  const byte NVBACKKEY3_INIT @0x0000FFB3 = <NVBACKKEY3_INITVAL>; */
#define _NVBACKKEY3 (*(const NVBACKKEY3STR * __far)0x0000FFB3)
#define NVBACKKEY3                      _NVBACKKEY3.Byte
#define NVBACKKEY3_KEY0                 _NVBACKKEY3.Bits.KEY0
#define NVBACKKEY3_KEY1                 _NVBACKKEY3.Bits.KEY1
#define NVBACKKEY3_KEY2                 _NVBACKKEY3.Bits.KEY2
#define NVBACKKEY3_KEY3                 _NVBACKKEY3.Bits.KEY3
#define NVBACKKEY3_KEY4                 _NVBACKKEY3.Bits.KEY4
#define NVBACKKEY3_KEY5                 _NVBACKKEY3.Bits.KEY5
#define NVBACKKEY3_KEY6                 _NVBACKKEY3.Bits.KEY6
#define NVBACKKEY3_KEY7                 _NVBACKKEY3.Bits.KEY7

#define NVBACKKEY3_KEY0_MASK            1U
#define NVBACKKEY3_KEY1_MASK            2U
#define NVBACKKEY3_KEY2_MASK            4U
#define NVBACKKEY3_KEY3_MASK            8U
#define NVBACKKEY3_KEY4_MASK            16U
#define NVBACKKEY3_KEY5_MASK            32U
#define NVBACKKEY3_KEY6_MASK            64U
#define NVBACKKEY3_KEY7_MASK            128U


/*** NVBACKKEY4 - Backdoor Comparison Key 4; 0x0000FFB4 ***/
typedef union {
  byte Byte;
  struct {
    byte KEY0        :1;                                       /* Backdoor Comparison Key 4 Bits, bit 0 */
    byte KEY1        :1;                                       /* Backdoor Comparison Key 4 Bits, bit 1 */
    byte KEY2        :1;                                       /* Backdoor Comparison Key 4 Bits, bit 2 */
    byte KEY3        :1;                                       /* Backdoor Comparison Key 4 Bits, bit 3 */
    byte KEY4        :1;                                       /* Backdoor Comparison Key 4 Bits, bit 4 */
    byte KEY5        :1;                                       /* Backdoor Comparison Key 4 Bits, bit 5 */
    byte KEY6        :1;                                       /* Backdoor Comparison Key 4 Bits, bit 6 */
    byte KEY7        :1;                                       /* Backdoor Comparison Key 4 Bits, bit 7 */
  } Bits;
} NVBACKKEY4STR;
/* Tip for register initialization in the user code:  const byte NVBACKKEY4_INIT @0x0000FFB4 = <NVBACKKEY4_INITVAL>; */
#define _NVBACKKEY4 (*(const NVBACKKEY4STR * __far)0x0000FFB4)
#define NVBACKKEY4                      _NVBACKKEY4.Byte
#define NVBACKKEY4_KEY0                 _NVBACKKEY4.Bits.KEY0
#define NVBACKKEY4_KEY1                 _NVBACKKEY4.Bits.KEY1
#define NVBACKKEY4_KEY2                 _NVBACKKEY4.Bits.KEY2
#define NVBACKKEY4_KEY3                 _NVBACKKEY4.Bits.KEY3
#define NVBACKKEY4_KEY4                 _NVBACKKEY4.Bits.KEY4
#define NVBACKKEY4_KEY5                 _NVBACKKEY4.Bits.KEY5
#define NVBACKKEY4_KEY6                 _NVBACKKEY4.Bits.KEY6
#define NVBACKKEY4_KEY7                 _NVBACKKEY4.Bits.KEY7

#define NVBACKKEY4_KEY0_MASK            1U
#define NVBACKKEY4_KEY1_MASK            2U
#define NVBACKKEY4_KEY2_MASK            4U
#define NVBACKKEY4_KEY3_MASK            8U
#define NVBACKKEY4_KEY4_MASK            16U
#define NVBACKKEY4_KEY5_MASK            32U
#define NVBACKKEY4_KEY6_MASK            64U
#define NVBACKKEY4_KEY7_MASK            128U


/*** NVBACKKEY5 - Backdoor Comparison Key 5; 0x0000FFB5 ***/
typedef union {
  byte Byte;
  struct {
    byte KEY0        :1;                                       /* Backdoor Comparison Key 5 Bits, bit 0 */
    byte KEY1        :1;                                       /* Backdoor Comparison Key 5 Bits, bit 1 */
    byte KEY2        :1;                                       /* Backdoor Comparison Key 5 Bits, bit 2 */
    byte KEY3        :1;                                       /* Backdoor Comparison Key 5 Bits, bit 3 */
    byte KEY4        :1;                                       /* Backdoor Comparison Key 5 Bits, bit 4 */
    byte KEY5        :1;                                       /* Backdoor Comparison Key 5 Bits, bit 5 */
    byte KEY6        :1;                                       /* Backdoor Comparison Key 5 Bits, bit 6 */
    byte KEY7        :1;                                       /* Backdoor Comparison Key 5 Bits, bit 7 */
  } Bits;
} NVBACKKEY5STR;
/* Tip for register initialization in the user code:  const byte NVBACKKEY5_INIT @0x0000FFB5 = <NVBACKKEY5_INITVAL>; */
#define _NVBACKKEY5 (*(const NVBACKKEY5STR * __far)0x0000FFB5)
#define NVBACKKEY5                      _NVBACKKEY5.Byte
#define NVBACKKEY5_KEY0                 _NVBACKKEY5.Bits.KEY0
#define NVBACKKEY5_KEY1                 _NVBACKKEY5.Bits.KEY1
#define NVBACKKEY5_KEY2                 _NVBACKKEY5.Bits.KEY2
#define NVBACKKEY5_KEY3                 _NVBACKKEY5.Bits.KEY3
#define NVBACKKEY5_KEY4                 _NVBACKKEY5.Bits.KEY4
#define NVBACKKEY5_KEY5                 _NVBACKKEY5.Bits.KEY5
#define NVBACKKEY5_KEY6                 _NVBACKKEY5.Bits.KEY6
#define NVBACKKEY5_KEY7                 _NVBACKKEY5.Bits.KEY7

#define NVBACKKEY5_KEY0_MASK            1U
#define NVBACKKEY5_KEY1_MASK            2U
#define NVBACKKEY5_KEY2_MASK            4U
#define NVBACKKEY5_KEY3_MASK            8U
#define NVBACKKEY5_KEY4_MASK            16U
#define NVBACKKEY5_KEY5_MASK            32U
#define NVBACKKEY5_KEY6_MASK            64U
#define NVBACKKEY5_KEY7_MASK            128U


/*** NVBACKKEY6 - Backdoor Comparison Key 6; 0x0000FFB6 ***/
typedef union {
  byte Byte;
  struct {
    byte KEY0        :1;                                       /* Backdoor Comparison Key 6 Bits, bit 0 */
    byte KEY1        :1;                                       /* Backdoor Comparison Key 6 Bits, bit 1 */
    byte KEY2        :1;                                       /* Backdoor Comparison Key 6 Bits, bit 2 */
    byte KEY3        :1;                                       /* Backdoor Comparison Key 6 Bits, bit 3 */
    byte KEY4        :1;                                       /* Backdoor Comparison Key 6 Bits, bit 4 */
    byte KEY5        :1;                                       /* Backdoor Comparison Key 6 Bits, bit 5 */
    byte KEY6        :1;                                       /* Backdoor Comparison Key 6 Bits, bit 6 */
    byte KEY7        :1;                                       /* Backdoor Comparison Key 6 Bits, bit 7 */
  } Bits;
} NVBACKKEY6STR;
/* Tip for register initialization in the user code:  const byte NVBACKKEY6_INIT @0x0000FFB6 = <NVBACKKEY6_INITVAL>; */
#define _NVBACKKEY6 (*(const NVBACKKEY6STR * __far)0x0000FFB6)
#define NVBACKKEY6                      _NVBACKKEY6.Byte
#define NVBACKKEY6_KEY0                 _NVBACKKEY6.Bits.KEY0
#define NVBACKKEY6_KEY1                 _NVBACKKEY6.Bits.KEY1
#define NVBACKKEY6_KEY2                 _NVBACKKEY6.Bits.KEY2
#define NVBACKKEY6_KEY3                 _NVBACKKEY6.Bits.KEY3
#define NVBACKKEY6_KEY4                 _NVBACKKEY6.Bits.KEY4
#define NVBACKKEY6_KEY5                 _NVBACKKEY6.Bits.KEY5
#define NVBACKKEY6_KEY6                 _NVBACKKEY6.Bits.KEY6
#define NVBACKKEY6_KEY7                 _NVBACKKEY6.Bits.KEY7

#define NVBACKKEY6_KEY0_MASK            1U
#define NVBACKKEY6_KEY1_MASK            2U
#define NVBACKKEY6_KEY2_MASK            4U
#define NVBACKKEY6_KEY3_MASK            8U
#define NVBACKKEY6_KEY4_MASK            16U
#define NVBACKKEY6_KEY5_MASK            32U
#define NVBACKKEY6_KEY6_MASK            64U
#define NVBACKKEY6_KEY7_MASK            128U


/*** NVBACKKEY7 - Backdoor Comparison Key 7; 0x0000FFB7 ***/
typedef union {
  byte Byte;
  struct {
    byte KEY0        :1;                                       /* Backdoor Comparison Key 7 Bits, bit 0 */
    byte KEY1        :1;                                       /* Backdoor Comparison Key 7 Bits, bit 1 */
    byte KEY2        :1;                                       /* Backdoor Comparison Key 7 Bits, bit 2 */
    byte KEY3        :1;                                       /* Backdoor Comparison Key 7 Bits, bit 3 */
    byte KEY4        :1;                                       /* Backdoor Comparison Key 7 Bits, bit 4 */
    byte KEY5        :1;                                       /* Backdoor Comparison Key 7 Bits, bit 5 */
    byte KEY6        :1;                                       /* Backdoor Comparison Key 7 Bits, bit 6 */
    byte KEY7        :1;                                       /* Backdoor Comparison Key 7 Bits, bit 7 */
  } Bits;
} NVBACKKEY7STR;
/* Tip for register initialization in the user code:  const byte NVBACKKEY7_INIT @0x0000FFB7 = <NVBACKKEY7_INITVAL>; */
#define _NVBACKKEY7 (*(const NVBACKKEY7STR * __far)0x0000FFB7)
#define NVBACKKEY7                      _NVBACKKEY7.Byte
#define NVBACKKEY7_KEY0                 _NVBACKKEY7.Bits.KEY0
#define NVBACKKEY7_KEY1                 _NVBACKKEY7.Bits.KEY1
#define NVBACKKEY7_KEY2                 _NVBACKKEY7.Bits.KEY2
#define NVBACKKEY7_KEY3                 _NVBACKKEY7.Bits.KEY3
#define NVBACKKEY7_KEY4                 _NVBACKKEY7.Bits.KEY4
#define NVBACKKEY7_KEY5                 _NVBACKKEY7.Bits.KEY5
#define NVBACKKEY7_KEY6                 _NVBACKKEY7.Bits.KEY6
#define NVBACKKEY7_KEY7                 _NVBACKKEY7.Bits.KEY7

#define NVBACKKEY7_KEY0_MASK            1U
#define NVBACKKEY7_KEY1_MASK            2U
#define NVBACKKEY7_KEY2_MASK            4U
#define NVBACKKEY7_KEY3_MASK            8U
#define NVBACKKEY7_KEY4_MASK            16U
#define NVBACKKEY7_KEY5_MASK            32U
#define NVBACKKEY7_KEY6_MASK            64U
#define NVBACKKEY7_KEY7_MASK            128U


/*** NVICGTRM1 - Non volatile storage of 250kHz ICG trim value; 0x0000FFBC ***/
typedef union {
  byte Byte;
  struct {
    byte NVTRIM0     :1;                                       /* ICG Trim Bit 0 */
    byte NVTRIM1     :1;                                       /* ICG Trim Bit 1 */
    byte NVTRIM2     :1;                                       /* ICG Trim Bit 2 */
    byte NVTRIM3     :1;                                       /* ICG Trim Bit 3 */
    byte NVTRIM4     :1;                                       /* ICG Trim Bit 4 */
    byte NVTRIM5     :1;                                       /* ICG Trim Bit 5 */
    byte NVTRIM6     :1;                                       /* ICG Trim Bit 6 */
    byte NVTRIM7     :1;                                       /* ICG Trim Bit 7 */
  } Bits;
} NVICGTRM1STR;
/* Tip for register initialization in the user code:  const byte NVICGTRM1_INIT @0x0000FFBC = <NVICGTRM1_INITVAL>; */
#define _NVICGTRM1 (*(const NVICGTRM1STR * __far)0x0000FFBC)
#define NVICGTRM1                       _NVICGTRM1.Byte
#define NVICGTRM1_NVTRIM0               _NVICGTRM1.Bits.NVTRIM0
#define NVICGTRM1_NVTRIM1               _NVICGTRM1.Bits.NVTRIM1
#define NVICGTRM1_NVTRIM2               _NVICGTRM1.Bits.NVTRIM2
#define NVICGTRM1_NVTRIM3               _NVICGTRM1.Bits.NVTRIM3
#define NVICGTRM1_NVTRIM4               _NVICGTRM1.Bits.NVTRIM4
#define NVICGTRM1_NVTRIM5               _NVICGTRM1.Bits.NVTRIM5
#define NVICGTRM1_NVTRIM6               _NVICGTRM1.Bits.NVTRIM6
#define NVICGTRM1_NVTRIM7               _NVICGTRM1.Bits.NVTRIM7

#define NVICGTRM1_NVTRIM0_MASK          1U
#define NVICGTRM1_NVTRIM1_MASK          2U
#define NVICGTRM1_NVTRIM2_MASK          4U
#define NVICGTRM1_NVTRIM3_MASK          8U
#define NVICGTRM1_NVTRIM4_MASK          16U
#define NVICGTRM1_NVTRIM5_MASK          32U
#define NVICGTRM1_NVTRIM6_MASK          64U
#define NVICGTRM1_NVTRIM7_MASK          128U


/*** NVPROT - Nonvolatile FLASH Protection Register; 0x0000FFBD ***/
typedef union {
  byte Byte;
  struct {
    byte FPDIS       :1;                                       /* FLASH Protection Disable */
    byte FPS1        :1;                                       /* FLASH Protect Select Bit 1 */
    byte FPS2        :1;                                       /* FLASH Protect Select Bit 2 */
    byte FPS3        :1;                                       /* FLASH Protect Select Bit 3 */
    byte FPS4        :1;                                       /* FLASH Protect Select Bit 4 */
    byte FPS5        :1;                                       /* FLASH Protect Select Bit 5 */
    byte FPS6        :1;                                       /* FLASH Protect Select Bit 6 */
    byte FPS7        :1;                                       /* FLASH Protect Select Bit 7 */
  } Bits;
  struct {
    byte         :1;
    byte grpFPS_1 :7;
  } MergedBits;
} NVPROTSTR;
/* Tip for register initialization in the user code:  const byte NVPROT_INIT @0x0000FFBD = <NVPROT_INITVAL>; */
#define _NVPROT (*(const NVPROTSTR * __far)0x0000FFBD)
#define NVPROT                          _NVPROT.Byte
#define NVPROT_FPDIS                    _NVPROT.Bits.FPDIS
#define NVPROT_FPS1                     _NVPROT.Bits.FPS1
#define NVPROT_FPS2                     _NVPROT.Bits.FPS2
#define NVPROT_FPS3                     _NVPROT.Bits.FPS3
#define NVPROT_FPS4                     _NVPROT.Bits.FPS4
#define NVPROT_FPS5                     _NVPROT.Bits.FPS5
#define NVPROT_FPS6                     _NVPROT.Bits.FPS6
#define NVPROT_FPS7                     _NVPROT.Bits.FPS7
#define NVPROT_FPS_1                    _NVPROT.MergedBits.grpFPS_1
#define NVPROT_FPS                      NVPROT_FPS_1

#define NVPROT_FPDIS_MASK               1U
#define NVPROT_FPS1_MASK                2U
#define NVPROT_FPS2_MASK                4U
#define NVPROT_FPS3_MASK                8U
#define NVPROT_FPS4_MASK                16U
#define NVPROT_FPS5_MASK                32U
#define NVPROT_FPS6_MASK                64U
#define NVPROT_FPS7_MASK                128U
#define NVPROT_FPS_1_MASK               254U
#define NVPROT_FPS_1_BITNUM             1U


/*** NVICGTRM - Non volatile ICG Trim Register; 0x0000FFBE ***/
typedef union {
  byte Byte;
  struct {
    byte NVTRIM0     :1;                                       /* ICG Trim Bit 0 */
    byte NVTRIM1     :1;                                       /* ICG Trim Bit 1 */
    byte NVTRIM2     :1;                                       /* ICG Trim Bit 2 */
    byte NVTRIM3     :1;                                       /* ICG Trim Bit 3 */
    byte NVTRIM4     :1;                                       /* ICG Trim Bit 4 */
    byte NVTRIM5     :1;                                       /* ICG Trim Bit 5 */
    byte NVTRIM6     :1;                                       /* ICG Trim Bit 6 */
    byte NVTRIM7     :1;                                       /* ICG Trim Bit 7 */
  } Bits;
} NVICGTRMSTR;
/* Tip for register initialization in the user code:  const byte NVICGTRM_INIT @0x0000FFBE = <NVICGTRM_INITVAL>; */
#define _NVICGTRM (*(const NVICGTRMSTR * __far)0x0000FFBE)
#define NVICGTRM                        _NVICGTRM.Byte
#define NVICGTRM_NVTRIM0                _NVICGTRM.Bits.NVTRIM0
#define NVICGTRM_NVTRIM1                _NVICGTRM.Bits.NVTRIM1
#define NVICGTRM_NVTRIM2                _NVICGTRM.Bits.NVTRIM2
#define NVICGTRM_NVTRIM3                _NVICGTRM.Bits.NVTRIM3
#define NVICGTRM_NVTRIM4                _NVICGTRM.Bits.NVTRIM4
#define NVICGTRM_NVTRIM5                _NVICGTRM.Bits.NVTRIM5
#define NVICGTRM_NVTRIM6                _NVICGTRM.Bits.NVTRIM6
#define NVICGTRM_NVTRIM7                _NVICGTRM.Bits.NVTRIM7

#define NVICGTRM_NVTRIM0_MASK           1U
#define NVICGTRM_NVTRIM1_MASK           2U
#define NVICGTRM_NVTRIM2_MASK           4U
#define NVICGTRM_NVTRIM3_MASK           8U
#define NVICGTRM_NVTRIM4_MASK           16U
#define NVICGTRM_NVTRIM5_MASK           32U
#define NVICGTRM_NVTRIM6_MASK           64U
#define NVICGTRM_NVTRIM7_MASK           128U


/*** NVOPT - Nonvolatile Flash Options Register; 0x0000FFBF ***/
typedef union {
  byte Byte;
  struct {
    byte SEC00       :1;                                       /* Security State Code, bit 0 */
    byte SEC01       :1;                                       /* Security State Code, bit 1 */
    byte             :1; 
    byte             :1; 
    byte             :1; 
    byte             :1; 
    byte FNORED      :1;                                       /* Vector Redirection Disable */
    byte KEYEN       :1;                                       /* Backdoor Key Mechanism Enable */
  } Bits;
  struct {
    byte grpSEC0 :2;
    byte         :1;
    byte         :1;
    byte         :1;
    byte         :1;
    byte         :1;
    byte         :1;
  } MergedBits;
} NVOPTSTR;
/* Tip for register initialization in the user code:  const byte NVOPT_INIT @0x0000FFBF = <NVOPT_INITVAL>; */
#define _NVOPT (*(const NVOPTSTR * __far)0x0000FFBF)
#define NVOPT                           _NVOPT.Byte
#define NVOPT_SEC00                     _NVOPT.Bits.SEC00
#define NVOPT_SEC01                     _NVOPT.Bits.SEC01
#define NVOPT_FNORED                    _NVOPT.Bits.FNORED
#define NVOPT_KEYEN                     _NVOPT.Bits.KEYEN
#define NVOPT_SEC0                      _NVOPT.MergedBits.grpSEC0

#define NVOPT_SEC00_MASK                1U
#define NVOPT_SEC01_MASK                2U
#define NVOPT_FNORED_MASK               64U
#define NVOPT_KEYEN_MASK                128U
#define NVOPT_SEC0_MASK                 3U
#define NVOPT_SEC0_BITNUM               0U



/* Flash commands */
#define mBlank                          0x05
#define mBurstProg                      0x25
#define mByteProg                       0x20
#define mMassErase                      0x41
#define mPageErase                      0x40


/***********************************************/
/**   D E P R E C I A T E D   S Y M B O L S   **/
/***********************************************/
#endif
/*lint -restore  +esym(961,18.4) +esym(961,19.7) Enable MISRA rule (1.1,18.4,6.4,19.7) checking. */
