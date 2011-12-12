/* ============================================================================================



                        G D M 7 010 4     M E M O R Y     M A P






 This file describes GDM7014 register map. Since GDM7014 uses 6 bit address for host interface,

 it ranges from 0x00 ~ 0x3F. All host APIs are based on the register addresses defined in this

 file.







                         GCT Semiconductor Inc. All Rights Reserved.
 


============================================================================================ */



/* ============================================================================================
                                 C O D E     H I S T O R Y
===============================================================================================

-----------------------------------------------------------------------------------------------
When              Who              What                        (in reverse chronological order)
-----------------------------------------------------------------------------------------------

Dec.14.2007       Kurt Yi       Created

-------------------------------------------------------------------------------------------- */

#ifndef _GDM7024_MEMORY_MAP_H
#define _GDM7024_MEMORY_MAP_H

/* ============================================================================================
                                     D E F I N T I O N S
============================================================================================ */

/* --------------------------------------------------------------------------------------------
                                      Internal Memory
-------------------------------------------------------------------------------------------- */

/* IROM */
#define GDM7024_IRAM_BASE               0x04020000
#define GDM_IROM_BASE                   GDM7024_IRAM_BASE

/* YMEM */
#define GDM7024_YMEM_BASE               0x38001bbc
#define GDM7024R0C_YMEM_BASE            0x38002600//0x38004400
#define GDM7024R1A_YMEM_BASE            0x38002A00
#define GDM_YMEM_BASE                   GDM7024R1A_YMEM_BASE
//#define GDM_YMEM_BASE                   GDM7024_YMEM_BASE


/* CHIP ID */
#define GDM7024_SYS_GLB_CTRL_CHIP_ID    0x3C006C88
#define GDM_SYS_GLB_CTRL_CHIP_ID        GDM7024_SYS_GLB_CTRL_CHIP_ID


#define GDM7024_IROM_BASE               0x4000000
#define GDM7024_IROM_SIZE              (68*1024)
#define GDM7024_BIN_INFO_SIZE          (40)

#define GDM7024_BIN_INFO_MREV_VER_OFFSET (17)

#define GDM7024_BIN_INFO_MREV_OFFSET     (16)

#define GDM7024_IRAM_BIN_SIZE          (1024*40)

/* --------------------------------------------------------------------------------------------
                                   CPU Debugger Registers
-------------------------------------------------------------------------------------------- */

/* Debug Command */
#define GDM7024_DEBUG_CMD               0x3C006400
#define GDM_DEBUG_CMD                   GDM7024_DEBUG_CMD



/* --------------------------------------------------------------------------------------------
                                   System Control Registers
-------------------------------------------------------------------------------------------- */

/* System Status */
#define GDM7024_SYS_STA                 0x3C006C84
#define GDM_SYS_STA                     GDM7024_SYS_STA



/* --------------------------------------------------------------------------------------------
                                        GHOST Registers
-------------------------------------------------------------------------------------------- */

/* Control Register */
#define GHOST_CTRL                      0x01

// Interrupt
#define GHOST_EXT_IS                    0x02
#define GHOST_EXT_IC                    0x03
#define GHOST_EXT_IM                    0x04

/* Host Should not use this */
#define GHOST_INT_IS                    0x05
#define GHOST_INT_IC                    0x06
#define GHOST_INT_IM                    0x07

/* Configuration Register Access */
#define GHOST_CR_CMD                    0x08
#define GHOST_CR_ADDR                   0x09
#define GHOST_CR_DATA                   0x0A

/* Internal Memory Access */
#define GHOST_MM_CMD                    0x0B
#define GHOST_MM_ADDR_H                 0x0C
#define GHOST_MM_ADDR_L                 0x0D
#define GHOST_MM_DATA_H                 0x0E
#define GHOST_MM_DATA_L                 0x0F

/* Internal Buffer Access */
#define GHOST_DAB0_FIC                  0x10
#define GHOST_DAB0_MSC                  0x1D
#define GHOST_DM                        0x1F


#define GHOST_DAB1_FIC                  0x30
#define GHOST_DAB1_MSC                  0x3D

/* --------------------------------------------------------------------------------------------
                                        GHOST CR Command
-------------------------------------------------------------------------------------------- */

#define GHOST_CR_CMD_ACTIVATE           0x0001

#define GHOST_CR_CMD_READ               0x0000
#define GHOST_CR_CMD_WRITE              0x0002

#define GHOST_CR_CMD_BIG_ENDIAN         0x0000
#define GHOST_CR_CMD_LITTLE_ENDIAN      0x0004



/* --------------------------------------------------------------------------------------------
                                        GHOST MM Command
-------------------------------------------------------------------------------------------- */

#define GHOST_MM_CMD_ACTIVATE           0x0001

#define GHOST_MM_CMD_READ               0x0000
#define GHOST_MM_CMD_WRITE              0x0002

#define GHOST_MM_CMD_LITTLE_ENDIAN      0x0004
#define GHOST_MM_CMD_BIG_ENDIAN         0x0000

#define GHOST_MM_CMD_BYTE               0x0000
#define GHOST_MM_CMD_HALFWORD           0x0008
#define GHOST_MM_CMD_WORD               0x0010

#define GHOST_MM_CMD_BURST              0x0020



/* --------------------------------------------------------------------------------------------
                           Overall Command CR(Configuration Register)
-------------------------------------------------------------------------------------------- */

/* System Check */
#define CR_DEV_ID                       0x0000

/* DM Control */
#define CR_DM_CTRL                      0x0100

/* --------------------------------------------------------------------------------------------
                          DAB0 Command CR(Configuration Register)
-------------------------------------------------------------------------------------------- */

/* Initial Settings */
#define CR_DAB0_MODE                    0x2000
#define CR_DAB0_FIC_WM                  0x2001
#define CR_DAB0_MSC_WM                  0x2002
#define CR_DAB0_ERR_TS_DISCARD          0x2003

/* Control */
#define CR_DAB0_SYS_CTRL                0x2100
#define CR_DAB0_DEMOD_STAT              0x2101
#define CR_DAB0_FREQ                    0x2102

/* Buffer Control */
#define CR_DAB0_STORED_FIB_SIZE         0x2200
#define CR_DAB0_FIC_CRC                 0x2201
#define CR_DAB0_STORED_MSC_SIZE         0x2202
#define CR_DAB0_CLR_MSC_BUF             0x2203
#define CR_DAB0_ENABLE_SERIAL_TS        0x2204
#define CR_DAB0_ENCAP_FIC_DM            0x2205

/* Sub Channel Settings */
#define CR_DAB0_SET_SUBCH_TS0           0x2300
#define CR_DAB0_SET_SUBCH_TS1           0x2301
/* Reserved : 0x2302 ~ 0x2303 */
#define CR_DAB0_SET_SUBCH_NTS0          0x2304
#define CR_DAB0_SET_SUBCH_NTS1          0x2305
#define CR_DAB0_SET_SUBCH_NTS2          0x2306
#define CR_DAB0_SET_SUBCH_NTS3          0x2307

/* Calibration */
#define CR_DAB0_CAL_MODE                0x2400
#define CR_DAB0_CAL_LNA_GAIN            0x2401
#define CR_DAB0_CAL_RSSI_DIFF0          0x2402
#define CR_DAB0_CAL_RSSI_DIFF1          0x2403
#define CR_DAB0_CAL_RSSI_DIFF3          0x2404


/* --------------------------------------------------------------------------------------------
                          DAB1 Command CR(Configuration Register)
-------------------------------------------------------------------------------------------- */

/* Initial Settings */
#define CR_DAB1_MODE                    0x3000
#define CR_DAB1_FIC_WM                  0x3001
#define CR_DAB1_MSC_WM                  0x3002
#define CR_DAB1_ERR_TS_DISCARD          0x3003

/* Control */
#define CR_DAB1_SYS_CTRL                0x3100
#define CR_DAB1_DEMOD_STAT              0x3101
#define CR_DAB1_FREQ                    0x3102

/* Buffer Control */
#define CR_DAB1_STORED_FIB_SIZE         0x3200
#define CR_DAB1_FIC_CRC                 0x3201
#define CR_DAB1_STORED_MSC_SIZE         0x3202
#define CR_DAB1_CLR_MSC_BUF             0x3203
#define CR_DAB1_ENABLE_SERIAL_TS        0x3204
#define CR_DAB1_ENCAP_FIC_DM            0x2205

/* Sub Channel Settings */
#define CR_DAB1_SET_SUBCH_TS0           0x3300
#define CR_DAB1_SET_SUBCH_TS1           0x3301
/* Reserved : 0x3302 ~ 0x3303 */
#define CR_DAB1_SET_SUBCH_NTS0          0x3304
#define CR_DAB1_SET_SUBCH_NTS1          0x3305
#define CR_DAB1_SET_SUBCH_NTS2          0x3306
#define CR_DAB1_SET_SUBCH_NTS3          0x3307

/* Calibration */
#define CR_DAB1_CAL_MODE                0x3400
#define CR_DAB1_CAL_LNA_GAIN            0x3401
#define CR_DAB1_CAL_RSSI_DIFF0          0x3402
#define CR_DAB1_CAL_RSSI_DIFF1          0x3403
#define CR_DAB1_CAL_RSSI_DIFF3          0x3404

#define EV_DAB_FIC                      0x0001
#define EV_DAB_MSC                      0x0002
#define EV_DAB_OV_MSC                   0x0004
#define EV_DAB_SYNC_LOSS                0x0008
#define EV_DAB_RECONF                   0x0010
#define EV_DAB_SUBCH_CONF_ERR           0x0020

#endif /* _GDM7024_MEMORY_MAP_H */

