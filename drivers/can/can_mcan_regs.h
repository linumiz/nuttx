/****************************************************************************
 * drivers/can/can_mcan_regs.h
 *
 * Bosch M_CAN register map and Message RAM element structs.
 * M_CAN User's Manual Rev 3.3.0.
 *
 * INTERNAL - included only by can_mcan.c.
 * SoC drivers must NOT include this directly; use the API in can_mcan.h.
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __DRIVERS_CAN_CAN_MCAN_REGS_H
#define __DRIVERS_CAN_CAN_MCAN_REGS_H

#include <stdint.h>
#include <nuttx/bits.h>

/* ------------------------------------------------------------------ */
/* Register offsets                                                    */
/* ------------------------------------------------------------------ */

#define CAN_MCAN_CREL         0x000
#define CAN_MCAN_CREL_REL     GENMASK(31, 28)
#define CAN_MCAN_CREL_STEP    GENMASK(27, 24)
#define CAN_MCAN_CREL_SUBSTEP GENMASK(23, 20)
#define CAN_MCAN_CREL_YEAR    GENMASK(19, 16)
#define CAN_MCAN_CREL_MON     GENMASK(15, 8)
#define CAN_MCAN_CREL_DAY     GENMASK(7, 0)

#define CAN_MCAN_ENDN         0x004
#define CAN_MCAN_CUST         0x008

#define CAN_MCAN_DBTP        0x00C
#define CAN_MCAN_DBTP_TDC    BIT(23)
#define CAN_MCAN_DBTP_DBRP   GENMASK(20, 16)
#define CAN_MCAN_DBTP_DTSEG1 GENMASK(12, 8)
#define CAN_MCAN_DBTP_DTSEG2 GENMASK(7, 4)
#define CAN_MCAN_DBTP_DSJW   GENMASK(3, 0)

#define CAN_MCAN_TEST       0x010
#define CAN_MCAN_TEST_SVAL  BIT(21)
#define CAN_MCAN_TEST_TXBNS GENMASK(20, 16)
#define CAN_MCAN_TEST_PVAL  BIT(13)
#define CAN_MCAN_TEST_TXBNP GENMASK(12, 8)
#define CAN_MCAN_TEST_RX    BIT(7)
#define CAN_MCAN_TEST_TX    GENMASK(6, 5)
#define CAN_MCAN_TEST_LBCK  BIT(4)

#define CAN_MCAN_RWD     0x014
#define CAN_MCAN_RWD_WDV GENMASK(15, 8)
#define CAN_MCAN_RWD_WDC GENMASK(7, 0)

#define CAN_MCAN_CCCR      0x018
#define CAN_MCAN_CCCR_NISO BIT(15)
#define CAN_MCAN_CCCR_TXP  BIT(14)
#define CAN_MCAN_CCCR_EFBI BIT(13)
#define CAN_MCAN_CCCR_PXHD BIT(12)
#define CAN_MCAN_CCCR_WMM  BIT(11)
#define CAN_MCAN_CCCR_UTSU BIT(10)
#define CAN_MCAN_CCCR_BRSE BIT(9)
#define CAN_MCAN_CCCR_FDOE BIT(8)
#define CAN_MCAN_CCCR_TEST BIT(7)
#define CAN_MCAN_CCCR_DAR  BIT(6)
#define CAN_MCAN_CCCR_MON  BIT(5)
#define CAN_MCAN_CCCR_CSR  BIT(4)
#define CAN_MCAN_CCCR_CSA  BIT(3)
#define CAN_MCAN_CCCR_ASM  BIT(2)
#define CAN_MCAN_CCCR_CCE  BIT(1)
#define CAN_MCAN_CCCR_INIT BIT(0)

#define CAN_MCAN_NBTP        0x01C
#define CAN_MCAN_NBTP_NSJW   GENMASK(31, 25)
#define CAN_MCAN_NBTP_NBRP   GENMASK(24, 16)
#define CAN_MCAN_NBTP_NTSEG1 GENMASK(15, 8)
#define CAN_MCAN_NBTP_NTSEG2 GENMASK(6, 0)

#define CAN_MCAN_TSCC     0x020
#define CAN_MCAN_TSCC_TCP GENMASK(19, 16)
#define CAN_MCAN_TSCC_TSS GENMASK(1, 0)

#define CAN_MCAN_TSCV     0x024
#define CAN_MCAN_TSCV_TSC GENMASK(15, 0)

#define CAN_MCAN_TOCC      0x028
#define CAN_MCAN_TOCC_TOP  GENMASK(31, 16)
#define CAN_MCAN_TOCC_TOS  GENMASK(2, 1)
#define CAN_MCAN_TOCC_ETOC BIT(1)

#define CAN_MCAN_TOCV     0x02C
#define CAN_MCAN_TOCV_TOC GENMASK(15, 0)

#define CAN_MCAN_ECR     0x040
#define CAN_MCAN_ECR_CEL GENMASK(23, 16)
#define CAN_MCAN_ECR_RP  BIT(15)
#define CAN_MCAN_ECR_REC GENMASK(14, 8)
#define CAN_MCAN_ECR_TEC GENMASK(7, 0)

#define CAN_MCAN_PSR      0x044
#define CAN_MCAN_PSR_TDCV GENMASK(22, 16)
#define CAN_MCAN_PSR_PXE  BIT(14)
#define CAN_MCAN_PSR_RFDF BIT(13)
#define CAN_MCAN_PSR_RBRS BIT(12)
#define CAN_MCAN_PSR_RESI BIT(11)
#define CAN_MCAN_PSR_DLEC GENMASK(10, 8)
#define CAN_MCAN_PSR_BO   BIT(7)
#define CAN_MCAN_PSR_EW   BIT(6)
#define CAN_MCAN_PSR_EP   BIT(5)
#define CAN_MCAN_PSR_ACT  GENMASK(4, 3)
#define CAN_MCAN_PSR_LEC  GENMASK(2, 0)

enum can_mcan_psr_lec
{
  CAN_MCAN_PSR_LEC_NO_ERROR    = 0,
  CAN_MCAN_PSR_LEC_STUFF_ERROR = 1,
  CAN_MCAN_PSR_LEC_FORM_ERROR  = 2,
  CAN_MCAN_PSR_LEC_ACK_ERROR   = 3,
  CAN_MCAN_PSR_LEC_BIT1_ERROR  = 4,
  CAN_MCAN_PSR_LEC_BIT0_ERROR  = 5,
  CAN_MCAN_PSR_LEC_CRC_ERROR   = 6,
  CAN_MCAN_PSR_LEC_NO_CHANGE   = 7
};

#define CAN_MCAN_TDCR      0x048
#define CAN_MCAN_TDCR_TDCO GENMASK(14, 8)
#define CAN_MCAN_TDCR_TDCF GENMASK(6, 0)

#define CAN_MCAN_IR      0x050
#define CAN_MCAN_IR_ARA  BIT(29)
#define CAN_MCAN_IR_PED  BIT(28)
#define CAN_MCAN_IR_PEA  BIT(27)
#define CAN_MCAN_IR_WDI  BIT(26)
#define CAN_MCAN_IR_BO   BIT(25)
#define CAN_MCAN_IR_EW   BIT(24)
#define CAN_MCAN_IR_EP   BIT(23)
#define CAN_MCAN_IR_ELO  BIT(22)
#define CAN_MCAN_IR_BEU  BIT(21)
#define CAN_MCAN_IR_BEC  BIT(20)
#define CAN_MCAN_IR_DRX  BIT(19)
#define CAN_MCAN_IR_TOO  BIT(18)
#define CAN_MCAN_IR_MRAF BIT(17)
#define CAN_MCAN_IR_TSW  BIT(16)
#define CAN_MCAN_IR_TEFL BIT(15)
#define CAN_MCAN_IR_TEFF BIT(14)
#define CAN_MCAN_IR_TEFW BIT(13)
#define CAN_MCAN_IR_TEFN BIT(12)
#define CAN_MCAN_IR_TFE  BIT(11)
#define CAN_MCAN_IR_TCF  BIT(10)
#define CAN_MCAN_IR_TC   BIT(9)
#define CAN_MCAN_IR_HPM  BIT(8)
#define CAN_MCAN_IR_RF1L BIT(7)
#define CAN_MCAN_IR_RF1F BIT(6)
#define CAN_MCAN_IR_RF1W BIT(5)
#define CAN_MCAN_IR_RF1N BIT(4)
#define CAN_MCAN_IR_RF0L BIT(3)
#define CAN_MCAN_IR_RF0F BIT(2)
#define CAN_MCAN_IR_RF0W BIT(1)
#define CAN_MCAN_IR_RF0N BIT(0)

#define CAN_MCAN_IE       0x054
#define CAN_MCAN_IE_ARAE  BIT(29)
#define CAN_MCAN_IE_PEDE  BIT(28)
#define CAN_MCAN_IE_PEAE  BIT(27)
#define CAN_MCAN_IE_WDIE  BIT(26)
#define CAN_MCAN_IE_BOE   BIT(25)
#define CAN_MCAN_IE_EWE   BIT(24)
#define CAN_MCAN_IE_EPE   BIT(23)
#define CAN_MCAN_IE_ELOE  BIT(22)
#define CAN_MCAN_IE_BEUE  BIT(21)
#define CAN_MCAN_IE_BECE  BIT(20)
#define CAN_MCAN_IE_DRXE  BIT(19)
#define CAN_MCAN_IE_TOOE  BIT(18)
#define CAN_MCAN_IE_MRAFE BIT(17)
#define CAN_MCAN_IE_TSWE  BIT(16)
#define CAN_MCAN_IE_TEFLE BIT(15)
#define CAN_MCAN_IE_TEFFE BIT(14)
#define CAN_MCAN_IE_TEFWE BIT(13)
#define CAN_MCAN_IE_TEFNE BIT(12)
#define CAN_MCAN_IE_TFEE  BIT(11)
#define CAN_MCAN_IE_TCFE  BIT(10)
#define CAN_MCAN_IE_TCE   BIT(9)
#define CAN_MCAN_IE_HPME  BIT(8)
#define CAN_MCAN_IE_RF1LE BIT(7)
#define CAN_MCAN_IE_RF1FE BIT(6)
#define CAN_MCAN_IE_RF1WE BIT(5)
#define CAN_MCAN_IE_RF1NE BIT(4)
#define CAN_MCAN_IE_RF0LE BIT(3)
#define CAN_MCAN_IE_RF0FE BIT(2)
#define CAN_MCAN_IE_RF0WE BIT(1)
#define CAN_MCAN_IE_RF0NE BIT(0)

#define CAN_MCAN_ILS       0x058
#define CAN_MCAN_ILS_ARAL  BIT(29)
#define CAN_MCAN_ILS_PEDL  BIT(28)
#define CAN_MCAN_ILS_PEAL  BIT(27)
#define CAN_MCAN_ILS_WDIL  BIT(26)
#define CAN_MCAN_ILS_BOL   BIT(25)
#define CAN_MCAN_ILS_EWL   BIT(24)
#define CAN_MCAN_ILS_EPL   BIT(23)
#define CAN_MCAN_ILS_ELOL  BIT(22)
#define CAN_MCAN_ILS_BEUL  BIT(21)
#define CAN_MCAN_ILS_BECL  BIT(20)
#define CAN_MCAN_ILS_DRXL  BIT(19)
#define CAN_MCAN_ILS_TOOL  BIT(18)
#define CAN_MCAN_ILS_MRAFL BIT(17)
#define CAN_MCAN_ILS_TSWL  BIT(16)
#define CAN_MCAN_ILS_TEFLL BIT(15)
#define CAN_MCAN_ILS_TEFFL BIT(14)
#define CAN_MCAN_ILS_TEFWL BIT(13)
#define CAN_MCAN_ILS_TEFNL BIT(12)
#define CAN_MCAN_ILS_TFEL  BIT(11)
#define CAN_MCAN_ILS_TCFL  BIT(10)
#define CAN_MCAN_ILS_TCL   BIT(9)
#define CAN_MCAN_ILS_HPML  BIT(8)
#define CAN_MCAN_ILS_RF1LL BIT(7)
#define CAN_MCAN_ILS_RF1FL BIT(6)
#define CAN_MCAN_ILS_RF1WL BIT(5)
#define CAN_MCAN_ILS_RF1NL BIT(4)
#define CAN_MCAN_ILS_RF0LL BIT(3)
#define CAN_MCAN_ILS_RF0FL BIT(2)
#define CAN_MCAN_ILS_RF0WL BIT(1)
#define CAN_MCAN_ILS_RF0NL BIT(0)

#define CAN_MCAN_ILE       0x05C
#define CAN_MCAN_ILE_EINT1 BIT(1)
#define CAN_MCAN_ILE_EINT0 BIT(0)

#define CAN_MCAN_GFC      0x080
#define CAN_MCAN_GFC_ANFS GENMASK(5, 4)
#define CAN_MCAN_GFC_ANFE GENMASK(3, 2)
#define CAN_MCAN_GFC_RRFS BIT(1)
#define CAN_MCAN_GFC_RRFE BIT(0)

#define CAN_MCAN_SIDFC       0x084
#define CAN_MCAN_SIDFC_LSS   GENMASK(23, 16)
#define CAN_MCAN_SIDFC_FLSSA GENMASK(15, 2)

#define CAN_MCAN_XIDFC       0x088
#define CAN_MCAN_XIDFC_LSS   GENMASK(22, 16)
#define CAN_MCAN_XIDFC_FLESA GENMASK(15, 2)

#define CAN_MCAN_XIDAM      0x090
#define CAN_MCAN_XIDAM_EIDM GENMASK(28, 0)

#define CAN_MCAN_HPMS      0x094
#define CAN_MCAN_HPMS_FLST BIT(15)
#define CAN_MCAN_HPMS_FIDX GENMASK(14, 8)
#define CAN_MCAN_HPMS_MSI  GENMASK(7, 6)
#define CAN_MCAN_HPMS_BIDX GENMASK(5, 0)

#define CAN_MCAN_NDAT1    0x098
#define CAN_MCAN_NDAT2    0x09C

#define CAN_MCAN_RXF0C      0x0A0
#define CAN_MCAN_RXF0C_F0OM BIT(31)
#define CAN_MCAN_RXF0C_F0WM GENMASK(30, 24)
#define CAN_MCAN_RXF0C_F0S  GENMASK(22, 16)
#define CAN_MCAN_RXF0C_F0SA GENMASK(15, 2)

#define CAN_MCAN_RXF0S      0x0A4
#define CAN_MCAN_RXF0S_RF0L BIT(25)
#define CAN_MCAN_RXF0S_F0F  BIT(24)
#define CAN_MCAN_RXF0S_F0PI GENMASK(21, 16)
#define CAN_MCAN_RXF0S_F0GI GENMASK(13, 8)
#define CAN_MCAN_RXF0S_F0FL GENMASK(6, 0)

#define CAN_MCAN_RXF0A      0x0A8
#define CAN_MCAN_RXF0A_F0AI GENMASK(5, 0)

#define CAN_MCAN_RXBC      0x0AC
#define CAN_MCAN_RXBC_RBSA GENMASK(15, 2)

#define CAN_MCAN_RXF1C      0x0B0
#define CAN_MCAN_RXF1C_F1OM BIT(31)
#define CAN_MCAN_RXF1C_F1WM GENMASK(30, 24)
#define CAN_MCAN_RXF1C_F1S  GENMASK(22, 16)
#define CAN_MCAN_RXF1C_F1SA GENMASK(15, 2)

#define CAN_MCAN_RXF1S      0x0B4
#define CAN_MCAN_RXF1S_RF1L BIT(25)
#define CAN_MCAN_RXF1S_F1F  BIT(24)
#define CAN_MCAN_RXF1S_F1PI GENMASK(21, 16)
#define CAN_MCAN_RXF1S_F1GI GENMASK(13, 8)
#define CAN_MCAN_RXF1S_F1FL GENMASK(6, 0)

#define CAN_MCAN_RXF1A      0x0B8
#define CAN_MCAN_RXF1A_F1AI GENMASK(5, 0)

#define CAN_MCAN_RXESC      0x0BC
#define CAN_MCAN_RXESC_RBDS GENMASK(10, 8)
#define CAN_MCAN_RXESC_F1DS GENMASK(6, 4)
#define CAN_MCAN_RXESC_F0DS GENMASK(2, 0)

#define CAN_MCAN_TXBC      0x0C0
#define CAN_MCAN_TXBC_TFQM BIT(30)
#define CAN_MCAN_TXBC_TFQS GENMASK(29, 24)
#define CAN_MCAN_TXBC_NDTB GENMASK(21, 16)
#define CAN_MCAN_TXBC_TBSA GENMASK(15, 2)

#define CAN_MCAN_TXFQS       0x0C4
#define CAN_MCAN_TXFQS_TFQF  BIT(21)
#define CAN_MCAN_TXFQS_TFQPI GENMASK(20, 16)
#define CAN_MCAN_TXFQS_TFGI  GENMASK(12, 8)
#define CAN_MCAN_TXFQS_TFFL  GENMASK(5, 0)

#define CAN_MCAN_TXESC      0x0C8
#define CAN_MCAN_TXESC_TBDS GENMASK(2, 0)

#define CAN_MCAN_TXBRP     0x0CC
#define CAN_MCAN_TXBRP_TRP GENMASK(31, 0)

#define CAN_MCAN_TXBAR    0x0D0
#define CAN_MCAN_TXBAR_AR GENMASK(31, 0)

#define CAN_MCAN_TXBCR    0x0D4
#define CAN_MCAN_TXBCR_CR GENMASK(31, 0)

#define CAN_MCAN_TXBTO    0x0D8
#define CAN_MCAN_TXBTO_TO GENMASK(31, 0)

#define CAN_MCAN_TXBCF    0x0DC
#define CAN_MCAN_TXBCF_CF GENMASK(31, 0)

#define CAN_MCAN_TXBTIE     0x0E0
#define CAN_MCAN_TXBTIE_TIE GENMASK(31, 0)

#define CAN_MCAN_TXBCIE      0x0E4
#define CAN_MCAN_TXBCIE_CFIE GENMASK(31, 0)

#define CAN_MCAN_TXEFC      0x0F0
#define CAN_MCAN_TXEFC_EFWM GENMASK(29, 24)
#define CAN_MCAN_TXEFC_EFS  GENMASK(21, 16)
#define CAN_MCAN_TXEFC_EFSA GENMASK(15, 2)

#define CAN_MCAN_TXEFS      0x0F4
#define CAN_MCAN_TXEFS_TEFL BIT(25)
#define CAN_MCAN_TXEFS_EFF  BIT(24)
#define CAN_MCAN_TXEFS_EFPI GENMASK(20, 16)
#define CAN_MCAN_TXEFS_EFGI GENMASK(12, 8)
#define CAN_MCAN_TXEFS_EFFL GENMASK(5, 0)

#define CAN_MCAN_TXEFA      0x0F8
#define CAN_MCAN_TXEFA_EFAI GENMASK(4, 0)

/* ------------------------------------------------------------------ */
/* Message RAM element structs - M_CAN UM s2.4                        */
/* ------------------------------------------------------------------ */

struct can_mcan_rx_fifo_hdr
{
  union
  {
    struct { uint32_t ext_id:29; uint32_t rtr:1; uint32_t xtd:1; uint32_t esi:1; };
    struct { uint32_t pad1:18;   uint32_t std_id:11; uint32_t pad2:3; };
  };
  uint32_t rxts : 16;
  uint32_t dlc  : 4;
  uint32_t brs  : 1;
  uint32_t fdf  : 1;
  uint32_t res  : 2;
  uint32_t fidx : 7;
  uint32_t anmf : 1;
} __attribute__((packed, aligned(4)));

struct can_mcan_rx_fifo
{
  struct can_mcan_rx_fifo_hdr hdr;
  union { uint8_t data[64]; uint32_t data_32[16]; };
} __attribute__((packed, aligned(4)));

struct can_mcan_tx_buffer_hdr
{
  union
  {
    struct { uint32_t ext_id:29; uint32_t rtr:1; uint32_t xtd:1; uint32_t esi:1; };
    struct { uint32_t pad1:18;   uint32_t std_id:11; uint32_t pad2:3; };
  };
  uint16_t res1;
  uint8_t  dlc:4; uint8_t brs:1; uint8_t fdf:1; uint8_t tsce:1; uint8_t efc:1;
  uint8_t  mm;
} __attribute__((packed, aligned(4)));

struct can_mcan_tx_buffer
{
  struct can_mcan_tx_buffer_hdr hdr;
  union { uint8_t data[64]; uint32_t data_32[16]; };
} __attribute__((packed, aligned(4)));

struct can_mcan_tx_event_fifo
{
  union
  {
    struct { uint32_t ext_id:29; uint32_t rtr:1; uint32_t xtd:1; uint32_t esi:1; };
    struct { uint32_t pad1:18;   uint32_t std_id:11; uint32_t pad2:3; };
  };
  uint16_t txts;
  uint8_t  dlc:4; uint8_t brs:1; uint8_t fdf:1; uint8_t et:2;
  uint8_t  mm;
} __attribute__((packed, aligned(4)));

/* Filter element config (SFEC/EFEC) */
#define CAN_MCAN_XFEC_DISABLE    0x0
#define CAN_MCAN_XFEC_FIFO0      0x1
#define CAN_MCAN_XFEC_FIFO1      0x2
#define CAN_MCAN_XFEC_REJECT     0x3
#define CAN_MCAN_XFEC_PRIO       0x4
#define CAN_MCAN_XFEC_PRIO_FIFO0 0x5
#define CAN_MCAN_XFEC_PRIO_FIFO1 0x7

/* Standard filter type (SFT) */
#define CAN_MCAN_SFT_RANGE    0x0
#define CAN_MCAN_SFT_DUAL     0x1
#define CAN_MCAN_SFT_CLASSIC  0x2
#define CAN_MCAN_SFT_DISABLED 0x3

struct can_mcan_std_filter
{
  uint32_t sfid2:11; uint32_t res:5; uint32_t sfid1:11;
  uint32_t sfec:3;   uint32_t sft:2;
} __attribute__((packed, aligned(4)));

/* Extended filter type (EFT) */
#define CAN_MCAN_EFT_RANGE_XIDAM 0x0
#define CAN_MCAN_EFT_DUAL        0x1
#define CAN_MCAN_EFT_CLASSIC     0x2
#define CAN_MCAN_EFT_RANGE       0x3

struct can_mcan_ext_filter
{
  uint32_t efid1:29; uint32_t efec:3;
  uint32_t efid2:29; uint32_t esync:1; uint32_t eft:2;
} __attribute__((packed, aligned(4)));

#endif /* __DRIVERS_CAN_CAN_MCAN_REGS_H */
