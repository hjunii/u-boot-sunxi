#include <common.h>
#include <malloc.h>

#include "sw_udc.h"

#include <asm/io.h>

#include <asm/arch/cpu.h>
#include <asm/arch/clock.h>

#define DRIVER_DESC     "SoftWinner USB Device Controller"
#define DRIVER_VERSION  "20080411"
#define DRIVER_AUTHOR   "SoftWinner USB Developer"

static const char gadget_name[] = "sw_usb_udc";
static const char driver_desc[] = DRIVER_DESC;

static struct sw_udc *the_controller = NULL;
static u32 usbd_port_no = 0;
static sw_udc_io_t g_sw_udc_io;
static u32 usb_connect = 0;
static u32 is_controller_alive = 0;
static u8 is_udc_enable = 0; /* is udc enable by gadget? */

static u8 crq_bRequest = 0;
static const unsigned char TestPkt[54] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAA,
	0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xEE, 0xEE, 0xEE,
	0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0xBF, 0xDF,
	0xEF, 0xF7, 0xFB, 0xFD, 0xFC, 0x7E, 0xBF, 0xDF, 0xEF, 0xF7,
	0xFB, 0xFD, 0x7E, 0x00};

static u32 USBC_ReadLenFromFifo(void __iomem *base, u32 ep_type)
{
	switch (ep_type) {
	case SUNXI_EP_TYPE_EP0:
		  return readw(base + SUNXI_COUNT0);
	case SUNXI_EP_TYPE_TX:
		  return 0;
	case SUNXI_EP_TYPE_RX:
		  return readw(base + SUNXI_RXCOUNT);
	default:
		  return 0;
	}
}

static u32 USBC_ReadPacket(void __iomem *base, u32 fifo, u32 cnt, void *buff)
{
    u32 len = 0;
    u32 i32 = 0;
    u32 i8  = 0;
    u8  *buf8  = NULL;
    u32 *buf32 = NULL;

    buf32 = buff;
    len   = cnt;

    i32 = len >> 2;
    i8  = len & 0x03;

    while (i32--) {
        *buf32++ = readl(fifo);
    }
    buf8 = (__u8 *)buf32;
    while (i8--) {
        *buf8++ = readb(fifo);
    }

    return len;
}

static u32 USBC_WritePacket(void __iomem *base, u32 fifo, u32 cnt, void *buff)
{
    u32 len = 0;
    u32 i32 = 0;
    u32 i8  = 0;
    u8  *buf8  = NULL;
    u32 *buf32 = NULL;

    buf32 = buff;
    len   = cnt;

    i32 = len >> 2;
    i8  = len & 0x03;

    while (i32--) {
        writel(*buf32++, fifo);
    }

    buf8 = (__u8 *)buf32;
    while (i8--){
        writeb(*buf8++, fifo);
    }

    return len;
}

static u32 USBC_GetActiveEp(void __iomem *base)
{
    return readb(base + SUNXI_EPIND);
}

static void USBC_SelectActiveEp(void __iomem *base, u8 ep_index)
{
    writeb(ep_index, base + SUNXI_EPIND);
}

static u32 USBC_SelectFIFO(void __iomem *base, u32 ep_index)
{
    return (u32) (base + SUNXI_EPFIFOx(ep_index));
}

static void __USBC_Dev_ep0_ReadDataComplete(void __iomem *base)
{
    writew((1 << SUNXI_BP_CSR0_D_SERVICED_RX_PKT_READY) | (1 << SUNXI_BP_CSR0_D_DATA_END),
            base + SUNXI_CSR0);
}

static void __USBC_Dev_Rx_ReadDataComplete(void __iomem *base)
{
    __u32 reg_val = 0;

    reg_val = readw(base + SUNXI_RXCSR);
    reg_val &= ~(1 << SUNXI_BP_RXCSR_D_RX_PKT_READY);
    reg_val &= ~(1 << SUNXI_BP_RXCSR_D_OVERRUN);
    reg_val &= ~(1 << SUNXI_BP_RXCSR_D_DATA_ERROR);
    writew(reg_val, base + SUNXI_RXCSR);
}

static s32 __USBC_Dev_ReadDataComplete(void __iomem *base, u32 ep_type)
{
    switch (ep_type) {
    case SUNXI_EP_TYPE_EP0:
        __USBC_Dev_ep0_ReadDataComplete(base);
        break;
    case SUNXI_EP_TYPE_TX:
        return -1;
    case SUNXI_EP_TYPE_RX:
        __USBC_Dev_Rx_ReadDataComplete(base);
        break;
    default:
        return -1;
    }
    return 0;
}

static void __USBC_Dev_ep0_ReadDataHalf(void __iomem *base)
{
    writew(1 << SUNXI_BP_CSR0_D_SERVICED_RX_PKT_READY, base + SUNXI_CSR0);
}

static void __USBC_Dev_Rx_ReadDataHalf(void __iomem *base)
{
    u32 reg_val = 0;

    reg_val = readw(base + SUNXI_RXCSR);
    reg_val &= ~(1 << SUNXI_BP_RXCSR_D_RX_PKT_READY);
    reg_val &= ~(1 << SUNXI_BP_RXCSR_D_OVERRUN);
    reg_val &= ~(1 << SUNXI_BP_RXCSR_D_DATA_ERROR);
    writew(reg_val, base + SUNXI_RXCSR);
}

static s32 __USBC_Dev_ReadDataHalf(void __iomem *base, u32 ep_type)
{
    switch (ep_type) {
    case SUNXI_EP_TYPE_EP0:
        __USBC_Dev_ep0_ReadDataHalf(base);
        break;
    case SUNXI_EP_TYPE_TX:
        return -1;
    case SUNXI_EP_TYPE_RX:
        __USBC_Dev_Rx_ReadDataHalf(base);
        break;
    default:
        return -1;
    }
    return 0;
}

static s32 USBC_Dev_ReadDataStatus(void __iomem *base, u32 ep_type, u32 complete)
{
    if (complete) {
        return __USBC_Dev_ReadDataComplete(base, ep_type);
    } else {
        return __USBC_Dev_ReadDataHalf(base, ep_type);
    }
}

static void __USBC_Dev_ep0_WriteDataComplete(void __iomem *base)
{
    writew((1 << SUNXI_BP_CSR0_D_TX_PKT_READY) | (1 << SUNXI_BP_CSR0_D_DATA_END),
            base + SUNXI_CSR0);
}

static void __USBC_Dev_Tx_WriteDataComplete(void __iomem *base)
{
    u16 ep_csr = 0;

    ep_csr = readw(base + SUNXI_TXCSR);
    ep_csr |= 1 << SUNXI_BP_TXCSR_D_TX_READY;
    ep_csr &= ~(1 << SUNXI_BP_TXCSR_D_UNDER_RUN);
    writew(ep_csr, base + SUNXI_TXCSR);
}

static s32 __USBC_Dev_WriteDataComplete(void __iomem *base, u32 ep_type)
{
    switch (ep_type) {
    case SUNXI_EP_TYPE_EP0:
        __USBC_Dev_ep0_WriteDataComplete(base);
        break;
    case SUNXI_EP_TYPE_TX:
        __USBC_Dev_Tx_WriteDataComplete(base);
        break;
    case SUNXI_EP_TYPE_RX:
        return -1;
    default:
        return -1;
    }

    return 0;
}

static void __USBC_Dev_ep0_WriteDataHalf(void __iomem *base)
{
    writew(1 << SUNXI_BP_CSR0_D_TX_PKT_READY, base + SUNXI_CSR0);
}

static void __USBC_Dev_Tx_WriteDataHalf(void __iomem *base)
{
    u16 ep_csr = 0;

    ep_csr = readw(base + SUNXI_TXCSR);
    ep_csr |= 1 << SUNXI_BP_TXCSR_D_TX_READY;
    ep_csr &= ~(1 << SUNXI_BP_TXCSR_D_UNDER_RUN);
    writew(ep_csr, base + SUNXI_TXCSR);
}

static s32 __USBC_Dev_WriteDataHalf(void __iomem *base, u32 ep_type)
{
    switch (ep_type) {
    case SUNXI_EP_TYPE_EP0:
        __USBC_Dev_ep0_WriteDataHalf(base);
        break;
    case SUNXI_EP_TYPE_TX:
        __USBC_Dev_Tx_WriteDataHalf(base);
        break;
    case SUNXI_EP_TYPE_RX:
        return -1;
    default:
        return -1;
    }
    return 0;
}

static s32 USBC_Dev_WriteDataStatus(void __iomem *base, u32 ep_type, u32 complete)
{
    if (complete) {
        return __USBC_Dev_WriteDataComplete(base, ep_type);
    } else {
        return __USBC_Dev_WriteDataHalf(base, ep_type);
    }
}

static u32 __USBC_Dev_ep0_IsEpStall(void __iomem *base)
{
    return readw(base + SUNXI_CSR0) & (1 << SUNXI_BP_CSR0_D_SENT_STALL);
}

static u32 __USBC_Dev_Tx_IsEpStall(void __iomem *base)
{
    return readw(base + SUNXI_TXCSR) & (1 << SUNXI_BP_TXCSR_D_SENT_STALL);
}

static u32 __USBC_Dev_Rx_IsEpStall(void __iomem *base)
{
    return readw(base + SUNXI_RXCSR) & (1 << SUNXI_BP_RXCSR_D_SENT_STALL);
}

static s32 USBC_Dev_IsEpStall(void __iomem *base, u32 ep_type)
{
    switch (ep_type) {
    case SUNXI_EP_TYPE_EP0:
        __USBC_Dev_ep0_IsEpStall(base);
        break;
    case SUNXI_EP_TYPE_TX:
        __USBC_Dev_Tx_IsEpStall(base);
        break;
    case SUNXI_EP_TYPE_RX:
        __USBC_Dev_Rx_IsEpStall(base);
        break;
    default:
        return -1;
    }
    return 0;
}

static u32 __USBC_Dev_ep0_IsReadDataReady(void __iomem *base)
{
    return readw(base + SUNXI_CSR0) & SUNXI_BP_CSR0_D_RX_PKT_READY;
}

static u32 __USBC_Dev_Rx_IsReadDataReady(void __iomem *base)
{
    return readw(base + SUNXI_RXCSR) & SUNXI_BP_RXCSR_D_RX_PKT_READY;
}

static u32 USBC_Dev_IsReadDataReady(void __iomem *base, u32 ep_type)
{
    switch (ep_type) {
    case SUNXI_EP_TYPE_EP0:
        return __USBC_Dev_ep0_IsReadDataReady(base);
    case SUNXI_EP_TYPE_TX:
        break;
    case SUNXI_EP_TYPE_RX:
        return __USBC_Dev_Rx_IsReadDataReady(base);
    default:
        break;
    }
    return 0;
}

static u32 __USBC_Dev_ep0_IsWriteDataReady(void __iomem *base)
{
    return readw(base + SUNXI_CSR0) & (1 << SUNXI_BP_CSR0_D_TX_PKT_READY);
}


static u32 __USBC_Dev_Tx_IsWriteDataReady(void __iomem *base)
{
    u32 temp = 0;

    temp = readw(base + SUNXI_TXCSR);
    temp &= (1 << SUNXI_BP_TXCSR_D_TX_READY) | (1 << SUNXI_BP_TXCSR_D_FIFO_NOT_EMPTY);

    return temp;
}

u32 USBC_Dev_IsWriteDataReady(void __iomem *base, u32 ep_type)
{
    switch (ep_type) {
    case SUNXI_EP_TYPE_EP0:
        return __USBC_Dev_ep0_IsWriteDataReady(base);
    case SUNXI_EP_TYPE_TX:
        return __USBC_Dev_Tx_IsWriteDataReady(base);
    case SUNXI_EP_TYPE_RX:
        break;
    default:
        break;
    }
    return 0;
}

static void __USBC_Dev_ep0_SendStall(void __iomem *base)
{
    setbits_le16(base + SUNXI_CSR0, 1 << SUNXI_BP_CSR0_D_SEND_STALL);
}

static void __USBC_Dev_Tx_SendStall(void __iomem *base)
{
    setbits_le16(base + SUNXI_TXCSR, 1 << SUNXI_BP_TXCSR_D_SEND_STALL);
}

static void __USBC_Dev_Rx_SendStall(void __iomem *base)
{
    setbits_le16(base + SUNXI_RXCSR, 1 << SUNXI_BP_RXCSR_D_SEND_STALL);
}

static s32 USBC_Dev_EpSendStall(void __iomem *base, u32 ep_type)
{
    switch (ep_type) {
    case SUNXI_EP_TYPE_EP0:
        __USBC_Dev_ep0_SendStall(base);
        break;
    case SUNXI_EP_TYPE_TX:
        __USBC_Dev_Tx_SendStall(base);
        break;
    case SUNXI_EP_TYPE_RX:
        __USBC_Dev_Rx_SendStall(base);
        break;
    default:
        return -1;
    }
    return 0;
}

static void __USBC_Dev_ep0_ClearStall(void __iomem *base)
{
    clrbits_le16(base + SUNXI_CSR0, (1 << SUNXI_BP_CSR0_D_SEND_STALL));
    clrbits_le16(base + SUNXI_CSR0, (1 << SUNXI_BP_CSR0_D_SENT_STALL));
}

static void __USBC_Dev_Tx_ClearStall(void __iomem *base)
{
    u32 reg_val;

    reg_val = readw(base + SUNXI_TXCSR);
    reg_val &= ~((1 << SUNXI_BP_TXCSR_D_SENT_STALL) | (1 << SUNXI_BP_TXCSR_D_SEND_STALL));
    writew(reg_val, base + SUNXI_TXCSR);
}

static void __USBC_Dev_Rx_ClearStall(void __iomem *base)
{
    clrbits_le16(base + SUNXI_RXCSR, (1 << SUNXI_BP_RXCSR_D_SEND_STALL));
    clrbits_le16(base + SUNXI_RXCSR, (1 << SUNXI_BP_RXCSR_D_SENT_STALL));
}

static s32 USBC_Dev_EpClearStall(void __iomem *base, u32 ep_type)
{
    switch (ep_type) {
    case SUNXI_EP_TYPE_EP0:
        __USBC_Dev_ep0_ClearStall(base);
        break;
    case SUNXI_EP_TYPE_TX:
        __USBC_Dev_Tx_ClearStall(base);
        break;
    case SUNXI_EP_TYPE_RX:
        __USBC_Dev_Rx_ClearStall(base);
        break;
    default:
        return -1;
    }
    return 0;
}

static u32 __USBC_Dev_ep0_IsSetupEnd(void __iomem *base)
{
        return readw(base + SUNXI_CSR0) & (1 << SUNXI_BP_CSR0_D_SETUP_END);
}

static u32 USBC_Dev_Ctrl_IsSetupEnd(void __iomem *base)
{
    return __USBC_Dev_ep0_IsSetupEnd(base);
}

static void __USBC_Dev_ep0_ClearSetupEnd(void __iomem *base)
{
    setbits_le16(base + SUNXI_CSR0, 1 << SUNXI_BP_CSR0_D_SERVICED_SETUP_END);
}

static void USBC_Dev_Ctrl_ClearSetupEnd(void __iomem *base)
{
    __USBC_Dev_ep0_ClearSetupEnd(base);
}

void USBC_Dev_SetAddress_default(void __iomem *base)
{
    writeb(0x00, base + SUNXI_FADDR);
}

static void USBC_Dev_SetAddress(void __iomem *base, u8 address)
{
    writeb(address, base + SUNXI_FADDR);
}

static void USBC_EnterMode_TestPacket(void __iomem *base)
{
    setbits_8(base + SUNXI_TMCTL, 1 << SUNXI_BP_TMCTL_TEST_PACKET);
}

static void USBC_INT_ClearMiscPending(void __iomem *base, u32 mask)
{
    writeb(mask, base + SUNXI_INTUSB);
}

static void __USBC_INT_ClearTxPendingAll(void __iomem *base)
{
    writew(0xffff, base + SUNXI_INTTx);
}

static void __USBC_INT_ClearRxPendingAll(void __iomem *base)
{
    writew(0xffff, base + SUNXI_INTRx);
}

static void USBC_INT_ClearEpPendingAll(void __iomem *base, u32 ep_type)
{
    switch (ep_type) {
    case SUNXI_EP_TYPE_EP0:
    case SUNXI_EP_TYPE_TX:
        __USBC_INT_ClearTxPendingAll(base);
        break;
    case SUNXI_EP_TYPE_RX:
        __USBC_INT_ClearRxPendingAll(base);
        break;
    default:
        break;
    }
}

static void USBC_INT_ClearMiscPendingAll(void __iomem *base)
{
    writeb(0xff, base + SUNXI_INTUSB);
}

static u32 __USBC_INT_TxPending(void __iomem *base)
{
    return readw(base + SUNXI_INTTx);
}

static u32 __USBC_INT_RxPending(void __iomem *base)
{
    return readw(base + SUNXI_INTRx);
}

static u32 USBC_INT_EpPending(void __iomem *base, u32 ep_type)
{
    switch(ep_type){
    case SUNXI_EP_TYPE_EP0:
    case SUNXI_EP_TYPE_TX:
        return __USBC_INT_TxPending(base);        
    case SUNXI_EP_TYPE_RX:
        return __USBC_INT_RxPending(base);
    default:
        return 0;
    }
}

static u32 USBC_INT_MiscPending(void __iomem *base)
{
    return readb(base + SUNXI_INTUSB);
}

static void __USBC_INT_ClearTxPending(void __iomem *base, u8 ep_index)
{
    writew((1 << ep_index), base + SUNXI_INTTx);
}

static void __USBC_INT_ClearRxPending(void __iomem *base, u8 ep_index)
{
    writew((1 << ep_index), base + SUNXI_INTRx);
}

static void USBC_INT_ClearEpPending(void __iomem *base, u32 ep_type, u8 ep_index)
{
    switch(ep_type) {
    case SUNXI_EP_TYPE_EP0:
    case SUNXI_EP_TYPE_TX:
        __USBC_INT_ClearTxPending(base, ep_index);
        break;
    case SUNXI_EP_TYPE_RX:
        __USBC_INT_ClearRxPending(base, ep_index);
        break;   
    default:
        break;
    }
    return;
}

static u32 USBC_Dev_QueryTransferMode(void __iomem *base)
{
    if (readb(base + SUNXI_PCTL) & (1 << SUNXI_BP_POWER_D_HIGH_SPEED_FLAG))
        return SUNXI_TS_MODE_HS;
    else
        return SUNXI_TS_MODE_FS;
}

static void __USBC_Dev_ep0_ConfigEp0(void __iomem *base)
{
    writew(1 << SUNXI_BP_CSR0_D_FLUSH_FIFO, base + SUNXI_CSR0);
}

static void __USBC_Dev_Tx_EnableIsoEp(void __iomem *base)
{
    setbits_le16(base + SUNXI_TXCSR, 1 << SUNXI_BP_TXCSR_D_ISO);
}

static void __USBC_Dev_Tx_EnableIntEp(void __iomem *base)
{
    clrbits_le16(base + SUNXI_TXCSR, 1 << SUNXI_BP_TXCSR_D_ISO);
}

static void __USBC_Dev_Tx_EnableBulkEp(void __iomem *base)
{
    clrbits_le16(base + SUNXI_TXCSR, 1 << SUNXI_BP_TXCSR_D_ISO);
}

static void __USBC_Dev_Tx_ConfigEp(void __iomem *base, u32 ts_type, u32 is_double_fifo, u32 ep_MaxPkt)
{
    u16 reg_val = 0;
    u16 temp = 0;

    reg_val = (1 << SUNXI_BP_TXCSR_D_MODE);
    reg_val |= (1 << SUNXI_BP_TXCSR_D_CLEAR_DATA_TOGGLE);
    reg_val |= (1 << SUNXI_BP_TXCSR_D_FLUSH_FIFO);
    writew(reg_val, base + SUNXI_TXCSR);

    if (is_double_fifo) {
        writew(reg_val, base + SUNXI_TXCSR);
    }

    reg_val = readw(base + SUNXI_TXMAXP);
    temp    = ep_MaxPkt & ((1 << SUNXI_BP_TXMAXP_PACKET_COUNT) - 1);
    reg_val |= temp;
    writew(reg_val, base + SUNXI_TXMAXP);

    switch (ts_type) {
    case SUNXI_TS_TYPE_ISO:
        __USBC_Dev_Tx_EnableIsoEp(base);
        break;
    case SUNXI_TS_TYPE_INT:
        __USBC_Dev_Tx_EnableIntEp(base);
        break;
    case SUNXI_TS_TYPE_BULK:
        __USBC_Dev_Tx_EnableBulkEp(base);
        break;
    default:
        __USBC_Dev_Tx_EnableBulkEp(base);
    }
}

static void __USBC_Dev_Rx_EnableIsoEp(void __iomem *base)
{
    setbits_le16(base + SUNXI_RXCSR, 1 << SUNXI_BP_RXCSR_D_ISO);
}

static void __USBC_Dev_Rx_EnableIntEp(void __iomem *base)
{
    clrbits_le16(base + SUNXI_RXCSR, 1 << SUNXI_BP_RXCSR_D_ISO);
}

static void __USBC_Dev_Rx_EnableBulkEp(void __iomem *base)
{
    clrbits_le16(base + SUNXI_RXCSR, 1 << SUNXI_BP_RXCSR_D_ISO);
}

static void __USBC_Dev_Rx_ConfigEp(void __iomem *base, u32 ts_type, u32 is_double_fifo, u32 ep_MaxPkt)
{
    u16 reg_val = 0;
    u16 temp = 0;

    writew((1 << SUNXI_BP_RXCSR_D_CLEAR_DATA_TOGGLE) | (1 << SUNXI_BP_RXCSR_D_FLUSH_FIFO),
            base + SUNXI_RXCSR);

    if (is_double_fifo) {
        writew((1 << SUNXI_BP_RXCSR_D_CLEAR_DATA_TOGGLE) | (1 << SUNXI_BP_RXCSR_D_FLUSH_FIFO),
                base + SUNXI_RXCSR);
    }

    reg_val = readw(base + SUNXI_RXMAXP);
    temp    = ep_MaxPkt & ((1 << SUNXI_BP_RXMAXP_PACKET_COUNT) - 1);
    reg_val |= temp;
    writew(reg_val, base + SUNXI_RXMAXP);

    switch (ts_type) {
    case SUNXI_TS_TYPE_ISO:
        __USBC_Dev_Rx_EnableIsoEp(base);
        break;
    case SUNXI_TS_TYPE_INT:
        __USBC_Dev_Rx_EnableIntEp(base);
        break;
    case SUNXI_TS_TYPE_BULK:
        __USBC_Dev_Rx_EnableBulkEp(base);
        break;
    default:
        __USBC_Dev_Rx_EnableBulkEp(base);
    }
}

static s32 USBC_Dev_ConfigEp(void __iomem *base, u32 ts_type, u32 ep_type, u32 is_double_fifo, u32 ep_MaxPkt)
{
    switch (ep_type) {
    case SUNXI_EP_TYPE_EP0:
        __USBC_Dev_ep0_ConfigEp0(base);
        break;
    case SUNXI_EP_TYPE_TX:
        __USBC_Dev_Tx_ConfigEp(base, ts_type, is_double_fifo,  ep_MaxPkt);
        break;
    case SUNXI_EP_TYPE_RX:
        __USBC_Dev_Rx_ConfigEp(base, ts_type, is_double_fifo, ep_MaxPkt);
        break;
    default:
        return -1;
    }
    return 0;
}

static void __USBC_ConfigFifo_TxEp(void __iomem *base, u32 is_double_fifo, u32 fifo_size, u32 fifo_addr)
{
    u32 temp = 0;
    u32 size = 0;
    u32 addr = 0;

    temp = fifo_size + 511;
    temp &= ~511;
    temp >>= 3;
    temp >>= 1;
    while (temp) {
        size++;
        temp >>= 1;
    }

    addr = fifo_addr >> 3;

    writew(addr, base + SUNXI_TXFIFOAD);

    writeb((size & 0x0f), base + SUNXI_TXFIFOSZ);
    if (is_double_fifo) {
        setbits_8(base + SUNXI_TXFIFOSZ, 1 << SUNXI_BP_TXFIFOSZ_DPB);
    }
}

static void __USBC_ConfigFifo_RxEp(void __iomem *base, u32 is_double_fifo, u32 fifo_size, u32 fifo_addr)
{
    u32 temp = 0;
    u32 size = 0;
    u32 addr = 0;

    temp = fifo_size + 511;
    temp &= ~511;
    temp >>= 3;
    temp >>= 1;
    while (temp) {
        size++;
        temp >>= 1;
    }

    addr = fifo_addr >> 3;

    writew(addr, base + SUNXI_RXFIFOAD);

    writeb((size & 0x0f), base + SUNXI_RXFIFOSZ);
    if (is_double_fifo) {
        setbits_8(base + SUNXI_RXFIFOSZ, 1 << SUNXI_BP_RXFIFOSZ_DPB);
    }
}

static void USBC_ConfigFifo(void __iomem *base, u32 ep_type, u32 is_double_fifo, u32 fifo_size, u32 fifo_addr)
{
    switch (ep_type) {
    case SUNXI_EP_TYPE_EP0:
        break;
    case SUNXI_EP_TYPE_TX:
        __USBC_ConfigFifo_TxEp(base, is_double_fifo, fifo_size, fifo_addr);
        break;
    case SUNXI_EP_TYPE_RX:
        __USBC_ConfigFifo_RxEp(base, is_double_fifo, fifo_size, fifo_addr);
        break;
    default:
        break;
    }
}

static void __USBC_INT_EnableTxEp(void __iomem *base, u8 ep_index)
{
    setbits_le16(base + SUNXI_INTTxE, ep_index);
}

static void __USBC_INT_EnableRxEp(void __iomem *base, u8 ep_index)
{
    setbits_le16(base + SUNXI_INTRxE, ep_index);
}

static void USBC_INT_EnableEp(void __iomem *base, u32 ep_type, u8 ep_index)
{
    switch (ep_type) {
    case SUNXI_EP_TYPE_TX:
        __USBC_INT_EnableTxEp(base, ep_index);
        break;
    case SUNXI_EP_TYPE_RX:
        __USBC_INT_EnableRxEp(base, ep_index);
        break;
    default:
        break;
    }
    return;
}

static void __USBC_Dev_ep0_ConfigEp0_Default(void __iomem *base)
{
    writew(1 << SUNXI_BP_CSR0_D_FLUSH_FIFO, base + SUNXI_CSR0);
}

static void __USBC_Dev_Tx_ConfigEp_Default(void __iomem *base)
{
    writew(0x00, base + SUNXI_TXCSR);
    writew(0x00, base + SUNXI_TXMAXP);
}

static void __USBC_Dev_Rx_ConfigEp_Default(void __iomem *base)
{
    writew(0x00, base + SUNXI_RXCSR);
    writew(0x00, base + SUNXI_RXMAXP);
}

static s32 USBC_Dev_ConfigEp_Default(void __iomem *base, u32 ep_type)
{
    switch (ep_type) {
    case SUNXI_EP_TYPE_EP0:
        __USBC_Dev_ep0_ConfigEp0_Default(base);
        break;
    case SUNXI_EP_TYPE_TX:
        __USBC_Dev_Tx_ConfigEp_Default(base);
        break;
    case SUNXI_EP_TYPE_RX:
        __USBC_Dev_Rx_ConfigEp_Default(base);
        break;
    default:
        return -1;
    }
    return 0;
}

static void __USBC_INT_DisableTxEp(void __iomem *base, u8 ep_index)
{
    clrbits_le16(base + SUNXI_INTTxE, 1 << ep_index);
}

static void __USBC_INT_DisableRxEp(void __iomem *base, u8 ep_index)
{
    clrbits_le16(base + SUNXI_INTRxE, 1 << ep_index);
}

void USBC_INT_DisableEp(void __iomem *base, u32 ep_type, u8 ep_index)
{
    switch (ep_type) {
    case SUNXI_EP_TYPE_TX:
        __USBC_INT_DisableTxEp(base, ep_index);
        break;
    case SUNXI_EP_TYPE_RX:
        __USBC_INT_DisableRxEp(base, ep_index);
        break;
    default:
        break;
    }
    return;
}

static void __USBC_Dev_TsType_default(void __iomem *base)
{
    clrbits_8(base + SUNXI_PCTL, 1 << SUNXI_BP_POWER_D_ISO_UPDATE_EN);
}

static void __USBC_Dev_TsType_Ctrl(void __iomem *base)
{
    clrbits_8(base + SUNXI_PCTL, 1 << SUNXI_BP_POWER_D_ISO_UPDATE_EN);
}

static void __USBC_Dev_TsType_Iso(void __iomem *base)
{
    setbits_8(base + SUNXI_PCTL, 1 << SUNXI_BP_POWER_D_ISO_UPDATE_EN);
}

static void __USBC_Dev_TsType_Int(void __iomem *base)
{
    clrbits_8(base + SUNXI_PCTL, 1 << SUNXI_BP_POWER_D_ISO_UPDATE_EN);
}

static void __USBC_Dev_TsType_Bulk(void __iomem *base)
{
    clrbits_8(base + SUNXI_PCTL, 1 << SUNXI_BP_POWER_D_ISO_UPDATE_EN);
}

static void __USBC_Dev_TsMode_default(void __iomem *base)
{
    clrbits_8(base + SUNXI_PCTL, 1 << SUNXI_BP_POWER_D_HIGH_SPEED_EN);
}

static void __USBC_Dev_TsMode_Hs(void __iomem *base)
{
    setbits_8(base + SUNXI_PCTL, 1 << SUNXI_BP_POWER_D_HIGH_SPEED_EN);
}

static void __USBC_Dev_TsMode_Fs(void __iomem *base)
{
    clrbits_8(base + SUNXI_PCTL, 1 << SUNXI_BP_POWER_D_HIGH_SPEED_EN);
}

static void __USBC_Dev_TsMode_Ls(void __iomem *base)
{
    __USBC_Dev_TsMode_Fs(base);
}

void USBC_Dev_ConfigTransferMode(void __iomem *base, u8 ts_type, u8 speed_mode)
{
    switch (ts_type) {
    case SUNXI_TS_TYPE_CTRL:
        __USBC_Dev_TsType_Ctrl(base);
        break;
    case SUNXI_TS_TYPE_ISO:
        __USBC_Dev_TsType_Iso(base);
        break;
    case SUNXI_TS_TYPE_INT:
        __USBC_Dev_TsType_Int(base);
        break;
    case SUNXI_TS_TYPE_BULK:
        __USBC_Dev_TsType_Bulk(base);
        break;
    default:
        __USBC_Dev_TsType_default(base);
    }

    switch (speed_mode) {
    case SUNXI_TS_MODE_HS:
        __USBC_Dev_TsMode_Hs(base);
        break;
    case SUNXI_TS_MODE_FS:
        __USBC_Dev_TsMode_Fs(base);
        break;
    case SUNXI_TS_MODE_LS:
        __USBC_Dev_TsMode_Ls(base);
        break;
    default:
        __USBC_Dev_TsMode_default(base);
    }
}

static void USBC_INT_EnableUsbMiscUint(void __iomem *base, u32 mask)
{
    u32 reg_val = 0;

    reg_val = readb(base + SUNXI_INTUSBE);
    reg_val |= mask;
    writeb(reg_val, SUNXI_INTUSBE);
}

static void USBC_INT_DisableUsbMiscAll(void __iomem *base)
{
    writeb(0, base + SUNXI_INTUSBE);
}

static void __USBC_INT_DisableTxAll(void __iomem *base)
{
    writew(0, base + SUNXI_INTTxE);
}

static void __USBC_INT_DisableRxAll(void __iomem *base)
{
    writew(0, base + SUNXI_INTRxE);
}

static void USBC_INT_DisableEpAll(void __iomem *base, u32 ep_type)
{
    switch (ep_type) {
    case SUNXI_EP_TYPE_TX:
        __USBC_INT_DisableTxAll(base);
        break;
    case SUNXI_EP_TYPE_RX:
        __USBC_INT_DisableRxAll(base);
        break;
    default:
        break;
    }
    return;
}

static void USBC_Dev_ConectSwitch(void __iomem *base, u32 is_on)
{
    if (is_on == SUNXI_DEVICE_SWITCH_ON) {
        setbits_8(base + SUNXI_PCTL, 1 << SUNXI_BP_POWER_D_SOFT_CONNECT);
    } else {
        clrbits_8(base + SUNXI_PCTL, 1 << SUNXI_BP_POWER_D_SOFT_CONNECT);
    }
}

static void cfg_udc_command(enum sw_udc_cmd_e cmd);

static u32 is_peripheral_active(void)
{
	return is_controller_alive;
}

static void sw_udc_done(struct sw_udc_ep *ep,
		struct sw_udc_request *req, int status)
{
	unsigned halted = ep->halted;

	debug("d: ep(0x%p, %d), req(0x%p, 0x%p, %d, %d)\n\n",
			ep, ep->num,
			req, &(req->req), req->req.length, req->req.actual);

	list_del_init(&req->queue);

	if (likely (req->req.status == -EINPROGRESS))
		req->req.status = status;
	else
		status = req->req.status;

	ep->halted = 1;
	spin_unlock(&ep->dev->lock);
	req->req.complete(&ep->ep, &req->req);
	spin_lock(&ep->dev->lock);
	ep->halted = halted;
}

static void sw_udc_nuke(struct sw_udc *udc, struct sw_udc_ep *ep, int status)
{
	/* Sanity check */
	if (&ep->queue == NULL)
		return;

	while (!list_empty (&ep->queue)) {
		struct sw_udc_request *req;
		req = list_entry (ep->queue.next, struct sw_udc_request,
				queue);
		debug("nuke: ep num is %d\n", ep->num);
		sw_udc_done(ep, req, status);
	}
}

static inline int sw_udc_fifo_count_out(u8 ep_index)
{
	if (ep_index) {
		return USBC_ReadLenFromFifo(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_RX);
	} else {
		return USBC_ReadLenFromFifo(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_EP0);
	}
}

static inline int sw_udc_write_packet(int fifo,
		struct sw_udc_request *req,
		unsigned max)
{
	unsigned len = min(req->req.length - req->req.actual, max);
	u8 *buf = req->req.buf + req->req.actual;

	prefetch(buf);

	debug("W: req.actual(%d), req.length(%d), len(%d), total(%d)\n",
			req->req.actual, req->req.length, len, req->req.actual + len);

	req->req.actual += len;

	udelay(5);
	USBC_WritePacket(g_sw_udc_io.usb_vbase, fifo, len, buf);

	return len;
}

static int sw_udc_write_fifo(struct sw_udc_ep *ep, struct sw_udc_request *req)
{
	unsigned count = 0;
	int is_last = 0;
	u32 idx = 0;
	int fifo_reg = 0;
	s32 ret = 0;
	u8 old_ep_index = 0;

	idx = ep->bEndpointAddress & 0x7F;

	/* select ep */
	old_ep_index = USBC_GetActiveEp(g_sw_udc_io.usb_vbase);
	USBC_SelectActiveEp(g_sw_udc_io.usb_vbase, idx);

	/* select fifo */
	fifo_reg = USBC_SelectFIFO(g_sw_udc_io.usb_vbase, idx);

	count = sw_udc_write_packet(fifo_reg, req, ep->ep.maxpacket);

	/* last packet is often short (sometimes a zlp) */
	if (count != ep->ep.maxpacket) {
		is_last = 1;
	} else if (req->req.length != req->req.actual || req->req.zero) {
		is_last = 0;
	} else {
		is_last = 2;
	}

	debug("pw: ep(0x%p, %d), req(0x%p, 0x%p, %d, %d), cnt(%d, %d)\n",
			ep, ep->num,
			req, &(req->req), req->req.length, req->req.actual,
			count, is_last);

	if (idx) { // ep1~4
		ret = USBC_Dev_WriteDataStatus(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_TX, is_last);
        if (ret != 0) {
            printf("ERR: USBC_Dev_WriteDataStatus, failed\n");
            req->req.status = -EOVERFLOW;
        }
	} else { // ep0
		ret = USBC_Dev_WriteDataStatus(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_EP0, is_last);
        if (ret != 0) {
            printf("ERR: USBC_Dev_WriteDataStatus, failed\n");
            req->req.status = -EOVERFLOW;
        }
	}

	USBC_SelectActiveEp(g_sw_udc_io.usb_vbase, old_ep_index);

	if (is_last) {
		if (!idx) {  /* ep0 */
			ep->dev->ep0state = EP0_IDLE;
			sw_udc_done(ep, req, 0);
		}

		is_last = 1;
	}

	return is_last;
}

static inline int sw_udc_read_packet(int fifo, u8 *buf,
		struct sw_udc_request *req, unsigned avail)
{
	unsigned len = 0;

	len = min(req->req.length - req->req.actual, avail);
	req->req.actual += len;

	debug("R: req.actual(%d), req.length(%d), len(%d), total(%d)\n",
			req->req.actual, req->req.length, len, req->req.actual + len);

	USBC_ReadPacket(g_sw_udc_io.usb_vbase, fifo, len, buf);

	return len;
}

static int sw_udc_read_fifo_crq(struct usb_ctrlrequest *crq)
{
	u32 fifo_count  = 0;
	u32 i = 0;
	u8  *pOut = (u8 *) crq;
	u32 fifo = 0;

	fifo = USBC_SelectFIFO(g_sw_udc_io.usb_vbase, 0);
	fifo_count = USBC_ReadLenFromFifo(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_EP0);

	if (fifo_count != 8) {
		i = 0;

		while (i < 16 && (fifo_count != 8)) {
			fifo_count = USBC_ReadLenFromFifo(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_EP0);
			i++;
		}

		if (i >= 16) {
			printf("ERR: get ep0 fifo len failed\n");
		}
	}

	return USBC_ReadPacket(g_sw_udc_io.usb_vbase, fifo, fifo_count, pOut);
}

static int sw_udc_read_fifo(struct sw_udc_ep *ep, struct sw_udc_request *req)
{
	u8 *buf = NULL;
	unsigned bufferspace = 0;
	int is_last = 1;
	unsigned avail = 0;
	int fifo_count = 0;
	u32 idx = 0;
	int fifo_reg = 0;
	s32 ret = 0;
	u8 old_ep_index = 0;

	idx = ep->bEndpointAddress & 0x7F;

	/* select fifo */
	fifo_reg = USBC_SelectFIFO(g_sw_udc_io.usb_vbase, idx);

	if (!req->req.length) {
		printf("ERR: req->req.length == 0\n");
		return 1;
	}

	buf = req->req.buf + req->req.actual;
	bufferspace = req->req.length - req->req.actual;
	if (!bufferspace) {
		printf("ERR: buffer full!\n");
		return -1;
	}

	/* select ep */
	old_ep_index = USBC_GetActiveEp(g_sw_udc_io.usb_vbase);
	USBC_SelectActiveEp(g_sw_udc_io.usb_vbase, idx);

	fifo_count = sw_udc_fifo_count_out(idx);
	if (fifo_count > ep->ep.maxpacket) {
		avail = ep->ep.maxpacket;
	} else {
		avail = fifo_count;
	}

	fifo_count = sw_udc_read_packet(fifo_reg, buf, req, avail);

	/* checking this with ep0 is not accurate as we already
	 * read a control request
	 */
	if (idx != 0 && fifo_count < ep->ep.maxpacket) {
		is_last = 1;
		/* overflowed this request?  flush extra data */
		if (fifo_count != avail)
			req->req.status = -EOVERFLOW;
	} else {
		is_last = (req->req.length <= req->req.actual) ? 1 : 0;
	}

	debug("pr: ep(0x%p, %d), req(0x%p, 0x%p, %d, %d), cnt(%d, %d)\n",
			ep, ep->num,
			req, &(req->req), req->req.length, req->req.actual,
			fifo_count, is_last);

	if (idx) {
		ret = USBC_Dev_ReadDataStatus(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_RX, is_last);
        if (ret != 0){
            printf("ERR: sw_udc_read_fifo: USBC_Dev_WriteDataStatus, failed\n");
            req->req.status = -EOVERFLOW;
        }
	} else {
		ret = USBC_Dev_ReadDataStatus(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_EP0, is_last);
        if (ret != 0){
            printf("ERR: sw_udc_read_fifo: USBC_Dev_WriteDataStatus, failed\n");
            req->req.status = -EOVERFLOW;
        }
	}

	USBC_SelectActiveEp(g_sw_udc_io.usb_vbase, old_ep_index);

	if (is_last) {
		if (!idx) {
			ep->dev->ep0state = EP0_IDLE;
		}

		sw_udc_done(ep, req, 0);
		is_last = 1;
	}

	return is_last;
}

static int sw_udc_get_status(struct sw_udc *dev, struct usb_ctrlrequest *crq)
{
	u16 status = 0;
	u8 buf[8];
	u8 ep_num = crq->wIndex & 0x7F;
	u8 is_in = crq->wIndex & USB_DIR_IN;
	u32 fifo = 0;

	switch (crq->bRequestType & USB_RECIP_MASK) {
    case USB_RECIP_INTERFACE:
        buf[0] = 0x00;
        buf[1] = 0x00;
        break;

    case USB_RECIP_DEVICE:
        status = dev->devstatus;
        buf[0] = 0x01;
        buf[1] = 0x00;
        break;
        
    case USB_RECIP_ENDPOINT:
        if (ep_num > 4 || crq->wLength > 2) {
            return 1;
        }
        
        USBC_SelectActiveEp(g_sw_udc_io.usb_vbase, ep_num);
        if (ep_num == 0) {
            status = USBC_Dev_IsEpStall(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_EP0);
        } else {
            if (is_in) {
                status = USBC_Dev_IsEpStall(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_TX);
            } else {
                status = USBC_Dev_IsEpStall(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_RX);
            }
        }
        status = status ? 1 : 0;
        if (status) {
            buf[0] = 0x01;
            buf[1] = 0x00;
        } else {
            buf[0] = 0x00;
            buf[1] = 0x00;
        }
        break;

    default:
        return 1;
	}

	/* Seems to be needed to get it working. ouch :( */
	udelay(5);
	USBC_Dev_ReadDataStatus(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_EP0, 0);

	fifo = USBC_SelectFIFO(g_sw_udc_io.usb_vbase, 0);
	USBC_WritePacket(g_sw_udc_io.usb_vbase, fifo, crq->wLength, buf);
	USBC_Dev_WriteDataStatus(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_EP0, 1);

	return 0;
}

static int sw_udc_set_halt(struct usb_ep *_ep, int value);

static void sw_udc_handle_ep0_idle(struct sw_udc *dev,
		struct sw_udc_ep *ep,
		struct usb_ctrlrequest *crq,
		u32 ep0csr)
{
	int len = 0, ret = 0, tmp = 0;

	/* start control request? */
	if (!USBC_Dev_IsReadDataReady(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_EP0)) {
		debug("ERR: data is ready, can not read data.\n");
		return;
	}

	sw_udc_nuke(dev, ep, -EPROTO);

	len = sw_udc_read_fifo_crq(crq);
	if (len != sizeof(*crq)) {
		printf("setup begin: fifo READ ERROR"
				" wanted %d bytes got %d. Stalling out...\n",
				sizeof(*crq), len);

		USBC_Dev_ReadDataStatus(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_EP0, 0);
		USBC_Dev_EpSendStall(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_EP0);

		return;
	}

	debug("ep0: bRequest = %d bRequestType %d wLength = %d\n",
			crq->bRequest, crq->bRequestType, crq->wLength);

	/* cope with automagic for some standard requests. */
	dev->req_std        = ((crq->bRequestType & USB_TYPE_MASK) == USB_TYPE_STANDARD);
	dev->req_config     = 0;
	dev->req_pending    = 1;

	if (dev->req_std) {   //standard request
		switch (crq->bRequest) {
		case USB_REQ_SET_CONFIGURATION:
            debug("USB_REQ_SET_CONFIGURATION ... \n");
            
            if (crq->bRequestType == USB_RECIP_DEVICE) {
                dev->req_config = 1;
            }
            break;
            
        case USB_REQ_SET_INTERFACE:
            debug("USB_REQ_SET_INTERFACE ... \n");
            
            if (crq->bRequestType == USB_RECIP_INTERFACE) {
                dev->req_config = 1;
            }
            break;
            
        case USB_REQ_SET_ADDRESS:
            debug("USB_REQ_SET_ADDRESS ... \n");
            
            if (crq->bRequestType == USB_RECIP_DEVICE) {
                tmp = crq->wValue & 0x7F;
                dev->address = tmp;
                
                USBC_Dev_ReadDataStatus(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_EP0, 1);
                dev->ep0state = EP0_END_XFER;
                crq_bRequest = USB_REQ_SET_ADDRESS;
                
                return;
            }
            break;
            
        case USB_REQ_GET_STATUS:
            debug("USB_REQ_GET_STATUS ... \n");
            
            if (!sw_udc_get_status(dev, crq)) {
                return;
            }
            break;
            
        case USB_REQ_CLEAR_FEATURE:
            if (crq->bRequestType & (1 << 7)) {
                printf("USB_REQ_CLEAR_FEATURE: data is not host to device\n");
                break;
            }
            
            USBC_Dev_ReadDataStatus(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_EP0, 1);
				
            if (crq->bRequestType == USB_RECIP_DEVICE) {
                /* wValue 0-1 */
                if (crq->wValue) {
                    dev->devstatus &= ~(1 << USB_DEVICE_REMOTE_WAKEUP);
                } else {
                    int k = 0;
                    for (k = 0; k < SW_UDC_ENDPOINTS; k++) {
                        sw_udc_set_halt(&dev->ep[k].ep, 0);
                    }
                }                
            } else if(crq->bRequestType == USB_RECIP_INTERFACE) {
            } else if(crq->bRequestType == USB_RECIP_ENDPOINT) {
                /* wValue 0-1 */
                if (crq->wValue) {
                    dev->devstatus &= ~(1 << USB_DEVICE_REMOTE_WAKEUP);
                } else {
                    sw_udc_set_halt(&dev->ep[crq->wIndex & 0x7f].ep, 0);
                }                
            } else {
                printf("PANIC : nonsupport set feature request. (%d)\n", crq->bRequestType);
                USBC_Dev_EpSendStall(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_EP0);
            }
            
            dev->ep0state = EP0_IDLE;
           
            return;
        case USB_REQ_SET_FEATURE:
            if (crq->bRequestType & (1 << 7)) {
                debug("USB_REQ_SET_FEATURE: data is not host to device\n");
                break;
            }

            if (crq->bRequestType == USB_RECIP_DEVICE) {
                if ((crq->wValue == USB_DEVICE_TEST_MODE) && (crq->wIndex == 0x0400)) {
                    USBC_Dev_ReadDataStatus(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_EP0, 1);
                    
                    dev->ep0state = EP0_END_XFER;
                    crq_bRequest = USB_REQ_SET_FEATURE;
                    
                    return;
                }

				USBC_Dev_ReadDataStatus(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_EP0, 1);
                dev->devstatus |= (1 << USB_DEVICE_REMOTE_WAKEUP);
                
            } else if (crq->bRequestType == USB_RECIP_INTERFACE) {
                USBC_Dev_ReadDataStatus(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_EP0, 1);
            } else if (crq->bRequestType == USB_RECIP_ENDPOINT) {
                USBC_Dev_ReadDataStatus(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_EP0, 1);
                sw_udc_set_halt(&dev->ep[crq->wIndex & 0x7f].ep, 1);
            } else {
                printf("PANIC : nonsupport set feature request. (%d)\n", crq->bRequestType);
                USBC_Dev_ReadDataStatus(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_EP0, 1);
                USBC_Dev_EpSendStall(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_EP0);
            }
            
            dev->ep0state = EP0_IDLE;
            
            return;
            
        default:
            USBC_Dev_ReadDataStatus(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_EP0, 0);
            break;
        }
    } else {
        USBC_Dev_ReadDataStatus(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_EP0, 0);
	}

	if (crq->bRequestType & USB_DIR_IN) {
		dev->ep0state = EP0_IN_DATA_PHASE;
	} else {
		dev->ep0state = EP0_OUT_DATA_PHASE;
	}

	spin_unlock(&dev->lock);
	ret = dev->driver->setup(&dev->gadget, crq);
	spin_lock(&dev->lock);
	if (ret < 0) {
		if (dev->req_config) {
			printf("ERR: config change %02x fail %d?\n", crq->bRequest, ret);
			return;
		}

		if (ret == -EOPNOTSUPP) {
			printf("ERR: Operation not supported\n");
		} else {
			printf("ERR: dev->driver->setup failed. (%d)\n", ret);
		}

		udelay(5);

		USBC_Dev_ReadDataStatus(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_EP0, 1);
		USBC_Dev_EpSendStall(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_EP0);

		dev->ep0state = EP0_IDLE;
		/* deferred i/o == no response yet */
	} else if (dev->req_pending) {
		dev->req_pending = 0;
	}

	if (crq->bRequest == USB_REQ_SET_CONFIGURATION || crq->bRequest == USB_REQ_SET_INTERFACE) {
		USBC_Dev_ReadDataStatus(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_EP0, 1);
	}

	return;
}

static void sw_udc_handle_ep0(struct sw_udc *dev)
{
	u32 ep0csr = 0;
	struct sw_udc_ep *ep = &dev->ep[0];
	struct sw_udc_request *req = NULL;
	struct usb_ctrlrequest crq;

	debug("sw_udc_handle_ep0--1--\n");

	if (list_empty(&ep->queue)) {
		req = NULL;
	} else {
		req = list_entry(ep->queue.next, struct sw_udc_request, queue);
	}

	debug("sw_udc_handle_ep0--2--\n");

	/* We make the assumption that sw_udc_UDC_IN_CSR1_REG equal to
	 * sw_udc_UDC_EP0_CSR_REG when index is zero */
	USBC_SelectActiveEp(g_sw_udc_io.usb_vbase, 0);

	/* clear stall status */
	if (USBC_Dev_IsEpStall(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_EP0)) {
		printf("ERR: ep0 stall\n");

		sw_udc_nuke(dev, ep, -EPIPE);
		USBC_Dev_EpClearStall(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_EP0);
		dev->ep0state = EP0_IDLE;

		return;
	}

	/* clear setup end */
	if (USBC_Dev_Ctrl_IsSetupEnd(g_sw_udc_io.usb_vbase)) {
		debug("handle_ep0: ep0 setup end\n");

		sw_udc_nuke(dev, ep, 0);
		USBC_Dev_Ctrl_ClearSetupEnd(g_sw_udc_io.usb_vbase);
		dev->ep0state = EP0_IDLE;
	}

	debug("sw_udc_handle_ep0--3--%d\n", dev->ep0state);

	ep0csr = readw(g_sw_udc_io.usb_vbase + SUNXI_RXCSR);

	switch (dev->ep0state) {
    case EP0_IDLE:
        sw_udc_handle_ep0_idle(dev, ep, &crq, ep0csr);
        break;
        
    case EP0_IN_DATA_PHASE:                 /* GET_DESCRIPTOR etc */
        debug("EP0_IN_DATA_PHASE ... what now?\n");
        
        if (!USBC_Dev_IsWriteDataReady(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_EP0) && req) {
            sw_udc_write_fifo(ep, req);
        }
        break;
        
    case EP0_OUT_DATA_PHASE:                /* SET_DESCRIPTOR etc */
        debug("EP0_OUT_DATA_PHASE ... what now?\n");
        
        if (USBC_Dev_IsReadDataReady(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_EP0) && req ) {
            sw_udc_read_fifo(ep,req);
        }
        break;

    case EP0_END_XFER:
        debug("EP0_END_XFER ... what now?\n");
        debug("crq_bRequest = 0x%x\n", crq_bRequest);
        switch (crq_bRequest) {
        case USB_REQ_SET_ADDRESS:
            USBC_SelectActiveEp(g_sw_udc_io.usb_vbase, 0);
            
            USBC_Dev_Ctrl_ClearSetupEnd(g_sw_udc_io.usb_vbase);
            USBC_Dev_SetAddress(g_sw_udc_io.usb_vbase, dev->address);
            
            debug("Set address %d\n", dev->address);
            break;
            
        case USB_REQ_SET_FEATURE:
            {
                u32 fifo = 0;

                fifo = USBC_SelectFIFO(g_sw_udc_io.usb_vbase, 0);
                USBC_WritePacket(g_sw_udc_io.usb_vbase, fifo, 54, (u32 *)TestPkt);
                USBC_EnterMode_TestPacket(g_sw_udc_io.usb_vbase);
                USBC_Dev_WriteDataStatus(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_EP0, 1);
            }
            break;
        default:
            break;
        }

        crq_bRequest = 0;
        
        dev->ep0state = EP0_IDLE;
        break;
        
    case EP0_STALL:
        debug("EP0_STALL ... what now?\n");
        dev->ep0state = EP0_IDLE;
        break;
	}

	debug("sw_udc_handle_ep0--4--%d\n", dev->ep0state);

	return;
}

static void sw_udc_handle_ep(struct sw_udc_ep *ep)
{
	struct sw_udc_request *req = NULL;
	int is_in = ep->bEndpointAddress & USB_DIR_IN;
	u32 idx = 0;
	u8 old_ep_index = 0;
	u32 is_done = 0;

	idx = ep->bEndpointAddress & 0x7F;

	old_ep_index = USBC_GetActiveEp(g_sw_udc_io.usb_vbase);
	USBC_SelectActiveEp(g_sw_udc_io.usb_vbase, idx);

	if (is_in) {
        if (USBC_Dev_IsEpStall(g_sw_udc_io.usb_vbase,
                    SUNXI_EP_TYPE_TX)) {
            printf("ERR: tx ep(%d) is stall\n", idx);
            USBC_Dev_EpClearStall(g_sw_udc_io.usb_vbase,
                    SUNXI_EP_TYPE_TX);
            goto end;
		}
	} else {
		if (USBC_Dev_IsEpStall(g_sw_udc_io.usb_vbase,
					SUNXI_EP_TYPE_RX)) {
			printf("ERR: rx ep(%d) is stall\n", idx);
			USBC_Dev_EpClearStall(g_sw_udc_io.usb_vbase,
					SUNXI_EP_TYPE_RX);
			goto end;
		}
	}

	/* get req */
	if (likely(!list_empty(&ep->queue)))
		req = list_entry(ep->queue.next, struct sw_udc_request, queue);
	else
		req = NULL;

	if (req) {

		/*DMSG_INFO("b: (0x%p, %d, %d)\n", &(req->req),
		  req->req.length, req->req.actual);*/

		if (is_in) {
			if (req->req.length <= req->req.actual) {
				sw_udc_done(ep, req, 0);
				is_done = 1;
				goto next;
			} else {
                if (!USBC_Dev_IsWriteDataReady(
                            g_sw_udc_io.usb_vbase,
                            SUNXI_EP_TYPE_TX))
                    sw_udc_write_fifo(ep, req);
			}
		} else {
            if (USBC_Dev_IsReadDataReady(g_sw_udc_io.usb_vbase,
                        SUNXI_EP_TYPE_RX))
                is_done = sw_udc_read_fifo(ep, req);
		}
	}

next:
	/* do next req */
	if (is_done) {
		if (likely(!list_empty(&ep->queue)))
			req = list_entry(ep->queue.next, struct sw_udc_request,
					queue);
		else
			req = NULL;

		if (req) {

			/*DMSG_INFO("n: (0x%p, %d, %d)\n", &(req->req),
			 req->req.length, req->req.actual);*/

			USBC_SelectActiveEp(g_sw_udc_io.usb_vbase, idx);

            if (is_in && !USBC_Dev_IsWriteDataReady(
                        g_sw_udc_io.usb_vbase,
                        SUNXI_EP_TYPE_TX))
                sw_udc_write_fifo(ep, req);
            else if (!is_in && USBC_Dev_IsReadDataReady(
                        g_sw_udc_io.usb_vbase,
                        SUNXI_EP_TYPE_RX))
                sw_udc_read_fifo(ep, req);
		}
	}

end:
	USBC_SelectActiveEp(g_sw_udc_io.usb_vbase, old_ep_index);

	return;
}

static u32 filtrate_irq_misc(u32 irq_misc)
{
	u32 irq = irq_misc;

	irq &= ~(SUNXI_INTUSB_VBUS_ERROR | SUNXI_INTUSB_SESSION_REQ | SUNXI_INTUSB_CONNECT | SUNXI_INTUSB_SOF);
	USBC_INT_ClearMiscPending(g_sw_udc_io.usb_vbase, SUNXI_INTUSB_VBUS_ERROR);
	USBC_INT_ClearMiscPending(g_sw_udc_io.usb_vbase, SUNXI_INTUSB_SESSION_REQ);
	USBC_INT_ClearMiscPending(g_sw_udc_io.usb_vbase, SUNXI_INTUSB_CONNECT);
	USBC_INT_ClearMiscPending(g_sw_udc_io.usb_vbase, SUNXI_INTUSB_SOF);

	return irq;
}

static void clear_all_irq(void)
{
	USBC_INT_ClearEpPendingAll(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_TX);
	USBC_INT_ClearEpPendingAll(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_RX);
	USBC_INT_ClearMiscPendingAll(g_sw_udc_io.usb_vbase);
}

static void throw_away_all_urb(struct sw_udc *dev)
{
	int k = 0;

	debug("irq: reset happen, throw away all urb\n");
	for(k = 0; k < SW_UDC_ENDPOINTS; k++){
		sw_udc_nuke(dev, (struct sw_udc_ep * )&(dev->ep[k]), -ESHUTDOWN);
	}
}

static int sw_udc_irq(int dummy, void *_dev)
{
	struct sw_udc *dev = _dev;
	int usb_irq = 0;
	int tx_irq = 0;
	int rx_irq = 0;
	int i = 0;
	u32 old_ep_idx = 0;
	unsigned long flags = 0;

	spin_lock_irqsave(&dev->lock, flags);

	/* Driver connected ? */
	if (!dev->driver || !is_peripheral_active()) {
		printf("ERR: functoin driver is not exist, or udc is not active.\n");

		/* Clear interrupts */
		clear_all_irq();

		spin_unlock_irqrestore(&dev->lock, flags);

		return IRQ_HANDLED;
	}

	/* Save index */
	old_ep_idx = USBC_GetActiveEp(g_sw_udc_io.usb_vbase);

	/* Read status registers */
	usb_irq = USBC_INT_MiscPending(g_sw_udc_io.usb_vbase);
	tx_irq  = USBC_INT_EpPending(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_TX);
	rx_irq  = USBC_INT_EpPending(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_RX);

	debug("\n\nirq: usb_irq=%02x, tx_irq=%02x, rx_irq=%02x\n",
			usb_irq, tx_irq, rx_irq);

	usb_irq = filtrate_irq_misc(usb_irq);

	/*
	 * Now, handle interrupts. There's two types :
	 * - Reset, Resume, Suspend coming -> usb_int_reg
	 * - EP -> ep_int_reg
	 */

	/* RESET */
	if (usb_irq & SUNXI_INTUSB_RESET) {
		debug("IRQ: reset\n");

		USBC_INT_ClearMiscPending(g_sw_udc_io.usb_vbase, SUNXI_INTUSB_RESET);
		clear_all_irq();

		usb_connect = 1;

		USBC_SelectActiveEp(g_sw_udc_io.usb_vbase, 0);
		USBC_Dev_SetAddress_default(g_sw_udc_io.usb_vbase);

		throw_away_all_urb(dev);
		dev->address = 0;
		dev->ep0state = EP0_IDLE;
		spin_unlock_irqrestore(&dev->lock, flags);

		return IRQ_HANDLED;
	}

	/* RESUME */
	if (usb_irq & SUNXI_INTUSB_RESUME) {
		debug("IRQ: resume\n");

		/* clear interrupt */
		USBC_INT_ClearMiscPending(g_sw_udc_io.usb_vbase, SUNXI_INTUSB_RESUME);

		if (dev->gadget.speed != USB_SPEED_UNKNOWN
				&& dev->driver
				&& dev->driver->resume) {
			spin_unlock(&dev->lock);
			dev->driver->resume(&dev->gadget);
			spin_lock(&dev->lock);
		}
	}

	/* SUSPEND */
	if (usb_irq & SUNXI_INTUSB_SUSPEND) {
		debug("IRQ: suspend\n");

		/* clear interrupt */
		USBC_INT_ClearMiscPending(g_sw_udc_io.usb_vbase, SUNXI_INTUSB_SUSPEND);

		if (dev->gadget.speed != USB_SPEED_UNKNOWN) {
			usb_connect = 0;
		} else {
			debug("ERR: usb speed is unkown\n");
		}

		if (dev->gadget.speed != USB_SPEED_UNKNOWN
				&& dev->driver
				&& dev->driver->suspend) {
			spin_unlock(&dev->lock);
			dev->driver->suspend(&dev->gadget);
			spin_lock(&dev->lock);
		}

		dev->ep0state = EP0_IDLE;
	}

	/* DISCONNECT */
	if (usb_irq & SUNXI_INTUSB_DISCONNECT) {
		debug("IRQ: disconnect\n");

		USBC_INT_ClearMiscPending(g_sw_udc_io.usb_vbase, SUNXI_INTUSB_DISCONNECT);

		dev->ep0state = EP0_IDLE;

		usb_connect = 0;
	}
	/* EP */
	/* control traffic */
	/* check on ep0csr != 0 is not a good idea as clearing in_pkt_ready
	 * generate an interrupt
	 */
	if (tx_irq & SUNXI_INTTx_FLAG_EP0) {
		debug("USB ep0 irq\n");

		/* Clear the interrupt bit by setting it to 1 */
		USBC_INT_ClearEpPending(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_TX, 0);

		if (dev->gadget.speed == USB_SPEED_UNKNOWN) {
			if (USBC_Dev_QueryTransferMode(g_sw_udc_io.usb_vbase) == SUNXI_TS_MODE_HS) {
				dev->gadget.speed = USB_SPEED_HIGH;

				debug("\n+++++++++++++++++++++++++++++++++++++\n");
				debug(" usb enter high speed.\n");
				debug("\n+++++++++++++++++++++++++++++++++++++\n");
			} else {
				dev->gadget.speed= USB_SPEED_FULL;

				debug("\n+++++++++++++++++++++++++++++++++++++\n");
				debug(" usb enter full speed.\n");
				debug("\n+++++++++++++++++++++++++++++++++++++\n");
			}
		}

		sw_udc_handle_ep0(dev);
	}

	/* rx endpoint data transfers */
	for (i = 1; i < SW_UDC_ENDPOINTS; i++) {
		u32 tmp = 1 << i;

		if (rx_irq & tmp) {
			debug("USB rx ep%d irq\n", i);

			/* Clear the interrupt bit by setting it to 1 */
			USBC_INT_ClearEpPending(g_sw_udc_io.usb_vbase,
					SUNXI_EP_TYPE_RX, i);

			sw_udc_handle_ep(&dev->ep[i]);
		}
	}

	/* tx endpoint data transfers */
	for (i = 1; i < SW_UDC_ENDPOINTS; i++) {
		u32 tmp = 1 << i;

		if (tx_irq & tmp) {
			debug("USB tx ep%d irq\n", i);

			/* Clear the interrupt bit by setting it to 1 */
			USBC_INT_ClearEpPending(g_sw_udc_io.usb_vbase,
					SUNXI_EP_TYPE_TX, i);

			sw_udc_handle_ep(&dev->ep[i]);
		}
	}

	debug("irq: %d irq end.\n", dev->irq_no);

	/* Restore old index */
	USBC_SelectActiveEp(g_sw_udc_io.usb_vbase, old_ep_idx);

	spin_unlock_irqrestore(&dev->lock, flags);

	return IRQ_HANDLED;
}

static inline struct sw_udc_ep *to_sw_udc_ep(struct usb_ep *ep)
{
	return container_of(ep, struct sw_udc_ep, ep);
}

static inline struct sw_udc *to_sw_udc(struct usb_gadget *gadget)
{
	return container_of(gadget, struct sw_udc, gadget);
}

static inline struct sw_udc_request *to_sw_udc_req(struct usb_request *req)
{
	return container_of(req, struct sw_udc_request, req);
}

static int sw_udc_ep_enable(struct usb_ep *_ep,
		const struct usb_endpoint_descriptor *desc)
{
	struct sw_udc *dev = NULL;
	struct sw_udc_ep *ep = NULL;
	u32 max	= 0;
	unsigned long flags = 0;
	u32 old_ep_index = 0;
	__u32 fifo_addr = 0;

	if (_ep == NULL || desc == NULL) {
		printf("ERR: invalid argment\n");
		return -EINVAL;
	}

	if (_ep->name == ep0name || desc->bDescriptorType != USB_DT_ENDPOINT) {
		printf("PANIC : _ep->name(%s) == ep0name || desc->bDescriptorType(%d) != USB_DT_ENDPOINT\n", _ep->name , desc->bDescriptorType);
		return -EINVAL;
	}

	ep = to_sw_udc_ep(_ep);
	if (ep == NULL) {
		printf("ERR: usbd_ep_enable, ep = NULL\n");
		return -EINVAL;
	}

	if (ep->desc) {
		printf("ERR: usbd_ep_enable, ep->desc is not NULL, ep%d(%s)\n", ep->num, _ep->name);
		return -EINVAL;
	}

	debug("ep enable: ep%d(0x%p, %s, %d, %d)\n",
			ep->num, _ep, _ep->name,
			(desc->bEndpointAddress & USB_DIR_IN), _ep->maxpacket);

	dev = ep->dev;
	if (!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN) {
		printf("PANIC : dev->driver = 0x%p ?= NULL  dev->gadget->speed =%d ?= USB_SPEED_UNKNOWN\n", dev->driver ,dev->gadget.speed);
		return -ESHUTDOWN;
	}

	max = usb_endpoint_maxp(desc) & 0x1fff;

	spin_lock_irqsave(&ep->dev->lock, flags);

	_ep->maxpacket          = max & 0x7ff;
	ep->desc                = desc;
	ep->halted              = 0;
	ep->bEndpointAddress    = desc->bEndpointAddress;

	fifo_addr = ep->num * 1024;

	if(!is_peripheral_active()){
		printf("ERR: usb device is not active\n");
		goto end;
	}

	old_ep_index = USBC_GetActiveEp(g_sw_udc_io.usb_vbase);
	USBC_SelectActiveEp(g_sw_udc_io.usb_vbase, ep->num);

	if ((ep->bEndpointAddress) & USB_DIR_IN)
	{ /* tx */
		USBC_Dev_ConfigEp(g_sw_udc_io.usb_vbase, SUNXI_TS_TYPE_BULK, SUNXI_EP_TYPE_TX, SW_UDC_FIFO_NUM, _ep->maxpacket & 0x7ff);
		USBC_ConfigFifo(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_TX, SW_UDC_FIFO_NUM, _ep->maxpacket, fifo_addr);
		USBC_INT_EnableEp(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_TX, ep->num);
	} 
	else
	{ /* rx */
		USBC_Dev_ConfigEp(g_sw_udc_io.usb_vbase, SUNXI_TS_TYPE_BULK, SUNXI_EP_TYPE_RX, SW_UDC_FIFO_NUM, _ep->maxpacket & 0x7ff);
		USBC_ConfigFifo(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_RX, SW_UDC_FIFO_NUM, 512, fifo_addr);
		USBC_INT_EnableEp(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_RX, ep->num);
	}

	USBC_SelectActiveEp(g_sw_udc_io.usb_vbase, old_ep_index);

end:
	spin_unlock_irqrestore(&ep->dev->lock, flags);

	sw_udc_set_halt(_ep, 0);

	return 0;
}

static int sw_udc_ep_disable(struct usb_ep *_ep)
{
	struct sw_udc_ep *ep = NULL;
	u32 old_ep_index = 0;
	unsigned long flags = 0;

	if (!_ep) {
		printf("ERR: invalid argment\n");
		return -EINVAL;
	}

	ep = to_sw_udc_ep(_ep);
	if(ep == NULL){
		printf("ERR: usbd_ep_disable: ep = NULL\n");
		return -EINVAL;
	}

	if (!ep->desc) {
		printf("ERR: %s not enabled\n", _ep ? ep->ep.name : NULL);
		return -EINVAL;
	}

	debug("ep disable: ep%d(0x%p, %s, %d, %x)\n",
			ep->num, _ep, _ep->name,
			(ep->bEndpointAddress & USB_DIR_IN), _ep->maxpacket);

	spin_lock_irqsave(&ep->dev->lock, flags);

	debug("ep_disable: %s\n", _ep->name);

	ep->desc = NULL;
	ep->halted = 1;

	sw_udc_nuke(ep->dev, ep, -ESHUTDOWN);

	if(!is_peripheral_active()){
		printf("ERR: usb device is not active\n");
		goto end;
	}

	old_ep_index = USBC_GetActiveEp(g_sw_udc_io.usb_vbase);
	USBC_SelectActiveEp(g_sw_udc_io.usb_vbase, ep->num);

	if ((ep->bEndpointAddress) & USB_DIR_IN)
	{ /* tx */
		USBC_Dev_ConfigEp_Default(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_TX);
		USBC_INT_DisableEp(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_TX, ep->num);
	}
	else
	{ /* rx */
		USBC_Dev_ConfigEp_Default(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_RX);
		USBC_INT_DisableEp(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_RX, ep->num);
	}

	USBC_SelectActiveEp(g_sw_udc_io.usb_vbase, old_ep_index);

end:
	spin_unlock_irqrestore(&ep->dev->lock, flags);

	debug("%s disabled\n", _ep->name);

	return 0;
}

static struct usb_request * sw_udc_alloc_request(struct usb_ep *_ep, gfp_t mem_flags)
{
	struct sw_udc_request *req = NULL;

	if (!_ep) {
		printf("ERR: invalid argment\n");
		return NULL;
	}

	req = memalign(CONFIG_SYS_CACHELINE_SIZE, sizeof(struct sw_udc_request));
	if (!req) {
		printf("ERR: kzalloc failed\n");
		return NULL;
	}

	memset(req, 0, sizeof(struct sw_udc_request));

	INIT_LIST_HEAD (&req->queue);

	debug("alloc request: ep(0x%p, %s, %d), req(0x%p)\n",
			_ep, _ep->name, _ep->maxpacket, req);

	return &req->req;
}

static void sw_udc_free_request(struct usb_ep *_ep, struct usb_request *_req)
{
	struct sw_udc_request   *req = NULL;

	if (_ep == NULL || _req == NULL) {
		printf("ERR: invalid argment\n");
		return;
	}

	req = to_sw_udc_req(_req);
	if (req == NULL) {
		printf("ERR: invalid argment\n");
		return;
	}

	debug("free request: ep(0x%p, %s, %d), req(0x%p)\n",
			_ep, _ep->name, _ep->maxpacket, req);

	kfree(req);

	return;
}

static int sw_udc_queue(struct usb_ep *_ep, struct usb_request *_req, gfp_t gfp_flags)
{
	struct sw_udc_request *req = NULL;
	struct sw_udc_ep *ep = NULL;
	struct sw_udc *dev = NULL;
	unsigned long flags = 0;
	u8 old_ep_index = 0;

	if (_ep == NULL || _req == NULL ) {
		printf("ERR: invalid argment\n");
		return -EINVAL;
	}

	ep = to_sw_udc_ep(_ep);
	if ((ep == NULL || (!ep->desc && _ep->name != ep0name))) {
		printf("ERR: sw_udc_queue: inval 2\n");
		return -EINVAL;
	}

	dev = ep->dev;
	if (!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN) {
		printf("ERR : dev->driver=0x%p, dev->gadget.speed=%x\n",
				dev->driver, dev->gadget.speed);
		return -ESHUTDOWN;
	}

	if (!_req->complete || !_req->buf) {
		printf("ERR: usbd_queue: _req is invalid\n");
		return -EINVAL;
	}

	req = to_sw_udc_req(_req);
	if (!req) {
		printf("ERR: req is NULL\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&ep->dev->lock, flags);

	_req->status = -EINPROGRESS;
	_req->actual = 0;

	list_add_tail(&req->queue, &ep->queue);

	if (!is_peripheral_active()) {
		printf("warn: peripheral is active\n");
		goto end;
	}

	debug("\n\nq: ep(0x%p, %d), req(0x%p, 0x%p, %d, %d)\n",
			ep, ep->num,
			req, _req, _req->length, _req->actual);

	old_ep_index = USBC_GetActiveEp(g_sw_udc_io.usb_vbase);
	if (ep->bEndpointAddress) {
		USBC_SelectActiveEp(g_sw_udc_io.usb_vbase, ep->bEndpointAddress & 0x7F);
	} else {
		USBC_SelectActiveEp(g_sw_udc_io.usb_vbase, 0);
	}

	if (!ep->halted && (&req->queue == ep->queue.next)) {
		if (ep->bEndpointAddress == 0 /* ep0 */) {
			switch (dev->ep0state) {
            case EP0_IN_DATA_PHASE:
                if (!USBC_Dev_IsWriteDataReady(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_TX)
                        && sw_udc_write_fifo(ep, req)) { 
                    dev->ep0state = EP0_IDLE;
                    req = NULL;
                }
                break;
                
            case EP0_OUT_DATA_PHASE:
                if ((!_req->length)
                        || (USBC_Dev_IsReadDataReady(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_RX)
                            && sw_udc_read_fifo(ep, req))) {
                    dev->ep0state = EP0_IDLE;
                    req = NULL;
                }
                break;
                
            default:
                spin_unlock_irqrestore(&ep->dev->lock, flags);
                return -EL2HLT;
            }
		} else if ((ep->bEndpointAddress & USB_DIR_IN) != 0
				&& !USBC_Dev_IsWriteDataReady(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_TX)) {
			if (sw_udc_write_fifo(ep, req)) {
				req = NULL;
			}
		} else if ((ep->bEndpointAddress & USB_DIR_IN) == 0
				&& USBC_Dev_IsReadDataReady(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_RX)) {
			if (sw_udc_read_fifo(ep, req)) {
				req = NULL;
			}
		}

	}

	USBC_SelectActiveEp(g_sw_udc_io.usb_vbase, old_ep_index);
end:

	spin_unlock_irqrestore(&ep->dev->lock, flags);

	return 0;
}

static int sw_udc_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct sw_udc_ep *ep = NULL;
	struct sw_udc *udc = NULL;
	int retval = -EINVAL;
	unsigned long flags = 0;
	struct sw_udc_request *req = NULL;

	debug("(%p,%p)\n", _ep, _req);

	if (!the_controller->driver) {
		printf("ERR: sw_udc_dequeue: driver is null\n");
		return -ESHUTDOWN;
	}

	if (!_ep || !_req) {
		printf("ERR: sw_udc_dequeue: invalid argment\n");
		return retval;
	}

	ep = to_sw_udc_ep(_ep);
	if (ep == NULL) {
		printf("ERR: ep == NULL\n");
		return -EINVAL;
	}

	udc = to_sw_udc(ep->gadget);
	if (udc == NULL) {
		printf("ERR: ep == NULL\n");
		return -EINVAL;
	}

	debug("dequeue: ep(0x%p, %d), _req(0x%p, %d, %d)\n",
			ep, ep->num,
			_req, _req->length, _req->actual);

	spin_lock_irqsave(&ep->dev->lock, flags);

	list_for_each_entry (req, &ep->queue, queue) {
		if (&req->req == _req) {
			list_del_init (&req->queue);
			_req->status = -ECONNRESET;
			retval = 0;
			break;
		}
	}

	if (retval == 0) {
		debug("dequeued req %p from %s, len %d buf %p\n",
				req, _ep->name, _req->length, _req->buf);

		sw_udc_done(ep, req, -ECONNRESET);
	}

	spin_unlock_irqrestore(&ep->dev->lock, flags);

	return retval;
}

static int sw_udc_set_halt(struct usb_ep *_ep, int value)
{
	struct sw_udc_ep *ep = NULL;
	unsigned long flags = 0;
	u32 idx = 0;
	u8 old_ep_index = 0;

	if (_ep == NULL) {
		printf("ERR: invalid argment\n");
		return -EINVAL;
	}

	ep = to_sw_udc_ep(_ep);
	if (ep == NULL) {
		printf("ERR: invalid argment\n");
		return -EINVAL;
	}

	if (!ep->desc && ep->ep.name != ep0name) {
		printf("ERR: !ep->desc && ep->ep.name != ep0name\n");
		return -EINVAL;
	}

	if (!is_peripheral_active()) {
		printf("ERR: usb device is not active\n");
		return 0;
	}

	spin_lock_irqsave(&ep->dev->lock, flags);

	idx = ep->bEndpointAddress & 0x7F;

	old_ep_index = USBC_GetActiveEp(g_sw_udc_io.usb_vbase);
	USBC_SelectActiveEp(g_sw_udc_io.usb_vbase, idx);

	if (idx == 0) {
		USBC_Dev_EpClearStall(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_EP0);
	} else {
		if ((ep->bEndpointAddress & USB_DIR_IN) != 0) {
			if (value) {
				USBC_Dev_EpSendStall(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_TX);
			} else {
				USBC_Dev_EpClearStall(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_TX);
			}
		} else {
			if (value) {
				USBC_Dev_EpSendStall(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_RX);
			}else{
				USBC_Dev_EpClearStall(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_RX);
			}
		}
	}

	ep->halted = value ? 1 : 0;

	USBC_SelectActiveEp(g_sw_udc_io.usb_vbase, old_ep_index);

	spin_unlock_irqrestore(&ep->dev->lock, flags);

	return 0;
}

static const struct usb_ep_ops sw_udc_ep_ops = {
	.enable                 = sw_udc_ep_enable,
	.disable                = sw_udc_ep_disable,

	.alloc_request  	= sw_udc_alloc_request,
	.free_request   	= sw_udc_free_request,

	.queue                  = sw_udc_queue,
	.dequeue                = sw_udc_dequeue,

	.set_halt               = sw_udc_set_halt,
};

static int sw_udc_get_frame(struct usb_gadget *_gadget)
{
	if (!is_peripheral_active()) {
		printf("ERR: usb device is not active\n");
		return 0;
	}

	return (int) (g_sw_udc_io.usb_vbase + SUNXI_FRNUM);
}

static int sw_udc_wakeup(struct usb_gadget *_gadget)
{
	if (!is_peripheral_active()) {
		printf("ERR: usb device is not active\n");
		return 0;
	}

	return 0;
}

static int sw_udc_set_selfpowered(struct usb_gadget *gadget, int value)
{
	if (!is_peripheral_active()) {
		printf("ERR: usb device is not active\n");
		return 0;
	}

	return 0;
}

static void sw_udc_disable(struct sw_udc *dev);
static void sw_udc_enable(struct sw_udc *dev);

static int sw_udc_set_pullup(struct sw_udc *udc, int is_on)
{
	debug("sw_udc_set_pullup\n");

	is_udc_enable = is_on;
	if (!is_peripheral_active()) {
		printf("ERR: usb device is not active\n");
		return 0;
	}


	if (is_on) {
		sw_udc_enable(udc);
	} else {
		if (udc->gadget.speed != USB_SPEED_UNKNOWN) {
			if (udc->driver && udc->driver->disconnect)
				udc->driver->disconnect(&udc->gadget);
		}

		sw_udc_disable(udc);
	}

	return 0;
}

static int sw_udc_vbus_session(struct usb_gadget *gadget, int is_active)
{
	struct sw_udc *udc = to_sw_udc(gadget);

	debug("sw_udc_vbus_session\n");

	if (!is_peripheral_active()) {
		printf("ERR: usb device is not active\n");
		return 0;
	}

	udc->vbus = (is_active != 0);
	sw_udc_set_pullup(udc, is_active);

	return 0;
}

static int sw_udc_pullup(struct usb_gadget *gadget, int is_on)
{
	struct sw_udc *udc = to_sw_udc(gadget);

	debug("sw_udc_pullup, is_on = %d\n", is_on);

	sw_udc_set_pullup(udc, is_on);

	return 0;
}

static int sw_udc_vbus_draw(struct usb_gadget *_gadget, unsigned ma)
{
	if (!is_peripheral_active()) {
		printf("ERR: usb device is not active\n");
		return 0;
	}

	debug("sw_udc_vbus_draw\n");

	return 0;
}

static void sw_udc_reinit(struct sw_udc *dev)
{
	u32 i = 0;

	/* device/ep0 records init */
	INIT_LIST_HEAD (&dev->gadget.ep_list);
	INIT_LIST_HEAD (&dev->gadget.ep0->ep_list);
	dev->ep0state = EP0_IDLE;

	for (i = 0; i < SW_UDC_ENDPOINTS; i++) {
		struct sw_udc_ep *ep = &dev->ep[i];

		if (i != 0) {
			list_add_tail (&ep->ep.ep_list, &dev->gadget.ep_list);
		}

		ep->dev     = dev;
		ep->desc    = NULL;
		ep->halted  = 0;
		INIT_LIST_HEAD (&ep->queue);
	}
}

static void sw_udc_enable(struct sw_udc *dev)
{
	debug("sw_udc_enable called\n");

	/* dev->gadget.speed = USB_SPEED_UNKNOWN; */
	dev->gadget.speed = USB_SPEED_UNKNOWN;

	debug("CONFIG_USB_GADGET_DUALSPEED\n");

	USBC_Dev_ConfigTransferMode(g_sw_udc_io.usb_vbase, SUNXI_TS_TYPE_BULK, SUNXI_TS_MODE_HS);

	/* Enable reset and suspend interrupt interrupts */
	USBC_INT_EnableUsbMiscUint(g_sw_udc_io.usb_vbase, SUNXI_BP_INTUSB_SUSPEND);
	USBC_INT_EnableUsbMiscUint(g_sw_udc_io.usb_vbase, SUNXI_BP_INTUSB_RESUME);
	USBC_INT_EnableUsbMiscUint(g_sw_udc_io.usb_vbase, SUNXI_BP_INTUSB_RESET);
	USBC_INT_EnableUsbMiscUint(g_sw_udc_io.usb_vbase, SUNXI_BP_INTUSB_DISCONNECT);

	/* Enable ep0 interrupt */
	USBC_INT_EnableEp(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_TX, 0);

	cfg_udc_command(SW_UDC_P_ENABLE);
}

static void sw_udc_disable(struct sw_udc *dev)
{
	debug("sw_udc_disable\n");

	/* Disable all interrupts */
	USBC_INT_DisableUsbMiscAll(g_sw_udc_io.usb_vbase);
	USBC_INT_DisableEpAll(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_RX);
	USBC_INT_DisableEpAll(g_sw_udc_io.usb_vbase, SUNXI_EP_TYPE_TX);

	/* Clear the interrupt registers */
	clear_all_irq();
	cfg_udc_command(SW_UDC_P_DISABLE);

	/* Set speed to unknown */
	dev->gadget.speed = USB_SPEED_UNKNOWN;
}

s32 usbd_start_work(void)
{
	debug("usbd_start_work\n");

	if (!is_peripheral_active()) {
		printf("ERR: usb device is not active\n");
		return 0;
	}

	USBC_Dev_ConectSwitch(g_sw_udc_io.usb_vbase, SUNXI_DEVICE_SWITCH_ON);

	return 0;
}

s32 usbd_stop_work(void)
{
	debug("usbd_stop_work\n");

	if (!is_peripheral_active()) {
		printf("ERR: usb device is not active\n");
		return 0;
	}

	USBC_Dev_ConectSwitch(g_sw_udc_io.usb_vbase, SUNXI_DEVICE_SWITCH_OFF);

	return 0;
}

static const struct usb_gadget_ops sw_udc_ops = {
	.get_frame              = sw_udc_get_frame,
	.wakeup                 = sw_udc_wakeup,
	.set_selfpowered	= sw_udc_set_selfpowered,
	.pullup                 = sw_udc_pullup,
	.vbus_session           = sw_udc_vbus_session,
	.vbus_draw              = sw_udc_vbus_draw,
};

int usb_gadget_register_driver(struct usb_gadget_driver *driver)
{
	struct sw_udc *dev = the_controller;
	int retval = 0;
	unsigned long flags;

	debug("%s: %s\n", __func__, "no name");

	if (!driver
	    || !driver->bind || !driver->disconnect || !driver->setup)
		return -EINVAL;
	if (!dev)
		return -ENODEV;
	if (dev->driver)
		return -EBUSY;

	spin_lock_irqsave(&dev->lock, flags);
	/* first hook up the driver ... */
	dev->driver = driver;
	spin_unlock_irqrestore(&dev->lock, flags);

	if (retval) { /* TODO */
		printf("target device_add failed, error %d\n", retval);
		return retval;
	}

	retval = driver->bind(&dev->gadget);
	if (retval) {
		printf("%s: bind to driver --> error %d\n",
				dev->gadget.name, retval);
		dev->driver = 0;
		return retval;
	}


	debug("Registered gadget driver %s\n", dev->gadget.name);
	sw_udc_enable(dev);

	return 0;
}

int usb_gadget_unregister_driver(struct usb_gadget_driver *driver)
{
	struct sw_udc *dev = the_controller;
	unsigned long flags;

	if (!dev)
		return -ENODEV;
	if (!driver || driver != dev->driver)
		return -EINVAL;

	spin_lock_irqsave(&dev->lock, flags);
	dev->driver = 0;
	spin_unlock_irqrestore(&dev->lock, flags);

	driver->unbind(&dev->gadget);

	sw_udc_disable(dev);

	return 0;
}

static struct sw_udc sw_udc = {
	.gadget = {
		.ops            = &sw_udc_ops,
		.ep0            = &sw_udc.ep[0].ep,
		.name           = gadget_name,
	},

	/* control endpoint */
	.ep[0] = {
		.num                    = 0,
		.ep = {
			.name           = ep0name,
			.ops            = &sw_udc_ep_ops,
			.maxpacket      = EP0_FIFO_SIZE,
		},
		.dev                    = &sw_udc,
	},

	/* first group of endpoints */
	.ep[1] = {
		.num                    = 1,
		.ep = {
			.name           = "ep1-bulk",
			.ops            = &sw_udc_ep_ops,
			.maxpacket      = SW_UDC_EP_FIFO_SIZE,
		},
		.dev                    = &sw_udc,
		.fifo_size              = (SW_UDC_EP_FIFO_SIZE * (SW_UDC_FIFO_NUM + 1)),
		.bEndpointAddress   = 1,
		.bmAttributes       = USB_ENDPOINT_XFER_BULK,
	},
	.ep[2] = {
		.num                    = 2,
		.ep = {
			.name           = "ep2-bulk",
			.ops            = &sw_udc_ep_ops,
			.maxpacket      = SW_UDC_EP_FIFO_SIZE,
		},
		.dev                    = &sw_udc,
		.fifo_size              = (SW_UDC_EP_FIFO_SIZE * (SW_UDC_FIFO_NUM + 1)),
		.bEndpointAddress   = 2,
		.bmAttributes       = USB_ENDPOINT_XFER_BULK,
	},

	.ep[3] = {
		.num                    = 3,
		.ep = {
			.name           = "ep3-bulk",
			.ops            = &sw_udc_ep_ops,
			.maxpacket      = SW_UDC_EP_FIFO_SIZE,
		},
		.dev                    = &sw_udc,
		.fifo_size              = (SW_UDC_EP_FIFO_SIZE * (SW_UDC_FIFO_NUM + 1)),
		.bEndpointAddress   = 3,
		.bmAttributes       = USB_ENDPOINT_XFER_BULK,
	},

	.ep[4] = {
		.num                    = 4,
		.ep = {
			.name           = "ep4-bulk",
			.ops            = &sw_udc_ep_ops,
			.maxpacket      = SW_UDC_EP_FIFO_SIZE,
		},
		.dev                    = &sw_udc,
		.fifo_size              = (SW_UDC_EP_FIFO_SIZE * (SW_UDC_FIFO_NUM + 1)),
		.bEndpointAddress   = 4,
		.bmAttributes       = USB_ENDPOINT_XFER_BULK,
	},

	.ep[5] = {
		.num                    = 5,
		.ep = {
			.name           = "ep5-int",
			.ops            = &sw_udc_ep_ops,
			.maxpacket      = SW_UDC_EP_FIFO_SIZE,
		},
		.dev                    = &sw_udc,
		.fifo_size              = (SW_UDC_EP_FIFO_SIZE * (SW_UDC_FIFO_NUM + 1)),
		.bEndpointAddress   = 5,
		.bmAttributes       = USB_ENDPOINT_XFER_INT,
	},
};

#define SW_VA_USB0_IO_BASE                0xf1c13000
static void sw_phy_write(__u32 addr, __u32 data, __u32 len)
{
	// USBC_Phy_GetCsr(0)
	// SW_VA_USB0_IO_BASE + 0x404;
	u32 temp = 0, dtmp = 0;
	u32 j=0;

	dtmp = data;
	for (j = 0; j < len; j++)
	{
		/* set  the bit address to be write */
		//temp = USBC_Readl(USBC_Phy_GetCsr(usbc_no));
		temp = readl(SW_VA_USB0_IO_BASE + 0x404);
		temp &= ~(0xff << 8);
		temp |= ((addr + j) << 8);
		writel(temp, SW_VA_USB0_IO_BASE + 0x404);

		temp = readb(SW_VA_USB0_IO_BASE + 0x404);
		temp &= ~(0x1 << 7);
		temp |= (dtmp & 0x1) << 7;
		temp &= ~(0x1);
		writeb(temp, SW_VA_USB0_IO_BASE + 0x404);

		temp = readb(SW_VA_USB0_IO_BASE + 0x404);
		temp |= (0x1);
		writeb(temp, SW_VA_USB0_IO_BASE + 0x404);

		temp = readb(SW_VA_USB0_IO_BASE + 0x404);
		temp &= ~(0x1);
		writeb(temp, SW_VA_USB0_IO_BASE + 0x404);
		dtmp >>= 1;
	}
}

__s32 sw_udc_bsp_init(__u32 usbc_no, sw_udc_io_t *sw_udc_io)
{
	u32 reg;

	//USBC_EnhanceSignal(sw_udc_io->usb_bsp_hdle);
	//NONE

	//USBC_EnableDpDmPullUp(sw_udc_io->usb_bsp_hdle);
	reg = readl(sw_udc_io->usb_vbase + SUNXI_ISCR);
	reg |= (1 << SUNXI_BP_ISCR_DPDM_PULLUP_EN);
	// reg_val = __USBC_WakeUp_ClearChangeDetect(reg_val)
	reg &= ~(1 << SUNXI_BP_ISCR_VBUS_CHANGE_DETECT);
	reg &= ~(1 << SUNXI_BP_ISCR_ID_CHANGE_DETECT);
	reg &= ~(1 << SUNXI_BP_ISCR_DPDM_CHANGE_DETECT);
	writel(reg, sw_udc_io->usb_vbase + SUNXI_ISCR);

	//USBC_EnableIdPullUp(sw_udc_io->usb_bsp_hdle);
	reg = readl(sw_udc_io->usb_vbase + SUNXI_ISCR);
	reg |= (1 << SUNXI_BP_ISCR_ID_PULLUP_EN);
	// reg_val = __USBC_WakeUp_ClearChangeDetect(reg_val)
	reg &= ~(1 << SUNXI_BP_ISCR_VBUS_CHANGE_DETECT);
	reg &= ~(1 << SUNXI_BP_ISCR_ID_CHANGE_DETECT);
	reg &= ~(1 << SUNXI_BP_ISCR_DPDM_CHANGE_DETECT);
	writel(reg, sw_udc_io->usb_vbase + SUNXI_ISCR);

	//USBC_ForceId(sw_udc_io->usb_bsp_hdle, USBC_ID_TYPE_DEVICE);
	// __USBC_ForceIdToHigh(usbc_otg->base_addr)
	reg = readl(sw_udc_io->usb_vbase + SUNXI_ISCR);
	reg |=  (0x03 << SUNXI_BP_ISCR_FORCE_ID);
	// reg_val = __USBC_WakeUp_ClearChangeDetect(reg_val)
	reg &= ~(1 << SUNXI_BP_ISCR_VBUS_CHANGE_DETECT);
	reg &= ~(1 << SUNXI_BP_ISCR_ID_CHANGE_DETECT);
	reg &= ~(1 << SUNXI_BP_ISCR_DPDM_CHANGE_DETECT);
	writel(reg, sw_udc_io->usb_vbase + SUNXI_ISCR);

	//USBC_ForceVbusValid(sw_udc_io->usb_bsp_hdle, USBC_VBUS_TYPE_HIGH);
	// __USBC_ForceVbusValidToHigh(usbc_otg->base_addr)
	reg = readl(sw_udc_io->usb_vbase + SUNXI_ISCR);
	reg |= (0x03 << SUNXI_BP_ISCR_FORCE_VBUS_VALID);
	// reg_val = __USBC_WakeUp_ClearChangeDetect(reg_val)
	reg &= ~(1 << SUNXI_BP_ISCR_VBUS_CHANGE_DETECT);
	reg &= ~(1 << SUNXI_BP_ISCR_ID_CHANGE_DETECT);
	reg &= ~(1 << SUNXI_BP_ISCR_DPDM_CHANGE_DETECT);
	writel(reg, sw_udc_io->usb_vbase + SUNXI_ISCR);

	//USBC_SelectBus(sw_udc_io->usb_bsp_hdle, USBC_IO_TYPE_PIO, 0, 0);
	reg = readb(sw_udc_io->usb_vbase + SUNXI_VEND0);
	reg &= 0x00;
	writeb(reg, sw_udc_io->usb_vbase + SUNXI_VEND0);

	return 0;
}

extern void sdelay(unsigned long loops);
static __s32 sw_udc_io_init(__u32 usbc_no, sw_udc_io_t *sw_udc_io)
{
	unsigned long flags = 0;
	u32 reg;

	sw_udc_io->usb_vbase  = (void __iomem *) SUNXI_USB0_BASE;
	sw_udc_io->sram_vbase = (void __iomem *) SUNXI_SRAMC_BASE;

	struct sunxi_ccm_reg *ccm = (struct sunxi_ccm_reg *) SUNXI_CCM_BASE;

	// open_usb_clock(udc.ccmu_base);
	setbits_le32(&ccm->ahb_gate0, 0x1 << AHB_GATE_OFFSET_USB);
	sdelay(10000);
	setbits_le32(&ccm->usb_clk_cfg, 0x1 << 8 | 0x1);
	sdelay(10000);

	sw_udc_io->clk_is_open = 1;

	//UsbPhyInit(0);
	//USBC_Phy_Write(usbc_no, 0x0c, 0x01, 1);
	sw_phy_write(0x0c, 0x01, 1);
	//USBC_Phy_Write(usbc_no, 0x20, 0x14, 5);
	sw_phy_write(0x20, 0x14, 5);
	//USBC_Phy_Write(usbc_no, 0x2a, 3, 2);
	sw_phy_write(0x2a, 3, 2);

	/* initialize usb bsp */
	sw_udc_bsp_init(usbc_no, sw_udc_io);

	/* config usb fifo */
	spin_lock_init(&lock);
	spin_lock_irqsave(&lock, flags);
	//USBC_ConfigFIFO_Base(sw_udc_io->usb_bsp_hdle, (u32)sw_udc_io->sram_vbase, USBC_FIFO_MODE_8K);
	reg = readl(sw_udc_io->sram_vbase + 0x04);
	reg |= (1 << 0);
	writel(reg, sw_udc_io->sram_vbase + 0x04);

	spin_unlock_irqrestore(&lock, flags);

	return 0;
}	

int sw_udc_probe(void)
{
	struct sw_udc *udc = &sw_udc;
	int ret = 0;

	memset(&g_sw_udc_io, 0, sizeof(sw_udc_io_t));

	ret = sw_udc_io_init(usbd_port_no, &g_sw_udc_io);
	if (ret != 0)
	{
		printf("ERR: sw_udc_io_init failed\n");
		return -1;
	}

	spin_lock_init (&udc->lock);

	is_controller_alive = 1;
	the_controller = udc;

	sw_udc_disable(udc);
	sw_udc_reinit(udc);

	udc->sw_udc_io = &g_sw_udc_io;
	udc->usbc_no = usbd_port_no;
	strcpy((char *)udc->driver_name, gadget_name);

	return 0;
}

static void cfg_udc_command(enum sw_udc_cmd_e cmd)
{
	struct sw_udc *udc = the_controller;

	switch (cmd)
	{
		case SW_UDC_P_ENABLE:
		{
			if (udc->driver) {
				usbd_start_work();
			} else {
				debug("udc->driver is null, udc is need not start\n");
			}
		}
		break;

		case SW_UDC_P_DISABLE:
		{
			if (udc->driver) {
				usbd_stop_work();
			} else {
				debug("udc->driver is null, udc is need not stop\n");
			}
		}
		break;

		case SW_UDC_P_RESET :
			printf("ERR: reset is not support\n");
			break;

		default:
			printf("ERR: unkown cmd(%d)\n", cmd);
			break;
	}
}

int usb_gadget_init_udc(void)
{
	return sw_udc_probe();
}

void usb_gadget_exit_udc(void)
{
}

int usb_gadget_handle_interrupts()
{
	return sw_udc_irq(0, the_controller);
}
