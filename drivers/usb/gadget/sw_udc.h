#ifndef  __SW_UDC_H__
#define  __SW_UDC_H__

#include <asm/errno.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/list.h>
#include <usb/lin_gadget_compat.h>

typedef struct sw_udc_ep {
	struct list_head			queue;
	unsigned long				last_io;
	struct usb_gadget			*gadget;
	struct sw_udc				*dev;
	const struct usb_endpoint_descriptor	*desc;
	struct usb_ep				ep;
	u8					num;

	unsigned short				fifo_size;
	u8					bEndpointAddress;
	u8					bmAttributes;

	unsigned				halted : 1;
	unsigned				already_seen : 1;
	unsigned				setup_stage : 1;
} sw_udc_ep_t;

#define  EP0_FIFO_SIZE				64

#define  SW_UDC_EP_FIFO_SIZE			512

#define  SW_UDC_EP_CTRL_INDEX			0x00
#define  SW_UDC_EP_BULK_IN_INDEX		0x01
#define  SW_UDC_EP_BULK_OUT_INDEX		0x02

#define  SW_UDC_FIFO_NUM			1

static const char ep0name [] = "ep0";

static const char *const ep_name[] = {
	ep0name,        /* everyone has ep0 */

	/* sw_udc four bidirectional bulk endpoints */
	"ep1-bulk",
	"ep2-bulk",
	"ep3-bulk",
	"ep4-bulk",
	"ep5-int",
};

#define SW_UDC_ENDPOINTS			6

struct sw_udc_request {
	struct list_head			queue;          /* ep's requests */
	struct usb_request			req;

	u32 is_queue;
};

enum ep0_state {
	EP0_IDLE,
	EP0_IN_DATA_PHASE,
	EP0_OUT_DATA_PHASE,
	EP0_END_XFER,
	EP0_STALL,
};

typedef struct sw_udc_io {
	struct resource *usb_base_res;          /* USB  resources               */
	struct resource *usb_base_req;          /* USB  resources               */
	void __iomem    *usb_vbase;             /* USB  base address    	*/

	struct resource *sram_base_res;         /* SRAM resources               */
	struct resource *sram_base_req;         /* SRAM resources               */
	void __iomem    *sram_vbase;            /* SRAM base address    	*/

	struct resource *clock_base_res;        /* clock resources              */
	struct resource *clock_base_req;        /* clock resources              */
	void __iomem    *clock_vbase;           /* clock base address   	*/

#if 0
	bsp_usbc_t usbc;                        /* usb bsp config               */
	__hdle usb_bsp_hdle;                    /* usb bsp handle               */
#endif

	u32 clk_is_open;                        /* is usb clock open?   */
	struct clk      *sie_clk;               /* SIE clock handle     */
	struct clk      *phy_clk;               /* PHY clock handle     */
	struct clk      *phy0_clk;              /* PHY0 clock handle    */

	long Drv_vbus_Handle;
} sw_udc_io_t;

typedef struct sw_udc {
	struct sw_udc_ep                    	ep[SW_UDC_ENDPOINTS];
	int                                     address;
	struct usb_gadget                   	gadget;
	struct usb_gadget_driver        	    *driver;
	struct sw_udc_request           	    fifo_req;
	u8                                      fifo_buf[SW_UDC_EP_FIFO_SIZE];
	u16                                     devstatus;

	u32                                     port_status;
	int                                     ep0state;

	unsigned                                got_irq : 1;

	unsigned                                req_std : 1;
	unsigned                                req_config : 1;
	unsigned                                req_pending : 1;
	u8                                      vbus;
	struct dentry                       	*regs_info;

	sw_udc_io_t                             *sw_udc_io;
	char                                    driver_name[32];
	u32                                     usbc_no;

	u32                                     stoped;
	u32                                     irq_no;
} sw_udc_t;

enum sw_udc_cmd_e {
	SW_UDC_P_ENABLE = 1,    /* Pull-up enable        */
	SW_UDC_P_DISABLE = 2,   /* Pull-up disable       */
	SW_UDC_P_RESET  = 3,    /* UDC reset, in case of */
};

typedef struct sw_udc_mach_info {
	struct usb_port_info *port_info;
	unsigned int usbc_base;
} sw_udc_mach_info_t;

/* usb OTG mode */
#define  SUNXI_OTG_HOST        0
#define  SUNXI_OTG_DEVICE      1

/* usb device type */
#define  SUNXI_DEVICE_HSDEV    0
#define  SUNXI_DEVICE_FSDEV    1
#define  SUNXI_DEVICE_LSDEV    2

/*  usb transfer type  */
#define  SUNXI_TS_TYPE_IDLE    0
#define  SUNXI_TS_TYPE_CTRL    1
#define  SUNXI_TS_TYPE_ISO     2
#define  SUNXI_TS_TYPE_INT     3
#define  SUNXI_TS_TYPE_BULK    4

/*  usb transfer mode  */
#define  SUNXI_TS_MODE_UNKOWN  0
#define  SUNXI_TS_MODE_LS      1
#define  SUNXI_TS_MODE_FS      2
#define  SUNXI_TS_MODE_HS      3

/* usb Vbus status */
#define  SUNXI_VBUS_STATUS_BELOW_SESSIONEND                 0
#define  SUNXI_VBUS_STATUS_ABOVE_SESSIONEND_BELOW_AVALID    1
#define  SUNXI_VBUS_STATUS_ABOVE_AVALID_BELOW_VBUSVALID     2
#define  SUNXI_VBUS_STATUS_ABOVE_VBUSVALID                  3

/* usb io type */
#define  SUNXI_IO_TYPE_PIO                    0
#define  SUNXI_IO_TYPE_DMA                    1

/* usb ep type */
#define  SUNXI_EP_TYPE_IDLE                   0
#define  SUNXI_EP_TYPE_EP0                    1
#define  SUNXI_EP_TYPE_TX                     2
#define  SUNXI_EP_TYPE_RX                     3

/* usb id type */
#define  SUNXI_ID_TYPE_DISABLE              0
#define  SUNXI_ID_TYPE_HOST                 1
#define  SUNXI_ID_TYPE_DEVICE               2

/* usb vbus valid type */
#define  SUNXI_VBUS_TYPE_DISABLE            0
#define  SUNXI_VBUS_TYPE_LOW                1
#define  SUNXI_VBUS_TYPE_HIGH               2

/* usb a valid source */
#define  SUNXI_A_VALID_SOURCE_UTMI_AVALID        0
#define  SUNXI_A_VALID_SOURCE_UTMI_VBUS          1

/* usb device switch */
#define  SUNXI_DEVICE_SWITCH_OFF    0
#define  SUNXI_DEVICE_SWITCH_ON     1

/* usb fifo config mode */
#define  SUNXI_FIFO_MODE_4K    0
#define  SUNXI_FIFO_MODE_8K    1

/*-------------------------------------------------------------------------*/
/* USB Power Control for Host only  */
#define  SUNXI_BP_POWER_H_HIGH_SPEED_EN			5
#define  SUNXI_BP_POWER_H_HIGH_SPEED_FLAG		4
#define  SUNXI_BP_POWER_H_RESET				3
#define  SUNXI_BP_POWER_H_RESUME			2
#define  SUNXI_BP_POWER_H_SUSPEND			1
#define  SUNXI_BP_POWER_H_SUEPEND_EN			0

/* USB Power Control for device only  */
#define  SUNXI_BP_POWER_D_ISO_UPDATE_EN			7
#define  SUNXI_BP_POWER_D_SOFT_CONNECT			6
#define  SUNXI_BP_POWER_D_HIGH_SPEED_EN			5
#define  SUNXI_BP_POWER_D_HIGH_SPEED_FLAG		4
#define  SUNXI_BP_POWER_D_RESET_FLAG			3
#define  SUNXI_BP_POWER_D_RESUME			2
#define  SUNXI_BP_POWER_D_SUSPEND			1
#define  SUNXI_BP_POWER_D_ENABLE_SUSPENDM		0

/* interrupt flags for ep0 and the Tx ep1~4 */
#define  SUNXI_BP_INTTx_FLAG_EP5			5
#define  SUNXI_BP_INTTx_FLAG_EP4			4
#define  SUNXI_BP_INTTx_FLAG_EP3			3
#define  SUNXI_BP_INTTx_FLAG_EP2		    	2
#define  SUNXI_BP_INTTx_FLAG_EP1  			1
#define  SUNXI_BP_INTTx_FLAG_EP0			0

/* interrupt flags for Rx ep1~4 */
#define  SUNXI_BP_INTRx_FLAG_EP5			5
#define  SUNXI_BP_INTRx_FLAG_EP4			4
#define  SUNXI_BP_INTRx_FLAG_EP3			3
#define  SUNXI_BP_INTRx_FLAG_EP2			2
#define  SUNXI_BP_INTRx_FLAG_EP1			1

/* interrupt enable for Tx ep0~4 */
#define  SUNXI_BP_INTTxE_EN_EP5				5
#define  SUNXI_BP_INTTxE_EN_EP4				4
#define  SUNXI_BP_INTTxE_EN_EP3				3
#define  SUNXI_BP_INTTxE_EN_EP2				2
#define  SUNXI_BP_INTTxE_EN_EP1				1
#define  SUNXI_BP_INTTxE_EN_EP0				0

/* interrupt enable for Rx ep1~4 */
#define  SUNXI_BP_INTRxE_EN_EP5				5
#define  SUNXI_BP_INTRxE_EN_EP4				4
#define  SUNXI_BP_INTRxE_EN_EP3				3
#define  SUNXI_BP_INTRxE_EN_EP2			    	2
#define  SUNXI_BP_INTRxE_EN_EP1  		    	1

/* USB interrupt */
#define  SUNXI_BP_INTUSB_VBUS_ERROR			7
#define  SUNXI_BP_INTUSB_SESSION_REQ			6
#define  SUNXI_BP_INTUSB_DISCONNECT			5
#define  SUNXI_BP_INTUSB_CONNECT			4
#define  SUNXI_BP_INTUSB_SOF				3
#define  SUNXI_BP_INTUSB_RESET				2
#define  SUNXI_BP_INTUSB_RESUME				1
#define  SUNXI_BP_INTUSB_SUSPEND			0

/* USB interrupt enable */
#define  SUNXI_BP_INTUSBE_EN_VBUS_ERROR			7
#define  SUNXI_BP_INTUSBE_EN_SESSION_REQ		6
#define  SUNXI_BP_INTUSBE_EN_DISCONNECT			5
#define  SUNXI_BP_INTUSBE_EN_CONNECT			4
#define  SUNXI_BP_INTUSBE_EN_SOF			3
#define  SUNXI_BP_INTUSBE_EN_RESET			2
#define  SUNXI_BP_INTUSBE_EN_RESUME			1
#define  SUNXI_BP_INTUSBE_EN_SUSPEND			0

/* Test Mode Control */
#define  SUNXI_BP_TMCTL_FORCE_HOST               	7
#define  SUNXI_BP_TMCTL_FIFO_ACCESS              	6
#define  SUNXI_BP_TMCTL_FORCE_FS                 	5
#define  SUNXI_BP_TMCTL_FORCE_HS                 	4
#define  SUNXI_BP_TMCTL_TEST_PACKET              	3
#define  SUNXI_BP_TMCTL_TEST_K                   	2
#define  SUNXI_BP_TMCTL_TEST_J                   	1
#define  SUNXI_BP_TMCTL_TEST_SE0_NAK             	0

/* Tx Max packet */
#define  SUNXI_BP_TXMAXP_PACKET_COUNT            	11
#define  SUNXI_BP_TXMAXP_MAXIMUM_PAYLOAD         	0

/* Control and Status Register for ep0 for Host only */
#define  SUNXI_BP_CSR0_H_DisPing 			11
#define  SUNXI_BP_CSR0_H_FlushFIFO			8
#define  SUNXI_BP_CSR0_H_NAK_Timeout			7
#define  SUNXI_BP_CSR0_H_StatusPkt			6
#define  SUNXI_BP_CSR0_H_ReqPkt				5
#define  SUNXI_BP_CSR0_H_Error				4
#define  SUNXI_BP_CSR0_H_SetupPkt			3
#define  SUNXI_BP_CSR0_H_RxStall			2
#define  SUNXI_BP_CSR0_H_TxPkRdy			1
#define  SUNXI_BP_CSR0_H_RxPkRdy			0

/* Control and Status Register for ep0 for device only */
#define  SUNXI_BP_CSR0_D_FLUSH_FIFO			8
#define  SUNXI_BP_CSR0_D_SERVICED_SETUP_END		7
#define  SUNXI_BP_CSR0_D_SERVICED_RX_PKT_READY   	6
#define  SUNXI_BP_CSR0_D_SEND_STALL			5
#define  SUNXI_BP_CSR0_D_SETUP_END			4
#define  SUNXI_BP_CSR0_D_DATA_END			3
#define  SUNXI_BP_CSR0_D_SENT_STALL			2
#define  SUNXI_BP_CSR0_D_TX_PKT_READY			1
#define  SUNXI_BP_CSR0_D_RX_PKT_READY			0

/* Tx ep Control and Status Register for Host only */
#define  SUNXI_BP_TXCSR_H_AUTOSET			15
#define  SUNXI_BP_TXCSR_H_RESERVED			14
#define  SUNXI_BP_TXCSR_H_MODE				13
#define  SUNXI_BP_TXCSR_H_DMA_REQ_EN			12
#define  SUNXI_BP_TXCSR_H_FORCE_DATA_TOGGLE		11
#define  SUNXI_BP_TXCSR_H_DMA_REQ_MODE			10
#define  SUNXI_BP_TXCSR_H_DATA_TOGGLE_WR_EN      	9
#define  SUNXI_BP_TXCSR_H_DATA_TOGGLE            	8
#define  SUNXI_BP_TXCSR_H_NAK_TIMEOUT			7
#define  SUNXI_BP_TXCSR_H_CLEAR_DATA_TOGGLE		6
#define  SUNXI_BP_TXCSR_H_TX_STALL			5
#define  SUNXI_BP_TXCSR_H_FLUSH_FIFO			3
#define  SUNXI_BP_TXCSR_H_ERROR				2
#define  SUNXI_BP_TXCSR_H_FIFO_NOT_EMPTY 		1
#define  SUNXI_BP_TXCSR_H_TX_READY			0

#define  SUNXI_TXCSR_H_WZC_BITS                  	((1 << SUNXI_BP_TXCSR_H_NAK_TIMEOUT) | (1 << SUNXI_BP_TXCSR_H_TX_STALL) \
                                                	| (1 << SUNXI_BP_TXCSR_H_ERROR) | (1 << SUNXI_BP_TXCSR_H_FIFO_NOT_EMPTY))

/* Tx ep Control and Status Register for Device only */
#define  SUNXI_BP_TXCSR_D_AUTOSET			15
#define  SUNXI_BP_TXCSR_D_ISO				14
#define  SUNXI_BP_TXCSR_D_MODE				13
#define  SUNXI_BP_TXCSR_D_DMA_REQ_EN			12
#define  SUNXI_BP_TXCSR_D_FORCE_DATA_TOGGLE		11
#define  SUNXI_BP_TXCSR_D_DMA_REQ_MODE			10
#define  SUNXI_BP_TXCSR_D_INCOMPLETE			7
#define  SUNXI_BP_TXCSR_D_CLEAR_DATA_TOGGLE		6
#define  SUNXI_BP_TXCSR_D_SENT_STALL			5
#define  SUNXI_BP_TXCSR_D_SEND_STALL			4
#define  SUNXI_BP_TXCSR_D_FLUSH_FIFO			3
#define  SUNXI_BP_TXCSR_D_UNDER_RUN			2
#define  SUNXI_BP_TXCSR_D_FIFO_NOT_EMPTY 		1
#define  SUNXI_BP_TXCSR_D_TX_READY			0

/* Rx Max Packet */
#define  SUNXI_BP_RXMAXP_PACKET_COUNT            	11
#define  SUNXI_BP_RXMAXP_MAXIMUM_PAYLOAD         	0

/* Rx ep Control and Status Register for Host only */
#define  SUNXI_BP_RXCSR_H_AUTO_CLEAR			15
#define  SUNXI_BP_RXCSR_H_AUTO_REQ			14
#define  SUNXI_BP_RXCSR_H_DMA_REQ_EN			13
#define  SUNXI_BP_RXCSR_H_DISNYET                	12  /* dis nyet */
#define  SUNXI_BP_RXCSR_H_PID_ERROR			12  /* pid error */
#define  SUNXI_BP_RXCSR_H_DMA_REQ_MODE		    	11
#define  SUNXI_BP_RXCSR_H_DATA_TOGGLE_WR_EN      	10
#define  SUNXI_BP_RXCSR_H_DATA_TOGGLE            	9
#define  SUNXI_BP_RXCSR_H_INCOMPLETE			8
#define  SUNXI_BP_RXCSR_H_CLEAR_DATA_TOGGLE	    	7
#define  SUNXI_BP_RXCSR_H_RX_STALL			6
#define  SUNXI_BP_RXCSR_H_REQ_PACKET			5
#define  SUNXI_BP_RXCSR_H_FLUSH_FIFO			4
#define  SUNXI_BP_RXCSR_H_DATA_ERR               	3   /* iso */
#define  SUNXI_BP_RXCSR_H_NAK_TIMEOUT		    	3   /* bulk */
#define  SUNXI_BP_RXCSR_H_ERROR				2
#define  SUNXI_BP_RXCSR_H_FIFO_FULL			1
#define  SUNXI_BP_RXCSR_H_RX_PKT_READY		    	0

#define  SUNXI_RXCSR_H_WZC_BITS                  	((1 << SUNXI_BP_RXCSR_H_RX_STALL) | (1 << SUNXI_BP_RXCSR_H_ERROR) \
							| (1 << SUNXI_BP_RXCSR_H_DATA_ERR) | (1 << SUNXI_BP_RXCSR_H_RX_PKT_READY))

/* Rx ep Control and Status Register for Device only */
#define  SUNXI_BP_RXCSR_D_AUTO_CLEAR			15
#define  SUNXI_BP_RXCSR_D_ISO				14
#define  SUNXI_BP_RXCSR_D_DMA_REQ_EN			13
#define  SUNXI_BP_RXCSR_D_DISABLE_NYET		    	12
#define  SUNXI_BP_RXCSR_D_DMA_REQ_MODE		    	11

#define  SUNXI_BP_RXCSR_D_INCOMPLETE			8
#define  SUNXI_BP_RXCSR_D_CLEAR_DATA_TOGGLE	    	7
#define  SUNXI_BP_RXCSR_D_SENT_STALL			6
#define  SUNXI_BP_RXCSR_D_SEND_STALL			5
#define  SUNXI_BP_RXCSR_D_FLUSH_FIFO			4
#define  SUNXI_BP_RXCSR_D_DATA_ERROR			3
#define  SUNXI_BP_RXCSR_D_OVERRUN			2
#define  SUNXI_BP_RXCSR_D_FIFO_FULL			1
#define  SUNXI_BP_RXCSR_D_RX_PKT_READY		    	0

/* Tx Type Register for host only */
#define  SUNXI_BP_TXTYPE_SPEED	                	6
#define  SUNXI_BP_TXTYPE_PROROCOL	            	4
#define  SUNXI_BP_TXTYPE_TARGET_EP_NUM           	0

/* Rx Type Register for host only */
#define  SUNXI_BP_RXTYPE_SPEED		            	6
#define  SUNXI_BP_RXTYPE_PROROCOL	           	4
#define  SUNXI_BP_RXTYPE_TARGET_EP_NUM           	0

/* Core Configueation */
#define  SUNXI_BP_CONFIGDATA_MPRXE               	7
#define  SUNXI_BP_CONFIGDATA_MPTXE               	6
#define  SUNXI_BP_CONFIGDATA_BIGENDIAN		    	5
#define  SUNXI_BP_CONFIGDATA_HBRXE			4
#define  SUNXI_BP_CONFIGDATA_HBTXE			3
#define  SUNXI_BP_CONFIGDATA_DYNFIFO_SIZING	    	2
#define  SUNXI_BP_CONFIGDATA_SOFTCONE		    	1
#define  SUNXI_BP_CONFIGDATA_UTMI_DATAWIDTH	    	0

/* OTG Device Control */
#define  SUNXI_BP_DEVCTL_B_DEVICE			7
#define  SUNXI_BP_DEVCTL_FS_DEV				6
#define  SUNXI_BP_DEVCTL_LS_DEV				5

#define  SUNXI_BP_DEVCTL_VBUS				3
#define  SUNXI_BP_DEVCTL_HOST_MODE			2
#define  SUNXI_BP_DEVCTL_HOST_REQ			1
#define  SUNXI_BP_DEVCTL_SESSION			0

/* Tx EP FIFO size control */
#define  SUNXI_BP_TXFIFOSZ_DPB				4
#define  SUNXI_BP_TXFIFOSZ_SZ				0

/* Rx EP FIFO size control */
#define  SUNXI_BP_RXFIFOSZ_DPB				4
#define  SUNXI_BP_RXFIFOSZ_SZ				0

/* vendor0 */
#define  SUNXI_BP_VEND0_DRQ_SEL				1
#define  SUNXI_BP_VEND0_BUS_SEL				0

/* hub address */
#define  SUNXI_BP_HADDR_MULTI_TT			7

/* Interface Status and Control */
#define  SUNXI_BP_ISCR_VBUS_VALID_FROM_DATA		30
#define  SUNXI_BP_ISCR_VBUS_VALID_FROM_VBUS		29
#define  SUNXI_BP_ISCR_EXT_ID_STATUS			28
#define  SUNXI_BP_ISCR_EXT_DM_STATUS			27
#define  SUNXI_BP_ISCR_EXT_DP_STATUS			26
#define  SUNXI_BP_ISCR_MERGED_VBUS_STATUS		25
#define  SUNXI_BP_ISCR_MERGED_ID_STATUS			24

#define  SUNXI_BP_ISCR_ID_PULLUP_EN			17
#define  SUNXI_BP_ISCR_DPDM_PULLUP_EN			16
#define  SUNXI_BP_ISCR_FORCE_ID				14
#define  SUNXI_BP_ISCR_FORCE_VBUS_VALID			12
#define  SUNXI_BP_ISCR_VBUS_VALID_SRC			10

#define  SUNXI_BP_ISCR_HOSC_EN                 		7
#define  SUNXI_BP_ISCR_VBUS_CHANGE_DETECT      		6
#define  SUNXI_BP_ISCR_ID_CHANGE_DETECT        		5
#define  SUNXI_BP_ISCR_DPDM_CHANGE_DETECT      		4
#define  SUNXI_BP_ISCR_IRQ_ENABLE              		3
#define  SUNXI_BP_ISCR_VBUS_CHANGE_DETECT_EN   		2
#define  SUNXI_BP_ISCR_ID_CHANGE_DETECT_EN     		1
#define  SUNXI_BP_ISCR_DPDM_CHANGE_DETECT_EN   		0

/* Register offset */
#define  SUNXI_FADDR			0x0098
#define  SUNXI_PCTL		    	0x0040
#define  SUNXI_INTTx		    	0x0044
#define  SUNXI_INTRx		    	0x0046
#define  SUNXI_INTTxE		    	0x0048
#define  SUNXI_INTRxE		    	0x004A
#define  SUNXI_INTUSB		    	0x004C
#define  SUNXI_INTUSBE         	    	0x0050
#define  SUNXI_FRNUM		    	0x0054
#define  SUNXI_EPIND		    	0x0042
#define  SUNXI_TMCTL		    	0x007C

#define  SUNXI_TXMAXP		    	0x0080
#define  SUNXI_CSR0		    	0x0082
#define  SUNXI_TXCSR		    	0x0082
#define  SUNXI_RXMAXP		    	0x0084
#define  SUNXI_RXCSR		    	0x0086
#define  SUNXI_COUNT0		    	0x0088
#define  SUNXI_RXCOUNT		    	0x0088
#define  SUNXI_EP0TYPE		    	0x008C
#define  SUNXI_TXTYPE		    	0x008C
#define  SUNXI_NAKLIMIT0	    	0x008D
#define  SUNXI_TXINTERVAL      	    	0x008D
#define  SUNXI_RXTYPE		    	0x008E
#define  SUNXI_RXINTERVAL	    	0x008F

#define  SUNXI_CONFIGDATA		0x00c0

#define  SUNXI_EPFIFO0		    	0x0000
#define  SUNXI_EPFIFO1		   	0x0004
#define  SUNXI_EPFIFO2		    	0x0008
#define  SUNXI_EPFIFO3		    	0x000C
#define  SUNXI_EPFIFO4		    	0x0010
#define  SUNXI_EPFIFO5		    	0x0014
#define  SUNXI_EPFIFOx(n)	    	(0x0000 + (n<<2))

#define  SUNXI_DEVCTL		    	0x0041

#define  SUNXI_TXFIFOSZ	    		0x0090
#define  SUNXI_RXFIFOSZ	    		0x0094
#define  SUNXI_TXFIFOAD	    		0x0092
#define  SUNXI_RXFIFOAD	    		0x0096

#define  SUNXI_VEND0		    	0x0043
#define  SUNXI_VEND1		    	0x007D
#define  SUNXI_VEND3		    	0x007E

#define  SUNXI_EPINFO		    	0x0078
#define  SUNXI_RAMINFO		    	0x0079
#define  SUNXI_LINKINFO	    		0x007A
#define  SUNXI_VPLEN		    	0x007B
#define  SUNXI_HSEOF		    	0x007C
#define  SUNXI_FSEOF		    	0x007D
#define  SUNXI_LSEOF		    	0x007E

#define  SUNXI_FADDR0          		0x0098
#define  SUNXI_HADDR0          		0x009A
#define  SUNXI_HPORT0          		0x009B
#define  SUNXI_TXFADDRx 		0x0098
#define  SUNXI_TXHADDRx			0x009A
#define  SUNXI_TXHPORTx			0x009B
#define  SUNXI_RXFADDRx			0x009C
#define  SUNXI_RXHADDRx			0x009E
#define  SUNXI_RXHPORTx			0x009F

#define  SUNXI_RPCOUNT			0x008A

#define  SUNXI_ISCR            		0x0400
#define  SUNXI_PHYCTL          		0x0404
#define  SUNXI_PHYBIST         		0x0408
#define  SUNXI_PHYTUNE         		0x040c

/* interrupt flags for ep0 and the Tx ep1~4 */
#define  SUNXI_INTTx_FLAG_EP5           (1 << SUNXI_BP_INTTx_FLAG_EP5)
#define  SUNXI_INTTx_FLAG_EP4           (1 << SUNXI_BP_INTTx_FLAG_EP4)
#define  SUNXI_INTTx_FLAG_EP3           (1 << SUNXI_BP_INTTx_FLAG_EP3)
#define  SUNXI_INTTx_FLAG_EP2           (1 << SUNXI_BP_INTTx_FLAG_EP2)
#define  SUNXI_INTTx_FLAG_EP1           (1 << SUNXI_BP_INTTx_FLAG_EP1)
#define  SUNXI_INTTx_FLAG_EP0           (1 << SUNXI_BP_INTTx_FLAG_EP0)

/* interrupt flags for Rx ep1~4 */
#define  SUNXI_INTRx_FLAG_EP5           (1 << SUNXI_BP_INTRx_FLAG_EP5)
#define  SUNXI_INTRx_FLAG_EP4           (1 << SUNXI_BP_INTRx_FLAG_EP4)
#define  SUNXI_INTRx_FLAG_EP3           (1 << SUNXI_BP_INTRx_FLAG_EP3)
#define  SUNXI_INTRx_FLAG_EP2           (1 << SUNXI_BP_INTRx_FLAG_EP2)
#define  SUNXI_INTRx_FLAG_EP1           (1 << SUNXI_BP_INTRx_FLAG_EP1)

/* USB interrupt */
#define  SUNXI_INTUSB_VBUS_ERROR         (1 << SUNXI_BP_INTUSB_VBUS_ERROR)
#define  SUNXI_INTUSB_SESSION_REQ        (1 << SUNXI_BP_INTUSB_SESSION_REQ)
#define  SUNXI_INTUSB_DISCONNECT         (1 << SUNXI_BP_INTUSB_DISCONNECT)
#define  SUNXI_INTUSB_CONNECT            (1 << SUNXI_BP_INTUSB_CONNECT)
#define  SUNXI_INTUSB_SOF                (1 << SUNXI_BP_INTUSB_SOF)
#define  SUNXI_INTUSB_RESET              (1 << SUNXI_BP_INTUSB_RESET)
#define  SUNXI_INTUSB_RESUME             (1 << SUNXI_BP_INTUSB_RESUME)
#define  SUNXI_INTUSB_SUSPEND            (1 << SUNXI_BP_INTUSB_SUSPEND)

#endif   //__SW_UDC_H__ 
