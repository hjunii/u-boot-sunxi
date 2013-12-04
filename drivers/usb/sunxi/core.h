/**
 * core.h
 */

#ifndef __DRIVERS_USB_SUNXI_CORE_H
#define __DRIVERS_USB_SUNXI_CORE_H

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/compiler.h>
#include "misc.h"

/* Global constants */
#define SUNXI_EP0_BOUNCE_SIZE	512
#define SUNXI_ENDPOINTS_NUM	6

#define SUNXI_EVENT_BUFFERS_SIZE	PAGE_SIZE
#define SUNXI_EVENT_TYPE_MASK	0xfe

#define SUNXI_EVENT_TYPE_DEV	0
#define SUNXI_EVENT_TYPE_CARKIT	3
#define SUNXI_EVENT_TYPE_I2C	4

#define SUNXI_DEVICE_EVENT_DISCONNECT		0
#define SUNXI_DEVICE_EVENT_RESET			1
#define SUNXI_DEVICE_EVENT_CONNECT_DONE		2
#define SUNXI_DEVICE_EVENT_LINK_STATUS_CHANGE	3
#define SUNXI_DEVICE_EVENT_WAKEUP		4
#define SUNXI_DEVICE_EVENT_EOPF			6
#define SUNXI_DEVICE_EVENT_SOF			7
#define SUNXI_DEVICE_EVENT_ERRATIC_ERROR		9
#define SUNXI_DEVICE_EVENT_CMD_CMPL		10
#define SUNXI_DEVICE_EVENT_OVERFLOW		11

#define SUNXI_GEVNTCOUNT_MASK	0xfffc
#define SUNXI_GSNPSID_MASK	0xffff0000
#define SUNXI_GSNPSREV_MASK	0xffff

/* Global Registers */
#define SUNXI_PCTL              0x0040
#define SUNXI_ISCR		0x0400
#define SUNXI_PHYCTL          	0x0404
#define SUNXI_PHYBIST         	0x0408
#define SUNXI_PHYTUNE         	0x040c

#define SUNXI_GSBUSCFG0		0xc100
#define SUNXI_GSBUSCFG1		0xc104
#define SUNXI_GTXTHRCFG		0xc108
#define SUNXI_GRXTHRCFG		0xc10c
#define SUNXI_GCTL		0xc110
#define SUNXI_GEVTEN		0xc114
#define SUNXI_GSTS		0xc118
#define SUNXI_GSNPSID		0xc120
#define SUNXI_GGPIO		0xc124
#define SUNXI_GUID		0xc128
#define SUNXI_GUCTL		0xc12c
#define SUNXI_GBUSERRADDR0	0xc130
#define SUNXI_GBUSERRADDR1	0xc134
#define SUNXI_GPRTBIMAP0		0xc138
#define SUNXI_GPRTBIMAP1		0xc13c
#define SUNXI_GHWPARAMS0		0xc140
#define SUNXI_GHWPARAMS1		0xc144
#define SUNXI_GHWPARAMS2		0xc148
#define SUNXI_GHWPARAMS3		0xc14c
#define SUNXI_GHWPARAMS4		0xc150
#define SUNXI_GHWPARAMS5		0xc154
#define SUNXI_GHWPARAMS6		0xc158
#define SUNXI_GHWPARAMS7		0xc15c
#define SUNXI_GDBGFIFOSPACE	0xc160
#define SUNXI_GDBGLTSSM		0xc164
#define SUNXI_GPRTBIMAP_HS0	0xc180
#define SUNXI_GPRTBIMAP_HS1	0xc184
#define SUNXI_GPRTBIMAP_FS0	0xc188
#define SUNXI_GPRTBIMAP_FS1	0xc18c

#define SUNXI_GUSB2PHYCFG(n)	(0xc200 + (n * 0x04))
#define SUNXI_GUSB2I2CCTL(n)	(0xc240 + (n * 0x04))

#define SUNXI_GUSB2PHYACC(n)	(0xc280 + (n * 0x04))

#define SUNXI_GUSB3PIPECTL(n)	(0xc2c0 + (n * 0x04))

#define SUNXI_GTXFIFOSIZ(n)	(0xc300 + (n * 0x04))
#define SUNXI_GRXFIFOSIZ(n)	(0xc380 + (n * 0x04))

#define SUNXI_GEVNTADRLO(n)	(0xc400 + (n * 0x10))
#define SUNXI_GEVNTADRHI(n)	(0xc404 + (n * 0x10))
#define SUNXI_GEVNTSIZ(n)	(0xc408 + (n * 0x10))
#define SUNXI_GEVNTCOUNT(n)	(0xc40c + (n * 0x10))

#define SUNXI_GHWPARAMS8		0xc600

/* Device Registers */
#define SUNXI_DCFG		0xc700
#define SUNXI_DCTL		0xc704
#define SUNXI_DEVTEN		0xc708
#define SUNXI_DSTS		0xc70c
#define SUNXI_DGCMDPAR		0xc710
#define SUNXI_DGCMD		0xc714
#define SUNXI_DALEPENA		0xc720
#define SUNXI_DEPCMDPAR2(n)	(0xc800 + (n * 0x10))
#define SUNXI_DEPCMDPAR1(n)	(0xc804 + (n * 0x10))
#define SUNXI_DEPCMDPAR0(n)	(0xc808 + (n * 0x10))
#define SUNXI_DEPCMD(n)		(0xc80c + (n * 0x10))

/* OTG Registers */
#define SUNXI_OCFG		0xcc00
#define SUNXI_OCTL		0xcc04
#define SUNXI_OEVTEN		0xcc08
#define SUNXI_OSTS		0xcc0C

/* Bit fields */

/* Global Configuration Register */
#define SUNXI_GCTL_PWRDNSCALE(n)	(n << 19)
#define SUNXI_GCTL_U2RSTECN	(1 << 16)
#define SUNXI_GCTL_RAMCLKSEL(x)	((x & SUNXI_GCTL_CLK_MASK) << 6)
#define SUNXI_GCTL_CLK_BUS	(0)
#define SUNXI_GCTL_CLK_PIPE	(1)
#define SUNXI_GCTL_CLK_PIPEHALF	(2)
#define SUNXI_GCTL_CLK_MASK	(3)

#define SUNXI_GCTL_PRTCAP(n)	(((n) & (3 << 12)) >> 12)
#define SUNXI_GCTL_PRTCAPDIR(n)	(n << 12)
#define SUNXI_GCTL_PRTCAP_DPDM_CHANGE_DETECT 4
#define SUNXI_GCTL_PRTCAP_ID_CHANGE_DETECT 5
#define SUNXI_GCTL_PRTCAP_VBUS_CHANGE_DETECT 6
#define SUNXI_GCTL_PRTCAP_FORCE_ID 14
#define SUNXI_GCTL_PRTCAP_HOST	1
#define SUNXI_GCTL_PRTCAP_DEVICE	2
#define SUNXI_GCTL_PRTCAP_OTG	3

#define SUNXI_GCTL_CORESOFTRESET	(1 << 11)
#define SUNXI_GCTL_SCALEDOWN(n)	(n << 4)
#define SUNXI_GCTL_DISSCRAMBLE	(1 << 3)
#define SUNXI_GCTL_DSBLCLKGTNG	(1 << 0)

/* Global USB2 PHY Configuration Register */
#define SUNXI_GUSB2PHYCFG_PHYSOFTRST (1 << 31)
#define SUNXI_GUSB2PHYCFG_SUSPHY	(1 << 6)

/* Global USB3 PIPE Control Register */
#define SUNXI_GUSB3PIPECTL_PHYSOFTRST (1 << 31)
#define SUNXI_GUSB3PIPECTL_SUSPHY (1 << 17)

/* Global HWPARAMS1 Register */
#define SUNXI_GHWPARAMS1_EN_PWROPT(n)	((n & (3 << 24)) >> 24)
#define SUNXI_GHWPARAMS1_EN_PWROPT_NO	0
#define SUNXI_GHWPARAMS1_EN_PWROPT_CLK	1

/* Device Configuration Register */
#define SUNXI_DCFG_DEVADDR(addr)	((addr) << 3)
#define SUNXI_DCFG_DEVADDR_MASK	SUNXI_DCFG_DEVADDR(0x7f)

/* Device Configuration Register */
#define SUNXI_DCFG_LPM_CAP	(1 << 22)

#define SUNXI_DCFG_SPEED_MASK	(7 << 0)
#define SUNXI_DCFG_SUPERSPEED	(4 << 0)
#define SUNXI_DCFG_HIGHSPEED	(0 << 0)
#define SUNXI_DCFG_FULLSPEED2	(1 << 0)
#define SUNXI_DCFG_LOWSPEED	(2 << 0)
#define SUNXI_DCFG_FULLSPEED1	(3 << 0)

/* Device Control Register */
#define SUNXI_DCTL_RUN_STOP	(1 << 31)
#define SUNXI_DCTL_CSFTRST	(1 << 30)
#define SUNXI_DCTL_LSFTRST	(1 << 29)

#define SUNXI_DCTL_HIRD_THRES_MASK	(0x1f << 24)
#define SUNXI_DCTL_HIRD_THRES(n)	(((n) & SUNXI_DCTL_HIRD_THRES_MASK) >> 24)

#define SUNXI_DCTL_APPL1RES	(1 << 23)
/* These apply for core versions 1.87a and earlier */
#define SUNXI_DCTL_TRGTULST_MASK		(0x0f << 17)
#define SUNXI_DCTL_TRGTULST(n)		((n) << 17)
#define SUNXI_DCTL_TRGTULST_U2		(SUNXI_DCTL_TRGTULST(2))
#define SUNXI_DCTL_TRGTULST_U3		(SUNXI_DCTL_TRGTULST(3))
#define SUNXI_DCTL_TRGTULST_SS_DIS	(SUNXI_DCTL_TRGTULST(4))
#define SUNXI_DCTL_TRGTULST_RX_DET	(SUNXI_DCTL_TRGTULST(5))
#define SUNXI_DCTL_TRGTULST_SS_INACT	(SUNXI_DCTL_TRGTULST(6))

/* These apply for core versions 1.94a and later */
#define SUNXI_DCTL_KEEP_CONNECT	(1 << 19)
#define SUNXI_DCTL_L1_HIBER_EN	(1 << 18)
#define SUNXI_DCTL_CRS		(1 << 17)
#define SUNXI_DCTL_CSS		(1 << 16)

#define SUNXI_DCTL_INITU2ENA	(1 << 12)
#define SUNXI_DCTL_ACCEPTU2ENA	(1 << 11)
#define SUNXI_DCTL_INITU1ENA	(1 << 10)
#define SUNXI_DCTL_ACCEPTU1ENA	(1 << 9)
#define SUNXI_DCTL_TSTCTRL_MASK	(0xf << 1)

#define SUNXI_DCTL_ULSTCHNGREQ_MASK	(0x0f << 5)
#define SUNXI_DCTL_ULSTCHNGREQ(n) (((n) << 5) & SUNXI_DCTL_ULSTCHNGREQ_MASK)

#define SUNXI_DCTL_ULSTCHNG_NO_ACTION	(SUNXI_DCTL_ULSTCHNGREQ(0))
#define SUNXI_DCTL_ULSTCHNG_SS_DISABLED	(SUNXI_DCTL_ULSTCHNGREQ(4))
#define SUNXI_DCTL_ULSTCHNG_RX_DETECT	(SUNXI_DCTL_ULSTCHNGREQ(5))
#define SUNXI_DCTL_ULSTCHNG_SS_INACTIVE	(SUNXI_DCTL_ULSTCHNGREQ(6))
#define SUNXI_DCTL_ULSTCHNG_RECOVERY	(SUNXI_DCTL_ULSTCHNGREQ(8))
#define SUNXI_DCTL_ULSTCHNG_COMPLIANCE	(SUNXI_DCTL_ULSTCHNGREQ(10))
#define SUNXI_DCTL_ULSTCHNG_LOOPBACK	(SUNXI_DCTL_ULSTCHNGREQ(11))

/* Device Event Enable Register */
#define SUNXI_DEVTEN_VNDRDEVTSTRCVEDEN	(1 << 12)
#define SUNXI_DEVTEN_EVNTOVERFLOWEN	(1 << 11)
#define SUNXI_DEVTEN_CMDCMPLTEN		(1 << 10)
#define SUNXI_DEVTEN_ERRTICERREN		(1 << 9)
#define SUNXI_DEVTEN_SOFEN		(1 << 7)
#define SUNXI_DEVTEN_EOPFEN		(1 << 6)
#define SUNXI_DEVTEN_WKUPEVTEN		(1 << 4)
#define SUNXI_DEVTEN_ULSTCNGEN		(1 << 3)
#define SUNXI_DEVTEN_CONNECTDONEEN	(1 << 2)
#define SUNXI_DEVTEN_USBRSTEN		(1 << 1)
#define SUNXI_DEVTEN_DISCONNEVTEN	(1 << 0)

/* Device Status Register */
#define SUNXI_DSTS_DCNRD			(1 << 29)

/* Device Status Register */
#define SUNXI_DSTS_PWRUPREQ		(1 << 24)
#define SUNXI_DSTS_COREIDLE		(1 << 23)
#define SUNXI_DSTS_DEVCTRLHLT		(1 << 22)

#define SUNXI_DSTS_USBLNKST_MASK		(0x0f << 18)
#define SUNXI_DSTS_USBLNKST(n)		(((n) & SUNXI_DSTS_USBLNKST_MASK) >> 18)

#define SUNXI_DSTS_RXFIFOEMPTY		(1 << 17)

#define SUNXI_DSTS_SOFFN_MASK		(0x3ff << 3)
#define SUNXI_DSTS_SOFFN(n)		(((n) & SUNXI_DSTS_SOFFN_MASK) >> 3)

#define SUNXI_DSTS_CONNECTSPD		(7 << 0)

#define SUNXI_DSTS_SUPERSPEED		(4 << 0)
#define SUNXI_DSTS_HIGHSPEED		(0 << 0)
#define SUNXI_DSTS_FULLSPEED2		(1 << 0)
#define SUNXI_DSTS_LOWSPEED		(2 << 0)
#define SUNXI_DSTS_FULLSPEED1		(3 << 0)

/* Device Generic Command Register */
#define SUNXI_DGCMD_SET_LMP		0x01
#define SUNXI_DGCMD_SET_PERIODIC_PAR	0x02
#define SUNXI_DGCMD_XMIT_FUNCTION	0x03
#define SUNXI_DGCMD_SELECTED_FIFO_FLUSH	0x09
#define SUNXI_DGCMD_ALL_FIFO_FLUSH	0x0a
#define SUNXI_DGCMD_SET_ENDPOINT_NRDY	0x0c
#define SUNXI_DGCMD_RUN_SOC_BUS_LOOPBACK	0x10

#define SUNXI_DGCMD_STATUS(n)		(((n) >> 15) & 1)
#define SUNXI_DGCMD_CMDACT		(1 << 10)
#define SUNXI_DGCMD_CMDIOC		(1 << 8)

/* Device Generic Command Parameter Register */
#define SUNXI_DGCMDPAR_FORCE_LINKPM_ACCEPT	(1 << 0)
#define SUNXI_DGCMDPAR_FIFO_NUM(n)		((n) << 0)
#define SUNXI_DGCMDPAR_RX_FIFO			(0 << 5)
#define SUNXI_DGCMDPAR_TX_FIFO			(1 << 5)
#define SUNXI_DGCMDPAR_LOOPBACK_DIS		(0 << 0)
#define SUNXI_DGCMDPAR_LOOPBACK_ENA		(1 << 0)

/* Device Endpoint Command Register */
#define SUNXI_DEPCMD_PARAM_SHIFT		16
#define SUNXI_DEPCMD_PARAM(x)		(x << SUNXI_DEPCMD_PARAM_SHIFT)
#define SUNXI_DEPCMD_GET_RSC_IDX(x)	((x >> SUNXI_DEPCMD_PARAM_SHIFT) & 0x7f)
#define SUNXI_DEPCMD_STATUS_MASK		(0x0f << 12)
#define SUNXI_DEPCMD_STATUS(x)		((x & SUNXI_DEPCMD_STATUS_MASK) >> 12)
#define SUNXI_DEPCMD_HIPRI_FORCERM	(1 << 11)
#define SUNXI_DEPCMD_CMDACT		(1 << 10)
#define SUNXI_DEPCMD_CMDIOC		(1 << 8)

#define SUNXI_DEPCMD_DEPSTARTCFG		(0x09 << 0)
#define SUNXI_DEPCMD_ENDTRANSFER		(0x08 << 0)
#define SUNXI_DEPCMD_UPDATETRANSFER	(0x07 << 0)
#define SUNXI_DEPCMD_STARTTRANSFER	(0x06 << 0)
#define SUNXI_DEPCMD_CLEARSTALL		(0x05 << 0)
#define SUNXI_DEPCMD_SETSTALL		(0x04 << 0)
#define SUNXI_DEPCMD_GETSEQNUMBER	(0x03 << 0)
#define SUNXI_DEPCMD_SETTRANSFRESOURCE	(0x02 << 0)
#define SUNXI_DEPCMD_SETEPCONFIG		(0x01 << 0)

/* The EP number goes 0..31 so ep0 is always out and ep1 is always in */
#define SUNXI_DALEPENA_EP(n)		(1 << n)

#define SUNXI_DEPCMD_TYPE_CONTROL	0
#define SUNXI_DEPCMD_TYPE_ISOC		1
#define SUNXI_DEPCMD_TYPE_BULK		2
#define SUNXI_DEPCMD_TYPE_INTR		3

/* bit position */

/* USB Power Control for device only  */                 
#define  SUNXI_BP_POWER_D_ISO_UPDATE_EN                  7
#define  SUNXI_BP_POWER_D_SOFT_CONNECT                   6
#define  SUNXI_BP_POWER_D_HIGH_SPEED_EN                  5
#define  SUNXI_BP_POWER_D_HIGH_SPEED_FLAG                4
#define  SUNXI_BP_POWER_D_RESET_FLAG                     3
#define  SUNXI_BP_POWER_D_RESUME                         2
#define  SUNXI_BP_POWER_D_SUSPEND                        1
#define  SUNXI_BP_POWER_D_ENABLE_SUSPENDM                0

/* Interface Status and Control */                       
#define  SUNXI_BP_ISCR_VBUS_VALID_FROM_DATA		30
#define  SUNXI_BP_ISCR_VBUS_VALID_FROM_VBUS        	29        
#define  SUNXI_BP_ISCR_EXT_ID_STATUS               	28
#define  SUNXI_BP_ISCR_EXT_DM_STATUS               	27
#define  SUNXI_BP_ISCR_EXT_DP_STATUS               	26                                                 
#define  SUNXI_BP_ISCR_MERGED_VBUS_STATUS          	25        
#define  SUNXI_BP_ISCR_MERGED_ID_STATUS            	24

#define  SUNXI_BP_ISCR_ID_PULLUP_EN                	17                                                 
#define  SUNXI_BP_ISCR_DPDM_PULLUP_EN              	16
#define  SUNXI_BP_ISCR_FORCE_ID                    	14
#define  SUNXI_BP_ISCR_FORCE_VBUS_VALID            	12
#define  SUNXI_BP_ISCR_VBUS_VALID_SRC             	10    

#define  SUNXI_BP_ISCR_HOSC_EN                   	 7        
#define  SUNXI_BP_ISCR_VBUS_CHANGE_DETECT        	 6
#define  SUNXI_BP_ISCR_ID_CHANGE_DETECT          	 5        
#define  SUNXI_BP_ISCR_DPDM_CHANGE_DETECT        	 4        
#define  SUNXI_BP_ISCR_IRQ_ENABLE                	 3        
#define  SUNXI_BP_ISCR_VBUS_CHANGE_DETECT_EN     	 2        
#define  SUNXI_BP_ISCR_ID_CHANGE_DETECT_EN       	 1        
#define  SUNXI_BP_ISCR_DPDM_CHANGE_DETECT_EN     	 0

/* Structures */

struct sunxi_trb_hw;

/**
 * struct sunxi_event_buffer - Software event buffer representation
 * @list: a list of event buffers
 * @buf: _THE_ buffer
 * @length: size of this buffer
 * @dma: dma_addr_t
 * @sunxi: pointer to DWC controller
 */
struct sunxi_event_buffer {
	void			*buf;
	unsigned		length;
	unsigned int		lpos;

	dma_addr_t		dma;

	struct sunxi		*sunxi;
};

#define SUNXI_EP_FLAG_STALLED	(1 << 0)
#define SUNXI_EP_FLAG_WEDGED	(1 << 1)

#define SUNXI_EP_DIRECTION_TX	true
#define SUNXI_EP_DIRECTION_RX	false

#define SUNXI_TRB_NUM		32
#define SUNXI_TRB_MASK		(SUNXI_TRB_NUM - 1)

/**
 * struct sunxi_ep - device side endpoint representation
 * @endpoint: usb endpoint
 * @request_list: list of requests for this endpoint
 * @req_queued: list of requests on this ep which have TRBs setup
 * @trb_pool: array of transaction buffers
 * @trb_pool_dma: dma address of @trb_pool
 * @free_slot: next slot which is going to be used
 * @busy_slot: first slot which is owned by HW
 * @desc: usb_endpoint_descriptor pointer
 * @sunxi: pointer to DWC controller
 * @flags: endpoint flags (wedged, stalled, ...)
 * @current_trb: index of current used trb
 * @number: endpoint number (1 - 15)
 * @type: set to bmAttributes & USB_ENDPOINT_XFERTYPE_MASK
 * @resource_index: Resource transfer index
 * @interval: the intervall on which the ISOC transfer is started
 * @name: a human readable name e.g. ep1out-bulk
 * @direction: true for TX, false for RX
 * @stream_capable: true when streams are enabled
 */
struct sunxi_ep {
	struct usb_ep		endpoint;
	struct list_head	request_list;
	struct list_head	req_queued;

	struct sunxi_trb_hw	*trb_pool;
	dma_addr_t		trb_pool_dma;
	u32			free_slot;
	u32			busy_slot;
	const struct usb_endpoint_descriptor *desc;
	const struct usb_ss_ep_comp_descriptor *comp_desc;
	struct sunxi		*sunxi;

	unsigned		flags;
#define SUNXI_EP_ENABLED		(1 << 0)
#define SUNXI_EP_STALL		(1 << 1)
#define SUNXI_EP_WEDGE		(1 << 2)
#define SUNXI_EP_BUSY		(1 << 4)
#define SUNXI_EP_PENDING_REQUEST	(1 << 5)

	/* This last one is specific to EP0 */
#define SUNXI_EP0_DIR_IN		(1 << 31)

	unsigned		current_trb;

	u8			number;
	u8			type;
	u8			resource_index;
	u32			interval;

	char			name[20];

	unsigned		direction:1;
	unsigned		stream_capable:1;
};

enum sunxi_phy {
	SUNXI_PHY_UNKNOWN = 0,
	SUNXI_PHY_USB3,
	SUNXI_PHY_USB2,
};

enum sunxi_ep0_next {
	SUNXI_EP0_UNKNOWN = 0,
	SUNXI_EP0_COMPLETE,
	SUNXI_EP0_NRDY_SETUP,
	SUNXI_EP0_NRDY_DATA,
	SUNXI_EP0_NRDY_STATUS,
};

enum sunxi_ep0_state {
	EP0_UNCONNECTED		= 0,
	EP0_SETUP_PHASE,
	EP0_DATA_PHASE,
	EP0_STATUS_PHASE,
};

enum sunxi_link_state {
	/* In SuperSpeed */
	SUNXI_LINK_STATE_U0		= 0x00, /* in HS, means ON */
	SUNXI_LINK_STATE_U1		= 0x01,
	SUNXI_LINK_STATE_U2		= 0x02, /* in HS, means SLEEP */
	SUNXI_LINK_STATE_U3		= 0x03, /* in HS, means SUSPEND */
	SUNXI_LINK_STATE_SS_DIS		= 0x04,
	SUNXI_LINK_STATE_RX_DET		= 0x05, /* in HS, means Early Suspend */
	SUNXI_LINK_STATE_SS_INACT	= 0x06,
	SUNXI_LINK_STATE_POLL		= 0x07,
	SUNXI_LINK_STATE_RECOV		= 0x08,
	SUNXI_LINK_STATE_HRESET		= 0x09,
	SUNXI_LINK_STATE_CMPLY		= 0x0a,
	SUNXI_LINK_STATE_LPBK		= 0x0b,
	SUNXI_LINK_STATE_MASK		= 0x0f,
};

enum sunxi_device_state {
	SUNXI_DEFAULT_STATE,
	SUNXI_ADDRESS_STATE,
	SUNXI_CONFIGURED_STATE,
};

/**
 * struct sunxi_trb - transfer request block
 * @bpl: lower 32bit of the buffer
 * @bph: higher 32bit of the buffer
 * @length: buffer size (up to 16mb - 1)
 * @pcm1: packet count m1
 * @trbsts: trb status
 *	0 = ok
 *	1 = missed isoc
 *	2 = setup pending
 * @hwo: hardware owner of descriptor
 * @lst: last trb
 * @chn: chain buffers
 * @csp: continue on short packets (only supported on isoc eps)
 * @trbctl: trb control
 *	1 = normal
 *	2 = control-setup
 *	3 = control-status-2
 *	4 = control-status-3
 *	5 = control-data (first trb of data stage)
 *	6 = isochronous-first (first trb of service interval)
 *	7 = isochronous
 *	8 = link trb
 *	others = reserved
 * @isp_imi: interrupt on short packet / interrupt on missed isoc
 * @ioc: interrupt on complete
 * @sid_sofn: Stream ID / SOF Number
 */
#if 1
struct sunxi_trb {
	u64             bplh;

	union {
		struct {
			u32             length:24;
			u32             pcm1:2;
			u32             reserved27_26:2;
			u32             trbsts:4;
#define SUNXI_TRB_STS_OKAY                       0
#define SUNXI_TRB_STS_MISSED_ISOC                1
#define SUNXI_TRB_STS_SETUP_PENDING              2
		};
		u32 len_pcm;
	};

	union {
		struct {
			u32             hwo:1;
			u32             lst:1;
			u32             chn:1;
			u32             csp:1;
			u32             trbctl:6;
			u32             isp_imi:1;
			u32             ioc:1;
			u32             reserved13_12:2;
			u32             sid_sofn:16;
			u32             reserved31_30:2;
		};
		u32 control;
	};
} __packed;
#endif
/* TRB Control */
#define SUNXI_TRB_CTRL_HWO		(1 << 0)
#define SUNXI_TRB_CTRL_LST		(1 << 1)
#define SUNXI_TRB_CTRL_CHN		(1 << 2)
#define SUNXI_TRB_CTRL_CSP		(1 << 3)
#define SUNXI_TRB_CTRL_TRBCTL(n)		(((n) & 0x3f) << 4)
#define SUNXI_TRB_CTRL_ISP_IMI		(1 << 10)
#define SUNXI_TRB_CTRL_IOC		(1 << 11)
#define SUNXI_TRB_CTRL_SID_SOFN(n)	(((n) & 0xffff) << 14)

/**
 * struct sunxi_trb_hw - transfer request block (hw format)
 * @bpl: DW0-3
 * @bph: DW4-7
 * @size: DW8-B
 * @trl: DWC-F
 */
struct sunxi_trb_hw {
	__le32		bpl;
	__le32		bph;
	__le32		size;
	__le32		ctrl;
} __packed;
#if 1
static inline void sunxi_trb_to_hw(struct sunxi_trb *nat, struct sunxi_trb_hw *hw)
{
	hw->bpl = cpu_to_le32(lower_32_bits(nat->bplh));
	hw->bph = cpu_to_le32(upper_32_bits(nat->bplh));
	hw->size = cpu_to_le32p(&nat->len_pcm);
	
	/* HWO is written last */
	hw->ctrl = cpu_to_le32p(&nat->control);
}

static inline void sunxi_trb_to_nat(struct sunxi_trb_hw *hw, struct sunxi_trb *nat)
{
	u64 bplh;

	bplh = le32_to_cpup(&hw->bpl);
	bplh |= (u64) le32_to_cpup(&hw->bph) << 32;
	nat->bplh = bplh;

	nat->len_pcm = le32_to_cpup(&hw->size);
	nat->control = le32_to_cpup(&hw->ctrl);
}
#endif
/**
 * sunxi_hwparams - copy of HWPARAMS registers
 * @hwparams0 - GHWPARAMS0
 * @hwparams1 - GHWPARAMS1
 * @hwparams2 - GHWPARAMS2
 * @hwparams3 - GHWPARAMS3
 * @hwparams4 - GHWPARAMS4
 * @hwparams5 - GHWPARAMS5
 * @hwparams6 - GHWPARAMS6
 * @hwparams7 - GHWPARAMS7
 * @hwparams8 - GHWPARAMS8
 */
struct sunxi_hwparams {
	u32	hwparams0;
	u32	hwparams1;
	u32	hwparams2;
	u32	hwparams3;
	u32	hwparams4;
	u32	hwparams5;
	u32	hwparams6;
	u32	hwparams7;
	u32	hwparams8;
};

/* HWPARAMS0 */
#define SUNXI_MODE(n)		((n) & 0x7)

#define SUNXI_MODE_DEVICE	0
#define SUNXI_MODE_HOST		1
#define SUNXI_MODE_DRD		2
#define SUNXI_MODE_HUB		3

/* HWPARAMS1 */
#define SUNXI_NUM_INT(n)	(((n) & (0x3f << 15)) >> 15)

struct sunxi_request {
	struct usb_request	request;
	struct list_head	list;
	struct sunxi_ep		*dep;

	u8			epnum;
	struct sunxi_trb_hw	*trb;
	dma_addr_t		trb_dma;

	unsigned		direction:1;
	unsigned		mapped:1;
	unsigned		queued:1;
};

/**
 * struct sunxi - representation of our controller
 * @ctrl_req: usb control request which is used for ep0
 * @ep0_trb: trb which is used for the ctrl_req
 * @ep0_bounce: bounce buffer for ep0
 * @setup_buf: used while precessing STD USB requests
 * @ctrl_req_addr: dma address of ctrl_req
 * @ep0_trb: dma address of ep0_trb
 * @ep0_usb_req: dummy req used while handling STD USB requests
 * @setup_buf_addr: dma address of setup_buf
 * @ep0_bounce_addr: dma address of ep0_bounce
 * @lock: for synchronizing
 * @dev: pointer to our struct device
 * @xhci: pointer to our xHCI child
 * @event_buffer_list: a list of event buffers
 * @gadget: device side representation of the peripheral controller
 * @gadget_driver: pointer to the gadget driver
 * @regs: base address for our registers
 * @regs_size: address space size
 * @num_event_buffers: calculated number of event buffers
 * @u1u2: only used on revisions <1.83a for workaround
 * @maximum_speed: maximum speed requested (mainly for testing purposes)
 * @revision: revision register contents
 * @mode: mode of operation
 * @usb2_phy: pointer to USB2 PHY
 * @usb3_phy: pointer to USB3 PHY
 * @dcfg: saved contents of DCFG register
 * @gctl: saved contents of GCTL register
 * @is_selfpowered: true when we are selfpowered
 * @three_stage_setup: set if we perform a three phase setup
 * @ep0_bounced: true when we used bounce buffer
 * @ep0_expect_in: true when we expect a DATA IN transfer
 * @start_config_issued: true when StartConfig command has been issued
 * @setup_packet_pending: true when there's a Setup Packet in FIFO. Workaround
 * @needs_fifo_resize: not all users might want fifo resizing, flag it
 * @resize_fifos: tells us it's ok to reconfigure our TxFIFO sizes.
 * @isoch_delay: wValue from Set Isochronous Delay request;
 * @u2sel: parameter from Set SEL request.
 * @u2pel: parameter from Set SEL request.
 * @u1sel: parameter from Set SEL request.
 * @u1pel: parameter from Set SEL request.
 * @num_out_eps: number of out endpoints
 * @num_in_eps: number of in endpoints
 * @ep0_next_event: hold the next expected event
 * @ep0state: state of endpoint zero
 * @link_state: link state
 * @speed: device speed (super, high, full, low)
 * @mem: points to start of memory which is used for this struct.
 * @hwparams: copy of hwparams registers
 */
struct sunxi {
	struct usb_ctrlrequest	*ctrl_req;
	struct sunxi_trb_hw	*ep0_trb;
	void			*ep0_bounce;
	u8			*setup_buf;
	dma_addr_t		ctrl_req_addr;
	dma_addr_t		ep0_trb_addr;
	dma_addr_t		setup_buf_addr;
	dma_addr_t		ep0_bounce_addr;
	struct sunxi_request	ep0_usb_req;

	/* device lock */
	spinlock_t		lock;

	struct device		*dev;

	struct platform_device	*xhci;
	struct resource		*res;

	struct sunxi_event_buffer **ev_buffs;
	struct sunxi_ep		*eps[SUNXI_ENDPOINTS_NUM];

	struct usb_gadget	gadget;
	struct usb_gadget_driver *gadget_driver;

	struct usb_phy		*usb2_phy;
	struct usb_phy		*usb3_phy;

	void __iomem		*regs;
	size_t			regs_size;

	/* used for suspend/resume */
	u32			dcfg;
	u32			gctl;

	u32			num_event_buffers;
	u32			u1u2;
	u32			maximum_speed;
	u32			revision;
	u32			mode;

#define SUNXI_REVISION_173A	0x5533173a
#define SUNXI_REVISION_175A	0x5533175a
#define SUNXI_REVISION_180A	0x5533180a
#define SUNXI_REVISION_183A	0x5533183a
#define SUNXI_REVISION_185A	0x5533185a
#define SUNXI_REVISION_187A	0x5533187a
#define SUNXI_REVISION_188A	0x5533188a
#define SUNXI_REVISION_190A	0x5533190a
#define SUNXI_REVISION_194A	0x5533194a
#define SUNXI_REVISION_200A	0x5533200a
#define SUNXI_REVISION_202A	0x5533202a
#define SUNXI_REVISION_210A	0x5533210a
#define SUNXI_REVISION_220A	0x5533220a
#define SUNXI_REVISION_230A	0x5533230a
#define SUNXI_REVISION_240A	0x5533240a
#define SUNXI_REVISION_250A	0x5533250a

	unsigned		is_selfpowered:1;
	unsigned		three_stage_setup:1;
	unsigned		ep0_bounced:1;
	unsigned		ep0_expect_in:1;
	unsigned		start_config_issued:1;
	unsigned		setup_packet_pending:1;
	unsigned		delayed_status:1;
	unsigned		needs_fifo_resize:1;
	unsigned		resize_fifos:1;
	unsigned		pullups_connected:1;

	enum sunxi_ep0_next	ep0_next_event;
	enum sunxi_ep0_state	ep0state;
	enum sunxi_link_state	link_state;
	enum sunxi_device_state	dev_state;

	u16			isoch_delay;
	u16			u2sel;
	u16			u2pel;
	u8			u1sel;
	u8			u1pel;

	u8			speed;
	void			*mem;

	struct sunxi_hwparams	hwparams;
	struct dentry		*root;

	u8			test_mode;
	u8			test_mode_nr;
};

/* -------------------------------------------------------------------------- */
#if 1
#define SUNXI_TRBSTS_OK			0
#define SUNXI_TRBSTS_MISSED_ISOC		1
#define SUNXI_TRBSTS_SETUP_PENDING	2

#define SUNXI_TRBCTL_NORMAL		1
#define SUNXI_TRBCTL_CONTROL_SETUP	2
#define SUNXI_TRBCTL_CONTROL_STATUS2	3
#define SUNXI_TRBCTL_CONTROL_STATUS3	4
#define SUNXI_TRBCTL_CONTROL_DATA	5
#define SUNXI_TRBCTL_ISOCHRONOUS_FIRST	6
#define SUNXI_TRBCTL_ISOCHRONOUS		7
#define SUNXI_TRBCTL_LINK_TRB		8
#endif

#define SUNXI_TRB_SIZE_MASK	(0x00ffffff)
#define SUNXI_TRB_SIZE_TRBSTS(n)	(((n) & (0x0f << 28)) >> 28)

/* -------------------------------------------------------------------------- */

struct sunxi_event_type {
	u32	is_devspec:1;
	u32	type:6;
	u32	reserved8_31:25;
} __packed;

#define SUNXI_DEPEVT_XFERCOMPLETE	0x01
#define SUNXI_DEPEVT_XFERINPROGRESS	0x02
#define SUNXI_DEPEVT_XFERNOTREADY	0x03
#define SUNXI_DEPEVT_RXTXFIFOEVT		0x04
#define SUNXI_DEPEVT_STREAMEVT		0x06
#define SUNXI_DEPEVT_EPCMDCMPLT		0x07

/**
 * struct sunxi_event_depvt - Device Endpoint Events
 * @one_bit: indicates this is an endpoint event (not used)
 * @endpoint_number: number of the endpoint
 * @endpoint_event: The event we have:
 *	0x00	- Reserved
 *	0x01	- XferComplete
 *	0x02	- XferInProgress
 *	0x03	- XferNotReady
 *	0x04	- RxTxFifoEvt (IN->Underrun, OUT->Overrun)
 *	0x05	- Reserved
 *	0x06	- StreamEvt
 *	0x07	- EPCmdCmplt
 * @reserved11_10: Reserved, don't use.
 * @status: Indicates the status of the event. Refer to databook for
 *	more information.
 * @parameters: Parameters of the current event. Refer to databook for
 *	more information.
 */
struct sunxi_event_depevt {
	u32	one_bit:1;
	u32	endpoint_number:5;
	u32	endpoint_event:4;
	u32	reserved11_10:2;
	u32	status:4;
#define DEPEVT_STATUS_BUSERR    (1 << 0)
#define DEPEVT_STATUS_SHORT     (1 << 1)
#define DEPEVT_STATUS_IOC       (1 << 2)
#define DEPEVT_STATUS_LST	(1 << 3)

/* Stream event only */
#define DEPEVT_STREAMEVT_FOUND		1
#define DEPEVT_STREAMEVT_NOTFOUND	2

/* Control-only Status */
#define DEPEVT_STATUS_CONTROL_SETUP	0
#define DEPEVT_STATUS_CONTROL_DATA	1
#define DEPEVT_STATUS_CONTROL_STATUS	2

	u32	parameters:16;
} __packed;

/**
 * struct sunxi_event_devt - Device Events
 * @one_bit: indicates this is a non-endpoint event (not used)
 * @device_event: indicates it's a device event. Should read as 0x00
 * @type: indicates the type of device event.
 *	0	- DisconnEvt
 *	1	- USBRst
 *	2	- ConnectDone
 *	3	- ULStChng
 *	4	- WkUpEvt
 *	5	- Reserved
 *	6	- EOPF
 *	7	- SOF
 *	8	- Reserved
 *	9	- ErrticErr
 *	10	- CmdCmplt
 *	11	- EvntOverflow
 *	12	- VndrDevTstRcved
 * @reserved15_12: Reserved, not used
 * @event_info: Information about this event
 * @reserved31_24: Reserved, not used
 */
struct sunxi_event_devt {
	u32	one_bit:1;
	u32	device_event:7;
	u32	type:4;
	u32	reserved15_12:4;
	u32	event_info:8;
	u32	reserved31_24:8;
} __packed;

/**
 * struct sunxi_event_gevt - Other Core Events
 * @one_bit: indicates this is a non-endpoint event (not used)
 * @device_event: indicates it's (0x03) Carkit or (0x04) I2C event.
 * @phy_port_number: self-explanatory
 * @reserved31_12: Reserved, not used.
 */
struct sunxi_event_gevt {
	u32	one_bit:1;
	u32	device_event:7;
	u32	phy_port_number:4;
	u32	reserved31_12:20;
} __packed;

/**
 * union sunxi_event - representation of Event Buffer contents
 * @raw: raw 32-bit event
 * @type: the type of the event
 * @depevt: Device Endpoint Event
 * @devt: Device Event
 * @gevt: Global Event
 */
union sunxi_event {
	u32				raw;
	struct sunxi_event_type		type;
	struct sunxi_event_depevt	depevt;
	struct sunxi_event_devt		devt;
	struct sunxi_event_gevt		gevt;
};

/*
 * SUNXI Features to be used as Driver Data
 */

#define SUNXI_HAS_PERIPHERAL		BIT(0)
#define SUNXI_HAS_XHCI			BIT(1)
#define SUNXI_HAS_OTG			BIT(3)

/* prototypes */
void sunxi_set_mode(struct sunxi *sunxi, u32 mode);

int sunxi_host_init(struct sunxi *sunxi);
static inline void sunxi_host_exit(struct sunxi *sunxi) {}

int sunxi_gadget_init(struct sunxi *sunxi);
void sunxi_gadget_exit(struct sunxi *sunxi);

extern int sunxi_get_device_id(void);
extern void sunxi_put_device_id(int id);

#endif /* __DRIVERS_USB_SUNXI_CORE_H */
