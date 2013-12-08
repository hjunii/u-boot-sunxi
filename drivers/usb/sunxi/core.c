/**
 * core.c
 */

#include <common.h>

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>

#include "core.h"
#include "gadget.h"
#include "io.h"

#include "debug.h"

#include <asm/arch/clock.h>
#include "palmas.h"
#include "sunxi_phy.h"

static char *maximum_speed = "super";
/* -------------------------------------------------------------------------- */

#define SUNXI_DEVS_POSSIBLE     32


int sunxi_get_device_id(void)
{
	static int sunxi_devs;

	return sunxi_devs++;
}

void sunxi_put_device_id(int id)
{
}

void sunxi_set_mode(struct sunxi *sunxi, u32 mode)
{
	u32 reg;

	// USBC_ForceId
	reg = sunxi_readl(sunxi->regs, SUNXI_ISCR);

	switch (mode) {
	case SUNXI_GCTL_PRTCAP_DEVICE:
		reg |=  (0x03 << SUNXI_BP_ISCR_FORCE_ID);
		break;
	case SUNXI_GCTL_PRTCAP_HOST:
		reg &= ~(0x03 << SUNXI_BP_ISCR_FORCE_ID);
		reg |=  (0x02 << SUNXI_BP_ISCR_FORCE_ID);
		break;
	default:
		reg &= ~(0x03 << SUNXI_BP_ISCR_FORCE_ID);
		break;
	}

	reg &= ~(1 << SUNXI_BP_ISCR_VBUS_CHANGE_DETECT);
	reg &= ~(1 << SUNXI_BP_ISCR_ID_CHANGE_DETECT);
	reg &= ~(1 << SUNXI_BP_ISCR_DPDM_CHANGE_DETECT);

	sunxi_writel(sunxi->regs, SUNXI_ISCR, reg);
}

/**
 * sunxi_core_soft_reset - Issues core soft reset and PHY reset
 * @sunxi: pointer to our context structure
 */
static void sunxi_core_soft_reset(struct sunxi *sunxi)
{
}

/**
 * sunxi_free_one_event_buffer - Frees one event buffer
 * @sunxi: Pointer to our controller context structure
 * @evt: Pointer to event buffer to be freed
 */
static void sunxi_free_one_event_buffer(struct sunxi *sunxi,
		struct sunxi_event_buffer *evt)
{
	dma_free_coherent(sunxi->dev, evt->length, evt->buf, evt->dma);
	kfree(evt);
}

/**
 * sunxi_alloc_one_event_buffer - Allocated one event buffer structure
 * @sunxi: Pointer to our controller context structure
 * @length: size of the event buffer
 *
 * Returns a pointer to the allocated event buffer structure on succes
 * otherwise ERR_PTR(errno).
 */
static struct sunxi_event_buffer *__devinit
sunxi_alloc_one_event_buffer(struct sunxi *sunxi, unsigned length)
{
	struct sunxi_event_buffer	*evt;

	evt = kzalloc(sizeof(*evt), GFP_KERNEL);
	if (!evt)
		return ERR_PTR(-ENOMEM);

	evt->sunxi	= sunxi;
	evt->length	= length;
	evt->buf	= dma_alloc_coherent(sunxi->dev, length,
			&evt->dma, GFP_KERNEL);
	if (!evt->buf) {
		kfree(evt);
		return ERR_PTR(-ENOMEM);
	}

	return evt;
}

/**
 * sunxi_free_event_buffers - frees all allocated event buffers
 * @sunxi: Pointer to our controller context structure
 */
static void sunxi_free_event_buffers(struct sunxi *sunxi)
{
	struct sunxi_event_buffer	*evt;
	int i;

	for (i = 0; i < sunxi->num_event_buffers; i++) {
		evt = sunxi->ev_buffs[i];
		if (evt) {
			sunxi_free_one_event_buffer(sunxi, evt);
			sunxi->ev_buffs[i] = NULL;
		}
	}
}

/**
 * sunxi_alloc_event_buffers - Allocates @num event buffers of size @length
 * @sunxi: Pointer to out controller context structure
 * @length: size of event buffer
 *
 * Returns 0 on success otherwise negative errno. In error the case, sunxi
 * may contain some buffers allocated but not all which were requested.
 */
static int __devinit sunxi_alloc_event_buffers(struct sunxi *sunxi, unsigned length)
{
	int			num;
	int			i;

	//num = SUNXI_NUM_INT(sunxi->hwparams.hwparams1);
	num = 1;
	sunxi->num_event_buffers = num;

	sunxi->ev_buffs = kzalloc(sizeof(*sunxi->ev_buffs) * num, GFP_KERNEL);
	if (!sunxi->ev_buffs) {
		dev_err(sunxi->dev, "can't allocate event buffers array\n");
		return -ENOMEM;
	}

	for (i = 0; i < num; i++) {
		struct sunxi_event_buffer	*evt;

		evt = sunxi_alloc_one_event_buffer(sunxi, length);
		if (IS_ERR(evt)) {
			dev_err(sunxi->dev, "can't allocate event buffer\n");
			return PTR_ERR(evt);
		}
		sunxi->ev_buffs[i] = evt;
	}

	return 0;
}

/**
 * sunxi_event_buffers_setup - setup our allocated event buffers
 * @sunxi: Pointer to out controller context structure
 *
 * Returns 0 on success otherwise negative errno.
 */
static int __devinit sunxi_event_buffers_setup(struct sunxi *sunxi)
{
	struct sunxi_event_buffer	*evt;
	int				n;

	for (n = 0; n < sunxi->num_event_buffers; n++) {
		evt = sunxi->ev_buffs[n];
		dev_dbg(sunxi->dev, "Event buf %p dma %08llx length %d\n",
				evt->buf, (unsigned long long) evt->dma,
				evt->length);
		evt->lpos = 0;
	}

	return 0;
}

static void sunxi_event_buffers_cleanup(struct sunxi *sunxi)
{
	int				n;

	for (n = 0; n < sunxi->num_event_buffers; n++) {
	}
}

/**
 * sunxi_core_init - Low-level initialization of SUNXI Core
 * @sunxi: Pointer to our controller context structure
 *
 * Returns 0 on success otherwise negative errno.
 */
static int sunxi_core_init(struct sunxi *sunxi)
{
	u32			reg;
	int			ret;
	
	sunxi_core_soft_reset(sunxi);

	ret = sunxi_event_buffers_setup(sunxi);
	if (ret) {
		dev_err(sunxi->dev, "failed to setup event buffers \n");
		goto err0;
	}
	
	return 0;

err0:
	return ret;
}

static void sunxi_core_exit(struct sunxi *sunxi)
{
	sunxi_event_buffers_cleanup(sunxi);
	sunxi_free_event_buffers(sunxi);
}

#define SUNXI_ALIGN_MASK		(16 - 1)

struct sunxi *global_sunxi;

void sunxi_wrapper_init(void);

int __devinit sunxi_probe(struct platform_device *pdev)
{
	struct sunxi		*sunxi;

	int			ret = -ENOMEM;

	void __iomem		*regs;
	void			*mem;

	u8			mode;

	/* Initialize all the clocks (system clocks/ optional clocks etc) */
	struct sunxi_ccm_reg *ccm = (struct sunxi_ccm_reg *)SUNXI_CCM_BASE;
	clrbits_le32(&ccm->ahb_gate0, 0x1 << AHB_GATE_OFFSET_USB);
	sdelay(10000);
	clrbits_le32(&ccm->usb_clk_cfg, 0x1 << 8 | 0x1);
	sdelay(10000);
	setbits_le32(&ccm->ahb_gate0, 0x1 << AHB_GATE_OFFSET_USB);
	sdelay(10000);
	setbits_le32(&ccm->usb_clk_cfg, 0x1 << 8 | 0x1);
	sdelay(10000);

	/* Initialize the wrapper registers */
	sunxi_wrapper_init();

	mem = kzalloc(sizeof(*sunxi) + SUNXI_ALIGN_MASK, GFP_KERNEL);
	if (!mem) {
		dev_err(&pdev->dev, "not enough memory\n");
		goto err0;
	}
	sunxi = PTR_ALIGN(mem, SUNXI_ALIGN_MASK + 1);
	sunxi->mem = mem;
	global_sunxi = sunxi;

	sunxi->regs	= SUNXI_USB0_BASE;
	sunxi->regs_size	= SUNXI_USB_REGS_SIZE;
	sunxi->dev	= &pdev->dev;

	if (!strncmp("super", maximum_speed, 5))
		sunxi->maximum_speed = SUNXI_DCFG_SUPERSPEED;
	else if (!strncmp("high", maximum_speed, 4))
		sunxi->maximum_speed = SUNXI_DCFG_HIGHSPEED;
	else if (!strncmp("full", maximum_speed, 4))
		sunxi->maximum_speed = SUNXI_DCFG_FULLSPEED1;
	else if (!strncmp("low", maximum_speed, 3))
		sunxi->maximum_speed = SUNXI_DCFG_LOWSPEED;
	else
		sunxi->maximum_speed = SUNXI_DCFG_SUPERSPEED;

	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);
	pm_runtime_forbid(&pdev->dev);

	ret = sunxi_alloc_event_buffers(sunxi, SUNXI_EVENT_BUFFERS_SIZE);

	if (ret) {
		dev_err(&pdev->dev, "failed to initialize core\n");
		goto err3;
	}

	ret = sunxi_core_init(sunxi);
	if (ret) {
		dev_err(dev, "failed to initialize core\n");
		goto err0;
	}

	mode = SUNXI_MODE_DEVICE /*SUNXI_MODE(sunxi->hwparams.hwparams0)*/;
	switch (mode) {
	case SUNXI_MODE_DEVICE:
		sunxi_set_mode(sunxi, SUNXI_GCTL_PRTCAP_DEVICE);
		ret = sunxi_gadget_init(sunxi);
		if (ret) {
			dev_err(&pdev->dev, "failed to initialize gadget\n");
			goto err4;
		}
		break;
	case SUNXI_MODE_HOST:
		sunxi_set_mode(sunxi, SUNXI_GCTL_PRTCAP_HOST);
		ret = sunxi_host_init(sunxi);
		if (ret) {
			dev_err(&pdev->dev, "failed to initialize host\n");
			goto err4;
		}
		break;
	case SUNXI_MODE_DRD:
		sunxi_set_mode(sunxi, SUNXI_GCTL_PRTCAP_OTG);
		ret = sunxi_host_init(sunxi);
		if (ret) {
			dev_err(&pdev->dev, "failed to initialize host\n");
			goto err4;
		}

		ret = sunxi_gadget_init(sunxi);
		if (ret) {
			dev_err(&pdev->dev, "failed to initialize gadget\n");
			goto err4;
		}
		break;
	default:
		dev_err(&pdev->dev, "Unsupported mode of operation %d\n", mode);
		goto err4;
	}
	sunxi->mode = mode;
	return 0;
err4:
	sunxi_core_exit(sunxi);

err3:
	iounmap(regs);
	kfree(sunxi->mem);
err0:
	return ret;
}

int __devexit sunxi_remove(struct platform_device *pdev)
{
	struct sunxi	*sunxi = global_sunxi;

	sunxi_debugfs_exit(sunxi);

	switch (sunxi->mode) {
	case SUNXI_MODE_DEVICE:
		sunxi_gadget_exit(sunxi);
		break;
	case SUNXI_MODE_HOST:
		sunxi_host_exit(sunxi);
		break;
	case SUNXI_MODE_DRD:
		sunxi_host_exit(sunxi);
		sunxi_gadget_exit(sunxi);
		break;
	default:
		/* do nothing */
		break;
	}

	sunxi_core_exit(sunxi);
	iounmap(sunxi->regs);
	kfree(sunxi->mem);

	return 0;
}


void sunxi_wrapper_init(void)
{
}

int usb_gadget_init_udc(void)
{
	return sunxi_probe(NULL);
}
void usb_gadget_exit_udc(void)
{
	sunxi_remove(NULL);
}

irqreturn_t sunxi_interrupt(int irq, void *_sunxi);
int usb_gadget_handle_interrupts(void)
{
	sunxi_interrupt(0, global_sunxi);
	return 0;
}
