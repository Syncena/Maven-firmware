/**
 * \file
 *
 * \brief SAM USB Driver.
 *
 * Copyright (c) 2014-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */

/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */
#include <inttypes.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "rtos.h"
#include "platform.h"
#include "platform_usb.h"
#include "sam_clocks.h"
#include "sam_cmcc.h"

#include "sam_usb.h"

#undef	SCW_USB_DEV_DEBUG
#undef	SCW_USB_EP_DEBUG
#undef	SCW_USB_STATS

#ifdef SCW_USB_DEV_DEBUG
#define	DEVPRINTF(fmt, ...)	do { printf(fmt, ##__VA_ARGS__); } while (0)
#else
#define	DEVPRINTF(fmt, ...)	do { } while (0)
#endif

#ifdef SCW_USB_EP_DEBUG
#define	EPPRINTF(fmt, ...)	do { printf(fmt, ##__VA_ARGS__); } while (0)
#else
#define	EPPRINTF(fmt, ...)	do { } while (0)
#endif

#ifndef UHD_BULK_INTERVAL_MIN
/** Minimal bulk interval value */
#  define UHD_BULK_INTERVAL_MIN 1
#endif

/** Fields definition from a LPM TOKEN  */
#define  USB_LPM_ATTRIBUT_BLINKSTATE_MASK      (0xF << 0)
#define  USB_LPM_ATTRIBUT_HIRD_MASK            (0xF << 4)
#define  USB_LPM_ATTRIBUT_REMOTEWAKE_MASK      (1 << 8)
#define  USB_LPM_ATTRIBUT_BLINKSTATE(value)    ((value & 0xF) << 0)
#define  USB_LPM_ATTRIBUT_HIRD(value)          ((value & 0xF) << 4)
#define  USB_LPM_ATTRIBUT_REMOTEWAKE(value)    ((value & 1) << 8)
#define  USB_LPM_ATTRIBUT_BLINKSTATE_L1        USB_LPM_ATTRIBUT_BLINKSTATE(1)

/**
 * \brief Mask selecting the index part of an endpoint address
 */
#define  USB_EP_ADDR_MASK     0x0f

/**
 * \brief Endpoint transfer direction is IN
 */
#define  USB_EP_DIR_IN        0x80

/**
 * \brief Endpoint transfer direction is OUT
 */
#define  USB_EP_DIR_OUT       0x00

/**
 * \name USB SRAM data containing pipe descriptor table
 * The content of the USB SRAM can be :
 * - modified by USB hardware interface to update pipe status.
 *   Thereby, it is read by software.
 * - modified by USB software to control pipe.
 *   Thereby, it is read by hardware.
 * This data section is volatile.
 *
 * @{
 */
__attribute__((packed,__aligned__(16)))
union {
	UsbDeviceDescriptor usb_endpoint_table[USB_EPT_NUM];
} usb_descriptor_table;
/** @} */

/* Device LPM callback variable */
static uint32_t device_callback_lpm_wakeup_enable;

/**
 * \brief Device endpoint callback parameter variable, used to transfer info to UDD wrapper layer
 */
static struct usb_endpoint_callback_parameter ep_callback_para;

/**
 * \internal USB Device IRQ Mask Bits Map
 */
static const uint16_t _usb_device_irq_bits[USB_DEVICE_CALLBACK_N] = {
	USB_DEVICE_INTFLAG_SOF,
	USB_DEVICE_INTFLAG_EORST,
	USB_DEVICE_INTFLAG_WAKEUP | USB_DEVICE_INTFLAG_EORSM | USB_DEVICE_INTFLAG_UPRSM,
	USB_DEVICE_INTFLAG_RAMACER,
	USB_DEVICE_INTFLAG_SUSPEND,
	USB_DEVICE_INTFLAG_LPMNYET,
	USB_DEVICE_INTFLAG_LPMSUSP,
};

#ifdef SCW_USB_STATS
static uint32_t _usb_device_irq_total;
static uint32_t _usb_device_irq_count[USB_DEVICE_CALLBACK_N];
#endif
#if defined(SCW_USB_STATS) || defined(SCW_USB_DEV_DEBUG)
static const char *_usb_device_irq_strings[USB_DEVICE_CALLBACK_N] = {
	"SOF", "RESET", "WAKEUP", "RAMACER", "SUSPEND", "LPMNYET", "LPMSUSP"
};
#endif

/**
 * \internal USB Device IRQ Mask Bits Map
 */
static const uint8_t _usb_endpoint_irq_bits[USB_DEVICE_EP_CALLBACK_N] = {
	USB_DEVICE_EPINTFLAG_TRCPT_Msk,
	USB_DEVICE_EPINTFLAG_TRFAIL_Msk,
	USB_DEVICE_EPINTFLAG_RXSTP,
	USB_DEVICE_EPINTFLAG_STALL_Msk
};

#ifdef SCW_USB_STATS
static uint32_t _usb_device_ep_irq_count[USB_EPT_NUM][USB_DEVICE_EP_CALLBACK_N];
#endif
#if defined(SCW_USB_STATS) || defined(SCW_USB_EP_DEBUG)
static const char *_usb_device_ep_irq_strings[USB_DEVICE_EP_CALLBACK_N] = {
	"TRCPT", "TRFAIL", "RXSTP", "STALL"
};
#endif

#ifdef SCW_USB_STATS
extern void dump_usb_counts(FILE *);
void
dump_usb_counts(FILE *fp)
{

	fprintf(fp, "Total interrupts: %" PRIu32 "\n\n", _usb_device_irq_total);

	for (unsigned int i = 0; i < USB_DEVICE_CALLBACK_N; i++)
		fprintf(fp, "%-9s", _usb_device_irq_strings[i]);
	fprintf(fp, " MASK STAT\n");

	for (unsigned int i = 0; i < USB_DEVICE_CALLBACK_N; i++)
		fprintf(fp, "%-9" PRIu32, _usb_device_irq_count[i]);
	fprintf(fp, " %04" PRIx16 " %04" PRIx16 "\n\n",
	    USB->DEVICE.INTENSET.reg, USB->DEVICE.INTFLAG.reg);

	fprintf(fp, "EP ");
	for (unsigned int i = 0; i < USB_DEVICE_EP_CALLBACK_N; i++)
		fprintf(fp, "%-11s", _usb_device_ep_irq_strings[i]);
	fprintf(fp, " MASK STAT\n");

	for (unsigned int ep = 0; ep < USB_EPT_NUM; ep++) {
		fprintf(fp, "%u  ", ep);
		for (unsigned int i = 0; i < USB_DEVICE_EP_CALLBACK_N; i++) {
			fprintf(fp, "%-11" PRIu32,
			    _usb_device_ep_irq_count[ep][i]);
		}
		fprintf(fp, " %02x   %02x\n",
		   (unsigned int)USB->DEVICE.DeviceEndpoint[ep].EPSTATUSSET.reg,
		    (unsigned int)USB->DEVICE.DeviceEndpoint[ep].EPINTFLAG.reg);
	}
}
#endif

/**
 * \brief Registers a USB device callback
 *
 * Registers a callback function which is implemented by the user.
 *
 * \note The callback must be enabled by \ref usb_device_enable_callback,
 * in order for the interrupt handler to call it when the conditions for the
 * callback type is met.
 *
 * \param[in]     module_inst   Pointer to USB software instance struct
 * \param[in]     callback_type Callback type given by an enum
 * \param[in]     callback_func Pointer to callback function
 *
 * \return Status of the registration operation.
 * \retval STATUS_OK    The callback was registered successfully.
 */
enum status_code usb_device_register_callback(struct usb_module *module_inst,
		enum usb_device_callback callback_type,
		usb_device_callback_t callback_func)
{
	/* Sanity check arguments */
	Assert(module_inst);
	Assert(callback_func);

	/* Register callback function */
	module_inst->device_callback[callback_type] = callback_func;

	/* Set the bit corresponding to the callback_type */
	module_inst->device_registered_callback_mask |= _usb_device_irq_bits[callback_type];

	return STATUS_OK;
}

/**
 * \brief Unregisters a USB device callback
 *
 * Unregisters an asynchronous callback implemented by the user. Removing it
 * from the internal callback registration table.
 *
 * \param[in]     module_inst   Pointer to USB software instance struct
 * \param[in]     callback_type Callback type given by an enum
 *
 * \return Status of the de-registration operation.
 * \retval STATUS_OK    The callback was unregistered successfully.
 */
enum status_code usb_device_unregister_callback(struct usb_module *module_inst,
		enum usb_device_callback callback_type)
{
	/* Sanity check arguments */
	Assert(module_inst);

	/* Unregister callback function */
	module_inst->device_callback[callback_type] = NULL;

	/* Clear the bit corresponding to the callback_type */
	module_inst->device_registered_callback_mask &= ~_usb_device_irq_bits[callback_type];

	return STATUS_OK;
}

/**
 * \brief Enables USB device callback generation for a given type.
 *
 * Enables asynchronous callbacks for a given logical type.
 * This must be called before USB device generate callback events.
 *
 * \param[in]     module_inst   Pointer to USB software instance struct
 * \param[in]     callback_type Callback type given by an enum
 *
 * \return Status of the callback enable operation.
 * \retval STATUS_OK    The callback was enabled successfully.
 */
enum status_code usb_device_enable_callback(struct usb_module *module_inst,
		enum usb_device_callback callback_type)
{
	/* Sanity check arguments */
	Assert(module_inst);
	Assert(module_inst->hw);

	/* clear related flag */
	module_inst->hw->DEVICE.INTFLAG.reg = _usb_device_irq_bits[callback_type];

	/* Enable callback */
	module_inst->device_enabled_callback_mask |= _usb_device_irq_bits[callback_type];

	module_inst->hw->DEVICE.INTENSET.reg = _usb_device_irq_bits[callback_type];

	DEVPRINTF("usb_device_enable_callback: type %s, bits 0x%04" PRIx16
	    " (0x%04" PRIx16 ")\n", _usb_device_irq_strings[callback_type],
	    _usb_device_irq_bits[callback_type],
	    module_inst->hw->DEVICE.INTENSET.reg);

	return STATUS_OK;
}

/**
 * \brief Disables USB device callback generation for a given type.
 *
 * Disables asynchronous callbacks for a given logical type.
 *
 * \param[in]     module_inst   Pointer to USB software instance struct
 * \param[in]     callback_type Callback type given by an enum
 *
 * \return Status of the callback disable operation.
 * \retval STATUS_OK    The callback was disabled successfully.
 */
enum status_code usb_device_disable_callback(struct usb_module *module_inst,
		enum usb_device_callback callback_type)
{
	/* Sanity check arguments */
	Assert(module_inst);
	Assert(module_inst->hw);

	/* Disable callback */
	module_inst->device_enabled_callback_mask &= ~_usb_device_irq_bits[callback_type];

	module_inst->hw->DEVICE.INTENCLR.reg = _usb_device_irq_bits[callback_type];

	DEVPRINTF("usb_device_disable_callback: type %s, bits 0x%04" PRIx16
	    "(0x%04" PRIx16 ")\n", _usb_device_irq_strings[callback_type],
	    _usb_device_irq_bits[callback_type],
	    module_inst->hw->DEVICE.INTENCLR.reg);

	return STATUS_OK;
}

/**
 * \brief Registers a USB device endpoint callback
 *
 * Registers a callback function which is implemented by the user.
 *
 * \note The callback must be enabled by \ref usb_device_endpoint_enable_callback,
 * in order for the interrupt handler to call it when the conditions for the
 * callback type is met.
 *
 * \param[in]     module_inst   Pointer to USB software instance struct
 * \param[in]     ep_num        Endpoint to configure
 * \param[in]     callback_type Callback type given by an enum
 * \param[in]     callback_func Pointer to callback function
 *
 * \return Status of the registration operation.
 * \retval STATUS_OK    The callback was registered successfully.
 */
enum status_code usb_device_endpoint_register_callback(
		struct usb_module *module_inst, uint8_t ep_num,
		enum usb_device_endpoint_callback callback_type,
		usb_device_endpoint_callback_t callback_func)
{
	/* Sanity check arguments */
	Assert(module_inst);
	Assert(ep_num < USB_EPT_NUM);
	Assert(callback_func);

	/* Register callback function */
	module_inst->device_endpoint_callback[ep_num][callback_type] = callback_func;

	/* Set the bit corresponding to the callback_type */
	module_inst->device_endpoint_registered_callback_mask[ep_num] |= _usb_endpoint_irq_bits[callback_type];

	EPPRINTF("usb_device_endpoint_register_callback: type %s, bits "
	    "0x%02x\n", _usb_device_ep_irq_strings[callback_type],
	    (unsigned int)_usb_endpoint_irq_bits[callback_type]);

	return STATUS_OK;
}

/**
 * \brief Unregisters a USB device endpoint callback
 *
 * Unregisters an callback implemented by the user. Removing it
 * from the internal callback registration table.
 *
 * \param[in]     module_inst   Pointer to USB software instance struct
 * \param[in]     ep_num        Endpoint to configure
 * \param[in]     callback_type Callback type given by an enum
 *
 * \return Status of the de-registration operation.
 * \retval STATUS_OK    The callback was unregistered successfully.
 */
enum status_code usb_device_endpoint_unregister_callback(
		struct usb_module *module_inst, uint8_t ep_num,
		enum usb_device_endpoint_callback callback_type)
{
	/* Sanity check arguments */
	Assert(module_inst);
	Assert(ep_num < USB_EPT_NUM);

	/* Unregister callback function */
	module_inst->device_endpoint_callback[ep_num][callback_type] = NULL;

	/* Clear the bit corresponding to the callback_type */
	module_inst->device_endpoint_registered_callback_mask[ep_num] &= ~_usb_endpoint_irq_bits[callback_type];

	EPPRINTF("usb_device_endpoint_unregister_callback: type %s, bits "
	    "0x%02x\n", _usb_device_ep_irq_strings[callback_type],
	    (unsigned int)_usb_endpoint_irq_bits[callback_type]);

	return STATUS_OK;
}

/**
 * \brief Enables USB device endpoint callback generation for a given type.
 *
 * Enables callbacks for a given logical type.
 * This must be called before USB device pipe generate callback events.
 *
 * \param[in]     module_inst   Pointer to USB software instance struct
 * \param[in]     ep            Endpoint to configure
 * \param[in]     callback_type Callback type given by an enum
 *
 * \return Status of the callback enable operation.
 * \retval STATUS_OK    The callback was enabled successfully.
 */
enum status_code usb_device_endpoint_enable_callback(
		struct usb_module *module_inst, uint8_t ep,
		enum usb_device_endpoint_callback callback_type)
{
	/* Sanity check arguments */
	Assert(module_inst);
	Assert(module_inst->hw);

	uint8_t ep_num = ep & USB_EP_ADDR_MASK;
	Assert(ep_num < USB_EPT_NUM);

	EPPRINTF("usb_device_endpoint_enable_callback: ep %02x, type "
	    "%s, bits 0x%02x\n", (unsigned int)ep,
	    _usb_device_ep_irq_strings[callback_type],
	    (unsigned int)_usb_endpoint_irq_bits[callback_type]);

	/* Enable callback */
	module_inst->device_endpoint_enabled_callback_mask[ep_num] |= _usb_endpoint_irq_bits[callback_type];

	if (callback_type == USB_DEVICE_ENDPOINT_CALLBACK_TRCPT) {
		if (ep_num == 0) { // control endpoint
			module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPINTENSET.reg = USB_DEVICE_EPINTENSET_TRCPT0 | USB_DEVICE_EPINTENSET_TRCPT1;
		} else if (ep & USB_EP_DIR_IN) {
			module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPINTENSET.reg = USB_DEVICE_EPINTENSET_TRCPT1;
		} else {
			module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPINTENSET.reg = USB_DEVICE_EPINTENSET_TRCPT0;
		}
	}

	if (callback_type == USB_DEVICE_ENDPOINT_CALLBACK_TRFAIL) {
		if (ep_num == 0) { // control endpoint
			module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPINTENSET.reg = USB_DEVICE_EPINTENSET_TRFAIL0 | USB_DEVICE_EPINTENSET_TRFAIL1;
		} else if (ep & USB_EP_DIR_IN) {
			module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPINTENSET.reg = USB_DEVICE_EPINTENSET_TRFAIL1;
		} else {
			module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPINTENSET.reg = USB_DEVICE_EPINTENSET_TRFAIL0;
		}
	}

	if (callback_type == USB_DEVICE_ENDPOINT_CALLBACK_RXSTP) {
		module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPINTENSET.reg = USB_DEVICE_EPINTENSET_RXSTP;
	}

	if (callback_type == USB_DEVICE_ENDPOINT_CALLBACK_STALL) {
		if (ep & USB_EP_DIR_IN) {
			module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPINTENSET.reg = USB_DEVICE_EPINTENSET_STALL1;
		} else {
			module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPINTENSET.reg = USB_DEVICE_EPINTENSET_STALL0;
		}
	}

	return STATUS_OK;
}

/**
 * \brief Disables USB device endpoint callback generation for a given type.
 *
 * Disables callbacks for a given logical type.
 *
 * \param[in]     module_inst   Pointer to USB software instance struct
 * \param[in]     ep            Endpoint to configure
 * \param[in]     callback_type Callback type given by an enum
 *
 * \return Status of the callback disable operation.
 * \retval STATUS_OK    The callback was disabled successfully.
 */
enum status_code usb_device_endpoint_disable_callback(
		struct usb_module *module_inst, uint8_t ep,
		enum usb_device_endpoint_callback callback_type)
{
	/* Sanity check arguments */
	Assert(module_inst);
	Assert(module_inst->hw);

	uint8_t ep_num = ep & USB_EP_ADDR_MASK;
	Assert(ep_num < USB_EPT_NUM);

	EPPRINTF("usb_device_endpoint_disable_callback: ep %02x, type %s, "
	    "bits 0x%02x\n", (unsigned int)ep,
	    _usb_device_ep_irq_strings[callback_type],
	    (unsigned int)_usb_endpoint_irq_bits[callback_type]);

	/* Enable callback */
	module_inst->device_endpoint_enabled_callback_mask[ep_num] &= ~_usb_endpoint_irq_bits[callback_type];

	if (callback_type == USB_DEVICE_ENDPOINT_CALLBACK_TRCPT) {
		if (ep_num == 0) { // control endpoint
			module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPINTENCLR.reg =  USB_DEVICE_EPINTENCLR_TRCPT0 | USB_DEVICE_EPINTENCLR_TRCPT1;
		} else if (ep & USB_EP_DIR_IN) {
			module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPINTENCLR.reg =  USB_DEVICE_EPINTENCLR_TRCPT1;
		} else {
			module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPINTENCLR.reg =  USB_DEVICE_EPINTENCLR_TRCPT0;
		}
	}

	if (callback_type == USB_DEVICE_ENDPOINT_CALLBACK_TRFAIL) {
		if (ep_num == 0) { // control endpoint
			module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPINTENCLR.reg = USB_DEVICE_EPINTENCLR_TRFAIL0 | USB_DEVICE_EPINTENCLR_TRFAIL1;
		} else if (ep & USB_EP_DIR_IN) {
			module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPINTENCLR.reg = USB_DEVICE_EPINTENCLR_TRFAIL1;
		} else {
			module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPINTENCLR.reg = USB_DEVICE_EPINTENCLR_TRFAIL0;
		}
	}

	if (callback_type == USB_DEVICE_ENDPOINT_CALLBACK_RXSTP) {
		module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPINTENCLR.reg = USB_DEVICE_EPINTENCLR_RXSTP;
	}

	if (callback_type == USB_DEVICE_ENDPOINT_CALLBACK_STALL) {
		if (ep & USB_EP_DIR_IN) {
			module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPINTENCLR.reg = USB_DEVICE_EPINTENCLR_STALL1;
		} else {
			module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPINTENCLR.reg = USB_DEVICE_EPINTENCLR_STALL0;
		}
	}

	return STATUS_OK;
}

/**
 * \brief Initializes an USB device endpoint configuration structure to defaults.
 *
 * Initializes a given USB device endpoint configuration structure to a
 * set of known default values. This function should be called on all new
 * instances of these configuration structures before being modified by the
 * user application.
 *
 * The default configuration is as follows:
 * \li endpoint address is 0
 * \li endpoint size is 8 bytes
 * \li auto_zlp is false
 * \li endpoint type is control
 *
 * \param[out] ep_config  Configuration structure to initialize to default values
 */
void usb_device_endpoint_get_config_defaults(struct usb_device_endpoint_config *ep_config)
{
	/* Sanity check arguments */
	Assert(ep_config);

	/* Write default config to config struct */
	ep_config->ep_address = 0;
	ep_config->ep_size = USB_ENDPOINT_8_BYTE;
	ep_config->auto_zlp = false;
	ep_config->ep_type = USB_DEVICE_ENDPOINT_TYPE_CONTROL;
}

/**
 * \brief Writes an USB device endpoint configuration to the hardware module.
 *
 * Writes out a given configuration of an USB device endpoint
 * configuration to the hardware module. If the pipe is already configured,
 * the new configuration will replace the existing one.
 *
 * \param[in] module_inst    Pointer to USB software instance struct
 * \param[in] ep_config      Configuration settings for the endpoint
 *
 * \return Status of the device endpoint configuration operation
 * \retval STATUS_OK         The device endpoint was configured successfully
 * \retval STATUS_ERR_DENIED The endpoint address is already configured
 */
enum status_code usb_device_endpoint_set_config(struct usb_module *module_inst,
		struct usb_device_endpoint_config *ep_config)
{
	/* Sanity check arguments */
	Assert(module_inst);
	Assert(ep_config);

	uint8_t ep_num = ep_config->ep_address & USB_EP_ADDR_MASK;
	uint8_t ep_bank = (ep_config->ep_address & USB_EP_DIR_IN) ? 1 : 0;

	switch (ep_config->ep_type) {
		case USB_DEVICE_ENDPOINT_TYPE_DISABLE:
			module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPCFG.reg = USB_DEVICE_EPCFG_EPTYPE0(0) |  USB_DEVICE_EPCFG_EPTYPE1(0);
			return STATUS_OK;

		case USB_DEVICE_ENDPOINT_TYPE_CONTROL:
			if ((module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPCFG.reg & USB_DEVICE_EPCFG_EPTYPE0_Msk) == 0 && \
				(module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPCFG.reg & USB_DEVICE_EPCFG_EPTYPE1_Msk) == 0) {
				module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPCFG.reg = USB_DEVICE_EPCFG_EPTYPE0(1) | USB_DEVICE_EPCFG_EPTYPE1(1);
				module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_BK0RDY;
				module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_BK1RDY;
			} else {
				return STATUS_ERR_DENIED;
			}
			CMCC_CleanInvalidateDCache_by_Addr(usb_descriptor_table.usb_endpoint_table[ep_num].DeviceDescBank,
			    sizeof(usb_descriptor_table.usb_endpoint_table[ep_num].DeviceDescBank));
			if (true == ep_config->auto_zlp) {
				usb_descriptor_table.usb_endpoint_table[ep_num].DeviceDescBank[0].PCKSIZE.reg |= USB_DEVICE_PCKSIZE_AUTO_ZLP;
				usb_descriptor_table.usb_endpoint_table[ep_num].DeviceDescBank[1].PCKSIZE.reg |= USB_DEVICE_PCKSIZE_AUTO_ZLP;
			} else {
				usb_descriptor_table.usb_endpoint_table[ep_num].DeviceDescBank[0].PCKSIZE.reg &= ~USB_DEVICE_PCKSIZE_AUTO_ZLP;
				usb_descriptor_table.usb_endpoint_table[ep_num].DeviceDescBank[1].PCKSIZE.reg &= ~USB_DEVICE_PCKSIZE_AUTO_ZLP;
			}
			usb_descriptor_table.usb_endpoint_table[ep_num].DeviceDescBank[0].PCKSIZE.bit.SIZE = ep_config->ep_size;
			usb_descriptor_table.usb_endpoint_table[ep_num].DeviceDescBank[1].PCKSIZE.bit.SIZE = ep_config->ep_size;
			return STATUS_OK;

		case USB_DEVICE_ENDPOINT_TYPE_ISOCHRONOUS:
			if (ep_bank) {
				if ((module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPCFG.reg & USB_DEVICE_EPCFG_EPTYPE1_Msk) == 0){
					module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPCFG.reg |= USB_DEVICE_EPCFG_EPTYPE1(2);
					module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_BK1RDY;
				} else {
					return STATUS_ERR_DENIED;
				}
			} else {
				if ((module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPCFG.reg & USB_DEVICE_EPCFG_EPTYPE0_Msk) == 0){
					module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPCFG.reg |= USB_DEVICE_EPCFG_EPTYPE0(2);
					module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_BK0RDY;
				} else {
					return STATUS_ERR_DENIED;
				}
			}
			break;

		case USB_DEVICE_ENDPOINT_TYPE_BULK:
			if (ep_bank) {
				if ((module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPCFG.reg & USB_DEVICE_EPCFG_EPTYPE1_Msk) == 0){
					module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPCFG.reg |= USB_DEVICE_EPCFG_EPTYPE1(3);
					module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_BK1RDY;
				} else {
					return STATUS_ERR_DENIED;
				}
			} else {
				if ((module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPCFG.reg & USB_DEVICE_EPCFG_EPTYPE0_Msk) == 0){
					module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPCFG.reg |= USB_DEVICE_EPCFG_EPTYPE0(3);
					module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_BK0RDY;
				} else {
					return STATUS_ERR_DENIED;
				}
			}
			break;

		case USB_DEVICE_ENDPOINT_TYPE_INTERRUPT:
			if (ep_bank) {
				if ((module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPCFG.reg & USB_DEVICE_EPCFG_EPTYPE1_Msk) == 0){
					module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPCFG.reg |= USB_DEVICE_EPCFG_EPTYPE1(4);
					module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_BK1RDY;
				} else {
					return STATUS_ERR_DENIED;
				}
			} else {
				if ((module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPCFG.reg & USB_DEVICE_EPCFG_EPTYPE0_Msk) == 0){
					module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPCFG.reg |= USB_DEVICE_EPCFG_EPTYPE0(4);
					module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_BK0RDY;
				} else {
					return STATUS_ERR_DENIED;
				}
			}
			break;

		default:
			break;
	}

	CMCC_CleanInvalidateDCache_by_Addr(&usb_descriptor_table.usb_endpoint_table[ep_num].DeviceDescBank[ep_bank],
	    sizeof(usb_descriptor_table.usb_endpoint_table[ep_num].DeviceDescBank[ep_bank]));
	usb_descriptor_table.usb_endpoint_table[ep_num].DeviceDescBank[ep_bank].PCKSIZE.bit.SIZE = ep_config->ep_size;

	if (true == ep_config->auto_zlp) {
		usb_descriptor_table.usb_endpoint_table[ep_num].DeviceDescBank[ep_bank].PCKSIZE.reg |= USB_DEVICE_PCKSIZE_AUTO_ZLP;
		} else {
		usb_descriptor_table.usb_endpoint_table[ep_num].DeviceDescBank[ep_bank].PCKSIZE.reg &= ~USB_DEVICE_PCKSIZE_AUTO_ZLP;
	}

	return STATUS_OK;
}

/**
 * \brief Check if current endpoint is configured
 *
 * \param module_inst   Pointer to USB software instance struct
 * \param ep            Endpoint address (direction & number)
 *
 * \return \c true if endpoint is configured and ready to use
 */
bool usb_device_endpoint_is_configured(struct usb_module *module_inst, uint8_t ep)
{
	uint8_t ep_num = ep & USB_EP_ADDR_MASK;
	uint8_t flag;

	if (ep & USB_EP_DIR_IN) {
		flag = (uint8_t)(module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPCFG.bit.EPTYPE1);
	} else {
		flag = (uint8_t)(module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPCFG.bit.EPTYPE0);
	}
	return ((enum usb_device_endpoint_type)(flag) != USB_DEVICE_ENDPOINT_TYPE_DISABLE);
}


/**
 * \brief Abort ongoing job on the endpoint
 *
 * \param module_inst Pointer to USB software instance struct
 * \param ep          Endpoint address
 */
void usb_device_endpoint_abort_job(struct usb_module *module_inst, uint8_t ep)
{
	uint8_t ep_num;
	ep_num = ep & USB_EP_ADDR_MASK;

	// Stop transfer
	if (ep & USB_EP_DIR_IN) {
		module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_BK1RDY;
		// Eventually ack a transfer occur during abort
		module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT1;
	} else {
		module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_BK0RDY;
		// Eventually ack a transfer occur during abort
		module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT0;
	}
}

/**
 * \brief Check if endpoint is halted
 *
 * \param module_inst Pointer to USB software instance struct
 * \param ep          Endpoint address
 *
 * \return \c true if the endpoint is halted
 */
bool usb_device_endpoint_is_halted(struct usb_module *module_inst, uint8_t ep)
{
	uint8_t ep_num = ep & USB_EP_ADDR_MASK;

	if (ep & USB_EP_DIR_IN) {
		return (module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPSTATUS.reg & USB_DEVICE_EPSTATUSSET_STALLRQ1);
	} else {
		return (module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPSTATUS.reg & USB_DEVICE_EPSTATUSSET_STALLRQ0);
	}
}

/**
 * \brief Halt the endpoint (send STALL)
 *
 * \param module_inst Pointer to USB software instance struct
 * \param ep          Endpoint address
 */
void usb_device_endpoint_set_halt(struct usb_module *module_inst, uint8_t ep)
{
	uint8_t ep_num = ep & USB_EP_ADDR_MASK;

	// Stall endpoint
	if (ep & USB_EP_DIR_IN) {
		module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_STALLRQ1;
	} else {
		module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_STALLRQ0;
	}
}

/**
 * \brief Clear endpoint halt state
 *
 * \param module_inst Pointer to USB software instance struct
 * \param ep          Endpoint address
 */
void usb_device_endpoint_clear_halt(struct usb_module *module_inst, uint8_t ep)
{
	uint8_t ep_num = ep & USB_EP_ADDR_MASK;

	if (ep & USB_EP_DIR_IN) {
		if (module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPSTATUS.reg & USB_DEVICE_EPSTATUSSET_STALLRQ1) {
			// Remove stall request
			module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_STALLRQ1;
			if (module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPINTFLAG.reg & USB_DEVICE_EPINTFLAG_STALL1) {
				module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_STALL1;
				// The Stall has occurred, then reset data toggle
				module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSSET_DTGLIN;
			}
		}
	} else {
		if (module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPSTATUS.reg & USB_DEVICE_EPSTATUSSET_STALLRQ0) {
			// Remove stall request
			module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_STALLRQ0;
			if (module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPINTFLAG.reg & USB_DEVICE_EPINTFLAG_STALL0) {
				module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_STALL0;
				// The Stall has occurred, then reset data toggle
				module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSSET_DTGLOUT;
			}
		}
	}
}

/**
 * \brief Start write buffer job on a endpoint
 *
 * \param module_inst Pointer to USB module instance
 * \param ep_num      Endpoint number
 * \param pbuf        Pointer to buffer
 * \param buf_size    Size of buffer
 *
 * \return Status of procedure
 * \retval STATUS_OK Job started successfully
 * \retval STATUS_ERR_DENIED Endpoint is not ready
 */
enum status_code usb_device_endpoint_write_buffer_job(struct usb_module *module_inst,uint8_t ep_num,
		uint8_t* pbuf, uint32_t buf_size)
{
	/* Sanity check arguments */
	Assert(module_inst);
	Assert(module_inst->hw);
	Assert(ep_num < USB_EPT_NUM);

	uint8_t flag;
	flag = (uint8_t)(module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPCFG.bit.EPTYPE1);
	if ((enum usb_device_endpoint_type)(flag) == USB_DEVICE_ENDPOINT_TYPE_DISABLE) {
		return STATUS_ERR_DENIED;
	};

	CMCC_CleanDCache_by_Addr(pbuf, buf_size);
	CMCC_CleanInvalidateDCache_by_Addr(&usb_descriptor_table.usb_endpoint_table[ep_num].DeviceDescBank[1],
	    sizeof(usb_descriptor_table.usb_endpoint_table[ep_num].DeviceDescBank[1]));

	/* get endpoint configuration from setting register */
	usb_descriptor_table.usb_endpoint_table[ep_num].DeviceDescBank[1].ADDR.reg = (uint32_t)pbuf;
	usb_descriptor_table.usb_endpoint_table[ep_num].DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;
	usb_descriptor_table.usb_endpoint_table[ep_num].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = buf_size;
	module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_BK1RDY;

	return STATUS_OK;
}

/**
 * \brief Start read buffer job on a endpoint
 *
 * \param module_inst Pointer to USB module instance
 * \param ep_num      Endpoint number
 * \param pbuf        Pointer to buffer
 * \param buf_size    Size of buffer
 *
 * \return Status of procedure
 * \retval STATUS_OK Job started successfully
 * \retval STATUS_ERR_DENIED Endpoint is not ready
 */
enum status_code usb_device_endpoint_read_buffer_job(struct usb_module *module_inst,uint8_t ep_num,
		uint8_t* pbuf, uint32_t buf_size)
{
	/* Sanity check arguments */
	Assert(module_inst);
	Assert(module_inst->hw);
	Assert(ep_num < USB_EPT_NUM);

	uint8_t flag;
	flag = (uint8_t)(module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPCFG.bit.EPTYPE0);
	if ((enum usb_device_endpoint_type)(flag) == USB_DEVICE_ENDPOINT_TYPE_DISABLE) {
		return STATUS_ERR_DENIED;
	};

	CMCC_CleanInvalidateDCache_by_Addr(pbuf, buf_size);
	CMCC_CleanInvalidateDCache_by_Addr(&usb_descriptor_table.usb_endpoint_table[ep_num].DeviceDescBank[0],
	    sizeof(usb_descriptor_table.usb_endpoint_table[ep_num].DeviceDescBank[0]));

	/* get endpoint configuration from setting register */
	usb_descriptor_table.usb_endpoint_table[ep_num].DeviceDescBank[0].ADDR.reg = (uint32_t)pbuf;
	usb_descriptor_table.usb_endpoint_table[ep_num].DeviceDescBank[0].PCKSIZE.bit.MULTI_PACKET_SIZE = buf_size;
	usb_descriptor_table.usb_endpoint_table[ep_num].DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT = 0;
	module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_BK0RDY;

	return STATUS_OK;
}

/**
 * \brief Start setup packet read job on a endpoint
 *
 * \param module_inst Pointer to USB device module instance
 * \param pbuf        Pointer to buffer
 *
 * \return Status of procedure
 * \retval STATUS_OK Job started successfully
 * \retval STATUS_ERR_DENIED Endpoint is not ready
 */
enum status_code usb_device_endpoint_setup_buffer_job(struct usb_module *module_inst,
		uint8_t* pbuf)
{
	/* Sanity check arguments */
	Assert(module_inst);
	Assert(module_inst->hw);

	CMCC_CleanInvalidateDCache_by_Addr(pbuf, 8);
	CMCC_CleanInvalidateDCache_by_Addr(&usb_descriptor_table.usb_endpoint_table[0].DeviceDescBank[0],
	    sizeof(usb_descriptor_table.usb_endpoint_table[0].DeviceDescBank[0]));

	/* get endpoint configuration from setting register */
	usb_descriptor_table.usb_endpoint_table[0].DeviceDescBank[0].ADDR.reg = (uint32_t)pbuf;
	usb_descriptor_table.usb_endpoint_table[0].DeviceDescBank[0].PCKSIZE.bit.MULTI_PACKET_SIZE = 8;
	usb_descriptor_table.usb_endpoint_table[0].DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT = 0;
	module_inst->hw->DEVICE.DeviceEndpoint[0].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_BK0RDY;

	return STATUS_OK;
}

static void _usb_device_interrupt_handler(struct usb_module *module_inst)
{
	uint16_t ep_inst;
	uint16_t flags, flags_run;
	ep_inst = module_inst->hw->DEVICE.EPINTSMRY.reg;

	/* device interrupt */
	if (0 == ep_inst) {
		int i;

		/* get interrupt flags */
		flags = module_inst->hw->DEVICE.INTFLAG.reg;
		flags_run = flags &
				module_inst->device_enabled_callback_mask &
				module_inst->device_registered_callback_mask;

		for (i = 0; i < USB_DEVICE_CALLBACK_N; i ++) {
			if (flags_run & _usb_device_irq_bits[i]) {
				module_inst->hw->DEVICE.INTFLAG.reg =
						_usb_device_irq_bits[i];
#ifdef SCW_USB_STATS
				_usb_device_irq_count[i]++;
#endif
			}
			if (flags_run & _usb_device_irq_bits[i]) {
				if (i == USB_DEVICE_CALLBACK_LPMSUSP) {
					CMCC_CleanInvalidateDCache_by_Addr(&usb_descriptor_table.usb_endpoint_table[0].DeviceDescBank[0],
					    sizeof(usb_descriptor_table.usb_endpoint_table[0].DeviceDescBank[0]));
					device_callback_lpm_wakeup_enable =
							usb_descriptor_table.usb_endpoint_table[0].DeviceDescBank[0].EXTREG.bit.VARIABLE
							& USB_LPM_ATTRIBUT_REMOTEWAKE_MASK;
				}
				(module_inst->device_callback[i])(module_inst, &device_callback_lpm_wakeup_enable);
			}
		}

	} else {
		/* endpoint interrupt */

		for (uint8_t i = 0; i < USB_EPT_NUM; i++) {

			if (ep_inst & (1 << i)) {
				flags = module_inst->hw->DEVICE.DeviceEndpoint[i].EPINTFLAG.reg;
				flags_run = flags &
						module_inst->device_endpoint_enabled_callback_mask[i] &
						module_inst->device_endpoint_registered_callback_mask[i];

				CMCC_CleanInvalidateDCache_by_Addr(usb_descriptor_table.usb_endpoint_table[i].DeviceDescBank,
				    sizeof(usb_descriptor_table.usb_endpoint_table[i].DeviceDescBank));

				// endpoint transfer stall interrupt
				if (flags & USB_DEVICE_EPINTFLAG_STALL_Msk) {
#ifdef SCW_USB_STATS
					_usb_device_ep_irq_count[i][USB_DEVICE_ENDPOINT_CALLBACK_STALL]++;
#endif
					if (module_inst->hw->DEVICE.DeviceEndpoint[i].EPINTFLAG.reg & USB_DEVICE_EPINTFLAG_STALL1) {
						module_inst->hw->DEVICE.DeviceEndpoint[i].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_STALL1;
						ep_callback_para.endpoint_address = USB_EP_DIR_IN | i;
					} else if (module_inst->hw->DEVICE.DeviceEndpoint[i].EPINTFLAG.reg & USB_DEVICE_EPINTFLAG_STALL0) {
						module_inst->hw->DEVICE.DeviceEndpoint[i].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_STALL0;
						ep_callback_para.endpoint_address = USB_EP_DIR_OUT | i;
					}

					if (flags_run & USB_DEVICE_EPINTFLAG_STALL_Msk) {
						(module_inst->device_endpoint_callback[i][USB_DEVICE_ENDPOINT_CALLBACK_STALL])(module_inst,&ep_callback_para);
					}
					return;
				}

				// endpoint received setup interrupt
				if (flags & USB_DEVICE_EPINTFLAG_RXSTP) {
#ifdef SCW_USB_STATS
					_usb_device_ep_irq_count[i][USB_DEVICE_ENDPOINT_CALLBACK_RXSTP]++;
#endif
					module_inst->hw->DEVICE.DeviceEndpoint[i].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_RXSTP;
					if(module_inst->device_endpoint_enabled_callback_mask[i] & _usb_endpoint_irq_bits[USB_DEVICE_ENDPOINT_CALLBACK_RXSTP]) {
						ep_callback_para.received_bytes = (uint16_t)(usb_descriptor_table.usb_endpoint_table[i].DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT);
						(module_inst->device_endpoint_callback[i][USB_DEVICE_ENDPOINT_CALLBACK_RXSTP])(module_inst,&ep_callback_para);
					}
					return;
				}

				// endpoint transfer complete interrupt
				if (flags & USB_DEVICE_EPINTFLAG_TRCPT_Msk) {
#ifdef SCW_USB_STATS
					_usb_device_ep_irq_count[i][USB_DEVICE_ENDPOINT_CALLBACK_TRCPT]++;
#endif
					if (module_inst->hw->DEVICE.DeviceEndpoint[i].EPINTFLAG.reg & USB_DEVICE_EPINTFLAG_TRCPT1) {
						module_inst->hw->DEVICE.DeviceEndpoint[i].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT1;
						ep_callback_para.endpoint_address = USB_EP_DIR_IN | i;
						ep_callback_para.sent_bytes = (uint16_t)(usb_descriptor_table.usb_endpoint_table[i].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT);

					} else if (module_inst->hw->DEVICE.DeviceEndpoint[i].EPINTFLAG.reg & USB_DEVICE_EPINTFLAG_TRCPT0) {
						module_inst->hw->DEVICE.DeviceEndpoint[i].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT0;
						ep_callback_para.endpoint_address = USB_EP_DIR_OUT | i;
						ep_callback_para.received_bytes = (uint16_t)(usb_descriptor_table.usb_endpoint_table[i].DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT);
						ep_callback_para.out_buffer_size = (uint16_t)(usb_descriptor_table.usb_endpoint_table[i].DeviceDescBank[0].PCKSIZE.bit.MULTI_PACKET_SIZE);
					}
					if(flags_run & USB_DEVICE_EPINTFLAG_TRCPT_Msk) {
						(module_inst->device_endpoint_callback[i][USB_DEVICE_ENDPOINT_CALLBACK_TRCPT])(module_inst,&ep_callback_para);
					}
					return;
				}

				// endpoint transfer fail interrupt
				if (flags & USB_DEVICE_EPINTFLAG_TRFAIL_Msk) {
#ifdef SCW_USB_STATS
					_usb_device_ep_irq_count[i][USB_DEVICE_ENDPOINT_CALLBACK_TRFAIL]++;
#endif
					if (module_inst->hw->DEVICE.DeviceEndpoint[i].EPINTFLAG.reg & USB_DEVICE_EPINTFLAG_TRFAIL1) {
						module_inst->hw->DEVICE.DeviceEndpoint[i].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRFAIL1;
						if (usb_descriptor_table.usb_endpoint_table[i].DeviceDescBank[1].STATUS_BK.reg & USB_DEVICE_STATUS_BK_ERRORFLOW) {
							usb_descriptor_table.usb_endpoint_table[i].DeviceDescBank[1].STATUS_BK.reg &= ~USB_DEVICE_STATUS_BK_ERRORFLOW;
						}
						ep_callback_para.endpoint_address = USB_EP_DIR_IN | i;
						if (module_inst->hw->DEVICE.DeviceEndpoint[i].EPINTFLAG.reg & USB_DEVICE_EPINTFLAG_TRCPT1) {
							return;
						}
					} else if(module_inst->hw->DEVICE.DeviceEndpoint[i].EPINTFLAG.reg & USB_DEVICE_EPINTFLAG_TRFAIL0) {
						module_inst->hw->DEVICE.DeviceEndpoint[i].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRFAIL0;
						if (usb_descriptor_table.usb_endpoint_table[i].DeviceDescBank[0].STATUS_BK.reg & USB_DEVICE_STATUS_BK_ERRORFLOW) {
							usb_descriptor_table.usb_endpoint_table[i].DeviceDescBank[0].STATUS_BK.reg &= ~USB_DEVICE_STATUS_BK_ERRORFLOW;
						}
						ep_callback_para.endpoint_address = USB_EP_DIR_OUT | i;
						if (module_inst->hw->DEVICE.DeviceEndpoint[i].EPINTFLAG.reg & USB_DEVICE_EPINTFLAG_TRCPT0) {
							return;
						}
					}

					if(flags_run & USB_DEVICE_EPINTFLAG_TRFAIL_Msk) {
						(module_inst->device_endpoint_callback[i][USB_DEVICE_ENDPOINT_CALLBACK_TRFAIL])(module_inst,&ep_callback_para);
					}
					return;
				}
			}
		}
	}
}

/**
 * \brief Enable the USB module peripheral
 *
 * \param module_inst pointer to USB module instance
 */
void usb_enable(struct usb_module *module_inst)
{
	Assert(module_inst);
	Assert(module_inst->hw);

	module_inst->hw->DEVICE.CTRLA.reg |= USB_CTRLA_ENABLE;
	while (module_inst->hw->DEVICE.SYNCBUSY.reg == USB_SYNCBUSY_ENABLE);
}

/**
 * \brief Disable the USB module peripheral
 *
 * \param module_inst pointer to USB module instance
 */
void usb_disable(struct usb_module *module_inst)
{
	Assert(module_inst);
	Assert(module_inst->hw);

	module_inst->hw->DEVICE.INTENCLR.reg = USB_DEVICE_INTENCLR_MASK;
	module_inst->hw->DEVICE.INTFLAG.reg = USB_DEVICE_INTFLAG_MASK;
	module_inst->hw->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE;
	while (module_inst->hw->DEVICE.SYNCBUSY.reg == USB_SYNCBUSY_ENABLE);
}

/**
 * \brief Interrupt handler for the USB module.
 */
static void USB_IRQ_Handler(void *arg)
{
	struct usb_module *module_inst = arg;

#ifdef SCW_USB_STATS
	_usb_device_irq_total++;
#endif

	/*device mode ISR */
	_usb_device_interrupt_handler(module_inst);
}

/**
 * \brief Get the default USB module settings
 *
 * \param[out] module_config  Configuration structure to initialize to default values
 */
void sam_usb_get_config_defaults(struct usb_config *module_config)
{
	Assert(module_config);

	/* Sanity check arguments */
	Assert(module_config);
	/* Write default configuration to config struct */
	module_config->select_host_mode = 0;
	module_config->run_in_standby = 1;
	module_config->speed_mode = USB_SPEED_FULL;
}

#define NVM_USB_PAD_TRANSN_POS  45
#define NVM_USB_PAD_TRANSN_SIZE 5
#define NVM_USB_PAD_TRANSP_POS  50
#define NVM_USB_PAD_TRANSP_SIZE 5
#define NVM_USB_PAD_TRIM_POS  55
#define NVM_USB_PAD_TRIM_SIZE 3

/**
 * \brief Initializes USB module instance
 *
 * Enables the clock and initializes the USB module, based on the given
 * configuration values.
 *
 * \param[in,out] module_inst   Pointer to the software module instance struct
 * \param[in]     hw            Pointer to the USB hardware module
 * \param[in]     module_config Pointer to the USB configuration options struct
 *
 * \return Status of the initialization procedure.
 *
 * \retval STATUS_OK           The module was initialized successfully
 */
enum status_code sam_usb_init(struct usb_module *module_inst, Usb *const hw,
		struct usb_config *module_config)
{
	/* Sanity check arguments */
	Assert(hw);
	Assert(module_inst);
	Assert(module_config);

	uint32_t i,j;
	uint16_t padcal;

	/* Associate the software module instance with the hardware module */
	module_inst->hw = hw;

	sam_usb_platform_init();

	/* Reset */
	hw->DEVICE.CTRLA.bit.SWRST = 1;
	while (hw->DEVICE.SYNCBUSY.bit.SWRST) {
		/* Sync wait */
	}

	/* Load Pad Calibration */
	padcal = sam_usb_get_padcal();
	hw->DEVICE.PADCAL.reg = padcal;

	/* Set the configuration */
	hw->DEVICE.CTRLA.bit.MODE = module_config->select_host_mode;
	hw->DEVICE.CTRLA.bit.RUNSTDBY = module_config->run_in_standby;
	hw->DEVICE.DESCADD.reg = (uint32_t)(&usb_descriptor_table.usb_endpoint_table[0]);
	if (USB_SPEED_FULL == module_config->speed_mode) {
		module_inst->hw->DEVICE.CTRLB.bit.SPDCONF = USB_DEVICE_CTRLB_SPDCONF_FS_Val;
	} else if(USB_SPEED_LOW == module_config->speed_mode) {
		module_inst->hw->DEVICE.CTRLB.bit.SPDCONF = USB_DEVICE_CTRLB_SPDCONF_LS_Val;
	}

	memset((uint8_t *)(&usb_descriptor_table.usb_endpoint_table[0]), 0,
			sizeof(usb_descriptor_table.usb_endpoint_table));

	/*  device callback related */
	for (i = 0; i < USB_DEVICE_CALLBACK_N; i++) {
		module_inst->device_callback[i] = NULL;
	}
	for (i = 0; i < USB_EPT_NUM; i++) {
		for(j = 0; j < USB_DEVICE_EP_CALLBACK_N; j++) {
			module_inst->device_endpoint_callback[i][j] = NULL;
		}
	}
	module_inst->device_registered_callback_mask = 0;
	module_inst->device_enabled_callback_mask = 0;
	for (j = 0; j < USB_EPT_NUM; j++) {
		module_inst->device_endpoint_registered_callback_mask[j] = 0;
		module_inst->device_endpoint_enabled_callback_mask[j] = 0;
	}

	/* Enable interrupts for this USB module */

	module_inst->irq[0] = rtos_irq_register_named("USB", USB_0_IRQn, HW_IPL_USB, USB_IRQ_Handler, module_inst);
	Assert(module_inst->irq[0] != NULL);

	module_inst->irq[1] = rtos_irq_register_named("USBSOF", USB_1_IRQn, HW_IPL_USB, USB_IRQ_Handler, module_inst);
	Assert(module_inst->irq[1] != NULL);

	module_inst->irq[2] = rtos_irq_register_named("USBTC0", USB_2_IRQn, HW_IPL_USB, USB_IRQ_Handler, module_inst);
	Assert(module_inst->irq[2] != NULL);

	module_inst->irq[3] = rtos_irq_register_named("USBTC1", USB_3_IRQn, HW_IPL_USB, USB_IRQ_Handler, module_inst);
	Assert(module_inst->irq[3] != NULL);

	return STATUS_OK;
}


void sam_usb_deinit(struct usb_module *module_inst)
{
	/* Reset */
	module_inst->hw->DEVICE.CTRLA.bit.SWRST = 1;
	while (module_inst->hw->DEVICE.SYNCBUSY.bit.SWRST) {
		/* Sync wait */
	}

	for (unsigned int i = i; i < 4; i++) {
		if (module_inst->irq[i] != NULL) {
			rtos_soft_irq_unregister(module_inst->irq[i]);
			module_inst->irq[i] = NULL;
		}
	}

	sam_usb_platform_deinit();
}
