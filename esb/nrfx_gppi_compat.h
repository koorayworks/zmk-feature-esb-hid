/*
 * Copyright (c) 2026 KeyboardBrain contributors
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/*
 * Compatibility shim for building the vendored ESB stack on plain Zephyr.
 *
 * The vendored esb_ppi.c / esb_dppi.c use the newer nrfx GPPI helper API
 * (paired with NCS), where channel allocation returns the channel number
 * (or a negative error) and free functions take a leading flags/handle
 * argument:
 *
 *   int  nrfx_gppi_channel_alloc(uint32_t flags);
 *   void nrfx_gppi_channel_free(uint32_t flags, uint8_t channel);
 *   int  nrfx_gppi_group_channel_alloc(uint32_t flags);
 *   void nrfx_gppi_group_channel_free(uint32_t flags, uint8_t group);
 *
 * Zephyr 4.1's bundled nrfx provides the older helper API with
 * out-parameters and nrfx_err_t returns:
 *
 *   nrfx_err_t nrfx_gppi_channel_alloc(uint8_t *p_channel);
 *   nrfx_err_t nrfx_gppi_channel_free(uint8_t channel);
 *   nrfx_err_t nrfx_gppi_group_alloc(nrfx_gppi_channel_group_t *p_group);
 *   nrfx_err_t nrfx_gppi_group_free(nrfx_gppi_channel_group_t group);
 *
 * This header includes the Zephyr helper header first (so later includes
 * of <helpers/nrfx_gppi.h> in the vendored sources are no-ops), then
 * maps the four newer entry points onto wrappers over the older API.
 * The flags argument is always 0 (default allocator) in the vendored
 * code and is ignored.
 *
 * It also covers NRFX_INSTANCE_IRQ_HANDLER_DEFINE(), used by esb.c:
 * the newer nrfx expects the application to define the instance IRQ
 * handler with this macro, while Zephyr 4.1's nrfx_timer/nrfx_rtc
 * drivers already generate nrfx_<periph>_<id>_irq_handler() for every
 * enabled instance (ESB connects to that symbol via irq_connect_dynamic
 * and the handler operates on the same per-instance control block).
 * Defining the macro as empty is therefore the equivalent behaviour.
 */

#ifndef ESB_NRFX_GPPI_COMPAT_H_
#define ESB_NRFX_GPPI_COMPAT_H_

#include <errno.h>

#include <helpers/nrfx_gppi.h>

static inline int esb_compat_gppi_channel_alloc(uint32_t flags)
{
	uint8_t channel;

	(void)flags;
	if ((nrfx_gppi_channel_alloc)(&channel) != NRFX_SUCCESS) {
		return -ENOMEM;
	}
	return (int)channel;
}

static inline void esb_compat_gppi_channel_free(uint32_t flags, uint8_t channel)
{
	(void)flags;
	(void)(nrfx_gppi_channel_free)(channel);
}

static inline int esb_compat_gppi_group_channel_alloc(uint32_t flags)
{
	nrfx_gppi_channel_group_t group;

	(void)flags;
	if ((nrfx_gppi_group_alloc)(&group) != NRFX_SUCCESS) {
		return -ENOMEM;
	}
	return (int)group;
}

static inline void esb_compat_gppi_group_channel_free(uint32_t flags, uint8_t group)
{
	(void)flags;
	(void)(nrfx_gppi_group_free)((nrfx_gppi_channel_group_t)group);
}

#define nrfx_gppi_channel_alloc(flags) esb_compat_gppi_channel_alloc(flags)
#define nrfx_gppi_channel_free(flags, channel) esb_compat_gppi_channel_free(flags, channel)
#define nrfx_gppi_group_channel_alloc(flags) esb_compat_gppi_group_channel_alloc(flags)
#define nrfx_gppi_group_channel_free(flags, group)                                             \
	esb_compat_gppi_group_channel_free(flags, group)

/* See the header comment: Zephyr 4.1's nrfx drivers already generate
 * the instance IRQ handlers, so this define is a no-op here.
 */
#ifndef NRFX_INSTANCE_IRQ_HANDLER_DEFINE
#define NRFX_INSTANCE_IRQ_HANDLER_DEFINE(periph, id, p_instance)
#endif

#endif /* ESB_NRFX_GPPI_COMPAT_H_ */
