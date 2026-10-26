/*
 * Copyright (c) 2026 KeyboardBrain contributors
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/*
 * Compatibility shim for building the vendored ESB stack on plain Zephyr.
 *
 * The NCS version of the nrfx MDK provides NRF_ERRATA_STATIC_CHECK() in
 * <nrf_erratas.h>. Zephyr's bundled nrfx MDK leaves <nrf_erratas.h> as an
 * empty stub and instead exposes per-errata macros named
 * NRF<series>_ERRATA_<number>_PRESENT and
 * NRF<series>_ERRATA_<number>_ENABLE_WORKAROUND in the per-series errata
 * headers (e.g. <nrf52_erratas.h>). Define the NCS-style check on top of
 * those macros when it is missing.
 *
 * Errata numbers that have no PRESENT macro in the included series
 * header evaluate to 0 in #if expressions, i.e. "workaround not
 * applicable", which matches a build for a different SoC series.
 */

#ifndef ESB_NRF_ERRATA_COMPAT_H_
#define ESB_NRF_ERRATA_COMPAT_H_

#include <nrf.h>

#if defined(NRF51_SERIES)
#include <nrf51_erratas.h>
#elif defined(NRF52_SERIES)
#include <nrf52_erratas.h>
#elif defined(NRF53_SERIES)
#include <nrf53_erratas.h>
#elif defined(NRF54L_SERIES)
#include <nrf54l_erratas.h>
#elif defined(NRF54H_SERIES)
#include <nrf54h_erratas.h>
#elif defined(NRF91_SERIES)
#include <nrf91_erratas.h>
#endif

#ifndef NRF_ERRATA_STATIC_CHECK
#define NRF_ERRATA_STATIC_CHECK(series, number)                                                \
	(NRF##series##_ERRATA_##number##_PRESENT &&                                            \
	 NRF##series##_ERRATA_##number##_ENABLE_WORKAROUND)
#endif

/*
 * NRF_ERRATA_DYNAMIC_CHECK(series, number) is the runtime counterpart:
 * in the NCS MDK it combines the ENABLE_WORKAROUND flag with the
 * per-erratum runtime applicability function from the series errata
 * header (nrf<series>_errata_<number>()). Dispatch on the series
 * argument; errata of a series other than the one being built for do
 * not apply and evaluate to false.
 */
#if defined(NRF52_SERIES)
#define ESB_ERRATA_DYNAMIC_CHECK_52(number)                                                    \
	(NRF52_ERRATA_##number##_ENABLE_WORKAROUND && nrf52_errata_##number())
#else
#define ESB_ERRATA_DYNAMIC_CHECK_52(number) (0)
#endif

#if defined(NRF53_SERIES)
#define ESB_ERRATA_DYNAMIC_CHECK_53(number)                                                    \
	(NRF53_ERRATA_##number##_ENABLE_WORKAROUND && nrf53_errata_##number())
#else
#define ESB_ERRATA_DYNAMIC_CHECK_53(number) (0)
#endif

#if defined(NRF54H_SERIES)
#define ESB_ERRATA_DYNAMIC_CHECK_54H(number)                                                   \
	(NRF54H_ERRATA_##number##_ENABLE_WORKAROUND && nrf54h_errata_##number())
#else
#define ESB_ERRATA_DYNAMIC_CHECK_54H(number) (0)
#endif

#if defined(NRF54L_SERIES)
#define ESB_ERRATA_DYNAMIC_CHECK_54L(number)                                                   \
	(NRF54L_ERRATA_##number##_ENABLE_WORKAROUND && nrf54l_errata_##number())
#else
#define ESB_ERRATA_DYNAMIC_CHECK_54L(number) (0)
#endif

#ifndef NRF_ERRATA_DYNAMIC_CHECK
#define NRF_ERRATA_DYNAMIC_CHECK(series, number) ESB_ERRATA_DYNAMIC_CHECK_##series(number)
#endif

/*
 * MPSL_TIMER0 is defined by <mpsl_hwres.h> in NCS (the timer instance
 * MPSL reserves). esb.c references it in a BUILD_ASSERT that is vacuous
 * when CONFIG_ESB_MPSL_TIMESLOT is off, but the identifier must exist.
 * Mirror the mpsl_hwres.h values per SoC series.
 */
#ifndef MPSL_TIMER0
#if defined(NRF52_SERIES)
#define MPSL_TIMER0 NRF_TIMER0
#elif defined(NRF53_SERIES)
#define MPSL_TIMER0 NRF_TIMER0_NS
#elif defined(NRF54L_SERIES)
#define MPSL_TIMER0 NRF_TIMER10
#elif defined(NRF54H_SERIES)
#define MPSL_TIMER0 NRF_TIMER020
#endif
#endif

#endif /* ESB_NRF_ERRATA_COMPAT_H_ */
