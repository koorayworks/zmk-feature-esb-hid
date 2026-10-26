/*
 * Copyright (c) 2026 KeyboardBrain contributors
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/*
 * MPSL stubs for building the vendored ESB stack on plain Zephyr
 * (without nRF Connect SDK / MPSL).
 *
 * This project never uses an external front-end module (FEM) and never
 * uses MPSL timeslots (CONFIG_ESB_MPSL_TIMESLOT is off), so the stubs
 * reproduce the "no FEM present" behaviour of the real MPSL:
 *
 * - mpsl_fem_*_configuration_set() fails, so ESB skips FEM activation
 *   and uses its normal radio ramp-up path.
 * - mpsl_fem_tx_power_split() assigns the full requested power to the
 *   radio stage and no power control to a FEM PA.
 * - mpsl_fem_pa_power_control_set() succeeds (ESB asserts otherwise).
 * - Timeslot session functions fail; they are unreachable unless
 *   CONFIG_ESB_MPSL_TIMESLOT is enabled.
 */

#include <mpsl_fem_protocol_api.h>
#include <mpsl_timeslot.h>
#include <nrf_errno.h>

void mpsl_fem_enable(void)
{
}

int32_t mpsl_fem_disable(void)
{
	return 0;
}

int32_t mpsl_fem_pa_configuration_set(const mpsl_fem_event_t *const p_activate_event,
				      const mpsl_fem_event_t *const p_deactivate_event)
{
	(void)p_activate_event;
	(void)p_deactivate_event;

	/* No FEM present: report failure so ESB does not enable FEM paths. */
	return -NRF_EINVAL;
}

int32_t mpsl_fem_pa_configuration_clear(void)
{
	return 0;
}

int32_t mpsl_fem_lna_configuration_set(const mpsl_fem_event_t *const p_activate_event,
				       const mpsl_fem_event_t *const p_deactivate_event)
{
	(void)p_activate_event;
	(void)p_deactivate_event;

	/* No FEM present: report failure so ESB does not enable FEM paths. */
	return -NRF_EINVAL;
}

int32_t mpsl_fem_lna_configuration_clear(void)
{
	return 0;
}

void mpsl_fem_deactivate_now(mpsl_fem_functionality_t type)
{
	(void)type;
}

int32_t mpsl_fem_pa_power_control_set(mpsl_fem_pa_power_control_t pa_power_control)
{
	(void)pa_power_control;

	return 0;
}

int8_t mpsl_fem_tx_power_split(const mpsl_tx_power_t power,
			       mpsl_tx_power_split_t *const p_tx_power_split,
			       mpsl_phy_t phy, uint16_t freq_mhz, bool tx_power_ceiling)
{
	(void)phy;
	(void)freq_mhz;
	(void)tx_power_ceiling;

	/* No FEM: the radio stage applies the full requested power. */
	p_tx_power_split->radio_tx_power = power;
	p_tx_power_split->fem_pa_power_control = 0;

	return power;
}

int32_t mpsl_timeslot_session_open(mpsl_timeslot_callback_t mpsl_timeslot_signal_callback,
				   mpsl_timeslot_session_id_t *p_session_id)
{
	(void)mpsl_timeslot_signal_callback;
	(void)p_session_id;

	/* Timeslots are unavailable without MPSL. */
	return -NRF_EPERM;
}

int32_t mpsl_timeslot_session_close(mpsl_timeslot_session_id_t session_id)
{
	(void)session_id;

	return -NRF_EPERM;
}

int32_t mpsl_timeslot_request(mpsl_timeslot_session_id_t session_id,
			      mpsl_timeslot_request_t const *p_request)
{
	(void)session_id;
	(void)p_request;

	return -NRF_EPERM;
}
