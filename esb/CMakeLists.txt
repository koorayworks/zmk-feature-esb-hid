#
# Copyright (c) 2019 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

zephyr_library()
zephyr_library_sources(esb.c)
zephyr_library_sources(esb_workarounds.c)
zephyr_library_sources(esb_glue.c)

# Plain-Zephyr compatibility: the vendored ESB sources come from nRF
# Connect SDK and use NRF_ERRATA_STATIC_CHECK() from the NCS nrfx MDK.
# Zephyr's bundled nrfx MDK does not provide that macro, so force-include
# a shim that defines it from the MDK per-errata PRESENT/ENABLE macros.
zephyr_library_compile_options(-include${CMAKE_CURRENT_SOURCE_DIR}/nrf_errata_compat.h)

# The vendored sources also use the newer nrfx GPPI helper allocation
# API; map it onto Zephyr 4.1's older helper API.
zephyr_library_compile_options(-include${CMAKE_CURRENT_SOURCE_DIR}/nrfx_gppi_compat.h)

# The vendored ESB sources also include MPSL headers unconditionally.
# Without NCS there is no MPSL; provide the genuine Nordic MPSL API
# headers plus stub implementations with "no FEM / no timeslot"
# behaviour (see mpsl_stub/mpsl_stubs.c). multithreading_lock.c is the
# genuine NCS implementation (a plain Zephyr k_mutex wrapper).
zephyr_library_include_directories(mpsl_stub/include)
zephyr_library_sources(mpsl_stub/mpsl_stubs.c)
zephyr_library_sources(mpsl_stub/multithreading_lock.c)

zephyr_library_sources_ifdef(CONFIG_HAS_HW_NRF_PPI esb_ppi.c)
zephyr_library_sources_ifndef(CONFIG_HAS_HW_NRF_PPI esb_dppi.c)
