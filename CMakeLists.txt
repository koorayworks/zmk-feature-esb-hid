# HID over ESB module
if(CONFIG_ZMK_ESB_HID OR CONFIG_ZMK_ESB_HID_RECEIVER)
  # Public headers: also needed by the vendored ESB library target,
  # not just the app target.
  zephyr_include_directories(include)
  add_subdirectory(esb)
  target_include_directories(app PRIVATE src)
  target_sources(app PRIVATE src/esb_hid_link.c)
endif()

if(CONFIG_ZMK_ESB_HID)
  target_sources(app PRIVATE src/esb_hid_tx.c)
  # The &esb_pair behavior driver compiles to nothing when no
  # zmk,behavior-esb-pair node exists in the devicetree
  # (DT_HAS_COMPAT_STATUS_OKAY guard inside the source).
  target_sources(app PRIVATE src/behaviors/behavior_esb_pair.c)
endif()

if(CONFIG_ZMK_ESB_HID_STUDIO)
  target_sources(app PRIVATE src/esb_hid_studio_tx.c)
endif()

if(CONFIG_ZMK_ESB_HID_RECEIVER)
  target_sources(app PRIVATE src/esb_hid_rx.c)
endif()

if(CONFIG_ZMK_ESB_HID_STUDIO_RX)
  target_sources(app PRIVATE src/esb_hid_studio_rx.c)
endif()

# Forked ST7789V driver with runtime orientation support
# (drivers/display/, Prospector-style zephyr_library_amend of the
# stock Zephyr driver). Gated on the driver actually being built:
# TX builds have no display and must not see the amend at all.
if(CONFIG_ST7789V)
  add_subdirectory(drivers/display)
endif()
