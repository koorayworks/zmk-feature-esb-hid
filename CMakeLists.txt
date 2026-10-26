# HID over ESB module
if(CONFIG_ZMK_ESB_HID OR CONFIG_ZMK_ESB_HID_RECEIVER)
  add_subdirectory(esb)
  target_include_directories(app PRIVATE include src)
  target_sources(app PRIVATE src/esb_hid_link.c)
endif()

if(CONFIG_ZMK_ESB_HID)
  target_sources(app PRIVATE src/esb_hid_tx.c)
endif()

if(CONFIG_ZMK_ESB_HID_RECEIVER)
  target_sources(app PRIVATE src/esb_hid_rx.c)
endif()
