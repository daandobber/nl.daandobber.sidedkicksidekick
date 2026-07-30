idf_component_register(
    SRCS
        "main.c"
        "lvgl_ui.c"
        "usb_audio_probe.c"
    PRIV_REQUIRES
        badge-bsp
        lvgl__lvgl
        usb
    INCLUDE_DIRS "."
)
