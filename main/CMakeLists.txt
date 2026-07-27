idf_component_register(
    SRCS
        "main.c"
        "usb_audio_probe.c"
    PRIV_REQUIRES
        badge-bsp
        usb
    INCLUDE_DIRS "."
)
