set(COMPONENT_SRCS "main.c"
	            "fb_data_acq.c"
                "fb_hdrive.c"
                "fb_mean_control.c"
                "fb_nvs.c"
                "fb_wifi.c"
		"neopixel.c"
		"ws2812_control.c"
                )

set(COMPONENT_ADD_INCLUDEDIRS ".")

register_component()


