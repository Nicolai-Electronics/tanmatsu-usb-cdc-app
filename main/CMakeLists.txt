idf_build_get_property(idf_target IDF_TARGET)
if("${idf_target}" STREQUAL "esp32p4" OR "${idf_target}" STREQUAL "esp32s3")
list(APPEND extra_requires esp_tinyusb)
endif()

idf_component_register(
	SRCS
		"main.c"
		"usb_serial_bridge.c"
	PRIV_REQUIRES
		esp_lcd
		fatfs
		nvs_flash
		badge-bsp
		esp_driver_uart
		${extra_requires}
	INCLUDE_DIRS
		"."
)

idf_build_set_property(COMPILE_OPTIONS "-Wno-error=unused-variable" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-Wno-error=unused-const-variable" APPEND)
