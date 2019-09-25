zephyr_library()

if(CONFIG_ICM20649)
zephyr_include_directories(.)
endif()
zephyr_library_sources_ifdef(CONFIG_ICM20649 icm20649.c)
