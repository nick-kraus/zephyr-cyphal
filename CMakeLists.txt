if(CONFIG_ZYPHAL)

    zephyr_include_directories("inc/")

    zephyr_library()
    zephyr_library_sources(
        "src/instance.c"
        "src/transmit.c"
    )

endif()
