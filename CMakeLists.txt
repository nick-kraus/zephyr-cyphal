if(CONFIG_ZCYPHAL)

    zephyr_include_directories("inc/")

    zephyr_library()
    zephyr_library_sources(
        "src/core.c"
    )

endif()
