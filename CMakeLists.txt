if(CONFIG_ZYPHAL)

    zephyr_library()

    zephyr_library_sources(
        "src/core.c"
    )

    zephyr_library_include_directories("inc/")

endif()
