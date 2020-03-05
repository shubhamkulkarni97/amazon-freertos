register_component()

target_compile_options(
    afr_3rdparty_mbedtls
    PUBLIC
    ${IDF_COMPILE_OPTIONS}
    $<$<COMPILE_LANGUAGE:C>:${IDF_C_COMPILE_OPTIONS}>
    $<$<COMPILE_LANGUAGE:CXX>:${IDF_CXX_COMPILE_OPTIONS}>
    "-Wno-maybe-uninitialized"
)

set(
    mbedtls_inc_dirs
    ${IDF_INCLUDE_DIRECTORIES}
    ${CMAKE_CURRENT_LIST_DIR}/port/include
)

set(
    mbedtls_esp_port_srcs
    "${CMAKE_CURRENT_LIST_DIR}/port/esp_bignum.c"
    "${CMAKE_CURRENT_LIST_DIR}/port/esp_mem.c"
    "${CMAKE_CURRENT_LIST_DIR}/port/esp_sha256.c"
    "${CMAKE_CURRENT_LIST_DIR}/port/esp_hardware.c"
    "${CMAKE_CURRENT_LIST_DIR}/port/esp_sha1.c"
    "${CMAKE_CURRENT_LIST_DIR}/port/esp_sha512.c"
    "${CMAKE_CURRENT_LIST_DIR}/port/mbedtls_debug.c"
)

if(AFR_ESP_LWIP)
    list(
        APPEND
        mbedtls_inc_dirs
        "${esp_idf_dir}/components/lwip/include/apps"
        "${esp_idf_dir}/components/lwip/include/apps/sntp"
        "${esp_idf_dir}/components/lwip/lwip/src/include"
        "${esp_idf_dir}/components/lwip/port/esp32/include"
        "${esp_idf_dir}/components/lwip/port/esp32/include/arch"
        "${esp_idf_dir}/components/lwip/include_compat"
    )

    afr_glob_src(mbedtls_src DIRECTORY ${AFR_3RDPARTY_DIR}/mbedtls)
    list(REMOVE_ITEM mbedtls_src "${AFR_3RDPARTY_DIR}/mbedtls/library/net_sockets.c")

    list(
        APPEND
        mbedtls_esp_port_srcs
        "${CMAKE_CURRENT_LIST_DIR}/port/net_sockets.c"
    )
endif()

target_include_directories(
    afr_3rdparty_mbedtls
    BEFORE
    PRIVATE
    "${AFR_VENDORS_DIR}/espressif/esp-idf/components/vfs/include"
    PUBLIC
    ${mbedtls_inc_dirs}
)

target_sources(
    afr_3rdparty_mbedtls
    PUBLIC
    ${mbedtls_esp_port_srcs}
)

target_compile_definitions(
    afr_3rdparty_mbedtls
    PUBLIC
    ${IDF_COMPILE_DEFINITIONS}
    -DMBEDTLS_CONFIG_FILE="${CMAKE_CURRENT_LIST_DIR}/port/include/mbedtls/esp_config.h"
)

target_link_libraries(
    ${COMPONENT_TARGET}
    INTERFACE
    afr_3rdparty_mbedtls
)
