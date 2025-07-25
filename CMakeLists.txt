cmake_policy(SET CMP0048 NEW)
cmake_minimum_required(VERSION 3.13.0)

get_filename_component(APPLICATION ${CMAKE_CURRENT_SOURCE_DIR} NAME_WE)

if (NOT "$ENV{APPLICATION_NAME}" STREQUAL "")
    set(APPLICATION_NAME $ENV{APPLICATION_NAME})
else()
    set(APPLICATION_NAME ${APPLICATION}_${CMAKE_VARIANT_BSP}_${CMAKE_BUILD_TYPE})
endif()

option(CLI_ENABLE "" ON)

option(BSP_NM180100EVB "" OFF)
option(BSP_NM180410 "" OFF)
option(BSP_NM180411 "" OFF)

option(FEATURE_RAT_BLE_ENABLE "" OFF)
option(FEATURE_RAT_LORAWAN_ENABLE "" OFF)
option(FEATURE_RAT_LORA_MESH_ENABLE "" ON)
option(FEATURE_TF_ENABLE "" OFF)

if (BSP_NM180100EVB)
add_definitions(-DBSP_NM180100EVB)
set(NM_TARGET "nm180100")
set(BSP_TARGET_DIR nm180100evb CACHE STRING "" FORCE)
endif()

if (BSP_NM180410)
add_definitions(-DBSP_NM180410)
set(NM_TARGET "nm180100")
set(BSP_TARGET_DIR nm180410 CACHE STRING "" FORCE)
endif()

if (BSP_NM180411)
add_definitions(-DBSP_NM180411)
set(NM_TARGET "nm180110")
set(BSP_TARGET_DIR nm180411 CACHE STRING "" FORCE)
endif()


add_definitions(-DNMI)

add_subdirectory(nmsdk2)

project(${APPLICATION})

add_executable(${APPLICATION})
set_target_properties(
    ${APPLICATION}
    PROPERTIES
        OUTPUT_NAME ${APPLICATION_NAME}
        SUFFIX ".axf"
)

add_subdirectory(version)
add_subdirectory(bsp)

if (BUILD_HAL)
    add_dependencies(${APPLICATION} hal)
endif()

if (BUILD_RTOS)
    add_dependencies(${APPLICATION} rtos)
endif()

target_link_libraries(
    ${APPLICATION}
    PUBLIC
        -T${TARGET_LINK_SCRIPT}
        -static
        -Wl,--start-group
        nm_sdk_version
        nm_app_version
        ${LORA_MESH_LIBS}
        ${RTOS_LIBS}
        bsp
        ${HAL_LIBS}
        -lm
        -lc
        -lgcc
        -Wl,--end-group
        -Wl,--gc-sections
        -Wl,-Map=$<TARGET_PROPERTY:OUTPUT_NAME>.map
        -Wl,--print-memory-usage
)

target_compile_definitions(
    ${APPLICATION}
    PRIVATE
)

target_include_directories(
    ${APPLICATION}
    PRIVATE
    ${HAL_INCLUDES}
    ${RTOS_INCLUDES}
    ${BSP_INCLUDES}
    ${PROJECT_SOURCE_DIR}/version
    ${PROJECT_SOURCE_DIR}
    ${SDK_DIR}/version
    ${SDK_DIR}/middleware/RTT/Config
    ${SDK_DIR}/middleware/RTT/RTT

    ${PROJECT_SOURCE_DIR}/ui

    ${LORA_MESH_INCLUDES}

    ${PROJECT_SOURCE_DIR}/comms/lrm
)

if (CLI_ENABLE)
    add_definitions(-DCLI_ENABLE)
    set(
        CLI_SOURCES
        console_task.c
        application_task_cli.c
        gpio_cli.c
        ui/led_task_cli.c
        comms/lrm/lrm_task_cli.c
    )
endif()

target_sources(
    ${APPLICATION}
    PRIVATE
    ${TARGET_STARTUP_SRC}
    ${TARGET_NEWLIB_STUB_SRC}
    ${CLI_SOURCES}
    main.c
    application_task.c
    application_lrm.c
    meshconnect_cli.c

    ${SDK_DIR}/middleware/RTT/RTT/SEGGER_RTT.c
    ${SDK_DIR}/middleware/RTT/RTT/SEGGER_RTT_printf.c

    ui/button_task.c
    ui/led_task.c
    ui/led_predefined_effects.c

    comms/lrm/lrm_task.c
)

add_custom_command(
    TARGET ${APPLICATION}
    POST_BUILD
    COMMAND echo "Creating bin file"
    COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE_NAME:${APPLICATION}> $<TARGET_FILE_BASE_NAME:${APPLICATION}>.bin
)
