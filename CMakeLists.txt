cmake_minimum_required(VERSION 3.16)
project(hal-stm32f407ighx C ASM)

set(TARGET ${PROJECT_NAME})
add_executable(${TARGET})
set_target_properties(${TARGET} PROPERTIES SUFFIX ".elf")

set(MCU_DEVICE STM32F407xx)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
    message(STATUS "no build type specified, defaulting to 'Debug'")
endif()

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F407IGHX_FLASH.ld)
set(CORE_DIR ${CMAKE_SOURCE_DIR}/Core)
set(USER_DIR ${CMAKE_SOURCE_DIR}/User)
set(LIB_DIR ${CMAKE_SOURCE_DIR}/Lib)
set(DRIVER_DIR ${CMAKE_SOURCE_DIR}/Drivers)
set(CMSIS_DIR ${DRIVER_DIR}/CMSIS)
set(CMSISCORE ${DRIVER_DIR}/CMSIS)
set(HAL_DIR ${DRIVER_DIR}/STM32F4xx_HAL_Driver)
set(STARTUP_DIR ${CMAKE_SOURCE_DIR}/startup)

file(GLOB_RECURSE USER_SOURCES ${USER_DIR}/*.c)
file(GLOB_RECURSE CORE_SOURCES ${CORE_DIR}/*.c)
file(GLOB_RECURSE HAL_SOURCES ${HAL_DIR}/*.c)
file(GLOB_RECURSE CMSIS_SOURCES ${CMSIS_DIR}/*.c)

set(ASM_SOURCES
    ${STARTUP_DIR}/startup_stm32f407ighx.s
)

set(PKG_BMI088D_DIR ${LIB_DIR}/BMI088D)
file(GLOB_RECURSE PKG_BMI088D_SOURCES ${PKG_BMI088D_DIR}/Src/*.c)

set(CMSISDSP ${LIB_DIR}/CMSIS-DAP)
add_subdirectory(${CMSISDSP}/Source bin_dsp)
target_link_libraries(${TARGET} CMSISDSP)

target_sources(${TARGET}
    PRIVATE
    ${USER_SOURCES}
    ${PKG_BMI088D_SOURCES}
    ${CORE_SOURCES}
    ${HAL_SOURCES}
    ${CMSIS_SOURCES}
    ${ASM_SOURCES}
)

target_include_directories(${TARGET}
    PUBLIC
    ${USER_DIR}
    ${PKG_BMI088D_DIR}/Inc
    ${CORE_DIR}/Inc
    ${CMSISDSP}/Include
    ${CMSIS_DIR}/Include
    ${CMSIS_DIR}/Device/ST/STM32F4xx/Include
    ${HAL_DIR}/Inc
)

# common compiler options
target_compile_options(${TARGET}
    PRIVATE
    -Wall
    -Wextra
    -ffunction-sections
    -fdata-sections
    -fstack-usage
    -fdiagnostics-color=always
    -fno-diagnostics-show-option
)

target_compile_options(${TARGET}
    PRIVATE
    $<$<CONFIG:Debug>:-Og -g3 -DDEBUG>
    $<$<CONFIG:Release>:-O2 -DNDEBUG>
    $<$<CONFIG:MinSizeRel>:-Os -DNDEBUG>
    $<$<CONFIG:RelWithDebInfo>:-O2 -g -DNDEBUG>
)

target_compile_definitions(${TARGET}
    PRIVATE
    STM32F40_41xxx
    ${MCU_DEVICE}
    USE_HAL_DRIVER
)

target_link_options(${TARGET}
    PRIVATE
    --specs=nano.specs
    --specs=nosys.specs
    -T${LINKER_SCRIPT}
    -Wl,-Map=${CMAKE_BINARY_DIR}/${TARGET}.map,--cref
    -Wl,--gc-sections
    -Wl,--print-memory-usage
)
