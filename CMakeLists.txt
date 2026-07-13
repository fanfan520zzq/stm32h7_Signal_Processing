cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME IIT6_Oscilliscope)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
set(CMSISCORE "${CMAKE_SOURCE_DIR}/Drivers/CMSIS/")

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)
add_definitions(-DARM_MATH_CM7 -D__FPU_PRESENT=1U)

set(DSP_LIB_FILE "${CMAKE_SOURCE_DIR}/Middlewares/ST/ARM/DSP/Lib/libarm_cortexM7lfdp_math.a")

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        Core/Task/LCD.c
        Core/Algorithm/lttb.c
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        Core/Src/Measure.c
        Core/Task/UARTTX.c
        Core/Task/CMD.c
        Core/Task/ADCTask.c
        Core/Task/FFTTask.c
        Core/Task/DDS.c
        Core/AD9833/ad9833_hal.c
        Core/Src/adc_sync.c
        Core/Src/sweep_grid.c
        Core/Src/sweep_engine.c
        Core/Src/goertzel.c
        Core/Src/calib.c
        Core/Src/hooks.c
        Core/Src/classify.c
        Core/Src/iir_runtime.c
        Core/Src/recon_analyzer.c
        Core/Src/recon_hlookup.c
        Core/Src/recon_synth.c
        Core/Src/recon_dds.c
        Core/Src/recon_pll.c
        Core/Src/known_model.c
)


file(GLOB DSP_LIB "${CMAKE_SOURCE_DIR}/Middlewares/ST/ARM/DSP/Lib/*.a")


# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        Core/Task
        Core/Inc
        Core/Algorithm
        Core/AD9833
        Core/si5351
        Drivers/CMSIS/DSP/Include
        Drivers/CMSIS/Include
        Middlewares/ST/ARM/DSP/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# 把"缺函数原型"类问题升级为编译错误, 从源头堵死硬浮点 ABI 传参错位的隐患.
# 只作用于本工程自己的源文件(不含 stm32cubemx 子目录的 HAL/CMSIS).
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    -Werror=implicit-function-declaration
    -Werror=implicit-int
)


# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
        "${DSP_LIB_FILE}"
)


target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u _printf_float)
