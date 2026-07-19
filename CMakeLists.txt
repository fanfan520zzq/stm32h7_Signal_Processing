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
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
        Core/Profiles/app_profile.c
        Core/Profiles/app_profile.h
        Core/Services/clock_service.c
        Core/Services/clock_service.h
        Core/Services/dpll_service.c
        Core/Services/dpll_service.h
        Core/Algorithms/measure.c
        Core/Algorithms/measure.h
        Core/Algorithms/phase_bridge.c
        Core/Algorithms/phase_bridge.h
        Core/Algorithms/dpll_controller.c
        Core/Algorithms/dpll_controller.h
        Core/Algorithms/dpll_b_mode.c
        Core/Algorithms/dpll_b_mode.h

        Core/Features/dac_dds.c
        Core/Features/dac_dds.h

        Core/Features/adc_capture.c
        Core/Features/adc_capture.h

        Core/Algorithms/fft_analysis.c
        Core/Algorithms/dft_separate.c
        Core/Algorithms/fft_analysis.h


        Core/Protocols/lcd_protocol.c
        Core/Protocols/lcd_protocol.h
        Core/Algorithms/lttb.c
        Core/Algorithms/lttb.h
        Core/Protocols/vofa_protocol.c
        Core/Protocols/vofa_protocol.h
        Core/Drivers/usart_driver.c
        Core/Drivers/spi_driver.c
        Core/Drivers/spi_driver.h
        Core/Drivers/timebase_driver.c
        Core/Drivers/timebase_driver.h
        Core/BSP/si5351/si5351.c
        Core/BSP/si5351/si5351.h
        Core/BSP/AD9833/ad9833_hal.c
        Core/BSP/AD9833/ad9833_hal.h
        Core/Features/fpga_link/fpga_ctrl.c
        Core/Features/fpga_link/fpga_ctrl.h
        Core/Protocols/fpga_spi_protocol.h
        Core/Protocols/fpga_spi_protocol.c
)


file(GLOB DSP_LIB "${CMAKE_SOURCE_DIR}/Middlewares/ST/ARM/DSP/Lib/*.a")


# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        Core/Inc
        Core/BSP/si5351
        Core/BSP/AD9833
        Core/Algorithms
        Core/Protocols
        Core/Features
        Core/Features/fpga_link
        Core/Services
        Core/Drivers
        Core/Profiles
        Drivers/CMSIS/DSP/Include
        Drivers/CMSIS/Include
        Middlewares/ST/ARM/DSP/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)


# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
        "${DSP_LIB_FILE}"
)


target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u _printf_float)
