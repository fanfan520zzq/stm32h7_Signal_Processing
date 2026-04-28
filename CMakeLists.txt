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
        Core/Task/LCD.h
        Core/Algorithm/lttb.c
        Core/Algorithm/lttb.h
        Core/Algorithm/protocol.c
        Core/Algorithm/protocol.h
        Core/si5351/si5351.c
        Core/si5351/si5351.h
        Core/si5351/si5351.h

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
        Core/Task/MSG.h
        Core/Task/UARTTX.c
        Core/Task/CMD.c
        Core/Task/DDS.c
        Core/Task/DDS.h

        Core/Task/ADCTask.c
        Core/Task/ADCTask.h

        Core/Task/FFTTask.c
        Core/Task/FFTTask.h


        Core/Task/LCD.c
        Core/Task/LCD.h
        Core/Algorithm/lttb.c
        Core/Algorithm/lttb.h
        Core/Algorithm/protocol.c
        Core/Algorithm/protocol.h
        Core/si5351/si5351.c
        Core/si5351/si5351.h
)


file(GLOB DSP_LIB "${CMAKE_SOURCE_DIR}/Middlewares/ST/ARM/DSP/Lib/*.a")


# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        Core/Task
        Core/Inc
        Core/Algorithm
        Drivers/CMSIS/DSP/Include
        Drivers/CMSIS/Include
        Middlewares/ST/ARM/DSP/Inc
    Core/si5351
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
