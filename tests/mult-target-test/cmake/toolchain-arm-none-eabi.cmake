set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Set toolchain paths (modify based on your setup)
set(TOOLCHAIN_PREFIX arm-none-eabi-)
set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}objcopy)
set(CMAKE_SIZE ${TOOLCHAIN_PREFIX}size)

# Compiler flags for Cortex-M (modify for your MCU)
set(CPU_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(CMAKE_C_FLAGS "${CPU_FLAGS} -Wall -Wextra -Os" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS "${CPU_FLAGS} -Wall -Wextra -Os -fno-exceptions -fno-rtti" CACHE STRING "" FORCE)
set(CMAKE_EXE_LINKER_FLAGS "-nostartfiles -Wl,--gc-sections" CACHE STRING "" FORCE)

# Specify linker script (modify for your board)
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/boards/ares/ldscript.ld)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T${LINKER_SCRIPT}" CACHE STRING "" FORCE)

# Don't use the system paths for includes and libraries
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
