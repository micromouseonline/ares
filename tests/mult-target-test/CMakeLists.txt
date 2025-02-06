include(${CMAKE_SOURCE_DIR}/cmake/project-boilerplate.cmake)

project(${APP} C CXX)

# Define project directories
set(SRC_DIR "src")
set(APP_DIR "${SRC_DIR}/app")
set(BOARDS_DIR "${SRC_DIR}/boards")
set(HAL_DIR "${SRC_DIR}/hal")
set(DRIVERS_DIR "${SRC_DIR}/drivers")
set(IMU_DIR "${DRIVERS_DIR}/imu")

# Include common directories
include_directories(
        ${SRC_DIR}
        ${APP_DIR}
        ${DRIVERS_DIR}
        ${BOARDS_DIR}
        ${HAL_DIR}
        #        ${IMU_DIR}
)

# Define common source files
set(COMMON_SOURCES
        ${SRC_DIR}/main.cpp
)

target_sources(${APP} PRIVATE
        src/main.cpp
)

# --- Ares Target (for Embedded Cortex-M) ---
add_executable(imu_ares
        ${COMMON_SOURCES}
)
target_include_directories(imu_ares PRIVATE
        ${BOARDS_DIR}/ares
)
target_compile_definitions(imu_ares PRIVATE
        BOARD_ARES
)

# --- MR32 Target (for Embedded Cortex-M) ---
add_executable(imu_mr32
        ${COMMON_SOURCES}
)

target_include_directories(imu_mr32 PRIVATE
        ${BOARDS_DIR}/mr32
)
target_compile_definitions(imu_mr32 PRIVATE
        BOARD_MR32
)

# Display build configuration
message(STATUS "Targets available: imu_ares, imu_mr32")

#include(FetchContent)
#FetchContent_Declare(
#        googletest
#        URL https://github.com/google/googletest/archive/refs/heads/main.zip
#)
#FetchContent_MakeAvailable(googletest)
#
#enable_testing()
#
#add_executable(test_robot
#        test-robot.cpp
#)
#target_link_libraries(test_robot
#        gtest
#        gtest_main
#        pthread
#)
