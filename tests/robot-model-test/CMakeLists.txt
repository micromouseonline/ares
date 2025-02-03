include(${CMAKE_SOURCE_DIR}/cmake/project-boilerplate.cmake)

project(${APP} C CXX)

target_sources(${APP} PRIVATE
        main.cpp
)


include(FetchContent)
FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/heads/main.zip
)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(test_robot
        test-robot.cpp
)
target_link_libraries(test_robot
        gtest
        gtest_main
        pthread
)
