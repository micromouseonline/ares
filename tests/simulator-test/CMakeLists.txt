include(${CMAKE_SOURCE_DIR}/cmake/project-boilerplate.cmake)

project(${APP} C CXX)

target_sources(${APP} PRIVATE
    main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/common/printf/printf.c
)
