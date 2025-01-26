include(${CMAKE_SOURCE_DIR}/cmake/project-boilerplate.cmake)

project(${APP} C CXX)

target_sources(${APP} PRIVATE
        main.cpp
)
