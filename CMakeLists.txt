cmake_minimum_required(VERSION 3.14)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS OFF)

project("mouse-simulator" LANGUAGES C CXX)

# Set project directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin/${CMAKE_BUILD_TYPE})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build/${CMAKE_GENERATOR_PLATFORM}/${CMAKE_BUILD_TYPE})
set(ASSETS_SOURCE_DIR "${CMAKE_SOURCE_DIR}/assets")
set(ASSETS_DEST_DIR "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/assets")

# ###############################################################
# set up the compiler and linker options
# static linking for easy installation
option(BUILD_SHARED_LIBS "Build shared libraries" OFF)
include(${CMAKE_SOURCE_DIR}/cmake/DefaultCompilerOptionsAndWarnings.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/print_compiler_info.cmake)

# ###############################################################
function(show_compiler_info target)
    add_custom_command(
            TARGET ${target} PRE_BUILD
            COMMAND ${CMAKE_COMMAND} -E echo "C++ Compiler ID: ${CMAKE_CXX_COMPILER_ID}"
            COMMAND ${CMAKE_COMMAND} -E echo "C++ Compiler Version: ${CMAKE_CXX_COMPILER_VERSION}"
    )
endfunction()

# ###############################################################
# be sure to copy any assets into the binary directory
function(copy_resources target)
    add_custom_command(
            TARGET ${target}
            COMMENT "Copy Resources directory"
            COMMAND ${CMAKE_COMMAND} -E echo " Copy Assets -"
            COMMAND ${CMAKE_COMMAND} -E echo "          from:  ${ASSETS_SOURCE_DIR}"
            COMMAND ${CMAKE_COMMAND} -E echo "            to:  ${ASSETS_DEST_DIR}"
            PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory ${ASSETS_SOURCE_DIR} ${ASSETS_DEST_DIR}

            # This second version is cmake >= 3.26 only
            # PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory_if_different ${ASSETS_SOURCE_DIR} ${ASSETS_DEST_DIR}
            VERBATIM
    )
endfunction()

# ###############################################################
# use this if needed for audio or other DLLs on windows platforms
function(copy_windows_dlls target)
    if (WIN32)
        add_custom_command(
                TARGET ${target}
                COMMENT "Copy OpenAL DLL"
                PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy ${SFML_SOURCE_DIR}/extlibs/bin/$<IF:$<EQUAL:${CMAKE_SIZEOF_VOID_P},8>,x64,x86>/openal32.dll $<TARGET_FILE_DIR:${target}>
                VERBATIM)
    endif ()
endfunction()

# ###############################################################
# Enable Fetchcontent to grab dependencies at configuration
include(FetchContent)

# ###############################################################
FetchContent_Declare(SFML

        # Main rendering library
        GIT_REPOSITORY https://github.com/SFML/SFML.git
        GIT_TAG 2.6.x)
FetchContent_MakeAvailable(SFML)

# ###############################################################
FetchContent_Declare(ImGui

        # Immediate Mode GUI Library
        GIT_REPOSITORY https://github.com/ocornut/imgui
        GIT_TAG v1.89.9-docking)
FetchContent_MakeAvailable(ImGui)
FetchContent_GetProperties(ImGui SOURCE_DIR IMGUI_DIR)
set(IMGUI_SFML_FIND_SFML OFF)

# ###############################################################
FetchContent_Declare(ImGui-SFML

        # ImGui back end for SFML
        GIT_REPOSITORY https://github.com/SFML/imgui-sfml
        GIT_TAG 2.6.x)
FetchContent_MakeAvailable(ImGui-SFML)

# ###############################################################
FetchContent_Declare(fmt

        # Enhanced formatting for IOSTREAM
        GIT_REPOSITORY https://github.com/fmtlib/fmt.git
        GIT_TAG 8.0.1
        # Do not change the version without checking spdlog below
        # as they may conflict
)
FetchContent_MakeAvailable(fmt)

# ###############################################################
include_directories(${imgui_SOURCE_DIR})

# Immediate Mode PLotting Library
add_subdirectory(libs/implot)
set(IMPLOT_LIBS PUBLIC implot)
include_directories(libs/implot)

# ###############################################################
set(SFML_LIBS PUBLIC sfml-graphics sfml-system sfml-window)
set(IMGUI_LIBS PUBLIC ImGui-SFML::ImGui-SFML)

# ###############################################################
FetchContent_Declare(spdlog
        GIT_REPOSITORY https://github.com/gabime/spdlog.git
        GIT_TAG v1.9.2
)
FetchContent_MakeAvailable(spdlog)

# ###############################################################
# Add GoogleTest via FetchContent
FetchContent_Declare(
        GoogleTest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG release-1.12.1  # You can use the latest stable release or a specific version
)
FetchContent_MakeAvailable(GoogleTest)
# ###############################################################

# Main application executable
set(APP mouse-simulator)
#add_definitions(-DWINDOW_TITLE=\"${APP}\")

add_executable(${APP} "")
copy_resources(${APP})
copy_windows_dlls(${APP})

target_compile_options(${APP} PUBLIC
        ${DEFAULT_COMPILER_OPTIONS_AND_WARNINGS}
)

target_link_libraries(${APP}
        ${SANITIZER_FLAGS}
        ${SFML_LIBS}
        ${IMGUI_LIBS}
        fmt::fmt
        spdlog::spdlog
)

include_directories(src)
include_directories(src/utils)

target_sources(${APP} PRIVATE
        src/main.cpp
        src/application/window.cpp
        src/world/mazedata.cpp
        src/common/printf/printf.c
)

# ###############################################################
# Add a new target for tests
enable_testing()

add_executable(google_test_mouse_simulator
        google-tests/test_main.cpp
        google-tests/test_set_mask.cpp
        google-tests/test_maze_class.cpp
        google-tests/test_pose.cpp
        google-tests/test_trajectory.cpp
        google-tests/test_path_runner.cpp
        google-tests/test_trapezoid.cpp
        google-tests/text_Spinturn.cpp
        google-tests/test_cubic.cpp
)

# Link GoogleTest and your project to the test executable
target_link_libraries(google_test_mouse_simulator
        PUBLIC
        GTest::gtest
        GTest::gtest_main
        ${SFML_LIBS}
        ${IMGUI_LIBS}
        fmt::fmt
        spdlog::spdlog
)

target_include_directories(google_test_mouse_simulator
        PUBLIC
        ${CMAKE_SOURCE_DIR}/src
        ${GTEST_INCLUDE_DIRS}
        #        ${CMAKE_SOURCE_DIR}/include  # If you have a separate include directory
)


# Run tests after building
add_test(NAME google_test_mouse_simulator COMMAND google_test_mouse_simulator)

############################################################
# other mini-test projects

add_subdirectory(tests/factory-test)
add_subdirectory(tests/simulator-test)
add_subdirectory(tests/callback-test)
add_subdirectory(tests/printf-callback-test)
add_subdirectory(tests/slip-angle-test)
add_subdirectory(tests/streaming-print-test)
