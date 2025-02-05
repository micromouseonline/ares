#pragma once

// Optionally, you can include common settings shared by all boards here
#define COMMON_FREQUENCY 1000  // Example: define a common frequency

// Define the target board based on CMake or other build system definitions
// These macros could be set in CMake, or you could manually define them here
#ifdef BOARD_ARES
#include "board-ares.h"

#elif defined(BOARD_MR32)
#include "board-mr32.h"

#else
#error "No board selected. Please define BOARD_ARES or BOARD_MR32."
#endif
