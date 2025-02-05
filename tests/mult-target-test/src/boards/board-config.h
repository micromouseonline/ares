#pragma once

// Define the target board based on CMake or other build system definitions
// These macros could be set in CMake, or you could manually define them here
#ifdef BOARD_ARES
#include "ares/board.h"
#elif defined(BOARD_MR32)
#include "mr32/board.h"
#else
#error "No board selected. Please define BOARD_ARES or BOARD_MR32."
#endif

// Optionally, you can also include common settings shared by all boards here
#define COMMON_FREQUENCY 1000  // Example: define a common frequency
