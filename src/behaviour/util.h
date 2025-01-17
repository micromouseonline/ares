//
// Created by peter on 17/01/25.
//

#pragma once

#ifndef kPI
#define kPI 3.14159265358979323846f  // More precise

#define RADIANS (kPI / 180.0f)
#define DEGREES (180.0f / kPI)

#endif

#ifndef BIT
#define BIT(b) (1UL << (b))
#endif

#ifndef SIGN
#define SIGN(x) ((0 < x) - (x < 0))
#endif
