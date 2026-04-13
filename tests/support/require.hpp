#pragma once

// Release-safe test assertion. Unlike <cassert>, this survives -DNDEBUG,
// which is the mode ctest will run tests in if the build is Release.

#include <cstdio>
#include <cstdlib>

#define REQUIRE(cond)                                                                   \
    do {                                                                                \
        if (!(cond)) {                                                                  \
            std::fprintf(stderr,                                                        \
                         "REQUIRE failed: %s\n  at %s:%d\n",                            \
                         #cond, __FILE__, __LINE__);                                    \
            std::exit(1);                                                               \
        }                                                                               \
    } while (0)
