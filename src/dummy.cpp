/**
 * @file dummy.cpp
 * @brief Placeholder source file for library compilation
 *
 * The OmniMotion library is primarily header-only, but this file
 * ensures the library can be built as a static/shared library.
 */

#include "omni/omnimotion.hpp"

// This file intentionally left mostly empty.
// The library is header-only for maximum portability.

namespace omni {

// Version information
const char* getVersion() {
    return Version::getString();
}

}  // namespace omni
