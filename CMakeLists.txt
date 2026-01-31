cmake_minimum_required(VERSION 3.14)
project(OmniMotion VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(OMNI_BUILD_EXAMPLES "Build example applications" ON)
option(OMNI_BUILD_TESTS "Build unit tests" ON)
option(OMNI_BUILD_DOCS "Build documentation" OFF)

# Platform detection
if(WIN32)
    set(OMNI_PLATFORM "windows")
elseif(UNIX AND NOT APPLE)
    set(OMNI_PLATFORM "linux")
elseif(APPLE)
    set(OMNI_PLATFORM "macos")
endif()

# Library type
option(OMNI_BUILD_SHARED "Build shared library" OFF)
if(OMNI_BUILD_SHARED)
    set(OMNI_LIB_TYPE SHARED)
else()
    set(OMNI_LIB_TYPE STATIC)
endif()

# Source files
file(GLOB_RECURSE OMNI_SOURCES
    "src/hal/*.cpp"
    "src/driver/*.cpp"
    "src/protocol/*.cpp"
    "src/control/*.cpp"
    "src/motion/*.cpp"
    "src/app/*.cpp"
    "src/utils/*.cpp"
)

# Create library
add_library(omnimotion ${OMNI_LIB_TYPE} ${OMNI_SOURCES})

target_include_directories(omnimotion
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Compiler warnings
if(MSVC)
    target_compile_options(omnimotion PRIVATE /W4)
else()
    target_compile_options(omnimotion PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Platform-specific HAL implementations
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/platform/${OMNI_PLATFORM}")
    file(GLOB_RECURSE PLATFORM_SOURCES "platform/${OMNI_PLATFORM}/*.cpp")
    target_sources(omnimotion PRIVATE ${PLATFORM_SOURCES})
    target_include_directories(omnimotion PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/platform/${OMNI_PLATFORM}
    )
endif()

# Examples
if(OMNI_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# Tests
if(OMNI_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# Documentation
if(OMNI_BUILD_DOCS)
    find_package(Doxygen)
    if(DOXYGEN_FOUND)
        add_subdirectory(docs)
    endif()
endif()

# Installation
include(GNUInstallDirs)
install(TARGETS omnimotion
    EXPORT omnimotion-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/omni
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT omnimotion-targets
    FILE omnimotion-targets.cmake
    NAMESPACE omni::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/omnimotion
)

# Package configuration
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/omnimotion-config-version.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/omnimotion-config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/omnimotion-config.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/omnimotion
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/omnimotion-config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/omnimotion-config-version.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/omnimotion
)
