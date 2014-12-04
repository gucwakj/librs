# build librs
project (LIBRS)

# set minimum cmake version
cmake_minimum_required (VERSION 2.8)

# set the version number
set (LIBRS_VERSION 0.0.70)
set (LIBRS_SO_VERSION 0)

# verbose ouptut
set (CMAKE_VERBOSE_MAKEFILE off)

# set build type
set (CMAKE_BUILD_TYPE "Release")

# set the build postfix extension
if (CMAKE_BUILD_TYPE MATCHES "Release")
    set (CMAKE_BUILD_POSTFIX "")
elseif (CMAKE_BUILD_TYPE MATCHES "MinSizeRel")
    set (CMAKE_BUILD_POSTFIX "s")
elseif (CMAKE_BUILD_TYPE MATCHES "RelWithDebInfo")
    set (CMAKE_BUILD_POSTFIX "rd")
elseif (CMAKE_BUILD_TYPE MATCHES "Debug")
    set (CMAKE_BUILD_POSTFIX "d")
else()
    set (CMAKE_BUILD_POSTFIX "")
endif()

# supress developer warnings
cmake_policy (SET CMP0015 NEW)

# set developer warnings
if (MSVC)
	add_definitions (-D_CRT_SECURE_NO_WARNINGS)
else (MSVC)
	set (CMAKE_CXX_FLAGS "-Wno-write-strings")
endif (MSVC)

# include directory
include_directories (
	"${PROJECT_SOURCE_DIR}/include"
	"${PROJECT_BINARY_DIR}"
)

# set output directory for shared objects
set (OUTPUT_LIBDIR "${LIBRS_BINARY_DIR}/lib")
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${OUTPUT_LIBDIR})
make_directory (${OUTPUT_LIBDIR})

# add libraries
add_subdirectory ("${LIBRS_SOURCE_DIR}/src")

