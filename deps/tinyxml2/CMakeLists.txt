project(tinyxml2)
include(GNUInstallDirs)

cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
cmake_policy(VERSION 2.6)

# set lib version
set(GENERIC_LIB_VERSION "1.0.11")
set(GENERIC_LIB_SOVERSION "1")

# add common source
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/.")

# set the build postfix extension
set (CMAKE_DEBUG_POSTFIX "d" CACHE STRING "add build postfix type")
set (CMAKE_RELEASE_POSTFIX "" CACHE STRING "add build postfix type")
set (CMAKE_MINSIZEREL_POSTFIX "s" CACHE STRING "add build postfix type")
set (CMAKE_RELWITHDEBINFO_POSTFIX "rd" CACHE STRING "add build postfix type")
if (CMAKE_BUILD_TYPE MATCHES "Release")
	set (CMAKE_BUILD_POSTFIX "")
elseif (CMAKE_BUILD_TYPE MATCHES "Debug")
	set (CMAKE_BUILD_POSTFIX "d")
elseif (CMAKE_BUILD_TYPE MATCHES "MinSizeRel")
	set (CMAKE_BUILD_POSTFIX "s")
elseif (CMAKE_BUILD_TYPE MATCHES "RelWithDebInfo")
	set (CMAKE_BUILD_POSTFIX "rd")
endif ()

# add definitions
if (WIN32)
	add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif (WIN32)

# set output directory for shared objects
set (OUTPUT_LIBDIR "${PROJECT_BINARY_DIR}/lib")
set (OUTPUT_BINDIR "${PROJECT_BINARY_DIR}/bin")
foreach (CONF ${CMAKE_CONFIGURATION_TYPES})
	string (TOUPPER "${CONF}" CONF)
	set ("CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${CONF}" "${OUTPUT_LIBDIR}")
	set ("CMAKE_RUNTIME_OUTPUT_DIRECTORY_${CONF}" "${OUTPUT_BINDIR}")
	if (WIN32)
		set ("CMAKE_LIBRARY_OUTPUT_DIRECTORY_${CONF}" "${OUTPUT_BINDIR}")
	else ()
		set ("CMAKE_LIBRARY_OUTPUT_DIRECTORY_${CONF}" "${OUTPUT_LIBDIR}")
	endif ()
endforeach ()
make_directory (${OUTPUT_LIBDIR})
make_directory (${OUTPUT_BINDIR})

# add targets
if (WIN32)
	add_library(tinyxml2 STATIC tinyxml2.cpp tinyxml2.h)
else ()
	add_library(tinyxml2 SHARED tinyxml2.cpp tinyxml2.h)
endif ()
set_target_properties(tinyxml2 PROPERTIES
	VERSION "${GENERIC_LIB_VERSION}"
	SOVERSION "${GENERIC_LIB_SOVERSION}")

