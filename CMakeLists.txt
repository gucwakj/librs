# build librs
project (LibRS)

# set minimum cmake version
cmake_minimum_required (VERSION 2.8)

# set the version number
set (LibRS_VERSION 0.0.70)
set (LibRS_SO_VERSION 0)

# verbose output
set (CMAKE_VERBOSE_MAKEFILE off)

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

# suppress developer warnings
cmake_policy (SET CMP0015 NEW)

# set developer warnings
if (WIN32)
	add_definitions (-D_CRT_SECURE_NO_WARNINGS)
else ()
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-write-strings")
endif ()

# include directory
include_directories (
	"${PROJECT_SOURCE_DIR}/include"
	"${PROJECT_BINARY_DIR}"
)

# link directory
link_directories ("${LibRS_BINARY_DIR}/lib")

# set output directory for shared objects
set (OUTPUT_LIBDIR "${LibRS_BINARY_DIR}/lib")
set (OUTPUT_BINDIR "${LibRS_BINARY_DIR}/bin")
foreach (CONF ${CMAKE_CONFIGURATION_TYPES})
	string (TOUPPER "${CONF}" CONF)
	set ("CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${CONF}" "${OUTPUT_LIBDIR}")
	#set ("CMAKE_RUNTIME_OUTPUT_DIRECTORY_${CONF}" "${OUTPUT_BINDIR}")
	if (WIN32)
		set ("CMAKE_LIBRARY_OUTPUT_DIRECTORY_${CONF}" "${OUTPUT_BINDIR}")
	else ()
		set ("CMAKE_LIBRARY_OUTPUT_DIRECTORY_${CONF}" "${OUTPUT_LIBDIR}")
	endif ()
endforeach ()
set ("CMAKE_LIBRARY_OUTPUT_DIRECTORY" "${OUTPUT_LIBDIR}")
if (WIN32)
	make_directory (${OUTPUT_BINDIR})
endif ()
make_directory (${OUTPUT_LIBDIR})

# options for individual robot compilation
set (ENABLE_LINKBOT ON)
set (ENABLE_MINDSTORMS ON)

# options for building pieces of whole library
option (ENABLE_GRAPHICS "Enable graphical library" ON)
option (ENABLE_SIM "Enable simulation library" ON)
option (ENABLE_XML "Enable xml library" ON)

# add libraries
add_subdirectory ("${LibRS_SOURCE_DIR}/src/rs")
add_subdirectory ("${LibRS_SOURCE_DIR}/src/rsRobots")
set (MODULES ${MODULES} "rs")
set (MODULES ${MODULES} "rsRobots")
if (ENABLE_SIM)
	add_subdirectory ("${LibRS_SOURCE_DIR}/src/rsSim")
	set (MODULES ${MODULES} "rsSim")
endif ()
find_package(OpenGL)
if (ENABLE_GRAPHICS AND OPENGL_FOUND)
	add_subdirectory ("${LibRS_SOURCE_DIR}/src/rsScene")
	add_subdirectory ("${LibRS_SOURCE_DIR}/src/rsCallback")
	set (MODULES ${MODULES} "rsScene")
	set (MODULES ${MODULES} "rsCallback")
endif ()
if (ENABLE_XML)
	add_subdirectory ("${LibRS_SOURCE_DIR}/src/rsXML")
	set (MODULES ${MODULES} "rsXML")
endif ()

# tell user which modules are configured for build
message ("Building librs modules: ${MODULES}")

