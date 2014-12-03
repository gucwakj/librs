# Build librobosim
project(librobosim)

# Set minimum cmake version
cmake_minimum_required(VERSION 2.8)

# Set the version number
set(LIBROBOSIM_VERSION 1.7.70)
set(LIBROBOSIM_SOVERSION 0)

# Supress developer warnings
cmake_policy(SET CMP0015 NEW)

# Verbose ouptut
set(CMAKE_VERBOSE_MAKEFILE off)

# Set build type
set(CMAKE_BUILD_TYPE "Release")

# Set developer warnings
if(MSVC)
	add_definitions(-D_CRT_SECURE_NO_WARNINGS)
else(MSVC)
	set(CMAKE_CXX_FLAGS "-Wno-write-strings")
endif(MSVC)

# include directory
include_directories(
	"${librobosim_SOURCE_DIR}/inc"
	"${librobosim_SOURCE_DIR}/tpl"
	"${librobosim_BINARY_DIR}"
)

# Set up a configure file
configure_file(
	"${librobosim_SOURCE_DIR}/config.h.in"
	"${librobosim_BINARY_DIR}/config.h"
)

# Add tinyxml2 library
include_directories("../tinyxml2")
if(MSVC)
	link_directories("../tinyxml2/build/Release")
else(MSVC)
	link_directories("../tinyxml2/build")
endif(MSVC)
set(EXTRA_LIBS ${EXTRA_LIBS} tinyxml2)

# Add ODE library
if(MSVC)
	include_directories("../opende/include")
	link_directories("../opende/lib/Release")
	link_directories("../opende/lib/Debug")
else(MSVC)
	include_directories("../opende/sys/include")
	link_directories("../opende/sys/lib")
endif(MSVC)
set(EXTRA_LIBS ${EXTRA_LIBS} optimized ode debug oded)

# Add base source files to library
set(SRCS ${SRCS}
	src/robot.cpp
	src/robosim.cpp
)

# Option to enable double precision
add_definitions(-DdDOUBLE)

# Option to enable assertion messages
option (ENABLE_MESSAGES "Disable message output." OFF)
if(ENABLE_MESSAGES)
else(ENABLE_MESSAGES)
	add_definitions(-DdNODEBUG)
endif(ENABLE_MESSAGES)

# Option for Linkbots
option(ENABLE_LINKBOT "Enable Linkbots." ON)
if(ENABLE_LINKBOT)
	set(SRCS ${SRCS} src/modularrobot.cpp src/linkbot.cpp)
endif(ENABLE_LINKBOT)

# Option for Mobots
option(ENABLE_MOBOT "Enable Mobots." ON)
if(ENABLE_MOBOT)
	set(SRCS ${SRCS} src/modularrobot.cpp src/mobot.cpp)
endif(ENABLE_MOBOT)

# Option for NXT
option(ENABLE_NXT "Enable NXT." ON)
if(ENABLE_NXT)
	set(SRCS ${SRCS} src/nxt.cpp)
endif(ENABLE_NXT)

# Option for Cubus
option(ENABLE_CUBUS "Enable Cubus." ON)
if(ENABLE_CUBUS)
	set(SRCS ${SRCS} src/cubus.cpp)
endif(ENABLE_CUBUS)

# Option to enable graphics
option(ENABLE_GRAPHICS "Enable graphics for simulation." ON)
if(ENABLE_GRAPHICS)
	include_directories("../osg/build/include")
	include_directories("../osg/include")
	link_directories("../osg/build/lib")
	set(EXTRA_LIBS ${EXTRA_LIBS} optimized osg debug osgd)
	set(EXTRA_LIBS ${EXTRA_LIBS} optimized osgDB debug osgDBd)
	set(EXTRA_LIBS ${EXTRA_LIBS} optimized osgGA debug osgGAd)
	set(EXTRA_LIBS ${EXTRA_LIBS} optimized osgShadow debug osgShadowd)
	set(EXTRA_LIBS ${EXTRA_LIBS} optimized osgText debug osgTextd)
	set(EXTRA_LIBS ${EXTRA_LIBS} optimized osgUtil debug osgUtild)
	set(EXTRA_LIBS ${EXTRA_LIBS} optimized osgViewer debug osgViewerd)
	set(EXTRA_LIBS ${EXTRA_LIBS} optimized OpenThreads debug OpenThreadsd)
	set(SRCS ${SRCS} src/graphics.cpp src/rgbhashtable.c)
	# Add openGL
	find_package(OpenGL REQUIRED)
	if(OPENGL_FOUND)
		include_directories(${OPENGL_INCLUDE_DIR})
		set(EXTRA_LIBS ${EXTRA_LIBS} ${OPENGL_LIBRARIES})
	endif(OPENGL_FOUND)
endif(ENABLE_GRAPHICS)

# Add pthreads
find_package(Threads)
set(EXTRA_LIBS ${EXTRA_LIBS} ${CMAKE_THREAD_LIBS_INIT})

# Set the build postfix extension according to what configuration is being built
if(CMAKE_BUILD_TYPE MATCHES "Release")
    set(CMAKE_BUILD_POSTFIX "")
elseif(CMAKE_BUILD_TYPE MATCHES "MinSizeRel")
    set(CMAKE_BUILD_POSTFIX "s")
elseif(CMAKE_BUILD_TYPE MATCHES "RelWithDebInfo")
    set(CMAKE_BUILD_POSTFIX "rd")
elseif(CMAKE_BUILD_TYPE MATCHES "Debug")
    set(CMAKE_BUILD_POSTFIX "d")
else()
    set(CMAKE_BUILD_POSTFIX "")
endif()

# Add the library
add_library(robosim SHARED ${SRCS})

# Link library against dependencies
target_link_libraries(robosim ${EXTRA_LIBS})

# set target version properties
set_target_properties(robosim PROPERTIES VERSION ${LIBROBOSIM_VERSION} SOVERSION ${LIBROBOSIM_SOVERSION})

