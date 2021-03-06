# build rsSim
project (rsSim)

# include directory
include_directories (
	"${LIBRS_SOURCE_DIR}/include/${PROJECT_NAME}"
	"${LIBRS_BINARY_DIR}"
)

# add ODE library
if (MSVC)
	include_directories ("${LIBRS_SOURCE_DIR}/deps/ode/include")
	link_directories ("${LIBRS_SOURCE_DIR}/deps/ode/lib/Release")
	link_directories ("${LIBRS_SOURCE_DIR}/deps/ode/lib/Debug")
else (MSVC)
	include_directories ("${LIBRS_SOURCE_DIR}/deps/ode/sys/include")
	link_directories ("${LIBRS_SOURCE_DIR}/deps/ode/sys/lib")
endif (MSVC)
set (EXTRA_LIBS ${EXTRA_LIBS} optimized ode debug oded)

# ODE options
add_definitions (-DdDOUBLE)
add_definitions (-DdNODEBUG)

# add source files to library
set (SRCS ${SRCS}
	robot.cpp
	modularRobot.cpp
	#cubus.cpp
	linkbot.cpp
	#mobot.cpp
	#nxt.cpp
	sim.cpp
)

# add the library
add_library (rsSim SHARED ${SRCS})

# link library against dependencies
target_link_libraries (rsSim ${EXTRA_LIBS})

# set version properties
set_target_properties (rsSim PROPERTIES VERSION ${LIBRS_VERSION} SOVERSION ${LIBRS_SO_VERSION})

