# build rsCallback
project (rsCallback)

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

# add rsRobots
set (EXTRA_LIBS ${EXTRA_LIBS} optimized rsRobots debug rsRobotsd)

# add rsSim
set (EXTRA_LIBS ${EXTRA_LIBS} optimized rsSim debug rsSimd)

# add OpenSceneGraph library
include_directories ("${LIBRS_SOURCE_DIR}/deps/osg/build/include")
include_directories ("${LIBRS_SOURCE_DIR}/deps/osg/include")
link_directories ("${LIBRS_SOURCE_DIR}/deps/osg/build/lib")
set (EXTRA_LIBS ${EXTRA_LIBS} optimized osg debug osgd)
set (EXTRA_LIBS ${EXTRA_LIBS} optimized osgText debug osgTextd)

# add rsScene
set (EXTRA_LIBS ${EXTRA_LIBS} optimized rsScene debug rsScened)

# add source files to library
set (SRCS ${SRCS}
	callback.cpp
	#cubusCallback.cpp
	groundCallback.cpp
	linkbotCallback.cpp
	#mobotCallback.cpp
	#nxtCallback.cpp
)

# add the library
add_library (rsCallback SHARED ${SRCS})

# link library against dependencies
target_link_libraries (rsCallback ${EXTRA_LIBS})

# set version properties
set_target_properties (rsCallback PROPERTIES VERSION ${LIBRS_VERSION} SOVERSION ${LIBRS_SO_VERSION})

