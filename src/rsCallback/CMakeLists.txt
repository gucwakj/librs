# build rsCallback
project (rsCallback)

# include directory
include_directories (
	"${LIBRS_SOURCE_DIR}/include/${PROJECT_NAME}"
	"${LIBRS_BINARY_DIR}"
)

# add ODE library
if (WIN32)
	include_directories ("${LIBRS_SOURCE_DIR}/deps/ode/include")
	link_directories ("${LIBRS_SOURCE_DIR}/deps/ode/lib/Release")
	link_directories ("${LIBRS_SOURCE_DIR}/deps/ode/lib/Debug")
else ()
	include_directories ("${LIBRS_SOURCE_DIR}/deps/ode/sys/include")
	link_directories ("${LIBRS_SOURCE_DIR}/deps/ode/sys/lib")
endif ()
set (EXTRA_LIBS ${EXTRA_LIBS} ode)

# ODE options
add_definitions (-DdDOUBLE)
add_definitions (-DdNODEBUG)

# add OpenSceneGraph library
include_directories ("${LIBRS_SOURCE_DIR}/deps/osg/build/include")
include_directories ("${LIBRS_SOURCE_DIR}/deps/osg/include")
link_directories ("${LIBRS_SOURCE_DIR}/deps/osg/build/lib")
set (EXTRA_LIBS ${EXTRA_LIBS} optimized osg debug osgd)
set (EXTRA_LIBS ${EXTRA_LIBS} optimized osgText debug osgTextd)

# add rsRobots
set (EXTRA_LIBS ${EXTRA_LIBS} optimized rsRobots debug rsRobotsd)
set (EXTRA_LIBS ${EXTRA_LIBS} optimized rsSim debug rsSimd)
set (EXTRA_LIBS ${EXTRA_LIBS} optimized rsScene debug rsScened)

# add source files to library
set (SRCS ${SRCS}
	callback.cpp
	obstacleCallback.cpp
)
if (ENABLE_LINKBOT)
	set (SRCS ${SRCS} linkbotCallback.cpp)
endif ()
if (ENABLE_MINDSTORMS)
	set (SRCS ${SRCS} mindstormsCallback.cpp)
endif ()

# add the library
add_library (rsCallback SHARED ${SRCS})

# link library against dependencies
target_link_libraries (rsCallback ${EXTRA_LIBS})

# set version properties
set_target_properties (rsCallback PROPERTIES VERSION ${LIBRS_VERSION} SOVERSION ${LIBRS_SO_VERSION})

