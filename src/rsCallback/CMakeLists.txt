# build rsCallback
project (rsCallback)

# include directory
include_directories (
	"${LibRS_SOURCE_DIR}/include/${PROJECT_NAME}"
	"${LibRS_BINARY_DIR}"
)

# add ODE library
if (WIN32)
	include_directories ("${LibRS_SOURCE_DIR}/deps/ode/include")
	link_directories ("${LibRS_SOURCE_DIR}/deps/ode/lib/Release")
	link_directories ("${LibRS_SOURCE_DIR}/deps/ode/lib/Debug")
else ()
	include_directories ("${LibRS_SOURCE_DIR}/deps/ode/sys/include")
	link_directories ("${LibRS_SOURCE_DIR}/deps/ode/sys/lib")
endif ()
set (EXTRA_LIBS ${EXTRA_LIBS} ode)

# ODE options
add_definitions (-DdDOUBLE)
add_definitions (-DdNODEBUG)

# add OpenSceneGraph library
include_directories ("${LibRS_SOURCE_DIR}/deps/osg3.4.0/build/include")
include_directories ("${LibRS_SOURCE_DIR}/deps/osg3.4.0/include")
link_directories ("${LibRS_SOURCE_DIR}/deps/osg3.4.0/build/lib")
set (EXTRA_LIBS ${EXTRA_LIBS} optimized osg debug osgd)
set (EXTRA_LIBS ${EXTRA_LIBS} optimized osgText debug osgTextd)

# add LibRS library
set (EXTRA_LIBS ${EXTRA_LIBS} optimized rs debug rsd)
set (EXTRA_LIBS ${EXTRA_LIBS} optimized rsRobots debug rsRobotsd)
set (EXTRA_LIBS ${EXTRA_LIBS} optimized rsSim debug rsSimd)
set (EXTRA_LIBS ${EXTRA_LIBS} optimized rsScene debug rsScened)

# add source files to library
set (SRCS ${SRCS}
	callback.cpp
	obstacle.cpp
)
if (RS_DOF)
	set (SRCS ${SRCS} dof.cpp)
endif ()
if (RS_LINKBOT)
	set (SRCS ${SRCS} linkbot.cpp)
endif ()
if (RS_MINDSTORMS)
	set (SRCS ${SRCS} mindstorms.cpp)
endif ()
if (RS_MINIDOF)
	set (SRCS ${SRCS} minidof.cpp)
endif ()

# add the library
add_library (rsCallback SHARED ${SRCS})

# link library against dependencies
target_link_libraries (rsCallback ${EXTRA_LIBS})

# set version properties
set_target_properties (rsCallback PROPERTIES VERSION ${LibRS_VERSION} SOVERSION ${LibRS_SO_VERSION})

