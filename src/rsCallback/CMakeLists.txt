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
else (MSVC)
	include_directories ("${LIBRS_SOURCE_DIR}/deps/ode/sys/include")
endif (MSVC)

# ODE options
add_definitions (-DdDOUBLE)
add_definitions (-DdNODEBUG)

# rsSim
include_directories ("${LIBRS_SOURCE_DIR}/include/rsSim")

# add OpenSceneGraph library
include_directories ("${LIBRS_SOURCE_DIR}/deps/osg/build/include")
include_directories ("${LIBRS_SOURCE_DIR}/deps/osg/include")

# rsScene
include_directories ("${LIBRS_SOURCE_DIR}/include/rsScene")

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

