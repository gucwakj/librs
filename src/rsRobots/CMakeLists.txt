# build rsRobots
project (rsRobots)

# include directory
include_directories (
	"${LIBRS_SOURCE_DIR}/include/${PROJECT_NAME}"
	"${LIBRS_BINARY_DIR}"
)

# add source files to library
set (SRCS ${SRCS}
	rgbhashtable.c
	robot.cpp
	#cubus.cpp
	linkbot.cpp
	#mobot.cpp
	#nxt.cpp
)

# add the library
add_library (rsRobots SHARED ${SRCS})

# link library against dependencies
target_link_libraries (rsRobots ${EXTRA_LIBS})

# set version properties
set_target_properties (rsRobots PROPERTIES VERSION ${LIBRS_VERSION} SOVERSION ${LIBRS_SO_VERSION})

