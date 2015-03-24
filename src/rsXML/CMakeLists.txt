# build rsXML
project (rsXML)

# include directory
include_directories (
	"${LIBRS_SOURCE_DIR}/include/${PROJECT_NAME}"
	"${LIBRS_BINARY_DIR}"
)

# add tinyxml2 library
include_directories ("${LIBRS_SOURCE_DIR}/deps/tinyxml2")
if (MSVC)
	link_directories ("${LIBRS_SOURCE_DIR}/deps/tinyxml2/build/Release")
else (MSVC)
	link_directories ("${LIBRS_SOURCE_DIR}/deps/tinyxml2/build")
endif (MSVC)
set (EXTRA_LIBS ${EXTRA_LIBS} tinyxml2)

# add rsRobots
set (EXTRA_LIBS ${EXTRA_LIBS} optimized rsRobots debug rsRobotsd)

# add source files to library
set (SRCS ${SRCS}
	conn.cpp
	ground.cpp
	marker.cpp
	robot.cpp
	reader.cpp
	writer.cpp
)
if (ENABLE_LINKBOT)
	set (SRCS ${SRCS} linkbot.cpp)
endif (ENABLE_LINKBOT)
if (ENABLE_MINDSTORMS)
	set (SRCS ${SRCS} mindstorms.cpp)
endif (ENABLE_MINDSTORMS)

# add the library
add_library (rsXML SHARED ${SRCS})

# link library against dependencies
target_link_libraries (rsXML ${EXTRA_LIBS})

# set version properties
set_target_properties (rsXML PROPERTIES VERSION ${LIBRS_VERSION} SOVERSION ${LIBRS_SO_VERSION})

