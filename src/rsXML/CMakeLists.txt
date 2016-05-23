# build rsXML
project (rsXML)

# include directory
include_directories (
	"${LibRS_SOURCE_DIR}/include/${PROJECT_NAME}"
	"${LibRS_BINARY_DIR}"
)

# include windows api
if (WIN32)
	set (EXTRA_LIBS ${EXTRA_LIBS} shlwapi)
endif ()

# add tinyxml2 library
include_directories ("${LibRS_SOURCE_DIR}/deps/tinyxml2")
link_directories ("${LibRS_SOURCE_DIR}/deps/tinyxml2/build/lib")
set (EXTRA_LIBS ${EXTRA_LIBS} optimized tinyxml2 debug tinyxml2d)

# add LibRS library
set (EXTRA_LIBS ${EXTRA_LIBS} optimized rs debug rsd)
set (EXTRA_LIBS ${EXTRA_LIBS} optimized rsRobots debug rsRobotsd)

# add source files to library
set (SRCS ${SRCS}
	conn.cpp
	marker.cpp
	obstacle.cpp
	robot.cpp
	backgroundreader.cpp
	reader.cpp
	writer.cpp
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

# add the library
add_library (rsXML SHARED ${SRCS})

# link library against dependencies
target_link_libraries (rsXML ${EXTRA_LIBS})

# set version properties
set_target_properties (rsXML PROPERTIES VERSION ${LibRS_VERSION} SOVERSION ${LibRS_SO_VERSION})

