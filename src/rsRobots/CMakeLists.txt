# build rsRobots
project (rsRobots)

# include directory
include_directories (
	"${LibRS_SOURCE_DIR}/include/${PROJECT_NAME}"
	"${LibRS_BINARY_DIR}"
)

# add LibRS library
set (EXTRA_LIBS ${EXTRA_LIBS} optimized rs debug rsd)

# add source files to library
set (SRCS ${SRCS}
	robot.cpp
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
add_library (rsRobots SHARED ${SRCS})

# link library against dependencies
target_link_libraries (rsRobots ${EXTRA_LIBS})

# set version properties
set_target_properties (rsRobots PROPERTIES VERSION ${LibRS_VERSION} SOVERSION ${LibRS_SO_VERSION})

