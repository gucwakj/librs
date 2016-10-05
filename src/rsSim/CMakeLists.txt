# build rsSim
project (rsSim)

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

# add threading
find_package (Threads)
set (EXTRA_LIBS ${EXTRA_LIBS} ${CMAKE_THREAD_LIBS_INIT})

# add gsl
if (RS_RESEARCH)
	find_package (GSL REQUIRED)
	include_directories (${GSL_INCLUDE_DIRS} ${GSLCBLAS_INCLUDE_DIRS})
	set (EXTRA_LIBS ${EXTRA_LIBS} ${GSL_LIBRARIES} ${GSLCBLAS_LIBRARIES})
endif ()

# add LibRS library
set (EXTRA_LIBS ${EXTRA_LIBS} optimized rs debug rsd)
set (EXTRA_LIBS ${EXTRA_LIBS} optimized rsRobots debug rsRobotsd)
if (RS_RESEARCH)
	set (EXTRA_LIBS ${EXTRA_LIBS} optimized rsResearch debug rsResearchd)
endif ()

# add source files to library
set (SRCS ${SRCS}
	robot.cpp
	modularrobot.cpp
	sim.cpp
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
add_library (rsSim SHARED ${SRCS})

# link library against dependencies
target_link_libraries (rsSim ${EXTRA_LIBS})

# set version properties
set_target_properties (rsSim PROPERTIES VERSION ${LibRS_VERSION} SOVERSION ${LibRS_SO_VERSION})

