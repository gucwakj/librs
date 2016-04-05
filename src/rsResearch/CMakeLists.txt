# build rsResearch
project (rsResearch)

# include directory
include_directories (
	"${LibRS_SOURCE_DIR}/include/${PROJECT_NAME}"
	"${LibRS_BINARY_DIR}"
)

# set module search path
set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${LibRS_SOURCE_DIR}/resources")

# add gsl
find_package (GSL REQUIRED)
include_directories (${GSL_INCLUDE_DIRS} ${GSLCBLAS_INCLUDE_DIRS})
set (EXTRA_LIBS ${EXTRA_LIBS} ${GSL_LIBRARIES} ${GSLCBLAS_LIBRARIES})

# add zmq
find_package (ZMQ REQUIRED)
include_directories (${ZMQ_INCLUDE_DIRS})
set (EXTRA_LIBS ${EXTRA_LIBS} ${ZMQ_LIBRARIES})

# add LibRS library
set (EXTRA_LIBS ${EXTRA_LIBS} optimized rs debug rsd)

# add source files to library
set (SRCS ${SRCS}
	integrator.cpp
	publisher.cpp
	replier.cpp
	requester.cpp
	subscriber.cpp
)

# add the library
add_library (rsResearch SHARED ${SRCS})

# link library against dependencies
target_link_libraries (rsResearch ${EXTRA_LIBS})

# set version properties
set_target_properties (rsResearch PROPERTIES VERSION ${LibRS_VERSION} SOVERSION ${LibRS_SO_VERSION})

