#
# Find the ZMQ includes and library
# 
#	This module defines
#		ZMQ_FOUND
#		ZMQ_INCLUDE_DIRS	where to find zmq.h
#		ZMQ_LIBRARIES		the library needed to use ZMQ

set (ZMQ_FOUND 0)

find_path (ZMQ_ROOT_DIR
	NAMES
		include/zmq.h
	PATHS
		/usr
		/usr/local
)

find_path (ZMQ_INCLUDE_DIRS
	NAMES
		zmq.h
	PATHS
		${ZMQ_ROOT_DIR}/include
)

find_library (ZMQ_LIBRARIES
	NAMES
		zmq
		${ZMQ_LIBRARY_NAME}
	PATHS
		/lib
		/usr/lib
		/usr/local/lib
		"${ZMQ_ROOT_DIR}/lib"
)

if (ZMQ_INCLUDE_DIRS AND ZMQ_LIBRARIES)
	set (ZMQ_FOUND 1)
	message (STATUS "Found ZMQ library: ${ZMQ_LIBRARIES}")
	message (STATUS "Found ZMQ headers: ${ZMQ_INCLUDE_DIRS}")
else ()
	message (FATAL_ERROR "Could not find ZMQ libraries/headers! Please install ZMQ with libraries and headers")
endif ()

