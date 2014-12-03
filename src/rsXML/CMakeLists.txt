# build rsXML
project (rsXML)

# include directory
include_directories (
	"${LIBRS_SOURCE_DIR}/inc/${PROJECT_NAME}"
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

# add source files to library
set (SRCS ${SRCS}
	store.cpp
	robot.cpp
	conn.cpp
)

# add the library
add_library (rsXML SHARED ${SRCS})

# link library against dependencies
target_link_libraries (rsXML ${EXTRA_LIBS})

# set version properties
set_target_properties (rsXML PROPERTIES VERSION ${LIBRS_VERSION} SOVERSION ${LIBRS_SO_VERSION})

