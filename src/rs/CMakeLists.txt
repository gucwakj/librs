# build rs
project (rs)

# include directory
include_directories (
	"${LIBRS_SOURCE_DIR}/include/${PROJECT_NAME}"
	"${LIBRS_BINARY_DIR}"
)

# add source files to library
set (SRCS ${SRCS}
	pos.cpp
	quat.cpp
)

# add the library
add_library (rs SHARED ${SRCS})

# link library against dependencies
target_link_libraries (rs ${EXTRA_LIBS})

# set version properties
set_target_properties (rs PROPERTIES VERSION ${LIBRS_VERSION} SOVERSION ${LIBRS_SO_VERSION})

