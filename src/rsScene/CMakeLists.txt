# build rsScene
project (rsScene)

# include directory
include_directories (
	"${LIBRS_SOURCE_DIR}/include/${PROJECT_NAME}"
	"${LIBRS_BINARY_DIR}"
)

# add OpenSceneGraph library
include_directories ("${LIBRS_SOURCE_DIR}/deps/osg/build/include")
include_directories ("${LIBRS_SOURCE_DIR}/deps/osg/include")
link_directories ("${LIBRS_SOURCE_DIR}/deps/osg/build/lib")
set(EXTRA_LIBS ${EXTRA_LIBS} optimized OpenThreads debug OpenThreadsd)
set(EXTRA_LIBS ${EXTRA_LIBS} optimized osg debug osgd)
set(EXTRA_LIBS ${EXTRA_LIBS} optimized osgDB debug osgDBd)
set(EXTRA_LIBS ${EXTRA_LIBS} optimized osgGA debug osgGAd)
set(EXTRA_LIBS ${EXTRA_LIBS} optimized osgShadow debug osgShadowd)
set(EXTRA_LIBS ${EXTRA_LIBS} optimized osgText debug osgTextd)
set(EXTRA_LIBS ${EXTRA_LIBS} optimized osgUtil debug osgUtild)
set(EXTRA_LIBS ${EXTRA_LIBS} optimized osgViewer debug osgViewerd)

# add openGL library
find_package (OpenGL REQUIRED)
if (OPENGL_FOUND)
	include_directories (${OPENGL_INCLUDE_DIR})
	set (EXTRA_LIBS ${EXTRA_LIBS} ${OPENGL_LIBRARIES})
endif (OPENGL_FOUND)

# add source files to library
set (SRCS ${SRCS}
	#cubusCallback.cpp
	keyboardHandler.cpp
	#linkbotCallback.cpp
	#mobotCallback.cpp
	mouseHandler.cpp
	#nxtCallback.cpp
	scene.cpp
	skyTransform.cpp
	textureCallback.cpp
)

# add the library
add_library (rsScene SHARED ${SRCS})

# link library against dependencies
target_link_libraries (rsScene ${EXTRA_LIBS})

# set version properties
set_target_properties (rsScene PROPERTIES VERSION ${LIBRS_VERSION} SOVERSION ${LIBRS_SO_VERSION})

