# build rsScene
project (rsScene)

# include directory
include_directories (
	"${LibRS_SOURCE_DIR}/include/${PROJECT_NAME}"
	"${LibRS_BINARY_DIR}"
)

# prevent windows from auto adding manifest
if (WIN32)
	set (CMAKE_SHARED_LINKER_FLAGS /MANIFEST:NO)
endif ()

# add OpenSceneGraph library
include_directories ("${LibRS_SOURCE_DIR}/deps/osg3.4.0/build/include")
include_directories ("${LibRS_SOURCE_DIR}/deps/osg3.4.0/include")
link_directories ("${LibRS_SOURCE_DIR}/deps/osg3.4.0/build/lib")
set (EXTRA_LIBS ${EXTRA_LIBS} optimized OpenThreads debug OpenThreadsd)
set (EXTRA_LIBS ${EXTRA_LIBS} optimized osg debug osgd)
set (EXTRA_LIBS ${EXTRA_LIBS} optimized osgDB debug osgDBd)
set (EXTRA_LIBS ${EXTRA_LIBS} optimized osgFX debug osgFXd)
set (EXTRA_LIBS ${EXTRA_LIBS} optimized osgGA debug osgGAd)
set (EXTRA_LIBS ${EXTRA_LIBS} optimized osgShadow debug osgShadowd)
set (EXTRA_LIBS ${EXTRA_LIBS} optimized osgText debug osgTextd)
set (EXTRA_LIBS ${EXTRA_LIBS} optimized osgUtil debug osgUtild)
set (EXTRA_LIBS ${EXTRA_LIBS} optimized osgViewer debug osgViewerd)

# add openGL library
find_package (OpenGL REQUIRED)
if (OPENGL_FOUND)
	include_directories (${OPENGL_INCLUDE_DIR})
	set (EXTRA_LIBS ${EXTRA_LIBS} ${OPENGL_LIBRARIES})
endif ()

# add opencv
find_package (OpenCV REQUIRED)
if (OpenCV_FOUND)
	include_directories (${OpenCV_INC})
	set (EXTRA_LIBS ${EXTRA_LIBS} ${OpenCV_LIBS})
endif ()

# add threading
find_package (Threads)
set (EXTRA_LIBS ${EXTRA_LIBS} ${CMAKE_THREAD_LIBS_INIT})

# add LibRS library
set (EXTRA_LIBS ${EXTRA_LIBS} optimized rs debug rsd)
set (EXTRA_LIBS ${EXTRA_LIBS} optimized rsRobots debug rsRobotsd)

# add source files to library
set (SRCS ${SRCS}
	texturecallback.cpp
	keyboardhandler.cpp
	mousehandler.cpp
	skytransform.cpp
	opencvoperation.cpp
	scene.cpp
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

# add the library
add_library (rsScene SHARED ${SRCS})

# link library against dependencies
target_link_libraries (rsScene ${EXTRA_LIBS})

# set version properties
set_target_properties (rsScene PROPERTIES VERSION ${LibRS_VERSION} SOVERSION ${LibRS_SO_VERSION})

# add windows manifest file for DPI scaling
if (WIN32)
	add_custom_command (TARGET rsScene
		POST_BUILD
		COMMAND "mt.exe" -manifest \"${CMAKE_CURRENT_SOURCE_DIR}\\..\\..\\resources\\dpiawarescaling.manifest\" -outputresource:\"${OUTPUT_BINDIR}\\rsScene.dll\"\;\#2
		COMMENT "Adding display aware manifest..."
	)
endif ()

