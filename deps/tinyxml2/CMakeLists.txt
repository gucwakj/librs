project(tinyxml2)

cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
cmake_policy(VERSION 2.6)

################################
# set lib version here

set(GENERIC_LIB_VERSION "1.0.11")
set(GENERIC_LIB_SOVERSION "1")


################################
# Add common source

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/.")


################################
# Add definitions

if(MSVC)
	add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif(MSVC)


################################
add_library(tinyxml2 SHARED tinyxml2.cpp tinyxml2.h)
set_target_properties(tinyxml2 PROPERTIES
	VERSION "${GENERIC_LIB_VERSION}"
	SOVERSION "${GENERIC_LIB_SOVERSION}")

