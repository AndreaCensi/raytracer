cmake_minimum_required(VERSION 2.4)

project(simplemap2)

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONC REQUIRED json-c)

IF(${JSONC_FOUND})
	MESSAGE("JSONC_LIBRARY_DIRS: ${JSONC_LIBRARY_DIRS}")
	MESSAGE("JSONC_LIBRARIES: ${JSONC_LIBRARIES}")
	MESSAGE("JSONC_INCLUDE_DIRS: ${CJSONC_INCLUDE_DIRS}")

	INCLUDE_DIRECTORIES(${JSONC_INCLUDE_DIRS})
	LINK_DIRECTORIES(${JSONC_LIBRARY_DIRS})

ELSE(${JSONC_FOUND})	
	MESSAGE(FATAL_ERROR "JSONC not found. Check that the environment variable PKG_CONFIG_PATH includes the path containing the file 'json-c.pc'.")
ENDIF(${JSONC_FOUND})		

set(raytracer_SRCS
	simplemap.cpp
	math_utils.cpp
	raytracer_json_load.cpp
)


#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ggdb")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

add_executable(raytracer2 raytracer2.cpp ${raytracer_SRCS})
target_link_libraries(raytracer2 ${JSONC_LIBRARIES})
INSTALL(PROGRAMS raytracer2  DESTINATION bin)


