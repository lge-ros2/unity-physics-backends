cmake_minimum_required(VERSION 3.10)
project(PhysX5Backend LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN ON)

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(PHYSX5_DEV_INSTALL_DIR "" CACHE STRING "Optional extra install directory (e.g. a Unity project Plugins folder)")

add_library(PhysX5Backend SHARED
	src/PluginMain.cpp
	src/Exports.cpp
	src/core/Dispatch.cpp
	src/core/StubWorld.cpp)

set_target_properties(PhysX5Backend PROPERTIES
	OUTPUT_NAME PhysX5Backend
	PREFIX "lib"
	SUFFIX ".so")

target_include_directories(PhysX5Backend PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
# SYSTEM: keep -Wall/-Wextra noise from the vendored Unity headers out of the build.
target_include_directories(PhysX5Backend SYSTEM PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/third_party/unity
	${CMAKE_CURRENT_SOURCE_DIR}/third_party/unity/PhysicsBackend)

target_compile_options(PhysX5Backend PRIVATE
	-Wall -Wextra -fPIC -fvisibility=hidden -fvisibility-inlines-hidden -O2)

# Statically link the C++ runtime and export only the Unity entry points.
target_link_libraries(PhysX5Backend PRIVATE
	-static-libstdc++ -static-libgcc
	-Wl,--version-script=${CMAKE_CURRENT_SOURCE_DIR}/exports.map
	-Wl,--no-undefined)
set_target_properties(PhysX5Backend PROPERTIES LINK_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/exports.map)

set(PHYSX5_DEFAULT_INSTALL_DIR
	${CMAKE_CURRENT_SOURCE_DIR}/../packages/com.lge-ros2.physics-backend.physx5/Plugins~/linux)

install(TARGETS PhysX5Backend LIBRARY DESTINATION ${PHYSX5_DEFAULT_INSTALL_DIR})

if (PHYSX5_DEV_INSTALL_DIR)
	install(TARGETS PhysX5Backend LIBRARY DESTINATION ${PHYSX5_DEV_INSTALL_DIR})
endif()
