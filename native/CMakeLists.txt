cmake_minimum_required(VERSION 3.10)
project(PhysicsBackend LANGUAGES CXX)

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
set(PHYSX_SDK 5 CACHE STRING "PhysX SDK to link: 5 = PhysX 5.x backend (shipped), 4 = PhysX 4.1 reference backend (verification only)")
set_property(CACHE PHYSX_SDK PROPERTY STRINGS 5 4)
option(PHYSX5_LINK_PHYSX "Link the PhysX SDK (third_party/PhysX4 or PhysX5) into the backend" ON)

if (PHYSX_SDK EQUAL 4)
	set(BACKEND_TARGET PhysX4RefBackend)
else()
	set(BACKEND_TARGET PhysX5Backend)
endif()

add_library(${BACKEND_TARGET} SHARED
	src/PluginMain.cpp
	src/Exports.cpp
	src/core/Dispatch.cpp
	src/core/CommandTrace.cpp
	src/core/StubWorld.cpp)

target_compile_definitions(${BACKEND_TARGET} PRIVATE BACKEND_PHYSX_SDK=${PHYSX_SDK})

if (PHYSX5_LINK_PHYSX)
	include(cmake/PhysXExternal.cmake)
	target_sources(${BACKEND_TARGET} PRIVATE src/physx/common/PhysXSdk.cpp)
	target_compile_definitions(${BACKEND_TARGET} PRIVATE PHYSX5_LINK_PHYSX=1)
	target_include_directories(${BACKEND_TARGET} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/physx/v${PHYSX_SDK})
	target_link_libraries(${BACKEND_TARGET} PRIVATE physx_sdk_libs)
endif()

set_target_properties(${BACKEND_TARGET} PROPERTIES
	OUTPUT_NAME ${BACKEND_TARGET}
	PREFIX "lib"
	SUFFIX ".so")

target_include_directories(${BACKEND_TARGET} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
# SYSTEM: keep -Wall/-Wextra noise from the vendored Unity headers out of the build.
target_include_directories(${BACKEND_TARGET} SYSTEM PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/third_party/unity
	${CMAKE_CURRENT_SOURCE_DIR}/third_party/unity/PhysicsBackend)

target_compile_options(${BACKEND_TARGET} PRIVATE
	-Wall -Wextra -fPIC -fvisibility=hidden -fvisibility-inlines-hidden -O2)

# Statically link the C++ runtime and export only the Unity entry points.
target_link_libraries(${BACKEND_TARGET} PRIVATE
	-static-libstdc++ -static-libgcc
	-Wl,--exclude-libs,ALL -Wl,-Bsymbolic
	-Wl,--version-script=${CMAKE_CURRENT_SOURCE_DIR}/exports.map
	-Wl,--no-undefined)
set_target_properties(${BACKEND_TARGET} PROPERTIES LINK_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/exports.map)

set(PHYSX5_DEFAULT_INSTALL_DIR
	${CMAKE_CURRENT_SOURCE_DIR}/../packages/com.lge-ros2.physics-backend.physx5/Plugins~/linux)

if (PHYSX_SDK EQUAL 5)
	install(TARGETS ${BACKEND_TARGET} LIBRARY DESTINATION ${PHYSX5_DEFAULT_INSTALL_DIR})
endif()

if (PHYSX5_DEV_INSTALL_DIR)
	install(TARGETS ${BACKEND_TARGET} LIBRARY DESTINATION ${PHYSX5_DEV_INSTALL_DIR})
endif()
