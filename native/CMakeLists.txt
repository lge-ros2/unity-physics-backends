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
option(BUILD_TESTS "Build the native test executables (tests/*.cpp)" OFF)

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
	target_sources(${BACKEND_TARGET} PRIVATE
		src/physx/common/PhysXSdk.cpp
		src/physx/common/PxObjects.cpp
		src/physx/common/PxFilter.cpp
		src/physx/common/PxMaterial.cpp
		src/physx/common/PxWorld.cpp
		src/physx/common/PxBody.cpp
		src/physx/common/PxShape.cpp
		src/physx/common/PxQuery.cpp
		src/physx/common/PxEvents.cpp
		src/physx/common/PxJoint.cpp
		src/physx/common/PxMesh.cpp
		src/physx/common/PxArticulation.cpp
		src/physx/v${PHYSX_SDK}/PxArticulationBackend.cpp
		src/physx/v${PHYSX_SDK}/PxCookingBackend.cpp)
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

if (BUILD_TESTS AND PHYSX5_LINK_PHYSX)
	# Loads the backend .so with dlopen and drives it through the Unity command entry point.
	add_executable(fall_test tests/fall_test.cpp)
	target_include_directories(fall_test SYSTEM PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/third_party/unity
		${CMAKE_CURRENT_SOURCE_DIR}/third_party/unity/PhysicsBackend)
	target_compile_options(fall_test PRIVATE -Wall -Wextra)
	target_link_libraries(fall_test PRIVATE dl)
	add_dependencies(fall_test ${BACKEND_TARGET})
	target_compile_definitions(fall_test PRIVATE
		FALL_TEST_BACKEND_PATH="$<TARGET_FILE:${BACKEND_TARGET}>"
		FALL_TEST_SDK=${PHYSX_SDK})

	add_executable(query_test tests/query_test.cpp)
	target_include_directories(query_test SYSTEM PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/third_party/unity
		${CMAKE_CURRENT_SOURCE_DIR}/third_party/unity/PhysicsBackend)
	target_compile_options(query_test PRIVATE -Wall -Wextra)
	target_link_libraries(query_test PRIVATE dl)
	add_dependencies(query_test ${BACKEND_TARGET})
	target_compile_definitions(query_test PRIVATE
		FALL_TEST_BACKEND_PATH="$<TARGET_FILE:${BACKEND_TARGET}>"
		FALL_TEST_SDK=${PHYSX_SDK})

	add_executable(mesh_test tests/mesh_test.cpp)
	target_include_directories(mesh_test SYSTEM PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/third_party/unity
		${CMAKE_CURRENT_SOURCE_DIR}/third_party/unity/PhysicsBackend)
	target_compile_options(mesh_test PRIVATE -Wall -Wextra)
	target_link_libraries(mesh_test PRIVATE dl)
	add_dependencies(mesh_test ${BACKEND_TARGET})
	target_compile_definitions(mesh_test PRIVATE
		FALL_TEST_BACKEND_PATH="$<TARGET_FILE:${BACKEND_TARGET}>"
		FALL_TEST_SDK=${PHYSX_SDK})

	add_executable(event_test tests/event_test.cpp)
	target_include_directories(event_test SYSTEM PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/third_party/unity
		${CMAKE_CURRENT_SOURCE_DIR}/third_party/unity/PhysicsBackend)
	target_compile_options(event_test PRIVATE -Wall -Wextra)
	target_link_libraries(event_test PRIVATE dl pthread)
	add_dependencies(event_test ${BACKEND_TARGET})
	target_compile_definitions(event_test PRIVATE
		FALL_TEST_BACKEND_PATH="$<TARGET_FILE:${BACKEND_TARGET}>"
		FALL_TEST_SDK=${PHYSX_SDK})

	add_executable(joint_test tests/joint_test.cpp)
	target_include_directories(joint_test SYSTEM PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/third_party/unity
		${CMAKE_CURRENT_SOURCE_DIR}/third_party/unity/PhysicsBackend)
	target_compile_options(joint_test PRIVATE -Wall -Wextra)
	target_link_libraries(joint_test PRIVATE dl)
	add_dependencies(joint_test ${BACKEND_TARGET})
	target_compile_definitions(joint_test PRIVATE
		FALL_TEST_BACKEND_PATH="$<TARGET_FILE:${BACKEND_TARGET}>"
		FALL_TEST_SDK=${PHYSX_SDK})

	add_executable(articulation_test tests/articulation_test.cpp)
	target_include_directories(articulation_test SYSTEM PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/third_party/unity
		${CMAKE_CURRENT_SOURCE_DIR}/third_party/unity/PhysicsBackend)
	target_compile_options(articulation_test PRIVATE -Wall -Wextra)
	target_link_libraries(articulation_test PRIVATE dl)
	add_dependencies(articulation_test ${BACKEND_TARGET})
	target_compile_definitions(articulation_test PRIVATE
		FALL_TEST_BACKEND_PATH="$<TARGET_FILE:${BACKEND_TARGET}>"
		FALL_TEST_SDK=${PHYSX_SDK})

	enable_testing()
	add_test(NAME articulation_test COMMAND articulation_test)
	add_test(NAME joint_test COMMAND joint_test)
	add_test(NAME event_test COMMAND event_test)
	add_test(NAME mesh_test COMMAND mesh_test)
	add_test(NAME fall_test COMMAND fall_test)
	add_test(NAME query_test COMMAND query_test)
endif()

set(PHYSX5_DEFAULT_INSTALL_DIR
	${CMAKE_CURRENT_SOURCE_DIR}/../packages/com.lge-ros2.physics-backend.physx5/Plugins~/linux)

if (PHYSX_SDK EQUAL 5)
	install(TARGETS ${BACKEND_TARGET} LIBRARY DESTINATION ${PHYSX5_DEFAULT_INSTALL_DIR})
endif()

if (PHYSX5_DEV_INSTALL_DIR)
	install(TARGETS ${BACKEND_TARGET} LIBRARY DESTINATION ${PHYSX5_DEV_INSTALL_DIR})
endif()
