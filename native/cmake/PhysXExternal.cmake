# Builds the PhysX SDK (CPU only, static, PIC) from the third_party/PhysX submodule.
# PhysX's own generator wants packman; we call its CMake project directly instead.
# Note: PhysX writes its generated PxConfig.h into the source tree (physx/include).

set(PHYSX_SOURCE_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/third_party/PhysX/physx)
if (NOT EXISTS ${PHYSX_SOURCE_ROOT}/compiler/public/CMakeLists.txt)
	message(FATAL_ERROR "PhysX submodule is missing. Run: git submodule update --init --depth 1 native/third_party/PhysX")
endif()

include(ExternalProject)

set(PHYSX_BUILD_DIR ${CMAKE_BINARY_DIR}/physx-build)
set(PHYSX_LIB_DIR ${PHYSX_BUILD_DIR}/out/bin/linux.x86_64/release)

set(PHYSX_LIB_NAMES
	PhysX
	PhysXExtensions
	PhysXCooking
	PhysXCommon
	PhysXFoundation
	PhysXPvdSDK
	PhysXCharacterKinematic
	PhysXVehicle2)

set(PHYSX_LIB_FILES)
foreach(name IN LISTS PHYSX_LIB_NAMES)
	list(APPEND PHYSX_LIB_FILES ${PHYSX_LIB_DIR}/lib${name}_static_64.a)
endforeach()

ExternalProject_Add(physx_sdk
	SOURCE_DIR ${PHYSX_SOURCE_ROOT}/compiler/public
	BINARY_DIR ${PHYSX_BUILD_DIR}
	CONFIGURE_COMMAND ${CMAKE_COMMAND} -E env PHYSX_ROOT_DIR=${PHYSX_SOURCE_ROOT}
		${CMAKE_COMMAND} -S <SOURCE_DIR> -B <BINARY_DIR>
		-DTARGET_BUILD_PLATFORM=linux
		-DPX_OUTPUT_ARCH=x86
		-DCMAKE_BUILD_TYPE=release
		-DPHYSX_ROOT_DIR=${PHYSX_SOURCE_ROOT}
		-DPX_OUTPUT_LIB_DIR=${PHYSX_BUILD_DIR}/out
		-DPX_OUTPUT_BIN_DIR=${PHYSX_BUILD_DIR}/out
		-DPX_OUTPUT_EXE_DIR=${PHYSX_BUILD_DIR}/out
		-DPX_OUTPUT_DLL_DIR=${PHYSX_BUILD_DIR}/out
		-DPX_GENERATE_STATIC_LIBRARIES=ON
		-DPX_BUILDSNIPPETS=OFF
		-DPX_BUILDPVDRUNTIME=OFF
		-DPX_GENERATE_GPU_PROJECTS=OFF
		-DCMAKE_POSITION_INDEPENDENT_CODE=ON
		"-DCMAKE_CXX_FLAGS=-fPIC -fvisibility=hidden"
		"-DCMAKE_C_FLAGS=-fPIC"
	BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> -j
	INSTALL_COMMAND ""
	BUILD_BYPRODUCTS ${PHYSX_LIB_FILES})

add_library(physx_sdk_libs INTERFACE)
add_dependencies(physx_sdk_libs physx_sdk)
target_include_directories(physx_sdk_libs SYSTEM INTERFACE ${PHYSX_SOURCE_ROOT}/include)
# Static libraries reference each other, so wrap them in a link group.
target_link_libraries(physx_sdk_libs INTERFACE
	-Wl,--start-group ${PHYSX_LIB_FILES} -Wl,--end-group pthread dl rt)
