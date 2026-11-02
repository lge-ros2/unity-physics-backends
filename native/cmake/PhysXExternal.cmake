# Builds the selected PhysX SDK (CPU only, static, PIC) from its submodule.
#   PHYSX_SDK=5 -> third_party/PhysX5 (PhysX 5.x, NVIDIA-Omniverse/PhysX)
#   PHYSX_SDK=4 -> third_party/PhysX4 (PhysX 4.1, NVIDIAGameWorks/PhysX, reference backend only)
# PhysX's own generators want packman; we call their CMake projects directly instead.

if (PHYSX_SDK EQUAL 5)
	set(PHYSX_SUBMODULE ${CMAKE_CURRENT_SOURCE_DIR}/third_party/PhysX5)
	set(PHYSX_ROOT ${PHYSX_SUBMODULE}/physx)
	set(PHYSX_PLATFORM_DIR linux.x86_64)
	set(PHYSX_INCLUDE_DIRS ${PHYSX_ROOT}/include)
	set(PHYSX_ENV PHYSX_ROOT_DIR=${PHYSX_ROOT})
	set(PHYSX_EXTRA_ARGS
		-DPX_BUILDPVDRUNTIME=OFF
		-DPX_GENERATE_GPU_PROJECTS=OFF)
	set(PHYSX_LIB_NAMES PhysX PhysXExtensions PhysXCooking PhysXCommon PhysXFoundation PhysXPvdSDK
		PhysXCharacterKinematic PhysXVehicle2)
	set(PHYSX_SUBMODULE_HINT "git submodule update --init --depth 1 native/third_party/PhysX5")
elseif (PHYSX_SDK EQUAL 4)
	set(PHYSX_SUBMODULE ${CMAKE_CURRENT_SOURCE_DIR}/third_party/PhysX4)
	set(PHYSX_ROOT ${PHYSX_SUBMODULE}/physx)
	set(PHYSX_PLATFORM_DIR linux.clang)
	set(PHYSX_INCLUDE_DIRS ${PHYSX_ROOT}/include ${PHYSX_SUBMODULE}/pxshared/include)
	set(PHYSX_ENV PHYSX_ROOT_DIR=${PHYSX_ROOT}
		PM_CMakeModules_PATH=${PHYSX_SUBMODULE}/externals/cmakemodules
		PM_PxShared_PATH=${PHYSX_SUBMODULE}/pxshared)
	set(PHYSX_EXTRA_ARGS
		-DCMAKEMODULES_PATH=${PHYSX_SUBMODULE}/externals/cmakemodules
		-DCMAKEMODULES_NAME=CMakeModules
		-DCMAKEMODULES_VERSION=1.27
		-DPX_BUILDPUBLICSAMPLES=OFF)
	set(PHYSX_LIB_NAMES PhysX PhysXExtensions PhysXCooking PhysXCommon PhysXFoundation PhysXPvdSDK
		PhysXCharacterKinematic PhysXVehicle)
	set(PHYSX_SUBMODULE_HINT "git submodule update --init --depth 1 --checkout native/third_party/PhysX4")
	# PhysX 4.1 predates gcc 13 and trips -Werror on several warnings; silence them for the SDK build only.
	set(PHYSX_WARNING_FLAGS " -w")
else()
	message(FATAL_ERROR "PHYSX_SDK must be 4 or 5")
endif()

if (NOT EXISTS ${PHYSX_ROOT}/compiler/public/CMakeLists.txt)
	message(FATAL_ERROR "PhysX submodule is missing. Run: ${PHYSX_SUBMODULE_HINT}")
endif()

include(ExternalProject)

# PHYSX_PREBUILT_LIB_DIR: reuse already built static libraries (e.g. another build directory)
# instead of compiling PhysX again.
set(PHYSX_PREBUILT_LIB_DIR "" CACHE PATH "Directory with prebuilt PhysX static libraries (skips the PhysX build)")

set(PHYSX_BUILD_DIR ${CMAKE_BINARY_DIR}/physx-build)
if (PHYSX_PREBUILT_LIB_DIR)
	set(PHYSX_LIB_DIR ${PHYSX_PREBUILT_LIB_DIR})
else()
	set(PHYSX_LIB_DIR ${PHYSX_BUILD_DIR}/out/bin/${PHYSX_PLATFORM_DIR}/release)
endif()

set(PHYSX_LIB_FILES)
foreach(name IN LISTS PHYSX_LIB_NAMES)
	list(APPEND PHYSX_LIB_FILES ${PHYSX_LIB_DIR}/lib${name}_static_64.a)
endforeach()

if (NOT PHYSX_PREBUILT_LIB_DIR)
ExternalProject_Add(physx_sdk
	SOURCE_DIR ${PHYSX_ROOT}/compiler/public
	BINARY_DIR ${PHYSX_BUILD_DIR}
	CONFIGURE_COMMAND ${CMAKE_COMMAND} -E env ${PHYSX_ENV}
		${CMAKE_COMMAND} -Wno-dev -S <SOURCE_DIR> -B <BINARY_DIR>
		-DTARGET_BUILD_PLATFORM=linux
		-DPX_OUTPUT_ARCH=x86
		-DCMAKE_BUILD_TYPE=release
		-DPHYSX_ROOT_DIR=${PHYSX_ROOT}
		-DPX_OUTPUT_LIB_DIR=${PHYSX_BUILD_DIR}/out
		-DPX_OUTPUT_BIN_DIR=${PHYSX_BUILD_DIR}/out
		-DPX_OUTPUT_EXE_DIR=${PHYSX_BUILD_DIR}/out
		-DPX_OUTPUT_DLL_DIR=${PHYSX_BUILD_DIR}/out
		-DPX_GENERATE_STATIC_LIBRARIES=ON
		-DPX_BUILDSNIPPETS=OFF
		-DCMAKE_POSITION_INDEPENDENT_CODE=ON
		"-DCMAKE_CXX_FLAGS=-fPIC -fvisibility=hidden${PHYSX_WARNING_FLAGS}"
		"-DCMAKE_C_FLAGS=-fPIC${PHYSX_WARNING_FLAGS}"
		${PHYSX_EXTRA_ARGS}
	BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> -j
	INSTALL_COMMAND ""
	BUILD_BYPRODUCTS ${PHYSX_LIB_FILES})
endif()

add_library(physx_sdk_libs INTERFACE)
if (NOT PHYSX_PREBUILT_LIB_DIR)
	add_dependencies(physx_sdk_libs physx_sdk)
endif()
target_include_directories(physx_sdk_libs SYSTEM INTERFACE ${PHYSX_INCLUDE_DIRS})
# Static libraries reference each other, so wrap them in a link group.
target_link_libraries(physx_sdk_libs INTERFACE
	-Wl,--start-group ${PHYSX_LIB_FILES} -Wl,--end-group pthread dl rt)
