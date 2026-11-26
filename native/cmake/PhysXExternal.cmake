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

if (WIN32)
	if (NOT PHYSX_SDK EQUAL 5)
		message(FATAL_ERROR "The PhysX 4.1 reference backend is only built on Linux")
	endif()
	# PhysX names its output folder after the toolset and the CRT: win.x86_64.vc143.mt = MSVC 2022, static CRT.
	if (MSVC_VERSION GREATER_EQUAL 1930)
		set(PHYSX_PLATFORM_DIR win.x86_64.vc143.mt)
	else()
		set(PHYSX_PLATFORM_DIR win.x86_64.vc142.mt)
	endif()
	set(PHYSX_TARGET_PLATFORM windows)
	set(PHYSX_LIB_PREFIX "")
	set(PHYSX_LIB_SUFFIX ".lib")
	# compiler/public forces PUBLIC_RELEASE=1, and the Windows project then copies freeglut.dll (only needed by the
	# PhysX samples, which are not built) from $PM_freeglut_PATH/bin/win64 and fails when the file is missing.
	# Empty placeholders satisfy the copy.
	set(PHYSX_FREEGLUT_STUB ${CMAKE_BINARY_DIR}/physx-stubs/freeglut)
	file(MAKE_DIRECTORY ${PHYSX_FREEGLUT_STUB}/bin/win64)
	file(WRITE ${PHYSX_FREEGLUT_STUB}/bin/win64/freeglut.dll "")
	file(WRITE ${PHYSX_FREEGLUT_STUB}/bin/win64/freeglutd.dll "")
	list(APPEND PHYSX_ENV PM_freeglut_PATH=${PHYSX_FREEGLUT_STUB})
	set(PHYSX_PLATFORM_ARGS -DNV_USE_STATIC_WINCRT=ON -DPX_GENERATE_STATIC_LIBRARIES=ON)
	set(PHYSX_CXX_FLAGS_ARGS "")
	set(PHYSX_C_FLAGS_ARGS "")
	set(PHYSX_BUILD_CONFIG_ARGS --config release)
	set(PHYSX_GENERATOR_ARGS -G "${CMAKE_GENERATOR}")
	if (CMAKE_GENERATOR_PLATFORM)
		list(APPEND PHYSX_GENERATOR_ARGS -A ${CMAKE_GENERATOR_PLATFORM})
	endif()
else()
	set(PHYSX_TARGET_PLATFORM linux)
	set(PHYSX_LIB_PREFIX lib)
	set(PHYSX_LIB_SUFFIX .a)
	set(PHYSX_PLATFORM_ARGS -DPX_GENERATE_STATIC_LIBRARIES=ON -DCMAKE_POSITION_INDEPENDENT_CODE=ON)
	set(PHYSX_CXX_FLAGS_ARGS "-DCMAKE_CXX_FLAGS=-fPIC -fvisibility=hidden${PHYSX_WARNING_FLAGS}")
	set(PHYSX_C_FLAGS_ARGS "-DCMAKE_C_FLAGS=-fPIC${PHYSX_WARNING_FLAGS}")
	set(PHYSX_BUILD_CONFIG_ARGS "")
	set(PHYSX_GENERATOR_ARGS "")
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
	list(APPEND PHYSX_LIB_FILES ${PHYSX_LIB_DIR}/${PHYSX_LIB_PREFIX}${name}_static_64${PHYSX_LIB_SUFFIX})
endforeach()

if (NOT PHYSX_PREBUILT_LIB_DIR)
ExternalProject_Add(physx_sdk
	SOURCE_DIR ${PHYSX_ROOT}/compiler/public
	BINARY_DIR ${PHYSX_BUILD_DIR}
	CONFIGURE_COMMAND ${CMAKE_COMMAND} -E env ${PHYSX_ENV}
		${CMAKE_COMMAND} -Wno-dev ${PHYSX_GENERATOR_ARGS} -S <SOURCE_DIR> -B <BINARY_DIR>
		-DTARGET_BUILD_PLATFORM=${PHYSX_TARGET_PLATFORM}
		-DPX_OUTPUT_ARCH=x86
		-DCMAKE_BUILD_TYPE=release
		-DPHYSX_ROOT_DIR=${PHYSX_ROOT}
		-DPX_OUTPUT_LIB_DIR=${PHYSX_BUILD_DIR}/out
		-DPX_OUTPUT_BIN_DIR=${PHYSX_BUILD_DIR}/out
		-DPX_OUTPUT_EXE_DIR=${PHYSX_BUILD_DIR}/out
		-DPX_OUTPUT_DLL_DIR=${PHYSX_BUILD_DIR}/out
		-DPX_BUILDSNIPPETS=OFF
		${PHYSX_PLATFORM_ARGS}
		${PHYSX_CXX_FLAGS_ARGS}
		${PHYSX_C_FLAGS_ARGS}
		${PHYSX_EXTRA_ARGS}
	BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> ${PHYSX_BUILD_CONFIG_ARGS} -j
	INSTALL_COMMAND ""
	BUILD_BYPRODUCTS ${PHYSX_LIB_FILES})
endif()

add_library(physx_sdk_libs INTERFACE)
if (NOT PHYSX_PREBUILT_LIB_DIR)
	add_dependencies(physx_sdk_libs physx_sdk)
endif()
target_include_directories(physx_sdk_libs SYSTEM INTERFACE ${PHYSX_INCLUDE_DIRS})
if (WIN32)
	# Without PX_PHYSX_STATIC_LIB the PhysX headers declare every API as __declspec(dllimport) (__imp_ symbols).
	target_compile_definitions(physx_sdk_libs INTERFACE PX_PHYSX_STATIC_LIB)
	target_link_libraries(physx_sdk_libs INTERFACE ${PHYSX_LIB_FILES} ws2_32 winmm)
else()
	# Static libraries reference each other, so wrap them in a link group.
	target_link_libraries(physx_sdk_libs INTERFACE
		-Wl,--start-group ${PHYSX_LIB_FILES} -Wl,--end-group pthread dl rt)
endif()
