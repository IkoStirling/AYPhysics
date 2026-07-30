cmake_minimum_required(VERSION 3.12)
project(AYPhysics)

set(MODULE_NAME AYPhysics)

# =============================================================================
# Public sources (interface/, include/, src/)
# =============================================================================

set(SRC_FILES
    src/AYPhysicsTypes.cpp
    src/AYPhysicsCommandQueue.cpp
    src/AYPhysicsManager.cpp
    src/AYPhysicsWorld3D.cpp
    src/AYPhysicsWorld2D.cpp
    src/AYPhysicsSubSystem.cpp
    src/Physics/PhysicsScene.cpp
    src/Physics/Rigidbody.cpp
    src/Physics/Collider.cpp
    src/Physics/Joint.cpp
)

set(HEAD_FILES
    AYPhysics.h
    include/AYPhysicsHandles.h
    include/AYPhysicsTypes.h
    include/AYPhysicsCommandQueue.h
    include/AYPhysicsManager.h
    include/AYPhysicsWorld3D.h
    include/AYPhysicsWorld2D.h
    include/AYPhysicsSubSystem.h
    include/AYPhysicsBackendTestAccess.h
    interface/IPhysicsBackend.h
    interface/IPhysicsBackend3D.h
    interface/IPhysicsBackend2D.h
    interface/PhysicsScene.h
)

# =============================================================================
# Backend implementations (always: Null + Mock; R1: Jolt stub excluded unless
# Jolt found via vcpkg find_package)
# =============================================================================

set(BACKEND_FILES
    backend/NullBackend3D.cpp
    backend/NullBackend2D.cpp
    backend/MockBackend3D.cpp
)

set(BACKEND_HEADERS
    backend/NullBackend3D.h
    backend/NullBackend2D.h
    backend/MockBackend3D.h
)

# =============================================================================
# Jolt integration (R1.5 real impl; stub before R1.5b body work)
# =============================================================================
# Three-tier fallback (mirrors AYAudio/CMakeLists.txt find_path pattern with
# vcpkg-aware defaults):
#   1) find_package(Jolt CONFIG) - vcpkg port `jolt-physics` exports its CMake
#      config at share/Jolt/JoltConfig.cmake with target name `Jolt::Jolt`.
#      Use this when present.
#   2) Header-only fallback via find_path(Jolt/Jolt.h) under $VCPKG_ROOT or the
#      project's vcpkg/installed/x64-windows/include. Requires the user to
#      provide Jolt.lib on the link line (JOLT_LIB CMake variable, set by
#      find_library inside this block).
#   3) AYPHYSICS_BUILD_JOLT=OFF -> define AYPHYSICS_NO_JOLT and exclude
#      JoltBackend3D TU entirely. PhysicsManager falls back to NullBackend3D
#      when kind3D == BackendKind::DefaultJolt (R1 behaviour preserved).
#
# Note: vcpkg port name is `jolt-physics` upstream; vcpkg's CMake config is
# exported under `Jolt` and the imported target is `Jolt::Jolt`. Do NOT use
# `find_package(joltphysics ...)` or `joltphysics::joltphysics` - those names
# do not exist and the find will silently fail.

option(AYPHYSICS_BUILD_JOLT "Build JoltBackend3D (requires vcpkg Jolt port)" OFF)

# Auto-enable when vcpkg Jolt is present and the user did not explicitly disable
# the option. We probe by file existence rather than find_package so the cache
# is not polluted for the subsequent real find_package call below.
if(NOT AYPHYSICS_BUILD_JOLT)
    set(_ayphys_jolt_probe_paths
        "$ENV{VCPKG_ROOT}/installed/x64-windows/include"
        "${CMAKE_SOURCE_DIR}/vcpkg/installed/x64-windows/include"
    )
    foreach(_p IN LISTS _ayphys_jolt_probe_paths)
        if(EXISTS "${_p}/Jolt/Jolt.h")
            set(AYPHYSICS_BUILD_JOLT ON CACHE BOOL "Build JoltBackend3D (requires vcpkg Jolt port)" FORCE)
            message(STATUS "AYPhysics: auto-enabled Jolt backend (vcpkg Jolt detected at ${_p})")
            break()
        endif()
    endforeach()
endif()

set(_ayphys_jolt_found FALSE)

if(AYPHYSICS_BUILD_JOLT)
    # Tier 1: vcpkg CMake config (target = Jolt::Jolt).
    find_package(Jolt CONFIG QUIET)
    if(Jolt_FOUND)
        set(_ayphys_jolt_found TRUE)
        set(AYPHYSICS_JOLT_TARGET "Jolt::Jolt")
        set(AYPHYSICS_JOLT_INCLUDE_DIR "")
        message(STATUS "AYPhysics: Jolt backend ENABLED via find_package(Jolt)")
    else()
        # Tier 2: header-only find_path fallback (mirrors AYAudio miniaudio).
        find_path(_ayphys_jolt_header "Jolt/Jolt.h")
        if(NOT _ayphys_jolt_header)
            if(DEFINED ENV{VCPKG_ROOT})
                set(_ayphys_jolt_inc "$ENV{VCPKG_ROOT}/installed/x64-windows/include")
                if(EXISTS "${_ayphys_jolt_inc}/Jolt/Jolt.h")
                    set(_ayphys_jolt_header "${_ayphys_jolt_inc}")
                endif()
            elseif(EXISTS "${CMAKE_SOURCE_DIR}/vcpkg/installed/x64-windows/include/Jolt/Jolt.h")
                set(_ayphys_jolt_header "${CMAKE_SOURCE_DIR}/vcpkg/installed/x64-windows/include")
            endif()
        endif()
        if(_ayphys_jolt_header)
            find_library(JOLT_LIB NAMES Jolt)
            if(JOLT_LIB)
                set(_ayphys_jolt_found TRUE)
                set(AYPHYSICS_JOLT_TARGET "${JOLT_LIB}")
                set(AYPHYSICS_JOLT_INCLUDE_DIR "${_ayphys_jolt_header}")
                message(STATUS "AYPhysics: Jolt backend ENABLED via find_path fallback (${JOLT_LIB})")
            else()
                message(WARNING "AYPhysics: Jolt/Jolt.h found but Jolt.lib missing - disabling Jolt backend")
            endif()
        endif()
    endif()

    if(_ayphys_jolt_found)
        list(APPEND BACKEND_FILES     backend/JoltBackend3D.cpp)
        list(APPEND BACKEND_HEADERS   backend/JoltBackend3D.h)
    else()
        message(WARNING "AYPhysics: Jolt not found; building without Jolt backend")
        set(AYPHYSICS_BUILD_JOLT OFF)
        set(AYPHYSICS_NO_JOLT TRUE)
    endif()
endif()

# =============================================================================
# Library
# =============================================================================

add_library(${MODULE_NAME} STATIC
    ${SRC_FILES} ${HEAD_FILES} ${BACKEND_FILES} ${BACKEND_HEADERS}
)

target_include_directories(${MODULE_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/interface
)

target_include_directories(${MODULE_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/backend
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Physics
)

target_link_libraries(${MODULE_NAME} PUBLIC
    AYMath
    AYPlatform   # Thread::yield/sleep used by _physicsThreadMain idle backoff
    AYTime       # Duration / TimePoint / Clock (header-only v1.2 value-type)
)

if(AYPHYSICS_BUILD_JOLT)
    target_link_libraries(${MODULE_NAME} PRIVATE ${AYPHYSICS_JOLT_TARGET})
    if(AYPHYSICS_JOLT_INCLUDE_DIR)
        target_include_directories(${MODULE_NAME} PRIVATE ${AYPHYSICS_JOLT_INCLUDE_DIR})
    endif()
    target_compile_definitions(${MODULE_NAME} PRIVATE AYPHYSICS_HAS_JOLT=1)
else()
    # When Jolt is absent we still publish a compile-define so JoltBackend3D.cpp
    # (when present) can guard Jolt-only includes if needed. Currently R1.5a
    # always builds the TU behind AYPHYSICS_BUILD_JOLT (excluded by the BACKEND
    # list above), so this define is unused in the stub.
    target_compile_definitions(${MODULE_NAME} PRIVATE AYPHYSICS_NO_JOLT=1)
endif()

target_compile_features(${MODULE_NAME} PRIVATE cxx_std_20)

target_compile_definitions(${MODULE_NAME} PRIVATE
    NOMINMAX
    WIN32_LEAN_AND_MEAN
)

if(MSVC)
    target_compile_options(${MODULE_NAME} PRIVATE /W4)
else()
    target_compile_options(${MODULE_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()

enable_testing()
add_subdirectory(unittest)