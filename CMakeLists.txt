cmake_minimum_required(VERSION 3.12)
project(AYPhysics
    VERSION 0.4.0
    DESCRIPTION "AYPhysics — Jolt 3D + Box2D 2D + Box2DBridge 2D physics layer"
)

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
    src/AYPhysicsEventBridge.cpp
    src/AYPhysicsQueryAdapter.cpp
    src/Physics/PhysicsScene.cpp
    src/Physics/Rigidbody.cpp
    src/Physics/Collider.cpp
    src/Physics/Joint.cpp
)

set(HEAD_FILES
    AYPhysics.h
    include/AYPhysics/PhysicsHandles.h
    include/AYPhysics/PhysicsTypes.h
    include/AYPhysics/PhysicsCommandQueue.h
    include/AYPhysics/PhysicsManager.h
    include/AYPhysics/PhysicsWorld3D.h
    include/AYPhysics/PhysicsWorld2D.h
    include/AYPhysics/PhysicsSubSystem.h
    include/AYPhysics/PhysicsEventBridge.h
    include/AYPhysics/PhysicsQueryAdapter.h
    include/AYPhysics/PhysicsBackendTestAccess.h
    interface/AYPhysics/IPhysicsBackend.h
    interface/AYPhysics/IPhysicsBackend3D.h
    interface/AYPhysics/IPhysicsBackend2D.h
    interface/AYPhysics/IPhysicsQuery.h
    interface/AYPhysics/PhysicsScene.h
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
# Resolution uses find_package so vcpkg manifest mode, a custom toolchain, and
# ordinary CMAKE_PREFIX_PATH installs all work without machine-specific paths.
# AYPHYSICS_BUILD_JOLT=OFF defines AYPHYSICS_NO_JOLT and excludes
#      JoltBackend3D TU entirely. PhysicsManager falls back to NullBackend3D
#      when kind3D == BackendKind::DefaultJolt (R1 behaviour preserved).
#
# Note: vcpkg port name is `jolt-physics` upstream; vcpkg's CMake config is
# exported under `Jolt` and the imported target is `Jolt::Jolt`. Do NOT use
# `find_package(joltphysics ...)` or `joltphysics::joltphysics` - those names
# do not exist and the find will silently fail.

option(AYPHYSICS_BUILD_JOLT "Build JoltBackend3D (requires Jolt package)" ON)

set(_ayphys_jolt_found FALSE)

if(AYPHYSICS_BUILD_JOLT)
    find_package(Jolt CONFIG QUIET)
    if(Jolt_FOUND)
        set(_ayphys_jolt_found TRUE)
        set(AYPHYSICS_JOLT_TARGET "Jolt::Jolt")
        set(AYPHYSICS_JOLT_INCLUDE_DIR "")
        message(STATUS "AYPhysics: Jolt backend ENABLED via find_package(Jolt)")
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
# Box2D integration (R2.5 real 2D impl)
# =============================================================================
option(AYPHYSICS_BUILD_BOX2D "Build Box2DBackend2D (requires box2d package)" ON)

set(_ayphys_box2d_found FALSE)

if(AYPHYSICS_BUILD_BOX2D)
    find_package(box2d CONFIG QUIET)
    if(box2d_FOUND)
        set(_ayphys_box2d_found TRUE)
        set(AYPHYSICS_BOX2D_TARGET "box2d::box2d")
        message(STATUS "AYPhysics: Box2D backend ENABLED via find_package(box2d)")
    endif()

    if(_ayphys_box2d_found)
        list(APPEND BACKEND_FILES     backend/Box2DBackend2D.cpp)
        list(APPEND BACKEND_HEADERS   backend/Box2DBackend2D.h)
    else()
        message(WARNING "AYPhysics: Box2D not found; building without Box2D backend")
        set(AYPHYSICS_BUILD_BOX2D OFF)
        set(AYPHYSICS_NO_BOX2D TRUE)
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
    AYGameLoop   # E-1: PhysicsSubSystem : ISubSystem
)
# E-3: PhysicsEventBridge posts PhysicsCollisionEvent onto EventBus.
target_link_libraries(${MODULE_NAME} PRIVATE AYEventSystem)

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

if(AYPHYSICS_BUILD_BOX2D)
    target_link_libraries(${MODULE_NAME} PRIVATE ${AYPHYSICS_BOX2D_TARGET})
    if(AYPHYSICS_BOX2D_INCLUDE_DIR)
        target_include_directories(${MODULE_NAME} PRIVATE ${AYPHYSICS_BOX2D_INCLUDE_DIR})
    endif()
    target_compile_definitions(${MODULE_NAME} PRIVATE AYPHYSICS_HAS_BOX2D=1)
else()
    target_compile_definitions(${MODULE_NAME} PRIVATE AYPHYSICS_NO_BOX2D=1)
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

message(STATUS "AYPhysics v0.4.0 (Jolt 3D + Box2D 2D + R1.5c Bench_PhysicsStep §17.8 gate) — see ${CMAKE_CURRENT_SOURCE_DIR}/design.md")
