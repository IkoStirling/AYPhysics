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
# Jolt integration (R1.5 real impl; R1 only stub if port available)
# =============================================================================

option(AYPHYSICS_BUILD_JOLT "Build JoltBackend3D (requires vcpkg joltphysics)" OFF)

if(AYPHYSICS_BUILD_JOLT)
    find_package(joltphysics CONFIG QUIET)
    if(joltphysics_FOUND)
        list(APPEND BACKEND_FILES     backend/JoltBackend3D.cpp)
        list(APPEND BACKEND_HEADERS   backend/JoltBackend3D.h)
        message(STATUS "AYPhysics: Jolt backend ENABLED")
    else()
        message(WARNING "AYPhysics: joltphysics not found; building without Jolt backend")
        set(AYPHYSICS_BUILD_JOLT OFF)
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
    target_link_libraries(${MODULE_NAME} PRIVATE joltphysics::joltphysics)
    target_compile_definitions(${MODULE_NAME} PRIVATE AYPHYSICS_HAS_JOLT=1)
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