cmake_minimum_required(VERSION 3.15)

project(AYPhysics_Tests)

set(TEST_TARGET AYPhysics_Tests)

add_executable(${TEST_TARGET}
    main.cpp
    Test_PhysicsTypes.cpp
    Test_PhysicsHandles.cpp
    Test_PhysicsCommandQueue.cpp
    Test_PhysicsSnapshot.cpp
    Test_PhysicsManager.cpp
    Test_NullBackend3D.cpp
    Test_MockBackend3D.cpp
    Test_JoltBackend3D_Stub.cpp
    # R1.5b: real Jolt backend integration tests. Only built when Jolt is
    # present (the file is empty otherwise — it relies on Test_JoltBackend3D_Stub
    # for the no-Jolt regression path).
    Test_JoltBackend3D.cpp
    # MockBackend3D.cpp provides the testaccess:: namespace symbols; the static
    # lib drops them when no consumer in the lib references them, so we add the
    # TU to the test target directly.
    ${CMAKE_CURRENT_SOURCE_DIR}/../backend/MockBackend3D.cpp
)

target_link_libraries(${TEST_TARGET} PRIVATE
    AYTest
    AYPhysics
    AYMath
    AYTime
    AYPlatform
)

target_include_directories(${TEST_TARGET} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_CURRENT_SOURCE_DIR}/../backend
)

target_compile_definitions(${TEST_TARGET} PRIVATE
    NOMINMAX
    WIN32_LEAN_AND_MEAN
)

# Forward AYPHYSICS_HAS_JOLT / AYPHYSICS_NO_JOLT from the AYPhysics target so
# Test_JoltBackend3D_Stub.cpp's #if branches resolve to the same code path as
# the backend that got linked. Without this the test TU sees NO_JOLT (default)
# but links Jolt symbols from AYPhysics.lib, producing a runtime mismatch
# where Stub_ManagerFallsBackToNullWhenNoJolt calls init3D() that actually
# runs JPH::Factory::operator new against an uninitialised process state.
if(AYPHYSICS_BUILD_JOLT)
    target_compile_definitions(${TEST_TARGET} PRIVATE AYPHYSICS_HAS_JOLT=1)
else()
    target_compile_definitions(${TEST_TARGET} PRIVATE AYPHYSICS_NO_JOLT=1)
endif()

if(MSVC)
    target_compile_options(${TEST_TARGET} PRIVATE /W4 /wd4200)
else()
    target_compile_options(${TEST_TARGET} PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME ${TEST_TARGET} COMMAND ${TEST_TARGET})