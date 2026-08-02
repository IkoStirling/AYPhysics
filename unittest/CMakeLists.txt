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
    # present (the file is empty otherwise โ€?it relies on Test_JoltBackend3D_Stub
    # for the no-Jolt regression path).
    Test_JoltBackend3D.cpp
    # R2.0a: advanced collider shapes (ConvexHull / TriangleMesh / Heightfield).
    # 10 Jolt-gated cases + 1 Mock-only case (round-trip shared_ptr). Always
    # compiled; Mock case runs in Null mode (proves shared_ptr survives
    # create-pool without Jolt linkage).
    Test_JoltBackend3D_AdvancedShapes.cpp
    Test_Box2DBackend2D.cpp
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

if(AYPHYSICS_BUILD_BOX2D)
    target_compile_definitions(${TEST_TARGET} PRIVATE AYPHYSICS_HAS_BOX2D=1)
else()
    target_compile_definitions(${TEST_TARGET} PRIVATE AYPHYSICS_NO_BOX2D=1)
endif()

if(MSVC)
    target_compile_options(${TEST_TARGET} PRIVATE /W4 /wd4200)
else()
    target_compile_options(${TEST_TARGET} PRIVATE -Wall -Wextra -Wpedantic)
endif()

ay_configure_test_tmpdir(${TEST_TARGET})
add_test(NAME ${TEST_TARGET} COMMAND ${TEST_TARGET})

# R1.5c: standalone bench executable (NOT inside AYPhysics_Tests). Built only
# when Jolt is available (the bench measures real Jolt timing โ€?see design.md
# ยง17.2 / ยง17.8 item 8). Numbers from this binary are recorded in
# docs/perf_R15.md before the ยง17.8 gate can be ticked.
if(AYPHYSICS_BUILD_JOLT)
    set(BENCH_TARGET Bench_PhysicsStep)

    add_executable(${BENCH_TARGET}
        Bench_PhysicsStep.cpp
    )

    target_link_libraries(${BENCH_TARGET} PRIVATE
        AYPhysics
        AYMath
        AYTime
        AYPlatform
    )

    target_include_directories(${BENCH_TARGET} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/..
        ${CMAKE_CURRENT_SOURCE_DIR}/../backend
    )

    target_compile_definitions(${BENCH_TARGET} PRIVATE
        NOMINMAX
        WIN32_LEAN_AND_MEAN
        AYPHYSICS_HAS_JOLT=1
    )

    if(MSVC)
        target_compile_options(${BENCH_TARGET} PRIVATE /W4 /wd4200)
    else()
        target_compile_options(${BENCH_TARGET} PRIVATE -Wall -Wextra -Wpedantic)
    endif()

    # Not registered with add_test() โ€?bench numbers belong in perf_R15.md,
    # not in CI test results. Run manually with d:/tmp/build_ayphysics_jolt.bat.
endif()