#include "AYPhysicsHandles.h"
#include "PhysicsScene.h"

#include "AYTest.h"

using namespace ayt::physics;

TEST_SUITE(PhysicsHandlesTests)

    TEST_CASE(MakeHandleRoundTripsIndexAndGeneration) {
        const BodyHandle h = makeHandle(42, 7);
        CHECK_INT_EQ(handleIndex(h), 42u);
        CHECK_INT_EQ(handleGeneration(h), 7u);
    }

    TEST_CASE(InvalidHandleIsZeroAndNotLive) {
        HandleAllocator alloc;
        CHECK(alloc.initialize(64));
        CHECK_FALSE(alloc.isLive(InvalidBodyHandle));
        CHECK_INT_EQ(alloc.liveCount(), 0u);
        alloc.shutdown();
    }

    TEST_CASE(AllocateThenFreeThenReuseFailsValidation) {
        HandleAllocator alloc;
        CHECK(alloc.initialize(64));
        BodyHandle h1 = alloc.allocate();
        CHECK(isValidHandle(h1));
        CHECK(alloc.isLive(h1));
        CHECK_INT_EQ(alloc.liveCount(), 1u);

        const uint32_t gen1 = handleGeneration(h1);
        alloc.free(h1);
        CHECK_FALSE(alloc.isLive(h1));

        // Reuse: a new handle with different generation is issued.
        BodyHandle h2 = alloc.allocate();
        CHECK(isValidHandle(h2));
        const uint32_t gen2 = handleGeneration(h2);
        CHECK(gen2 != gen1);                  // generation bumped
        CHECK(alloc.isLive(h2));
        CHECK_FALSE(alloc.isLive(h1));        // stale handle no longer live
        alloc.shutdown();
    }

    TEST_CASE(GenerationBumpWrapsAtMaxGen) {
        // Generation is 12-bit (max 4095). Bump should wrap 4095 -> 1.
        CHECK_INT_EQ(bumpGeneration(0), 1u);
        CHECK_INT_EQ(bumpGeneration(1), 2u);
        CHECK_INT_EQ(bumpGeneration(4094), 4095u);
        CHECK_INT_EQ(bumpGeneration(4095), 1u);
    }

    TEST_CASE(AllocateExhaustionReturnsInvalid) {
        HandleAllocator alloc;
        CHECK(alloc.initialize(4));
        // capacity = 4 -> indices 1..3 available (0 reserved); 3 allocs then full.
        BodyHandle h1 = alloc.allocate();
        BodyHandle h2 = alloc.allocate();
        BodyHandle h3 = alloc.allocate();
        CHECK(isValidHandle(h1));
        CHECK(isValidHandle(h2));
        CHECK(isValidHandle(h3));
        BodyHandle h4 = alloc.allocate();
        CHECK_INT_EQ(h4, InvalidBodyHandle);
        alloc.shutdown();
    }

TEST_SUITE_END