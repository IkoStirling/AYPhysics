#include "AYPhysics/PhysicsTypes.h"

#include "AYTest.h"

using namespace ayt::physics;

TEST_SUITE(PhysicsTypesTests)

    TEST_CASE(PhysResultOkIsZero) {
        CHECK_INT_EQ(static_cast<uint8_t>(PhysResult::Ok), 0u);
    }

    TEST_CASE(PhysResultToStringCoversAll) {
        CHECK_NOT_NULL(toString(PhysResult::Ok));
        CHECK_NOT_NULL(toString(PhysResult::InvalidParam));
        CHECK_NOT_NULL(toString(PhysResult::NoMemory));
        CHECK_NOT_NULL(toString(PhysResult::AlreadyExists));
        CHECK_NOT_NULL(toString(PhysResult::NotFound));
        CHECK_NOT_NULL(toString(PhysResult::InvalidState));
        CHECK_NOT_NULL(toString(PhysResult::BackendError));
        CHECK_NOT_NULL(toString(PhysResult::OutOfRange));
        CHECK_NOT_NULL(toString(PhysResult::Unsupported));
        CHECK_NOT_NULL(toString(PhysResult::QueueFull));
    }

    TEST_CASE(InvalidHandleIsZero) {
        CHECK_INT_EQ(InvalidBodyHandle, 0u);
        CHECK_INT_EQ(InvalidColliderHandle, 0u);
        CHECK_INT_EQ(InvalidJointHandle, 0u);
    }

    TEST_CASE(LockstepStubReturnsFalse) {
        CHECK_FALSE(isLockstepActive());
    }

    TEST_CASE(DefaultDescHasSensibleDefaults) {
        PhysicsBackendDescriptor desc;
        CHECK_INT_EQ(static_cast<uint8_t>(desc.kind3D), static_cast<uint8_t>(BackendKind::DefaultJolt));
        CHECK_INT_EQ(static_cast<uint8_t>(desc.kind2D), static_cast<uint8_t>(BackendKind::Null));
        CHECK_FLOAT_EQ(desc.fixedDeltaTime, 1.0f / 60.0f, 1e-6f);
    }

TEST_SUITE_END