#include "AYPhysics/PhysicsManager.h"

#include "AYPhysics/PhysicsWorld3D.h"
#include "AYPhysics/PhysicsWorld2D.h"
#include "AYPhysics/IPhysicsBackend.h"
#include "AYPhysics/IPhysicsBackend3D.h"
#include "AYPhysics/IPhysicsBackend2D.h"
#include "AYPhysics/PhysicsScene.h"
#include "NullBackend3D.h"
#include "NullBackend2D.h"
#include "MockBackend3D.h"
#include "AYPhysics/PhysicsBackendTestAccess.h"

#if defined(AYPHYSICS_HAS_JOLT)
#include "JoltBackend3D.h"
#endif

#if defined(AYPHYSICS_HAS_BOX2D)
#include "Box2DBackend2D.h"
#endif

#include "AYPlatform/Thread.h"

#include <mutex>

namespace ayt::physics {

namespace {
constexpr uint32_t kSyncQueryMailboxCapacity = 64;

std::unique_ptr<IPhysicsBackend3D> createBackend3D(BackendKind kind,
                                                   const PhysicsBackendDescriptor& desc,
                                                   uint32_t* outMaxBodies) {
    switch (kind) {
        case BackendKind::Null:
            return std::make_unique<NullBackend3D>();
        case BackendKind::Mock:
            return std::make_unique<MockBackend3D>();
        case BackendKind::DefaultJolt:
#if defined(AYPHYSICS_HAS_JOLT)
            // R1.5a: Jolt stub wired through the manager; R1.5b replaces the
            // body of JoltBackend3D with the real Jolt integration.
            return std::make_unique<JoltBackend3D>();
#else
            // Jolt not built (-DAVPHYSICS_BUILD_JOLT=OFF or vcpkg port missing);
            // preserve R1 behaviour by falling back to Null.
            return std::make_unique<NullBackend3D>();
#endif
        case BackendKind::DefaultBox2D:
            // 3D path; Box2D is 2D-only.
            return std::make_unique<NullBackend3D>();
    }
    *outMaxBodies = desc.maxBodies;
    return nullptr;
}

std::unique_ptr<IPhysicsBackend2D> createBackend2D(BackendKind kind) {
    switch (kind) {
        case BackendKind::Null:
        case BackendKind::Mock:
        case BackendKind::DefaultJolt:
            return std::make_unique<NullBackend2D>();
        case BackendKind::DefaultBox2D:
#if defined(AYPHYSICS_HAS_BOX2D)
            return std::make_unique<Box2DBackend2D>();
#else
            return std::make_unique<NullBackend2D>();
#endif
    }
    return nullptr;
}
}  // namespace

PhysicsManager::~PhysicsManager() { shutdown(); }

std::unique_ptr<PhysicsManager> PhysicsManager::create(const PhysicsBackendDescriptor& desc) {
    auto mgr = std::unique_ptr<PhysicsManager>(new PhysicsManager());
    mgr->_descriptor = desc;

    // Backend selection.
    {
        uint32_t maxBodies = desc.maxBodies;
        mgr->_backend3D = createBackend3D(desc.kind3D, desc, &maxBodies);
        if (!mgr->_backend3D) return nullptr;
        if (!mgr->_backend3D->init3D(desc)) return nullptr;
        mgr->_backend3D->setGravity(desc.gravity3D);
    }
    mgr->_backend2D = createBackend2D(desc.kind2D);
    if (!mgr->_backend2D) return nullptr;
    if (!mgr->_backend2D->init2D(desc)) {
        // F-K — 3D is live at this point; tear it down before bailing so the
        // caller doesn't get a half-initialised manager leaking the 3D backend
        // (and any Jolt PhysicsSystem / Box2D world owned by it).
        mgr->_backend3D->stop();
        mgr->_backend3D.reset();
        return nullptr;
    }
    mgr->_backend2D->setGravity(desc.gravity2D);

    // Mock pointer installed for test inspection.
    if (desc.kind3D == BackendKind::Mock) {
        // Safe downcast: createBackend3D returned a MockBackend3D when Mock requested.
        testaccess::setCurrentMock(static_cast< ::ayt::physics::MockBackend3D*>(mgr->_backend3D.get()));
    } else {
        testaccess::setCurrentMock(static_cast< ::ayt::physics::MockBackend3D*>(nullptr));
    }

    // Queues (3D + 2D use independent SPSC rings; Step lives on the 3D queue).
    mgr->_queue       = std::make_unique<PhysicsCommandQueue>();
    mgr->_queue2D     = std::make_unique<PhysicsCommandQueue>();
    mgr->_createPool  = std::make_unique<PhysicsCreatePool>();
    mgr->_syncMailbox  = std::make_unique<SyncQueryMailbox>();
    mgr->_syncMailbox2D = std::make_unique<SyncQueryMailbox>();
    if (!mgr->_queue->initialize(desc.commandQueueCapacity)) return nullptr;
    if (!mgr->_queue2D->initialize(desc.commandQueueCapacity)) return nullptr;
    if (!mgr->_createPool->initialize(desc.createPoolCapacity)) return nullptr;
    if (!mgr->_syncMailbox->initialize(kSyncQueryMailboxCapacity)) return nullptr;
    if (!mgr->_syncMailbox2D->initialize(kSyncQueryMailboxCapacity)) return nullptr;

    // Start backends (physics thread will own; start here for fail-fast).
    PhysicsBackendInfo info = mgr->_backend3D->describe();
    info.maxBodies = desc.maxBodies;
    info.maxColliders = desc.maxBodies * 2;
    info.maxJoints    = desc.maxBodies;
    if (!mgr->_backend3D->start(info)) {
        // F-K — 2D was init'd above (b2World / Jolt system live); tear it down
        // before returning so the caller doesn't get a half-initialised manager.
        mgr->_backend2D->stop();
        mgr->_backend2D.reset();
        mgr->_backend3D.reset();
        return nullptr;
    }

    PhysicsBackendInfo info2D = mgr->_backend2D->describe();
    info2D.maxBodies = desc.maxBodies;
    // F-P1: Box2DBackend2D shares a single index pool across bodies, colliders,
    // and joints (every per-collider / per-joint slot is sized by maxBodies, see
    // Box2DBackend2D::init2D). Telling the backend maxColliders = 2*maxBodies
    // would silently reject the upper half as PhysResult::NotFound. Clamp to the
    // actual capacity the backend can honour until §6.4 separates the pools.
    info2D.maxColliders = desc.maxBodies;
    info2D.maxJoints    = desc.maxBodies;
    if (!mgr->_backend2D->start(info2D)) {
        // F-K — symmetric rollback: 3D was started above, must stop before we
        // bail so the PhysicsSystem / Box2D world it owns is shut down cleanly.
        mgr->_backend3D->stop();
        mgr->_backend3D.reset();
        mgr->_backend2D.reset();
        return nullptr;
    }

    // World3D + World2D get manager back-pointers (lifetime owned by manager).
    mgr->_world3D = std::unique_ptr<PhysicsWorld3D>(new PhysicsWorld3D());
    mgr->_world2D = std::unique_ptr<PhysicsWorld2D>(new PhysicsWorld2D());
    mgr->_world3D->_manager = mgr.get();
    mgr->_world2D->_manager = mgr.get();

    // Spawn physics thread.
    mgr->_running.store(true, std::memory_order_release);
    mgr->_physicsThread = std::thread(&PhysicsManager::_physicsThreadMain, mgr.get());

    return mgr;
}

PhysResult PhysicsManager::step(float deltaTime) {
    if (!_running.load(std::memory_order_acquire)) return PhysResult::InvalidState;
    PhysicsCommand cmd{};
    cmd.type = PhysicsCommandType::Step;
    cmd.u.step.deltaTime = deltaTime;
    cmd.u.step.completionSequence = 0;
    if (!_queue->tryPush(cmd)) {
        _queueRejectCount.fetch_add(1, std::memory_order_relaxed);
        return PhysResult::QueueFull;
    }
    return PhysResult::Ok;
}

PhysResult PhysicsManager::stepAndWait(float deltaTime) {
    return stepAndWait(deltaTime, _descriptor.fixedStepTimeout);
}

PhysResult PhysicsManager::stepAndWait(float deltaTime,
                                       ayt::time::Duration timeout) {
    if (!_running.load(std::memory_order_acquire)) return PhysResult::InvalidState;

    const uint64_t sequence =
        _nextStepCompletionSequence.fetch_add(1, std::memory_order_relaxed);
    PhysicsCommand cmd{};
    cmd.type = PhysicsCommandType::Step;
    cmd.u.step.deltaTime = deltaTime;
    cmd.u.step.completionSequence = sequence;
    if (!_queue->tryPush(cmd)) {
        _queueRejectCount.fetch_add(1, std::memory_order_relaxed);
        return PhysResult::QueueFull;
    }

    std::unique_lock<ayt::platform::Mutex> lock(_stepCompletionMutex);
    const bool completed = _stepCompletionCv.waitFor(
        lock,
        timeout.toChronoMicroseconds(),
        [this, sequence]() {
            return _completedStepSequence.load(std::memory_order_acquire) >= sequence
                || !_running.load(std::memory_order_acquire);
        });

    if (!completed) return PhysResult::BackendError;
    return _completedStepSequence.load(std::memory_order_acquire) >= sequence
        ? PhysResult::Ok
        : PhysResult::InvalidState;
}

const PhysFrameSnapshot& PhysicsManager::fetchResults() {
    const uint32_t idx = _frontIndex.load(std::memory_order_acquire);
    return _snapshots[idx];
}

void PhysicsManager::publishSnapshot() {
    // Atomic swap: the physics thread already wrote into the back buffer; publish
    // it as front and recycle the old front as the next back (cleared).
    const uint32_t oldFront = _frontIndex.load(std::memory_order_relaxed);
    const uint32_t newFront = oldFront ^ 1u;
    _snapshots[newFront].frameIndex =
        _frameIndex.fetch_add(1, std::memory_order_relaxed) + 1;
    _frontIndex.store(newFront, std::memory_order_release);
    _backIndex.store(oldFront, std::memory_order_release);
    _snapshots[oldFront].transforms.clear();
    _snapshots[oldFront].collisionEvents.clear();
    _snapshots[oldFront].queryResults.clear();
}

void PhysicsManager::shutdown() {
    if (!_running.exchange(false, std::memory_order_acq_rel)) return;
    _stepCompletionCv.notifyAll();
    if (_physicsThread.joinable()) _physicsThread.join();
    if (_backend3D) _backend3D->stop();
    if (_backend2D) _backend2D->stop();
    if (_queue)       _queue->shutdown();
    if (_queue2D)     _queue2D->shutdown();
    if (_createPool)  _createPool->shutdown();
    if (_syncMailbox) _syncMailbox->shutdown();
    if (_syncMailbox2D) _syncMailbox2D->shutdown();
    testaccess::setCurrentMock(nullptr);
}

void PhysicsManager::_physicsThreadMain() {
    // Snapshot publish bookkeeping for back buffer.
    float lastDt = 1.0f / 60.0f;

    while (_running.load(std::memory_order_acquire)) {
        // Drain commands up to budget. We treat Step as a "synchronisation"
        // boundary: drain everything queued up to the Step, then call backend step().
        // For R1 we simply drain up to maxDrainPerTick then step.
        uint32_t drained = 0;
        bool sawStep = false;
        uint64_t completionSequence = 0;
        PhysicsCommand cmd{};
        while (drained < _descriptor.maxDrainPerTick &&
               _queue->tryPop(cmd)) {
            if (cmd.type == PhysicsCommandType::Step) {
                sawStep = true;
                lastDt = cmd.u.step.deltaTime;
                completionSequence = cmd.u.step.completionSequence;
                break;
            }
            // Non-step command: take create payload if relevant, then execute.
            PhysicsCreatePayload payload{};
            const PhysicsCreatePayload* pPayload = nullptr;
            if (cmd.type == PhysicsCommandType::CreateRigidbody ||
                cmd.type == PhysicsCommandType::CreateCollider ||
                cmd.type == PhysicsCommandType::CreateJoint) {
                if (cmd.createSlot != InvalidCreateSlotId &&
                    _createPool->take(cmd.createSlot, payload)) {
                    pPayload = &payload;
                }
            }
            if (_backend3D) _backend3D->execute(cmd, pPayload);
            ++drained;
        }

        uint32_t drained2D = 0;
        while (drained2D < _descriptor.maxDrainPerTick &&
               _queue2D->tryPop(cmd)) {
            PhysicsCreatePayload payload{};
            const PhysicsCreatePayload* pPayload = nullptr;
            if (cmd.type == PhysicsCommandType::CreateRigidbody ||
                cmd.type == PhysicsCommandType::CreateCollider ||
                cmd.type == PhysicsCommandType::CreateJoint) {
                if (cmd.createSlot != InvalidCreateSlotId &&
                    _createPool->take(cmd.createSlot, payload)) {
                    pPayload = &payload;
                }
            }
            if (_backend2D) _backend2D->execute(cmd, pPayload);
            ++drained2D;
        }

        if (_syncMailbox && _backend3D) {
            _syncMailbox->serviceAll(static_cast<IPhysicsBackend*>(_backend3D.get()));
        }
        if (_syncMailbox2D && _backend2D) {
            _syncMailbox2D->serviceAll(static_cast<IPhysicsBackend*>(_backend2D.get()));
        }

        if (sawStep) {
            if (_backend3D) _backend3D->step(lastDt);
            if (_backend2D) _backend2D->step(lastDt);
            PhysFrameSnapshot& back = _snapshots[_backIndex.load(std::memory_order_relaxed)];
            back.stepSeconds = lastDt;
            if (_backend3D) _backend3D->publishSnapshot(back);
            if (_backend2D) _backend2D->publishSnapshot(back);
            publishSnapshot();
            if (completionSequence != 0) {
                _completedStepSequence.store(completionSequence,
                                             std::memory_order_release);
                _stepCompletionCv.notifyAll();
            }
        }

        if (drained == 0 && drained2D == 0 && !sawStep) {
            ayt::platform::Thread::yield();
        }
    }
}

} // namespace ayt::physics
