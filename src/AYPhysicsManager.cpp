#include "AYPhysicsManager.h"

#include "AYPhysicsWorld3D.h"
#include "AYPhysicsWorld2D.h"
#include "IPhysicsBackend.h"
#include "IPhysicsBackend3D.h"
#include "IPhysicsBackend2D.h"
#include "PhysicsScene.h"
#include "NullBackend3D.h"
#include "NullBackend2D.h"
#include "MockBackend3D.h"
#include "AYPhysicsBackendTestAccess.h"

#if defined(AYPHYSICS_HAS_JOLT)
#include "JoltBackend3D.h"
#endif

#include "ayplatform/Thread.h"

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
        case BackendKind::DefaultBox2D:        // R1: no Box2D yet
        case BackendKind::DefaultJolt:         // Jolt-2D is R1.5
        case BackendKind::Mock:                // R1: no Mock-2D
            return std::make_unique<NullBackend2D>();
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
    if (!mgr->_backend2D->init2D(desc)) return nullptr;
    mgr->_backend2D->setGravity(desc.gravity2D);

    // Mock pointer installed for test inspection.
    if (desc.kind3D == BackendKind::Mock) {
        // Safe downcast: createBackend3D returned a MockBackend3D when Mock requested.
        testaccess::setCurrentMock(static_cast< ::ayt::physics::MockBackend3D*>(mgr->_backend3D.get()));
    } else {
        testaccess::setCurrentMock(static_cast< ::ayt::physics::MockBackend3D*>(nullptr));
    }

    // Queues.
    mgr->_queue      = std::make_unique<PhysicsCommandQueue>();
    mgr->_createPool = std::make_unique<PhysicsCreatePool>();
    mgr->_syncMailbox = std::make_unique<SyncQueryMailbox>();
    if (!mgr->_queue->initialize(desc.commandQueueCapacity)) return nullptr;
    if (!mgr->_createPool->initialize(desc.createPoolCapacity)) return nullptr;
    if (!mgr->_syncMailbox->initialize(kSyncQueryMailboxCapacity)) return nullptr;

    // Start backend (physics thread will own; but start() is called here so we
    // can fail-fast on init).
    PhysicsBackendInfo info = mgr->_backend3D->describe();
    info.maxBodies = desc.maxBodies;
    info.maxColliders = desc.maxBodies * 2;       // heuristic; refined in R1.5
    info.maxJoints    = desc.maxBodies;
    if (!mgr->_backend3D->start(info)) return nullptr;

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
    if (!_queue->tryPush(cmd)) {
        _queueRejectCount.fetch_add(1, std::memory_order_relaxed);
        return PhysResult::QueueFull;
    }
    return PhysResult::Ok;
}

const PhysFrameSnapshot& PhysicsManager::fetchResults() {
    const uint32_t idx = _frontIndex.load(std::memory_order_acquire);
    return _snapshots[idx];
}

void PhysicsManager::publishSnapshot() {
    // Atomic swap: back becomes front, previous front becomes new back (cleared).
    const uint32_t oldFront = _frontIndex.load(std::memory_order_relaxed);
    const uint32_t newFront = oldFront ^ 1;
    _snapshots[newFront].transforms.clear();
    _snapshots[newFront].collisionEvents.clear();
    _snapshots[newFront].queryResults.clear();
    _snapshots[newFront].frameIndex = _frameIndex.fetch_add(1, std::memory_order_relaxed) + 1;
    _frontIndex.store(newFront, std::memory_order_release);
}

void PhysicsManager::shutdown() {
    if (!_running.exchange(false, std::memory_order_acq_rel)) return;
    if (_physicsThread.joinable()) _physicsThread.join();
    if (_backend3D) _backend3D->stop();
    if (_backend2D) _backend2D->stop();
    if (_queue)      _queue->shutdown();
    if (_createPool) _createPool->shutdown();
    if (_syncMailbox)_syncMailbox->shutdown();
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
        PhysicsCommand cmd{};
        while (drained < _descriptor.maxDrainPerTick &&
               _queue->tryPop(cmd)) {
            if (cmd.type == PhysicsCommandType::Step) {
                sawStep = true;
                lastDt = cmd.u.step.deltaTime;
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

        // Drain sync-query mailbox between commands and step.
        if (_syncMailbox && _backend3D) {
            _syncMailbox->serviceAll(static_cast<IPhysicsBackend*>(_backend3D.get()));
        }

        if (sawStep && _backend3D) {
            _backend3D->step(lastDt);
            PhysFrameSnapshot& back = _snapshots[_backIndex.load(std::memory_order_relaxed)];
            back.stepSeconds = lastDt;
            _backend3D->publishSnapshot(back);
            publishSnapshot();
        }

        // Idle backoff: yield the OS thread instead of microsecond sleeps.
        // Rationale: Thread::sleep(float seconds) only delivers ms granularity,
        // and a 50 us chrono sleep on Windows resolves to a 1 ms+ OS quantum
        // anyway. Yield is the canonical spin-wait primitive for SPSC ring
        // consumption; for the multi-ms level we let OS scheduling kick in.
        if (drained == 0 && !sawStep) {
            ayt::platform::Thread::yield();
        }
    }
}

} // namespace ayt::physics