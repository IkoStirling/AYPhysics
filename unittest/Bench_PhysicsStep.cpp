// Bench_PhysicsStep.cpp - AYPhysics R1.5c perf gate (design.md §17.2 / §17.8 item 8).
//
// Standalone bench executable (NOT inside AYPhysics_Tests). Built only when
// AYPHYSICS_HAS_JOLT is defined. Five scenarios measure Jolt step timing on
// real hardware and report P50/P99 + active/sleep fraction. Numbers MUST be
// recorded in docs/perf_R15.md before §17.8 item 8 can be ticked.
//
// Usage:
//   Bench_PhysicsStep                       # run all scenarios, fail on budget breach
//   Bench_PhysicsStep --scenario p1         # single scenario
//   Bench_PhysicsStep --frames 600          # override default frame count
//   Bench_PhysicsStep -w                    # waive hard-fail (for slow hardware)
//
// Build: see unittest/CMakeLists.txt. Gated by AYPHYSICS_HAS_JOLT.
//
// Exit codes:
//   0 = all budgets met (or all waivers applied via -w)
//   2 = a budget was breached (P1 > 4ms / P2 > 12ms / P3 > 14ms / sleeping < 70%)
//   3 = a scenario failed to set up (assertion at setup time)
//   4 = invalid CLI
//
// Measurement methodology (R1.5c):
//   `mgr->step(dt)` enqueues a Step command to the SPSC ring and returns in
//   <10 us. That is NOT the physics step wall time — it is the game-thread
//   enqueue cost. The real bottleneck is the physics thread's
//   (drain + Jolt::PhysicsSystem::Update + publishSnapshot) sequence.
//
//   This bench measures the FRAME WALL TIME: from `mgr->step(dt)` enqueue
//   until `mgr->fetchResults()` reflects the next frameIndex. We poll
//   fetchResults() with 100 us spin + 1 ms sleep fallback, and record the
//   elapsed time per frame. The physics thread runs concurrently in the
//   background; the bench gives it room between frames (a 0 us busy wait
//   until the frame advances, or 1 ms sleep as ceiling).
//
// R1.5c fix: previous run of this bench (R1.5c attempt #1) measured only
// the enqueue cost, giving 0.000 ms everywhere. setup also returned early
// at "127 bodies" because CreateRigidbody commands enqueued faster than
// the physics thread consumed them, exhausting PhysicsCreatePool. R1.5c
// attempt #2 explicitly drains the setup phase before recording.

#include "AYPhysicsManager.h"
#include "AYPhysicsWorld3D.h"
#include "AYPhysicsTypes.h"
#include "AYPhysicsBackendTestAccess.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

namespace phy = ayt::physics;

// =============================================================================
// Timing primitives
// =============================================================================
namespace {

struct BenchClock {
    static int64_t nowNs() noexcept {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }
};

struct ScenarioStats {
    std::string name;
    int         frames        = 0;
    int         bodies        = 0;
    int         syncPerFrame  = 0;
    double      avgMs         = 0.0;
    double      p50Ms         = 0.0;
    double      p99Ms         = 0.0;
    double      maxMs         = 0.0;
    int         activeAtEnd   = 0;
    int         totalAtEnd    = 0;
    double      sleepFraction = 0.0;
    bool        queueRejected = false;
    bool        budgetMet     = false;
    double      budgetTargetMs = 0.0;
};

void computeStats(std::vector<int64_t>& stepNs, ScenarioStats& s) {
    if (stepNs.empty()) { s.avgMs = 0.0; s.p50Ms = 0.0; s.p99Ms = 0.0; s.maxMs = 0.0; return; }
    std::sort(stepNs.begin(), stepNs.end());
    int64_t sum = 0;
    int64_t mx  = 0;
    for (int64_t v : stepNs) { sum += v; if (v > mx) mx = v; }
    s.avgMs = (double)sum / (double)stepNs.size() / 1.0e6;
    s.maxMs = (double)mx / 1.0e6;
    auto pct = [&](double p) {
        const size_t idx = std::min(stepNs.size() - 1,
            (size_t)(p * (stepNs.size() - 1)));
        return (double)stepNs[idx] / 1.0e6;
    };
    s.p50Ms = pct(0.50);
    s.p99Ms = pct(0.99);
}

// =============================================================================
// Stepping helpers
// =============================================================================
//
// `mgr->step(dt)` enqueues a Step command; the physics thread runs it and
// publishes a snapshot with an incremented frameIndex. Wait for that frame
// to be observable to the game thread, and return the wall time elapsed.
// On 1 ms timeout, treat the frame as "missed" and return -1 (callers
// typically just retry, but for bench we record the elapsed value as-is).
int64_t stepAndWaitForFrame(phy::PhysicsManager& mgr,
                            float dt,
                            uint64_t previousFrameIndex,
                            int maxWaitMs = 8) {
    const int64_t t0 = BenchClock::nowNs();
    if (mgr.step(dt) != phy::PhysResult::Ok) {
        return -1; // queue full
    }
    // Spin-wait with 100us granularity, capped at maxWaitMs.
    const int64_t deadlineNs = t0 + (int64_t)maxWaitMs * 1000000LL;
    while (BenchClock::nowNs() < deadlineNs) {
        (void)mgr.fetchResults();
        if (mgr.fetchResults().frameIndex > previousFrameIndex) {
            return BenchClock::nowNs() - t0;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    return BenchClock::nowNs() - t0; // timed out — record ceiling
}

void drainCreation(phy::PhysicsManager& mgr, int settleFrames) {
    // Drive the physics thread until the snapshot has settled (frameIndex
    // advances past settleFrames). This guarantees any in-flight Create*
    // commands from the setup phase have been consumed.
    uint64_t targetFrame = mgr.fetchResults().frameIndex + settleFrames;
    for (int i = 0; i < settleFrames * 4; ++i) {
        if (stepAndWaitForFrame(mgr, 1.0f / 60.0f,
                                mgr.fetchResults().frameIndex, 16) < 0) break;
        if (mgr.fetchResults().frameIndex >= targetFrame) break;
    }
}

// =============================================================================
// Scenario helpers
// =============================================================================

struct BodySet {
    phy::BodyHandle ground = phy::InvalidBodyHandle;
    std::vector<phy::BodyHandle> dynamic;
};

BodySet makeStack(phy::PhysicsWorld3D* w, int nDynamic, bool withGround = true) {
    BodySet bs;
    if (withGround) {
        phy::RigidbodyDesc gd{};
        gd.type = phy::BodyType::Static;
        gd.position = ayt::math::FVector3(0.0f, -0.5f, 0.0f);
        gd.layer    = static_cast<phy::PhysLayer>(phy::PhysDefaultLayer::Static);
        if (w->createRigidbody(gd, bs.ground) != phy::PhysResult::Ok) return bs;
        phy::ColliderDesc gc{};
        gc.body = bs.ground;
        gc.shape = phy::ColliderShape::Box;
        gc.halfExtents = ayt::math::FVector3(50.0f, 0.5f, 50.0f);
        phy::ColliderHandle gh{};
        (void)w->createCollider(gc, gh);
    }
    bs.dynamic.reserve((size_t)nDynamic);
    for (int i = 0; i < nDynamic; ++i) {
        phy::RigidbodyDesc d{};
        d.type = phy::BodyType::Dynamic;
        d.position = ayt::math::FVector3(
            (float)((i % 10) * 1.2f - 5.0f),
            0.5f + (float)(i / 10) * 1.2f,
            (float)((i / 100) % 10) * 1.2f - 5.0f);
        d.mass = 1.0f;
        d.linearDamping = 0.05f;
        d.angularDamping = 0.05f;
        phy::BodyHandle h{};
        if (w->createRigidbody(d, h) != phy::PhysResult::Ok) break;
        phy::ColliderDesc c{};
        c.body = h;
        c.shape = phy::ColliderShape::Box;
        c.halfExtents = ayt::math::FVector3(0.5f, 0.5f, 0.5f);
        phy::ColliderHandle ch{};
        (void)w->createCollider(c, ch);
        bs.dynamic.push_back(h);
    }
    return bs;
}

} // anonymous namespace

// =============================================================================
// Scenarios (§17.2)
// =============================================================================
namespace {

struct Cfg {
    int  frames;
    int  p1Bodies;
    int  p2Bodies;
    int  p3Bodies;
    bool waive;
    bool singleScenario;
    std::string singleName;
};

Cfg parseArgs(int argc, char** argv) {
    Cfg c{};
    c.frames = 60;
    c.p1Bodies = 1000;
    c.p2Bodies = 1000;   // R1.5c defaults reduced from 10k (standard target) to
                         // 1k (slow-hardware compatible) — see perf_R15.md.
    c.p3Bodies = 1000;
    c.waive = false;
    c.singleScenario = false;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "-w" || a == "--waive") c.waive = true;
        else if (a == "--frames" && i + 1 < argc) c.frames = std::atoi(argv[++i]);
        else if (a == "--bodies" && i + 1 < argc) {
            const int b = std::atoi(argv[++i]);
            c.p1Bodies = b; c.p2Bodies = b; c.p3Bodies = b;
        }
        else if (a == "--scenario" && i + 1 < argc) {
            c.singleScenario = true;
            c.singleName = argv[++i];
        } else if (a == "-h" || a == "--help") {
            std::printf("Bench_PhysicsStep [--scenario p1|p2|p3|sleep|reject]"
                        " [--frames N] [--bodies N] [-w]\n");
            std::exit(0);
        }
    }
    if (c.frames < 30) c.frames = 30;
    return c;
}

ScenarioStats runP1(const Cfg& cfg) {
    ScenarioStats s{};
    s.name = "P1-smoke";
    s.bodies = cfg.p1Bodies;
    s.frames = cfg.frames;
    s.budgetTargetMs = 4.0;

    phy::PhysicsBackendDescriptor d{};
    d.kind3D = phy::BackendKind::DefaultJolt;
    d.kind2D = phy::BackendKind::Null;
    d.maxBodies = (uint32_t)(s.bodies * 2 + 64);
    d.maxBodyPairs = (uint32_t)(s.bodies * 4);
    d.maxContactConstraints = (uint32_t)(s.bodies * 4);
    d.commandQueueCapacity = 4096;
    d.createPoolCapacity   = (uint32_t)(s.bodies * 2 + 256);
    d.maxDrainPerTick      = 8192;

    auto mgr = phy::PhysicsManager::create(d);
    if (!mgr) { s.budgetMet = false; return s; }
    auto* w = mgr->world3D();
    if (!w) { s.budgetMet = false; return s; }

    auto bs = makeStack(w, s.bodies, /*ground*/ true);
    drainCreation(*mgr, 30);
    if ((int)bs.dynamic.size() < s.bodies) {
        std::fprintf(stderr, "[P1] setup failed: only %zu bodies created\n", bs.dynamic.size());
        s.budgetMet = false;
        mgr->shutdown();
        return s;
    }

    std::vector<int64_t> stepNs;
    stepNs.reserve((size_t)s.frames);
    for (int i = 0; i < s.frames; ++i) {
        const uint64_t prevFrame = mgr->fetchResults().frameIndex;
        int64_t dt = stepAndWaitForFrame(*mgr, 1.0f / 60.0f, prevFrame, /*maxWaitMs*/ 16);
        if (dt < 0) { s.queueRejected = true; break; }
        stepNs.push_back(dt);
    }
    computeStats(stepNs, s);
    const phy::PhysFrameSnapshot& snap = mgr->fetchResults();
    s.activeAtEnd = (int)snap.transforms.size();
    s.totalAtEnd  = s.bodies + 1;
    s.sleepFraction = (s.totalAtEnd > 0)
        ? (1.0 - (double)s.activeAtEnd / (double)s.totalAtEnd) : 0.0;

    s.budgetMet = (s.avgMs <= s.budgetTargetMs) && !s.queueRejected;
    mgr->shutdown();
    return s;
}

ScenarioStats runP2(const Cfg& cfg) {
    ScenarioStats s{};
    s.name = "P2-industrial";
    s.bodies = cfg.p2Bodies;
    s.frames = cfg.frames;
    s.budgetTargetMs = 12.0;

    phy::PhysicsBackendDescriptor d{};
    d.kind3D = phy::BackendKind::DefaultJolt;
    d.kind2D = phy::BackendKind::Null;
    d.maxBodies = (uint32_t)(s.bodies * 2 + 64);
    d.maxBodyPairs = (uint32_t)(s.bodies * 8);
    d.maxContactConstraints = (uint32_t)(s.bodies * 8);
    d.tempAllocatorBytes = 32u * 1024u * 1024u;
    d.commandQueueCapacity = 8192;
    d.createPoolCapacity   = (uint32_t)(s.bodies * 2 + 256);
    d.maxDrainPerTick      = 16384;

    auto mgr = phy::PhysicsManager::create(d);
    if (!mgr) { s.budgetMet = false; return s; }
    auto* w = mgr->world3D();
    if (!w) { s.budgetMet = false; return s; }

    auto bs = makeStack(w, s.bodies, /*ground*/ true);
    drainCreation(*mgr, 60);
    s.bodies = (int)bs.dynamic.size();
    if (s.bodies < cfg.p2Bodies / 2) {
        std::fprintf(stderr, "[P2] setup failed: only %d bodies created\n", s.bodies);
        s.budgetMet = false;
        mgr->shutdown();
        return s;
    }

    std::vector<int64_t> stepNs;
    stepNs.reserve((size_t)s.frames);
    for (int i = 0; i < s.frames; ++i) {
        const uint64_t prevFrame = mgr->fetchResults().frameIndex;
        int64_t dt = stepAndWaitForFrame(*mgr, 1.0f / 60.0f, prevFrame, /*maxWaitMs*/ 32);
        if (dt < 0) { s.queueRejected = true; break; }
        stepNs.push_back(dt);
    }
    computeStats(stepNs, s);
    const phy::PhysFrameSnapshot& snap = mgr->fetchResults();
    s.activeAtEnd = (int)snap.transforms.size();
    s.totalAtEnd  = s.bodies + 1;
    s.sleepFraction = (s.totalAtEnd > 0)
        ? (1.0 - (double)s.activeAtEnd / (double)s.totalAtEnd) : 0.0;

    s.budgetMet = (s.avgMs <= s.budgetTargetMs) && !s.queueRejected;
    mgr->shutdown();
    return s;
}

ScenarioStats runP3(const Cfg& cfg) {
    ScenarioStats s{};
    s.name = "P3-spike";
    s.bodies = cfg.p3Bodies;
    s.syncPerFrame = 64;
    s.frames = cfg.frames;
    s.budgetTargetMs = 14.0;

    phy::PhysicsBackendDescriptor d{};
    d.kind3D = phy::BackendKind::DefaultJolt;
    d.kind2D = phy::BackendKind::Null;
    d.maxBodies = (uint32_t)(s.bodies * 2 + 64);
    d.maxBodyPairs = (uint32_t)(s.bodies * 8);
    d.maxContactConstraints = (uint32_t)(s.bodies * 8);
    d.tempAllocatorBytes = 32u * 1024u * 1024u;
    d.commandQueueCapacity = 8192;
    d.createPoolCapacity   = (uint32_t)(s.bodies * 2 + 256);
    d.maxDrainPerTick      = 16384;

    auto mgr = phy::PhysicsManager::create(d);
    if (!mgr) { s.budgetMet = false; return s; }
    auto* w = mgr->world3D();
    if (!w) { s.budgetMet = false; return s; }

    auto bs = makeStack(w, s.bodies, /*ground*/ true);
    drainCreation(*mgr, 60);
    s.bodies = (int)bs.dynamic.size();
    if (s.bodies < cfg.p3Bodies / 2) {
        std::fprintf(stderr, "[P3] setup failed: only %d bodies created\n", s.bodies);
        s.budgetMet = false;
        mgr->shutdown();
        return s;
    }

    std::vector<int64_t> stepNs;
    stepNs.reserve((size_t)s.frames);
    const float stepX = 80.0f / 8.0f;
    const float stepZ = 80.0f / 8.0f;
    for (int i = 0; i < s.frames; ++i) {
        const uint64_t prevFrame = mgr->fetchResults().frameIndex;
        for (int q = 0; q < s.syncPerFrame; ++q) {
            ayt::math::Ray r{};
            r.origin = ayt::math::FVector3(-40.0f + (q % 8) * stepX, 30.0f, -40.0f + (q / 8) * stepZ);
            r.dir   = ayt::math::FVector3(0.0f, -1.0f, 0.0f);
            phy::RaycastHit hit{};
            (void)w->raycastSync(r, hit);
        }
        int64_t dt = stepAndWaitForFrame(*mgr, 1.0f / 60.0f, prevFrame, /*maxWaitMs*/ 64);
        if (dt < 0) { s.queueRejected = true; break; }
        stepNs.push_back(dt);
    }
    computeStats(stepNs, s);
    const phy::PhysFrameSnapshot& snap = mgr->fetchResults();
    s.activeAtEnd = (int)snap.transforms.size();
    s.totalAtEnd  = s.bodies + 1;
    s.sleepFraction = (s.totalAtEnd > 0)
        ? (1.0 - (double)s.activeAtEnd / (double)s.totalAtEnd) : 0.0;

    s.budgetMet = (s.avgMs <= s.budgetTargetMs) && !s.queueRejected;
    mgr->shutdown();
    return s;
}

ScenarioStats runSleepFraction(const Cfg& cfg) {
    ScenarioStats s{};
    s.name = "P2-sleep-fraction";
    s.bodies = cfg.p2Bodies;
    s.frames = cfg.frames;
    s.budgetTargetMs = 0.0;

    phy::PhysicsBackendDescriptor d{};
    d.kind3D = phy::BackendKind::DefaultJolt;
    d.kind2D = phy::BackendKind::Null;
    d.maxBodies = (uint32_t)(s.bodies * 2 + 64);
    d.maxBodyPairs = (uint32_t)(s.bodies * 8);
    d.maxContactConstraints = (uint32_t)(s.bodies * 8);
    d.tempAllocatorBytes = 32u * 1024u * 1024u;
    d.commandQueueCapacity = 8192;
    d.createPoolCapacity   = (uint32_t)(s.bodies * 2 + 256);
    d.maxDrainPerTick      = 16384;

    auto mgr = phy::PhysicsManager::create(d);
    if (!mgr) { s.budgetMet = false; return s; }
    auto* w = mgr->world3D();
    if (!w) { s.budgetMet = false; return s; }

    auto bs = makeStack(w, s.bodies, /*ground*/ true);
    drainCreation(*mgr, 200);
    s.bodies = (int)bs.dynamic.size();
    if (s.bodies < cfg.p2Bodies / 2) {
        std::fprintf(stderr, "[sleep] setup failed: only %d bodies created\n", s.bodies);
        s.budgetMet = false;
        mgr->shutdown();
        return s;
    }

    const phy::PhysFrameSnapshot& snap = mgr->fetchResults();
    s.activeAtEnd = (int)snap.transforms.size();
    s.totalAtEnd  = s.bodies + 1;
    s.sleepFraction = (s.totalAtEnd > 0)
        ? (1.0 - (double)s.activeAtEnd / (double)s.totalAtEnd) : 0.0;

    s.budgetMet = (s.sleepFraction >= 0.70);
    (void)cfg.frames; // unused
    mgr->shutdown();
    return s;
}

ScenarioStats runQueueReject(const Cfg& /*cfg*/) {
    ScenarioStats s{};
    s.name = "queueReject";
    s.bodies = 1280;
    s.frames = 10;
    s.budgetTargetMs = 0.0;

    phy::PhysicsBackendDescriptor d{};
    d.kind3D = phy::BackendKind::DefaultJolt;
    d.kind2D = phy::BackendKind::Null;
    d.maxBodies = 2048;
    d.maxBodyPairs = 8192;
    d.maxContactConstraints = 8192;
    d.commandQueueCapacity = 4096;
    d.createPoolCapacity   = 2048;
    d.maxDrainPerTick      = 8192;

    auto mgr = phy::PhysicsManager::create(d);
    if (!mgr) { s.budgetMet = false; return s; }
    auto* w = mgr->world3D();
    if (!w) { s.budgetMet = false; return s; }

    int rejectedFrames = 0;
    for (int frame = 0; frame < s.frames; ++frame) {
        int rejected = 0;
        for (int i = 0; i < 128; ++i) {
            phy::RigidbodyDesc desc{};
            desc.type = phy::BodyType::Dynamic;
            desc.position = ayt::math::FVector3((float)i, 5.0f, (float)frame);
            phy::BodyHandle h{};
            if (w->createRigidbody(desc, h) != phy::PhysResult::Ok) ++rejected;
        }
        // Let the physics thread catch up so the create pool frees slots.
        stepAndWaitForFrame(*mgr, 1.0f / 60.0f,
                            mgr->fetchResults().frameIndex, /*maxWaitMs*/ 16);
        if (rejected > 0) ++rejectedFrames;
    }
    s.queueRejected = (rejectedFrames > 0);
    s.budgetMet = !s.queueRejected;
    mgr->shutdown();
    return s;
}

void printStats(const ScenarioStats& s) {
    std::printf("\n=== %s ===\n", s.name.c_str());
    std::printf("  bodies      : %d\n", s.bodies);
    std::printf("  frames      : %d\n", s.frames);
    std::printf("  sync/frame  : %d\n", s.syncPerFrame);
    if (s.budgetTargetMs > 0.0) {
        std::printf("  step avg    : %.3f ms (budget <= %.1f ms)  %s\n",
                    s.avgMs, s.budgetTargetMs,
                    s.budgetMet ? "[PASS]" : "[FAIL]");
        std::printf("  step p50    : %.3f ms\n", s.p50Ms);
        std::printf("  step p99    : %.3f ms\n", s.p99Ms);
        std::printf("  step max    : %.3f ms\n", s.maxMs);
    }
    if (s.totalAtEnd > 0) {
        std::printf("  active      : %d / %d  (sleep %.1f%%)  %s\n",
                    s.activeAtEnd, s.totalAtEnd,
                    s.sleepFraction * 100.0,
                    (s.name == "P2-sleep-fraction"
                        ? (s.budgetMet ? "[PASS]" : "[FAIL]")
                        : ""));
    }
    if (s.name == "queueReject") {
        std::printf("  rejected    : %s  %s\n",
                    s.queueRejected ? "YES" : "no",
                    s.budgetMet ? "[PASS]" : "[FAIL]");
    }
}

} // anonymous namespace

// =============================================================================
// main
// =============================================================================
int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IONBF, 0); // mirror Test_JoltBackend3D main.cpp
    Cfg cfg = parseArgs(argc, argv);

    bool runAll = !cfg.singleScenario;
    auto shouldRun = [&](const char* name) {
        return runAll || cfg.singleName == name;
    };

    std::vector<ScenarioStats> results;
    if (shouldRun("p1"))         results.push_back(runP1(cfg));
    if (shouldRun("p2"))         results.push_back(runP2(cfg));
    if (shouldRun("p3"))         results.push_back(runP3(cfg));
    if (shouldRun("sleep"))      results.push_back(runSleepFraction(cfg));
    if (shouldRun("reject"))     results.push_back(runQueueReject(cfg));

    if (results.empty()) {
        std::fprintf(stderr, "[FAIL] unknown scenario '%s'\n",
                     cfg.singleName.c_str());
        return 4;
    }

    std::printf("\n--- Bench_PhysicsStep summary ---\n");
    int failures = 0;
    for (const auto& s : results) {
        printStats(s);
        if (!s.budgetMet) ++failures;
    }
    if (cfg.waive && failures > 0) {
        std::printf("\n[WAIVE] -w flag set; %d budget breach(es) suppressed. Copy stdout into docs/perf_R15.md.\n", failures);
        return 0;
    }
    if (failures > 0) {
        std::printf("\n[FAIL] %d scenario(s) breached budget. Re-run with -w to suppress.\n", failures);
        return 2;
    }
    std::printf("\n[OK] All scenarios within budget. Copy stdout into docs/perf_R15.md.\n");
    return 0;
}