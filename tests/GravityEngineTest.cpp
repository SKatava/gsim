#include <gtest/gtest.h>
#include <cmath>
#include <cstdlib>

#include "gsim-simulation/GravityEngine.h"
#include "gsim-simulation/Integrator.h"
#include "gsim-simulation/ObjectData.h"

// ==========Helpers==========
static float kineticEnergy(const ObjectData& data) {
    float ke = 0.f;
    for (size_t i = 0; i < data.size(); ++i) {
        const auto& v = data.velocities[i];
        ke += 0.5f * data.masses[i] * (v.x*v.x + v.y*v.y + v.z*v.z);
    }
    return ke;
}

static ObjectData makeSymmetricPair() {
    ObjectData d;
    d.positions  = { glm::vec4( 1.f, 0.f, 0.f, 0.f),
                     glm::vec4(-1.f, 0.f, 0.f, 0.f) };
    d.velocities = { glm::vec4(0.f), glm::vec4(0.f) };
    d.masses     = { 1e10f, 1e10f };
    return d;
}

static ObjectData makeCollidingPair() {
    ObjectData d;
    d.positions  = { glm::vec4(0.f, 0.f,  100.f, 0.f),
                     glm::vec4(0.f, 0.f, -100.f, 0.f) };
    d.velocities = { glm::vec4(0.f), glm::vec4(0.f) };
    d.masses     = { 1e14f, 1e14f };
    return d;
}

static ObjectData makeNBody(int N, unsigned seed = 42) {
    ObjectData d;
    std::srand(seed);
    auto rf = []() { return (std::rand() / float(RAND_MAX)) * 800.f + 100.f; };
    for (int i = 0; i < N; ++i) {
        d.positions.emplace_back(rf(), rf(), rf(), 0.f);
        d.velocities.emplace_back(0.f);
        d.masses.push_back(1e12f + std::rand() % static_cast<int>(1e12f));
    }
    return d;
}

struct Sim {
    ObjectData    data;
    GravityEngine engine;
    Integrator    integrator;

    void init(ObjectData d, CalculationMethod method = CalculationMethod::DIRECT) {
        data = std::move(d);
        engine.setMethod(method);
        engine.init(data);
        integrator.init(data);
    }

    // Full tick: force accumulation then position integration.
    void step(float dt) {
        engine.step(dt);
        integrator.step(dt);
    }

    void stepN(int n, float dt) {
        for (int i = 0; i < n; ++i) step(dt);
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// Direct O(N²) algorithm tests
// ─────────────────────────────────────────────────────────────────────────────

//Only one body resulting with no force
TEST(DirectTest, SingleBodyNoSelfForce) {
    Sim sim;
    sim.init([] {
        ObjectData d;
        d.positions  = { glm::vec4(0.f) };
        d.velocities = { glm::vec4(0.f) };
        d.masses     = { 1e12f };
        return d;
    }());
    sim.step(1.f);

    EXPECT_FLOAT_EQ(sim.data.velocities[0].x, 0.f);
    EXPECT_FLOAT_EQ(sim.data.velocities[0].y, 0.f);
    EXPECT_FLOAT_EQ(sim.data.velocities[0].z, 0.f);
    EXPECT_FLOAT_EQ(sim.data.positions[0].x, 0.f);
    EXPECT_FLOAT_EQ(sim.data.positions[0].y, 0.f);
    EXPECT_FLOAT_EQ(sim.data.positions[0].z, 0.f);
}

//Two equal bodeies result in equal but oposite forces, resulting in no change of velocities
TEST(DirectTest, SymmetricPairNewtonThirdLaw) {
    Sim sim;
    sim.init(makeSymmetricPair());
    sim.engine.step(1.f);

    EXPECT_NEAR(sim.data.velocities[0].x + sim.data.velocities[1].x, 0.f, 1e-5f);
    EXPECT_NEAR(sim.data.velocities[0].y, 0.f, 1e-10f);
    EXPECT_NEAR(sim.data.velocities[1].y, 0.f, 1e-10f);
    EXPECT_NEAR(sim.data.velocities[0].z, 0.f, 1e-10f);
    EXPECT_NEAR(sim.data.velocities[1].z, 0.f, 1e-10f);
}

// Attraction direction: body at +X must gain negative vx, body at -X positive vx.
TEST(DirectTest, SymmetricPairAttractsCorrectly) {
    Sim sim;
    sim.init(makeSymmetricPair());
    sim.engine.step(1.f);

    EXPECT_LT(sim.data.velocities[0].x, 0.f);
    EXPECT_GT(sim.data.velocities[1].x, 0.f);
}

// Gravity does work: kinetic energy must increase for a pair starting at rest.
TEST(DirectTest, AttractingPairKineticEnergyIncreases) {
    Sim sim;
    sim.init(makeCollidingPair());

    const float ke0 = kineticEnergy(sim.data);
    sim.stepN(100, 0.1f);

    EXPECT_GT(kineticEnergy(sim.data), ke0);
}

// Doubling m[0] must double the acceleration of m[1] (F = G·m0·m1/r²).
TEST(DirectTest, ForceMagnitudeScalesWithMass) {
    ObjectData base;
    base.positions  = { glm::vec4(0.f), glm::vec4(10.f, 0.f, 0.f, 0.f) };
    base.velocities = { glm::vec4(0.f), glm::vec4(0.f) };
    base.masses     = { 1e12f, 1e12f };

    ObjectData doubled = base;
    doubled.masses[0] *= 2.f;

    Sim s1, s2;
    s1.init(base);
    s2.init(doubled);

    // Force step only — velocity directly reflects the impulse received.
    s1.engine.step(1.f);
    s2.engine.step(1.f);

    EXPECT_NEAR(std::abs(s2.data.velocities[1].x),
                std::abs(s1.data.velocities[1].x) * 2.f,
                std::abs(s1.data.velocities[1].x) * 2.f * 1e-4f);
}

// Integrator correctness: position update must equal pos + vel * dt (Euler forward).
TEST(DirectTest, IntegratorPositionUpdateIsEulerForward) {
    Sim sim;
    sim.init(makeSymmetricPair());

    sim.engine.step(1.f);
    const glm::vec4 vel0        = sim.data.velocities[0];
    const glm::vec4 pos0_before = sim.data.positions[0];

    sim.integrator.step(1.f);

    const glm::vec4 expected = pos0_before + vel0 * 1.f;
    EXPECT_NEAR(sim.data.positions[0].x, expected.x, 1e-5f);
    EXPECT_NEAR(sim.data.positions[0].y, expected.y, 1e-5f);
    EXPECT_NEAR(sim.data.positions[0].z, expected.z, 1e-5f);
}

// Euler drifts, but radius should stay within 10% over 5000 steps.
TEST(DirectTest, CircularOrbitStaysNearRadius) {
    constexpr float G = 6.67430e-11f;
    constexpr float M = 1e26f;
    constexpr float r = 1e8f;
    const float     v = std::sqrt(G * M / r);

    Sim sim;
    sim.init([&] {
        ObjectData d;
        d.positions  = { glm::vec4(0.f), glm::vec4(r, 0.f, 0.f, 0.f) };
        d.velocities = { glm::vec4(0.f), glm::vec4(0.f, v, 0.f, 0.f) };
        d.masses     = { M, 1.f };
        return d;
    }());
    sim.stepN(5000, 10.f);

    const auto& p    = sim.data.positions[1];
    const float dist = std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
    EXPECT_NEAR(dist, r, r * 0.10f);
}

// ─────────────────────────────────────────────────────────────────────────────
// Barnes-Hut O(NlogN) algorithm tests
// ─────────────────────────────────────────────────────────────────────────────

//Only one body resulting in no force
TEST(BarnesHutTest, SingleBodyNoSelfForce) {
    Sim sim;
    sim.init([] {
        ObjectData d;
        d.positions  = { glm::vec4(500.f, 500.f, 500.f, 0.f) };
        d.velocities = { glm::vec4(0.f) };
        d.masses     = { 1e12f };
        return d;
    }(), CalculationMethod::BARNESHUT);
    sim.step(1.f);

    EXPECT_NEAR(sim.data.velocities[0].x, 0.f, 1e-10f);
    EXPECT_NEAR(sim.data.velocities[0].y, 0.f, 1e-10f);
    EXPECT_NEAR(sim.data.velocities[0].z, 0.f, 1e-10f);
}

// Leaf nodes are never approximated regardless of theta — 2-body must be exact.
TEST(BarnesHutTest, TwoBodyMatchesDirect) {
    Sim direct, bht;
    direct.init(makeSymmetricPair(), CalculationMethod::DIRECT);
    bht.init(makeSymmetricPair(),    CalculationMethod::BARNESHUT);

    direct.engine.step(1.f);
    bht.engine.step(1.f);

    for (size_t i = 0; i < direct.data.size(); ++i) {
        EXPECT_NEAR(bht.data.velocities[i].x, direct.data.velocities[i].x, 1e-4f)
            << "body " << i << " vx mismatch";
        EXPECT_NEAR(bht.data.velocities[i].y, direct.data.velocities[i].y, 1e-4f)
            << "body " << i << " vy mismatch";
        EXPECT_NEAR(bht.data.velocities[i].z, direct.data.velocities[i].z, 1e-4f)
            << "body " << i << " vz mismatch";
    }
}

//Pair of sam objects resulting in same but oposite forces, meaning no change in velocity
TEST(BarnesHutTest, SymmetricPairAttractsCorrectly) {
    Sim sim;
    sim.init(makeSymmetricPair(), CalculationMethod::BARNESHUT);
    sim.engine.step(1.f);

    EXPECT_LT(sim.data.velocities[0].x, 0.f);
    EXPECT_GT(sim.data.velocities[1].x, 0.f);
}

//Excpecting increase in kinetic energy as force is applied
TEST(BarnesHutTest, AttractingPairKineticEnergyIncreases) {
    Sim sim;
    sim.init(makeCollidingPair(), CalculationMethod::BARNESHUT);

    const float ke0 = kineticEnergy(sim.data);
    sim.stepN(100, 0.1f);

    EXPECT_GT(kineticEnergy(sim.data), ke0);
}

// Per-body relative velocity error vs direct must stay under 5% (theta = 0.5).
TEST(BarnesHutTest, NBodyVelocityWithinToleranceOfDirect) {
    constexpr int   N   = 50;
    constexpr float TOL = 0.05f;

    Sim direct, bht;
    direct.init(makeNBody(N), CalculationMethod::DIRECT);
    bht.init(makeNBody(N),    CalculationMethod::BARNESHUT); // same seed → identical layout

    direct.engine.step(1.f);
    bht.engine.step(1.f);

    for (int i = 0; i < N; ++i) {
        const auto& vd  = direct.data.velocities[i];
        const auto& vb  = bht.data.velocities[i];
        const float mag = std::sqrt(vd.x*vd.x + vd.y*vd.y + vd.z*vd.z);
        if (mag < 1e-12f) continue;

        const float err = std::sqrt(
            (vd.x-vb.x)*(vd.x-vb.x) +
            (vd.y-vb.y)*(vd.y-vb.y) +
            (vd.z-vb.z)*(vd.z-vb.z)
        ) / mag;

        EXPECT_LT(err, TOL) << "body " << i << " relative velocity error "
                             << err << " exceeds " << TOL;
    }
}
