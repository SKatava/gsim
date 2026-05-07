#include <gtest/gtest.h>
#include <cmath>
#include <cstdlib>

#include "gsim-simulation/GravityEngine.h"
#include "gsim-simulation/Integrator.h"
#include "gsim-simulation/ObjectData.h"

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

static float kineticEnergy(const ObjectData& data) {
    float ke = 0.f;
    for (size_t i = 0; i < data.size(); ++i) {
        const auto& v = data.velocities[i];
        const float m = data.positions[i].w;
        ke += 0.5f * m * (v.x*v.x + v.y*v.y + v.z*v.z);
    }
    return ke;
}

// xyz = position, w = mass
static ObjectData makeSymmetricPair() {
    ObjectData d;
    d.positions  = { glm::vec4( 1.f, 0.f, 0.f, 1e10f),
                     glm::vec4(-1.f, 0.f, 0.f, 1e10f) };
    d.velocities = { glm::vec4(0.f), glm::vec4(0.f) };
    d.accels     = { glm::vec4(0.f), glm::vec4(0.f) };
    return d;
}

static ObjectData makeCollidingPair() {
    ObjectData d;
    d.positions  = { glm::vec4(0.f, 0.f,  100.f, 1e14f),
                     glm::vec4(0.f, 0.f, -100.f, 1e14f) };
    d.velocities = { glm::vec4(0.f), glm::vec4(0.f) };
    d.accels     = { glm::vec4(0.f), glm::vec4(0.f) };
    return d;
}

static ObjectData makeNBody(int N, unsigned seed = 42) {
    ObjectData d;
    std::srand(seed);
    auto rf = []() { return (std::rand() / float(RAND_MAX)) * 800.f + 100.f; };
    for (int i = 0; i < N; ++i) {
        float mass = 1e12f + std::rand() % static_cast<int>(1e12f);
        d.positions.emplace_back(rf(), rf(), rf(), mass);
        d.velocities.emplace_back(0.f);
        d.accels.emplace_back(0.f);
    }
    return d;
}

// ─────────────────────────────────────────────────────────────────────────────
// Sim: wires engine + integrator to the same ObjectData.
// Contract: engine.step() writes accels, integrator.step(dt) reads accels
//           and updates velocities then positions.
// ─────────────────────────────────────────────────────────────────────────────

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

    void step(float dt) {
        engine.step();
        integrator.step(dt);
    }

    void stepN(int n, float dt) {
        for (int i = 0; i < n; ++i) step(dt);
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// Direct O(N²)
// ─────────────────────────────────────────────────────────────────────────────

// No partner → no force → accels, velocities, positions all stay zero.
TEST(DirectTest, SingleBodyNoSelfForce) {
    Sim sim;
    sim.init([] {
        ObjectData d;
        d.positions  = { glm::vec4(0.f, 0.f, 0.f, 1e12f) };
        d.velocities = { glm::vec4(0.f) };
        d.accels     = { glm::vec4(0.f) };
        return d;
    }());
    sim.step(1.f);

    EXPECT_FLOAT_EQ(sim.data.accels[0].x,     0.f);
    EXPECT_FLOAT_EQ(sim.data.accels[0].y,     0.f);
    EXPECT_FLOAT_EQ(sim.data.accels[0].z,     0.f);
    EXPECT_FLOAT_EQ(sim.data.velocities[0].x, 0.f);
    EXPECT_FLOAT_EQ(sim.data.velocities[0].y, 0.f);
    EXPECT_FLOAT_EQ(sim.data.velocities[0].z, 0.f);
    EXPECT_FLOAT_EQ(sim.data.positions[0].x,  0.f);
    EXPECT_FLOAT_EQ(sim.data.positions[0].y,  0.f);
    EXPECT_FLOAT_EQ(sim.data.positions[0].z,  0.f);
    EXPECT_FLOAT_EQ(sim.data.positions[0].w,  1e12f);
}

// Equal masses → equal and opposite accelerations (Newton's third law).
// Checked after engine.step() only — accels are the direct output of the force kernel.
TEST(DirectTest, SymmetricPairNewtonThirdLaw) {
    Sim sim;
    sim.init(makeSymmetricPair());
    sim.engine.step();

    EXPECT_NEAR(sim.data.accels[0].x + sim.data.accels[1].x, 0.f, 1e-5f);
    EXPECT_NEAR(sim.data.accels[0].y, 0.f, 1e-10f);
    EXPECT_NEAR(sim.data.accels[1].y, 0.f, 1e-10f);
    EXPECT_NEAR(sim.data.accels[0].z, 0.f, 1e-10f);
    EXPECT_NEAR(sim.data.accels[1].z, 0.f, 1e-10f);
}

// Body at +X must be pulled toward -X, body at -X toward +X.
// Full tick needed: engine writes accels, integrator applies them to velocities.
TEST(DirectTest, SymmetricPairAttractsCorrectly) {
    Sim sim;
    sim.init(makeSymmetricPair());
    sim.step(1.f);

    EXPECT_LT(sim.data.velocities[0].x, 0.f);
    EXPECT_GT(sim.data.velocities[1].x, 0.f);
}

// Force kernel must not corrupt mass packed in positions.w.
TEST(DirectTest, ForceStepPreservesMass) {
    Sim sim;
    sim.init(makeSymmetricPair());
    sim.engine.step();

    EXPECT_FLOAT_EQ(sim.data.positions[0].w, 1e10f);
    EXPECT_FLOAT_EQ(sim.data.positions[1].w, 1e10f);
}

// Integrator must not corrupt mass packed in positions.w.
TEST(DirectTest, IntegratorPreservesMass) {
    Sim sim;
    sim.init(makeSymmetricPair());
    sim.step(1.f);

    EXPECT_FLOAT_EQ(sim.data.positions[0].w, 1e10f);
    EXPECT_FLOAT_EQ(sim.data.positions[1].w, 1e10f);
}

// Gravity does work: kinetic energy must increase for a pair starting at rest.
TEST(DirectTest, AttractingPairKineticEnergyIncreases) {
    Sim sim;
    sim.init(makeCollidingPair());

    const float ke0 = kineticEnergy(sim.data);
    sim.stepN(100, 0.1f);

    EXPECT_GT(kineticEnergy(sim.data), ke0);
}

// Doubling m[0] must double the acceleration of m[1] (a = G*m0/r²).
TEST(DirectTest, ForceMagnitudeScalesWithMass) {
    ObjectData base;
    base.positions  = { glm::vec4(0.f,  0.f, 0.f, 1e12f),
                        glm::vec4(10.f, 0.f, 0.f, 1e12f) };
    base.velocities = { glm::vec4(0.f), glm::vec4(0.f) };
    base.accels     = { glm::vec4(0.f), glm::vec4(0.f) };

    ObjectData doubled = base;
    doubled.positions[0].w *= 2.f;

    Sim s1, s2;
    s1.init(base);
    s2.init(doubled);

    // Force step only — accels directly reflect what the kernel computed.
    s1.engine.step();
    s2.engine.step();

    EXPECT_NEAR(std::abs(s2.data.accels[1].x),
                std::abs(s1.data.accels[1].x) * 2.f,
                std::abs(s1.data.accels[1].x) * 2.f * 1e-4f);
}

// Integrator contract: v += a*dt, then x += v*dt (symplectic Euler).
TEST(DirectTest, IntegratorPositionUpdateIsCorrect) {
    Sim sim;
    sim.init(makeSymmetricPair());

    // Populate accels without touching velocities or positions.
    sim.engine.step();

    const glm::vec4 accel0    = sim.data.accels[0];
    const glm::vec4 vel0      = sim.data.velocities[0];   // still zero
    const glm::vec4 pos0      = sim.data.positions[0];

    sim.integrator.step(1.f);

    const glm::vec4 expected_vel = vel0   + accel0        * 1.f;
    const glm::vec4 expected_pos = pos0   + expected_vel  * 1.f;  // symplectic: uses v_new

    EXPECT_NEAR(sim.data.velocities[0].x, expected_vel.x, 1e-5f);
    EXPECT_NEAR(sim.data.velocities[0].y, expected_vel.y, 1e-5f);
    EXPECT_NEAR(sim.data.velocities[0].z, expected_vel.z, 1e-5f);
    EXPECT_NEAR(sim.data.positions[0].x,  expected_pos.x, 1e-5f);
    EXPECT_NEAR(sim.data.positions[0].y,  expected_pos.y, 1e-5f);
    EXPECT_NEAR(sim.data.positions[0].z,  expected_pos.z, 1e-5f);
    EXPECT_FLOAT_EQ(sim.data.positions[0].w, 1e10f);  // mass untouched
}

// Euler drifts — baseline only. Radius must stay within 20% over 5000 steps.
TEST(DirectTest, CircularOrbitStaysNearRadius) {
    constexpr float G = 6.67430e-11f;
    constexpr float M = 1e26f;
    constexpr float r = 1e8f;
    const float     v = std::sqrt(G * M / r);

    Sim sim;
    sim.init([&] {
        ObjectData d;
        d.positions  = { glm::vec4(0.f, 0.f, 0.f, M),
                         glm::vec4(r,   0.f, 0.f, 1.f) };
        d.velocities = { glm::vec4(0.f),
                         glm::vec4(0.f, v, 0.f, 0.f) };
        d.accels     = { glm::vec4(0.f), glm::vec4(0.f) };
        return d;
    }());
    sim.stepN(5000, 10.f);

    const auto& p    = sim.data.positions[1];
    const float dist = std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
    EXPECT_NEAR(dist, r, r * 0.20f);  // 20%: Euler drifts, this is a sanity check not accuracy
}

// ─────────────────────────────────────────────────────────────────────────────
// Barnes-Hut O(NlogN)
// ─────────────────────────────────────────────────────────────────────────────

TEST(BarnesHutTest, SingleBodyNoSelfForce) {
    Sim sim;
    sim.init([] {
        ObjectData d;
        d.positions  = { glm::vec4(500.f, 500.f, 500.f, 1e12f) };
        d.velocities = { glm::vec4(0.f) };
        d.accels     = { glm::vec4(0.f) };
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

    direct.engine.step();
    bht.engine.step();

    for (size_t i = 0; i < direct.data.size(); ++i) {
        EXPECT_NEAR(bht.data.accels[i].x, direct.data.accels[i].x, 1e-4f)
            << "body " << i << " ax mismatch";
        EXPECT_NEAR(bht.data.accels[i].y, direct.data.accels[i].y, 1e-4f)
            << "body " << i << " ay mismatch";
        EXPECT_NEAR(bht.data.accels[i].z, direct.data.accels[i].z, 1e-4f)
            << "body " << i << " az mismatch";
    }
}

// Full tick needed to see velocity change.
TEST(BarnesHutTest, SymmetricPairAttractsCorrectly) {
    Sim sim;
    sim.init(makeSymmetricPair(), CalculationMethod::BARNESHUT);
    sim.step(1.f);

    EXPECT_LT(sim.data.velocities[0].x, 0.f);
    EXPECT_GT(sim.data.velocities[1].x, 0.f);
}

TEST(BarnesHutTest, AttractingPairKineticEnergyIncreases) {
    Sim sim;
    sim.init(makeCollidingPair(), CalculationMethod::BARNESHUT);

    const float ke0 = kineticEnergy(sim.data);
    sim.stepN(100, 0.1f);

    EXPECT_GT(kineticEnergy(sim.data), ke0);
}

// Per-body relative acceleration error vs direct must stay under 5% (theta = 0.5).
// Checked on accels directly — cleaner than velocities since no integration error accumulates.
TEST(BarnesHutTest, NBodyAccelerationWithinToleranceOfDirect) {
    constexpr int   N   = 50;
    constexpr float TOL = 0.05f;

    Sim direct, bht;
    direct.init(makeNBody(N), CalculationMethod::DIRECT);
    bht.init(makeNBody(N),    CalculationMethod::BARNESHUT);

    direct.engine.step();
    bht.engine.step();

    for (int i = 0; i < N; ++i) {
        const auto& ad  = direct.data.accels[i];
        const auto& ab  = bht.data.accels[i];
        const float mag = std::sqrt(ad.x*ad.x + ad.y*ad.y + ad.z*ad.z);
        if (mag < 1e-12f) continue;

        const float err = std::sqrt(
            (ad.x-ab.x)*(ad.x-ab.x) +
            (ad.y-ab.y)*(ad.y-ab.y) +
            (ad.z-ab.z)*(ad.z-ab.z)
        ) / mag;

        EXPECT_LT(err, TOL) << "body " << i << " relative acceleration error "
                             << err << " exceeds " << TOL;
    }
}
