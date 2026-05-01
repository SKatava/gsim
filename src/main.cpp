#include <iostream>
#include <iomanip>
#include <cassert>
#include <math.h>
#include "gsim-simulation/GravityEngine.h"


// ── helpers ────────────────────────────────────────────────────────────────

static void printState(const ObjectData& data, const char* label) {
    std::cout << "\n[" << label << "]\n";
    std::cout << std::fixed << std::setprecision(6);
    for (size_t i = 0; i < data.size(); ++i) {
        const auto& p = data.positions[i];
        const auto& v = data.velocities[i];
        std::cout
            << "  obj[" << i << "]"
            << "  pos=(" << p.x << ", " << p.y << ", " << p.z << ")"
            << "  vel=(" << v.x << ", " << v.y << ", " << v.z << ")"
            << "  mass=" << data.masses[i] << "\n";
    }
}

// Returns total kinetic energy — must decrease (or stay equal) over time
// for a purely gravitational system with no energy injection.
static float kineticEnergy(const ObjectData& data) {
    float ke = 0.f;
    for (size_t i = 0; i < data.size(); ++i) {
        const auto& v = data.velocities[i];
        float v2 = v.x*v.x + v.y*v.y + v.z*v.z;
        ke += 0.5f * data.masses[i] * v2;
    }
    return ke;
}

// ── tests ───────────────────────────────────────────────────────────────────

// Two equal masses placed symmetrically — net force on each should be equal
// in magnitude, opposite in direction → after one step velocities must be
// equal in magnitude, opposite in sign (along the axis joining them).
static void testSymmetricPair() {
    ObjectData data;
    data.positions  = { glm::vec4(1.f, 0.f, 0.f, 0.f),
                        glm::vec4(-1.f, 0.f, 0.f, 0.f) };
    data.velocities = { glm::vec4(0.f), glm::vec4(0.f) };
    data.masses     = { 1e10f, 1e10f };

    GravityEngine engine;
    engine.setMethod(CalculationMethod::DIRECT);
    engine.init(data);
    engine.step(1.f);

    printState(data, "testSymmetricPair — after 1 step");

    // Velocities must be equal magnitude, opposite direction on X axis
    assert(std::abs(data.velocities[0].x + data.velocities[1].x) < 1e-5f);
    assert(std::abs(data.velocities[0].y) < 1e-10f);
    assert(std::abs(data.velocities[1].y) < 1e-10f);
    std::cout << "  ✓ symmetry preserved\n";
}

// A single object must not move — no partner to exert force.
static void testSingleObject() {
    ObjectData data;
    data.positions  = { glm::vec4(0.f, 0.f, 0.f, 0.f) };
    data.velocities = { glm::vec4(0.f) };
    data.masses     = { 1e12f };

    GravityEngine engine;
    engine.setMethod(CalculationMethod::DIRECT);
    engine.init(data);
    engine.step(1.f);

    assert(data.velocities[0] == glm::vec4(0.f));
    std::cout << "\n[testSingleObject]\n  ✓ no self-force\n";
}

// Kinetic energy must increase for an attracting pair starting from rest
// (potential energy converts to kinetic).
static void testEnergyConservationTrend() {
    ObjectData data;
    data.positions  = { glm::vec4(0.f, 0.f,  100.f, 0.f),
                        glm::vec4(0.f, 0.f, -100.f, 0.f) };
    data.velocities = { glm::vec4(0.f), glm::vec4(0.f) };
    data.masses     = { 1e14f, 1e14f };

    GravityEngine engine;
    engine.setMethod(CalculationMethod::DIRECT);
    engine.init(data);

    float ke0 = kineticEnergy(data);   // 0 — both at rest
    for (int i = 0; i < 100; ++i)
        engine.step(0.1f);
    float ke1 = kineticEnergy(data);

    std::cout << "\n[testEnergyConservationTrend]\n";
    std::cout << "  KE before: " << ke0 << "  KE after: " << ke1 << "\n";
    assert(ke1 > ke0);                 // objects accelerated toward each other
    std::cout << "  ✓ kinetic energy increased (gravity did work)\n";
}

// Longer integration — watch two objects orbit and print every N steps.
static void testOrbitIntegration() {
    // Circular orbit setup (Newtonian): v = sqrt(G*M/r)
    constexpr float G   = 6.67430e-11f;
    constexpr float M   = 1e26f;   // heavy central body
    constexpr float r   = 1e8f;    // orbital radius (m)
    const float     v   = std::sqrt(G * M / r);

    ObjectData data;
    data.positions  = { glm::vec4(0.f, 0.f, 0.f, 0.f),          // central
                        glm::vec4(r,   0.f, 0.f, 0.f) };         // orbiter
    data.velocities = { glm::vec4(0.f, 0.f, 0.f, 0.f),
                        glm::vec4(0.f, v,   0.f, 0.f) };         // tangential
    data.masses     = { M, 1.f };   // massless orbiter approximation

    GravityEngine engine;
    engine.setMethod(CalculationMethod::DIRECT);
    engine.init(data);

    constexpr float dt    = 10.f;
    constexpr int   STEPS = 5000;
    constexpr int   PRINT = 500;

    std::cout << "\n[testOrbitIntegration]\n";
    for (int i = 0; i < STEPS; ++i) {
        engine.step(dt);
        if (i % PRINT == 0) {
            const auto& p = data.positions[1];
            float dist = std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
            std::cout << "  step " << std::setw(4) << i
                      << "  r=" << dist << " m"
                      << "  KE=" << kineticEnergy(data) << "\n";
        }
    }
    std::cout << "  ✓ integration completed\n";
}

// ── entry point ─────────────────────────────────────────────────────────────

int main() {
    testSingleObject();
    testSymmetricPair();
    testEnergyConservationTrend();
    testOrbitIntegration();

    std::cout << "\nAll tests passed.\n";
}
