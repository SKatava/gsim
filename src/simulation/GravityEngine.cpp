#include "gsim-simulation/GravityEngine.h"

#include "gsim-simulation/Tree.h"

#include <glm/glm.hpp>

//=====Core Logic=====

//Sest the pointer to the data it will work on
void GravityEngine::init(ObjectData& data) {
    m_data = &data;
    m_initialized = true;
}

void GravityEngine::setMethod(CalculationMethod method) {
    m_method = method;
}

//One simulation step based on time change(dt)
void GravityEngine::step(float dt) {
    assertValid();
    switch (m_method) {
        case CalculationMethod::DIRECT:
            directImplementation(dt);
            break;

        case CalculationMethod::BARNESHUT:
            barnesHutImplementation(dt);
            break;
    } 
}


//=====Private Functions=====

//Check if engine is properly initialized
void GravityEngine::assertValid() const noexcept{
    assert(m_initialized);
    assert(m_data != nullptr);
}

//Uses no aproximation and Newton's formula for gravity
void GravityEngine::directImplementation(float dt) const {
    auto& pos = m_data->positions;
    auto& vel = m_data->velocities;
    auto& mass = m_data->masses;

    const size_t N = m_data->size();
    constexpr float G = 6.67430e-11f;
    constexpr float EPSILON = 1e-5f; //to soften the force at critical position(radius = 0 => infinite force)

    for(size_t i {0}; i < N; ++i) {
        for(size_t j {i+1}; j < N; ++j) {
            glm::vec4 dir = glm::vec4(pos[j] - pos[i]);
            float dist2 = glm::dot(dir, dir) + EPSILON;

            float invDist = 1.0f / sqrt(dist2);
            glm::vec4 forceDir = dir * invDist;

            float force = G * mass[i] * mass[j] / dist2;

            glm::vec4 accel = forceDir * force;

            vel[i] += (accel / mass[i]) * dt;
            vel[j] -= (accel / mass[j]) * dt;
        }
    } 
}

//Uses popular Barnes-Hut aproximation by seperating the objects in octrees
void GravityEngine::barnesHutImplementation(float dt) const {
    Tree tree(m_data); 
    tree.buildTree();       
    tree.calculateForces(0.5f, 6.674e-11f, 1e-15);
}


