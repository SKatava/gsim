#include "gsim-simulation/GravityEngine.h"

#include "gsim-simulation/Tree.h"

#include <glm/glm.hpp>

//=====Core Logic=====

//Sets the pointer to the data it will work on
void GravityEngine::init(ObjectData& data) {
    m_data = &data;
    m_initialized = true;
}

void GravityEngine::setMethod(CalculationMethod method) {
    m_method = method;
}

//One simulation step 
void GravityEngine::step() {
    assertValid();
    switch (m_method) {
        case CalculationMethod::DIRECT:     directImplementation();       break;
        case CalculationMethod::BARNESHUT:  barnesHutImplementation();    break;
    } 
}

//=====Private Functions=====

//Check if engine is properly initialized
void GravityEngine::assertValid() const noexcept{
    assert(m_initialized);
    assert(m_data != nullptr);
}

//Uses no aproximation and Newton's formula for gravity(I know it is kind of aproximation my dear physicists)
void GravityEngine::directImplementation() const {
    auto& pos = m_data->positions;
    auto& acc = m_data->accels;
    
    for(int i{}; i < acc.size(); ++i) acc[i] = glm::vec4(0.f, 0.f, 0.f, 0.f);  

    const size_t N = m_data->size();
    constexpr float G = 6.67430e-11f;
    constexpr float EPSILON = 1e-5f; //to soften the force at critical position(radius = 0 => infinite force)

    for(size_t i {0}; i < N; ++i) {
        for(size_t j {i+1}; j < N; ++j) {
            glm::vec4 dir = glm::vec4(pos[j] - pos[i]);
            dir.w = 0.f;
            float dist2 = glm::dot(dir, dir) + EPSILON;
            
            float invDist = 1.0f / sqrt(dist2);
            glm::vec4 forceDir = dir * invDist;

            float forceVal = G * pos[i].w * pos[j].w / dist2;

            glm::vec4 force = forceDir * forceVal;

            acc[i] += force / pos[i].w;
            acc[j] -= force / pos[j].w;
        }
    } 
}

//Uses popular Barnes-Hut aproximation by seperating the objects in octrees
void GravityEngine::barnesHutImplementation() const {
    Tree tree(m_data); 
    tree.buildTree();       
    tree.calculateForces(0.5f, 6.674e-11f, 1e-15);
}


