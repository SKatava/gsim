#include "gsim-simulation/Integrator.h"

#include <glm/glm.hpp>

//=====Core Logic=====

//Sest the pointer to the data it will work on
void Integrator::init(ObjectData& data) {
    m_data = &data;
    m_initialized = true;
}

void Integrator::step(float dt) {
    assertValid();
    for(int i {}; i < m_data->size(); ++i) {
        m_data->velocities[i] += m_data->accels[i] * dt; 
        m_data->positions[i] += m_data->velocities[i] * dt;
    }
}

void Integrator::assertValid() const noexcept {
    assert(m_initialized);
    assert(m_data != nullptr);
}

