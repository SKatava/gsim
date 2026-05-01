#include "gsim-simulation/ObjectView.h"

//=====Construction/Destruction=====

ObjectView::ObjectView(uint32_t id, ObjectData& data) 
    : m_id(id), m_data(&data) {}

//=====Getter/Setters=====

const glm::vec4& ObjectView::Position() const noexcept {
    assert(m_id < m_data->positions.size());
    return m_data->positions[m_id];
}

glm::vec4& ObjectView::Position() noexcept {
    assert(m_id < m_data->positions.size());
    return m_data->positions[m_id];
}

const glm::vec4& ObjectView::Velocity() const noexcept {
    assert(m_id < m_data->velocities.size());
    return m_data->velocities[m_id];
}

glm::vec4& ObjectView::Velocity() noexcept {
    assert(m_id < m_data->velocities.size());
    return m_data->velocities[m_id];
}

const float& ObjectView::Mass() const noexcept {
    assert(m_id < m_data->masses.size());
    return m_data->masses[m_id];
}

float& ObjectView::Mass() noexcept {
    assert(m_id < m_data->masses.size());
    return m_data->masses[m_id];
}
