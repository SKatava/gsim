#include "gsim-simulation/ObjectView.h"

//=====Construction/Destruction=====

ObjectView::ObjectView(uint32_t id, ObjectData& data) 
    : m_id(id), m_data(&data) {}

//=====Getter/Setters=====

void ObjectView::setPosition(const glm::vec3& p) noexcept {
    assert(m_id < m_data->positions.size());
    m_data->positions[m_id].x = p.x;
    m_data->positions[m_id].y = p.y;
    m_data->positions[m_id].z = p.z;
}

glm::vec3 ObjectView::velocity() const noexcept {
    assert(m_id < m_data->velocities.size());
    return glm::vec3(m_data->velocities[m_id]);
}

void ObjectView::setVelocity(const glm::vec3& v) noexcept {
    assert(m_id < m_data->velocities.size());
    m_data->velocities[m_id].x = v.x;
    m_data->velocities[m_id].y = v.y;
    m_data->velocities[m_id].z = v.z;
}

glm::vec3 ObjectView::acceleration() const noexcept {
    assert(m_id < m_data->accels.size());
    return glm::vec3(m_data->accels[m_id]);
}

void ObjectView::setAcceleration(const glm::vec3& a) noexcept {
    assert(m_id < m_data->accels.size());
    m_data->accels[m_id].x = a.x;
    m_data->accels[m_id].y = a.y;
    m_data->accels[m_id].z = a.z;
}

float ObjectView::mass() const noexcept {
    assert(m_id < m_data->positions.size());
    return m_data->positions[m_id].w;
}

void ObjectView::setMass(float m) noexcept {
    assert(m_id < m_data->positions.size());
    m_data->positions[m_id].w = m;
}
