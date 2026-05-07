#pragma once

#include "ObjectData.h"
#include <glm/vec3.hpp>

class ObjectView {
    public:
        //=====Construction/Destruction=====
        explicit ObjectView(uint32_t id, ObjectData& data);
        ~ObjectView() = default;

        //=====Getter/Setters=====

        glm::vec3 position() const noexcept;
        glm::vec3 velocity() const noexcept;
        glm::vec3 acceleration() const noexcept;
        float mass() const noexcept;

        void setPosition (const glm::vec3& p) noexcept;
        void setVelocity (const glm::vec3& v) noexcept;
        void setAcceleration (const glm::vec3& a) noexcept;
        void setMass (float m) noexcept;

    private:
        uint32_t m_id;
        ObjectData* m_data;
};
