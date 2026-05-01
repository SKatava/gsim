#pragma once

#include "ObjectData.h"

class ObjectView {
    public:
        //=====Construction/Destruction=====
        explicit ObjectView(uint32_t id, ObjectData& data);
        ~ObjectView() = default;

        //=====Getter/Setters=====
        [[nodiscard]] const glm::vec4& Position() const noexcept;
        [[nodiscard]] glm::vec4& Position() noexcept;
        [[nodiscard]] const glm::vec4& Velocity() const noexcept;
        [[nodiscard]] glm::vec4& Velocity() noexcept;
        [[nodiscard]] const float& Mass() const noexcept;
        [[nodiscard]] float& Mass() noexcept;
        
    private:
        uint32_t m_id;
        ObjectData* m_data;
};
