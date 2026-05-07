#pragma once

#include "ObjectData.h"

class Integrator {
    public:
        //=====Construction/Destruction=====

        Integrator() = default;

        Integrator(const Integrator&) = delete;
        Integrator& operator=(const Integrator&) = delete;

        Integrator(Integrator&&) noexcept = default;
        Integrator& operator=(Integrator&&) noexcept = default;

        ~Integrator() = default;

        //=====Core Logic=====
        void init(ObjectData& data);
        void step(float dt);
        
    private:
        ObjectData* m_data {nullptr};
        bool m_initialized {false};

        //=====Private Functions=====
        void assertValid() const noexcept; 

};
