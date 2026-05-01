#pragma once

#include "ObjectData.h"

enum class CalculationMethod {
    DIRECT,
    BARNESHUT
};

class GravityEngine {
    public:
        //=====Construction/Destruction=====

        GravityEngine() = default;

        GravityEngine(const GravityEngine&) = delete;
        GravityEngine& operator=(const GravityEngine&) = delete;

        GravityEngine(GravityEngine&&) noexcept = default; 
        GravityEngine& operator=(GravityEngine&&) noexcept = default; 

        ~GravityEngine() = default;

        //=====Core Logic=====
        
        void init(ObjectData& data);
        void setMethod(CalculationMethod method);
        void step(float dt);

    private:
        ObjectData* m_data {nullptr};
        bool m_initialized {false};
        CalculationMethod m_method {CalculationMethod::DIRECT};

        //=====Private Functions=====
        void assertValid() const noexcept; 
        void directImplementation(float dt) const;
        void barnesHutImplementation(float dt) const;
};
