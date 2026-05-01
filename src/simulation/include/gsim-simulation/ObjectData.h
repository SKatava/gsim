#pragma once

#include <vector>
#include <glm/vec4.hpp>

struct ObjectData {
    std::vector<glm::vec4> positions {};    //16 bytes 
    std::vector<glm::vec4> velocities {};   //16 bytes
    std::vector<float> masses {};           //4 bytes
                                            
    size_t size() const noexcept { return masses.size(); }
};
