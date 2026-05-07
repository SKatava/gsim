#pragma once

#include <vector>
#include <glm/vec4.hpp>

struct ObjectData {
    std::vector<glm::vec4> positions {};    //16 bytes, w-mass 
    std::vector<glm::vec4> velocities {};   //16 bytes, w-empty
    std::vector<glm::vec4> accels {};       //16 bytes, w-empty
                                            
    size_t size() const noexcept    { return positions.size(); }
};
