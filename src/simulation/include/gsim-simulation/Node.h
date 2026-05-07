#pragma once

#include <algorithm>
#include <vector>

#include <glm/vec4.hpp>

struct Node {
    float mass {};
    glm::vec4 cm {};
    glm::vec4 center {};
    float size {};
    int body {-1}; // (-1) - empty, (-2) - internal, else leaf
    int children[8];

    Node() {
        std::fill(std::begin(children), std::end(children), -1);
    }
};
