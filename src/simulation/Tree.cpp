#include "gsim-simulation/Tree.h"

#include <math.h>
#include <glm/glm.hpp>

Tree::Tree(ObjectData* data) {
    m_data = data;
    m_nodes.reserve(4 * m_data->size() + 8); 
    m_nodes.emplace_back();                 
    m_nodes[0].size   = 1000.f;
    m_nodes[0].center = glm::vec4(500.f, 500.f, 500.f, 0.f);
}

void Tree::buildTree() {
    for (int i = 0; i < static_cast<int>(m_data->size()); ++i) {
        insertToNode(i, 0);
    }
    accumulateMass(0);
}

void Tree::insertToNode(int objectIndex, int nodeIndex) {
    Node& node = m_nodes[nodeIndex];

    if (node.body == -1) {
        node.body = objectIndex;
    }
    else if (node.body == -2) {
        int quad = getQuadrant(objectIndex, nodeIndex);
        if (m_nodes[nodeIndex].children[quad] == -1) {
            glm::vec4 childCenter = getCenter(nodeIndex, quad);
            float childSize = m_nodes[nodeIndex].size * 0.5f;

            m_nodes.emplace_back();
            int child = static_cast<int>(m_nodes.size()) - 1;
            m_nodes[child].center = childCenter;
            m_nodes[child].size = childSize;
            m_nodes[nodeIndex].children[quad] = child; 
        }
        insertToNode(objectIndex, m_nodes[nodeIndex].children[quad]);
    }
    else {
        int existing = node.body;
        m_nodes[nodeIndex].body = -2; 

        insertToNode(existing, nodeIndex);
        insertToNode(objectIndex, nodeIndex);
    }
}

void Tree::accumulateMass(int nodeIndex) {
    Node& node = m_nodes[nodeIndex];

    if (node.body >= 0) {
        node.mass = m_data->positions[node.body].w;
        node.cm = m_data->positions[node.body];
        return;
    }

    if (node.body == -2) {
        node.mass = 0.f;
        node.cm = glm::vec4(0.f);
        for (int c : node.children) {
            if (c == -1) continue;
            accumulateMass(c);
            node.mass += m_nodes[c].mass;
            node.cm += m_nodes[c].cm * m_nodes[c].mass;
        }
        if (node.mass > 0.f)
            node.cm /= node.mass;
    }
}

int Tree::getQuadrant(int oInd, int nInd) {
    const auto& pos = m_data->positions[oInd];
    const auto& c = m_nodes[nInd].center;
    return
        ((pos.x > c.x) << 2) |
        ((pos.y > c.y) << 1) |
        ((pos.z > c.z));
}

glm::vec4 Tree::getCenter(int nInd, int quad) {
    const glm::vec4& c = m_nodes[nInd].center;
    const float off = m_nodes[nInd].size * 0.25f; 

    return glm::vec4(
            c.x + ((quad & 4) ? +off : -off),
            c.y + ((quad & 2) ? +off : -off),
            c.z + ((quad & 1) ? +off : -off),
            0
            );
}

void Tree::calculateForces(float theta, float G, float epsilon2) {
    for (int i = 0; i < static_cast<int>(m_data->size()); ++i) {
        glm::vec4 force = traverseNode(i, 0, theta, G, epsilon2);
        m_data->accels[i] = (force / m_data->positions[i].w); 
    } 
}

glm::vec4 Tree::traverseNode(int bodyIndex, int nodeIndex, float theta, float G, float epsilon2) {
    const Node& node = m_nodes[nodeIndex];

    if (node.body == -1) return glm::vec4(0.f);

    glm::vec4 diff = node.cm - m_data->positions[bodyIndex];
    diff.w = 0;
    const float d2 = glm::dot(diff, diff) + epsilon2;

    if (node.body >= 0) {
        if (node.body == bodyIndex) return glm::vec4(0.f);

        const float invD3 = 1.f / (d2 * std::sqrt(d2));
        return G * node.mass * m_data->positions[bodyIndex].w * invD3 * diff;
    }

    const float d = std::sqrt(d2);
    if (node.size / d < theta) {
        const float invD3 = 1.f / (d2 * std::sqrt(d2));
        return G * node.mass * m_data->positions[bodyIndex].w * invD3 * diff;
    }

    glm::vec4 force(0.f);
    for (int c : node.children) {
        if (c != -1) force += traverseNode(bodyIndex, c, theta, G, epsilon2);
    }
    return force; 
}
