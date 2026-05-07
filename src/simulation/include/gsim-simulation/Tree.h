#pragma once

#include <vector>

#include "Node.h"
#include "ObjectData.h"

class Tree {
    public:
        Tree(ObjectData* data);
        void buildTree();
        void calculateForces(float theta, float G, float epsilon2);
    
    private:
        std::vector<Node> m_nodes;
        ObjectData* m_data;

        glm::vec4 traverseNode(int bodyIndex, int nodeIndex, float theta, float G, float epsilon2);
        void insertToNode(int objectIndex, int nodeIndex);
        int getQuadrant(int objectIndex, int nodeIndex);
        void computeMassDistribution();
        glm::vec4 getCenter(int nInd, int quad);
        void accumulateMass(int nodeIndex);

};

