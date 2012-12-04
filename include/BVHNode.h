#pragma once

#include <list>
#include "CollisionSphere.h"

class BVHLeafNode;
class BVHParentNode;

class BVHNode
{
public:
    bool isLeaf;
    CollisionSphere* boundingSphere;
    bool firstChild;
    BVHParentNode* parentNode;

    BVHNode(bool leaf, bool first, BVHParentNode* parent);
    virtual ~BVHNode();

    CollisionSphere* getBoundingSphere() {return boundingSphere;};

    void getCollision(BVHNode* check);
    virtual void getCollision() = 0;
    virtual bool getCollision(BVHLeafNode* leaf) = 0;
    virtual bool getCollision(BVHParentNode* node) = 0;

    virtual void insert(BVHLeafNode*) = 0;
    virtual void buildBoundingSphere() = 0;
};