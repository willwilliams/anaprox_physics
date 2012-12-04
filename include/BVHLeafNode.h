#pragma once

#include "BVHNode.h"

class BVHLeafNode : public BVHNode
{
public:
    Collision* mObject;

    BVHLeafNode(Collision* object, bool first, BVHParentNode* parent);
    ~BVHLeafNode();

    virtual void buildBoundingSphere();
    virtual void insert(BVHLeafNode* toInsert);
	void remove();

    virtual void getCollision() {};
    virtual bool getCollision(BVHLeafNode* leaf);
    virtual bool getCollision(BVHParentNode* node);
};