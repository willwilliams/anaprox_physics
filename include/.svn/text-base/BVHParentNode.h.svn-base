#pragma once

#include "BVHNode.h"

class BVHParentNode : public BVHNode
{
public:
    BVHParentNode(BVHNode* firstChildNode, BVHNode* secondChildNode, bool first, BVHParentNode* parent);
    ~BVHParentNode();

    virtual void buildBoundingSphere();

    virtual void insert(BVHLeafNode* toInsert);
    void newChild(bool firstCh, BVHParentNode* child);
    void removeChild(bool firstCh);
    void removeChild(bool firstCh, BVHNode* other);

    virtual void getCollision();
    virtual bool getCollision(BVHLeafNode* leaf);
    virtual bool getCollision(BVHParentNode* node);

private:
    BVHNode* mFirst;
    BVHNode* mSecond;
};