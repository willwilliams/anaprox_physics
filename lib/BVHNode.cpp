#include "BVHNode.h"
#include "BVHLeafNode.h"
#include "BVHParentNode.h"

BVHNode::BVHNode(bool leaf, bool first, BVHParentNode *parent)
    : isLeaf(leaf), firstChild(first), parentNode(parent), boundingSphere(NULL)
{}

BVHNode::~BVHNode()
{
    delete boundingSphere;
}

void BVHNode::getCollision(BVHNode* check)
{
    if(check->isLeaf)
        getCollision((BVHLeafNode*)check);
    else
        getCollision((BVHParentNode*)check);
}