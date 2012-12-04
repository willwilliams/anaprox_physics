#include "BVHParentNode.h"
#include "BVHLeafNode.h"
#include "World.h"

BVHParentNode::BVHParentNode(BVHNode *firstChildNode, BVHNode *secondChildNode, bool first, BVHParentNode *parent)
    : BVHNode(false, first, parent), mFirst(firstChildNode), mSecond(secondChildNode)
{
	buildBoundingSphere();
    if(parent != NULL)
        parent->buildBoundingSphere();
}

BVHParentNode::~BVHParentNode()
{}

void BVHParentNode::buildBoundingSphere()
{
    CollisionSphere* newSphere = mFirst->boundingSphere->getBoundingSphere(*mSecond->boundingSphere);
    if(boundingSphere != NULL && boundingSphere->radius == newSphere->radius && boundingSphere->centre == newSphere->centre)
        delete newSphere;
    else
    {
        delete boundingSphere;
        boundingSphere = newSphere;
        if(parentNode != NULL)
            parentNode->buildBoundingSphere();
    }
}

void BVHParentNode::insert(BVHLeafNode *toInsert)
{
    if(mFirst->boundingSphere->checkBoundingSphere(*toInsert->boundingSphere) < mSecond->boundingSphere->checkBoundingSphere(*toInsert->boundingSphere))
    {
        mFirst->insert(toInsert);
    }
    else
    {
        mSecond->insert(toInsert);
    }
}

void BVHParentNode::newChild(bool firstCh, BVHParentNode *child)
{
    if(firstCh)
        mFirst = child;
    else
        mSecond = child;
}

void BVHParentNode::removeChild(bool firstCh)
{
    if(firstCh)
    {
		mFirst->parentNode = NULL;
		mSecond->parentNode = parentNode;
        if(parentNode != NULL)
            parentNode->removeChild(firstChild, mSecond);
        else
            World::getWorld()->setRoot(mSecond);
    }
    else
    {
		mSecond->parentNode = NULL;
		mFirst->parentNode = parentNode;
        if(parentNode != NULL)
            parentNode->removeChild(firstChild, mFirst);
        else
            World::getWorld()->setRoot(mFirst);
    }
}

void BVHParentNode::removeChild(bool firstCh, BVHNode *other)
{
    if(firstCh)
    {
        delete mFirst;
        mFirst = other;
    }
    else
    {
        delete mSecond;
        mSecond = other;
    }
}

void BVHParentNode::getCollision()
{
    mFirst->getCollision(mSecond);
    mFirst->getCollision();
    mSecond->getCollision();
}

bool BVHParentNode::getCollision(BVHLeafNode *leaf)
{
    if(leaf->boundingSphere != NULL)
    {
        if(boundingSphere->Collides(*leaf->boundingSphere))
        {
            mFirst->getCollision(leaf);
            mSecond->getCollision(leaf);
			return true;
        }
		else
			return false;
    }
    else
    {
        if(boundingSphere->Collides(*(CollisionPlane*)leaf->mObject))
        {
            mFirst->getCollision(leaf);
            mSecond->getCollision(leaf);
			return true;
        }
		else
			return false;
    }
}

bool BVHParentNode::getCollision(BVHParentNode *node)
{
    if(boundingSphere->Collides(*node->boundingSphere))
    {
        mFirst->getCollision(node);
        mSecond->getCollision(node);
		return true;
    }
	else
		return false;
}