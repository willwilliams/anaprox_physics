#include "BVHLeafNode.h"
#include "BVHParentNode.h"
#include "World.h"

BVHLeafNode::BVHLeafNode(Collision *object, bool first, BVHParentNode *parent)
    : BVHNode(true, first, parent), mObject(object)
{
	buildBoundingSphere();
}

BVHLeafNode::~BVHLeafNode()
{
    remove();
}

bool BVHLeafNode::getCollision(BVHLeafNode *leaf)
{
    if(leaf->boundingSphere != NULL)
    {
        if(boundingSphere->Collides(*leaf->boundingSphere))
        {
            std::list<Contact*> contacts =  mObject->getContacts(leaf->mObject);
            mObject->resolveContacts(contacts);
			return true;
        }
		else
			return false;
    }
    else
    {
        if(boundingSphere->Collides(*(CollisionPlane*)leaf->mObject))
        {
            std::list<Contact*> contacts = mObject->getContacts(leaf->mObject);
            mObject->resolveContacts(contacts);
			return true;
        }
		else
			return false;
    }
}

bool BVHLeafNode::getCollision(BVHParentNode *node)
{
    return node->getCollision(this);
}

void BVHLeafNode::buildBoundingSphere()
{
    switch(mObject->mType)
    {
    case Collision::SPHERE:
        {
            boundingSphere = new CollisionSphere(*((CollisionSphere*) mObject));
            break;
        }
    case Collision::BOX:
        {
            boundingSphere = ((CollisionBox*)mObject)->getBoundingSphere();
            break;
        }
    case Collision::PLANE:
        boundingSphere = NULL;
    };
    if(parentNode != NULL)
        parentNode->buildBoundingSphere();
}

void BVHLeafNode::insert(BVHLeafNode *toInsert)
{
    BVHParentNode* newParent = new BVHParentNode(this, toInsert, firstChild, parentNode);
    if(parentNode != NULL)
        parentNode->newChild(firstChild, newParent);
    parentNode = newParent;
    firstChild = true;
    toInsert->parentNode = newParent;
    toInsert->firstChild = false;
    parentNode->buildBoundingSphere();
}

void BVHLeafNode::remove()
{
    if(parentNode != NULL)
    {
        parentNode->removeChild(firstChild);
    }
    else
    {
        World::getWorld()->nullRoot();
    }
    parentNode = NULL;
}