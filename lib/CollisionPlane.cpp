#include "CollisionPlane.h"
#include "BVHLeafNode.h"

CollisionPlane::CollisionPlane(precision offset_in, Vector normal_in)
    : Collision(PLANE, NULL), offset(offset_in), normal(normal_in)
{
    normal.normalise();
    myNode = World::getWorld()->insertPlane(this);
}

CollisionPlane::~CollisionPlane()
{
    World::getWorld()->removePlane(myNode);
}

std::list<Contact*> CollisionPlane::getContacts(CollisionSphere* sphere)
{
    
	std::list<Contact*> results;
    return results;
}

std::list<Contact*> CollisionPlane::getContacts(CollisionBox* box)
{
	std::list<Contact*> results;
    return results;
}

std::list<Contact*> CollisionPlane::getContacts(CollisionPlane* plane)
{
    
	std::list<Contact*> results;
    return results;
}

void CollisionPlane::resolveContacts(std::list<Contact*> conts)
{}