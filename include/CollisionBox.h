#pragma once

#include "Collision.h"
#include "CollisionSphere.h"
#include "CollisionPlane.h"
#include "Vector.h"
#include "Dataprecision.h"
#include <list>

class BVHLeafNode;

class CollisionBox : public Collision
{
public:
    CollisionBox(RigidBody* obj, Vector centre_in = Vector(), Vector halfLength_in = Vector(), Quaternion orientation_in = Quaternion());
    ~CollisionBox();

    CollisionSphere* getBoundingSphere();
    virtual std::list<Contact*> getContacts(CollisionSphere* sphere);
    virtual std::list<Contact*> getContacts(CollisionBox* box);

    // Box - Plane contact genereation
	virtual std::list<Contact*> getContacts(CollisionPlane* plane);
    
	virtual void resolveContacts(std::list<Contact*> conts);
    virtual void update(Vector position, Quaternion orientation);
	

	// Box - Box functions
	Vector getAxis(int axisNumber);
	precision projectToAxis(CollisionBox b, const Vector& axis);
	bool overlaps(CollisionBox b1, CollisionBox b2, const Vector& axis);
	bool earlyOut(CollisionBox b1, CollisionBox b2);
	std::list<Contact*> pointAndBox(CollisionBox* b1, Vector v, std::list<Contact*> conts);



    Vector centre;
    Vector halfLength;
	Quaternion q;
private:
    BVHLeafNode* myNode;
};