#pragma once

#include "Collision.h"
#include "CollisionBox.h"
#include "CollisionPlane.h"
#include "Vector.h"
#include "Dataprecision.h"

class BVHLeafNode;

class CollisionSphere : public Collision
{
public:
    CollisionSphere(Vector centre_in = Vector(), precision radius_in = 0, RigidBody* obj = NULL);
    ~CollisionSphere();

    Vector centre;
    precision radius;

    bool Collides(const CollisionSphere& sphere);
    bool Collides(const CollisionPlane& plane);

    precision checkBoundingSphere(const CollisionSphere& sphere);
    CollisionSphere* getBoundingSphere(const CollisionSphere& sphere);

	// Sphere - Sphere contact genereation
    virtual std::list<Contact*> getContacts(CollisionSphere* sphere);

	// Sphere - Plane contact genereation
    virtual std::list<Contact*> getContacts(CollisionBox* box);

    virtual std::list<Contact*> getContacts(CollisionPlane* plane);
    virtual void resolveContacts(std::list<Contact*> conts);
    virtual void update(Vector position, Quaternion orientation);

private:
    BVHLeafNode* myNode;
};