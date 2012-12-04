#pragma once

#include "Collision.h"
#include "CollisionBox.h"
#include "CollisionSphere.h"
#include "Vector.h"
#include "Dataprecision.h"

class BVHLeafNode;

class CollisionPlane : public Collision
{
public:
    precision offset;
    Vector normal;

    CollisionPlane(precision offset_in, Vector normal_in = Vector());
    ~CollisionPlane();
    bool Collides(const CollisionSphere& sphere);
    virtual std::list<Contact*> getContacts(CollisionSphere* sphere);
    virtual std::list<Contact*> getContacts(CollisionBox* box);
    virtual std::list<Contact*> getContacts(CollisionPlane* plane);
    virtual void resolveContacts(std::list<Contact*> conts);
    virtual void update(Vector position, Quaternion orientation) {};

private:
    std::list<BVHLeafNode*>::iterator myNode;
};