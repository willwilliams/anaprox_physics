#pragma once

#include "RigidBody.h"
#include "Contact.h"
#include <list>

class CollisionSphere;
class CollisionBox;
class CollisionPlane;

class Collision
{
public:
    enum CollisionType {SPHERE, BOX, PLANE};
    CollisionType mType;
    RigidBody* object;

    Collision(CollisionType type, RigidBody* obj = NULL);
    virtual ~Collision();
    std::list<Contact*> getContacts(Collision* check);
    virtual std::list<Contact*> getContacts(CollisionSphere*) = 0;
    virtual std::list<Contact*> getContacts(CollisionBox*) = 0;
    virtual std::list<Contact*> getContacts(CollisionPlane*) = 0;
    void resolveContacts(std::list<Contact*>);
    virtual void update(Vector position, Quaternion orientation) = 0;
};