#pragma once

#include "Collision.h"

class Collision;

class Contact
{
public:
	Contact(Vector pos, precision penDepth, Vector normal, Collision* o1, Collision* o2) :
	  position(pos), penetrationDepth(penDepth),
		contactNormal(normal), object1(o1), object2(o2){}

	// Contact point
    Vector position;

	// Depth of the penetration of the two objects
    precision penetrationDepth;

	// Normal of the contact
	Vector contactNormal;

	// The two offending objects who have decided to collide
    Collision* object1;
    Collision* object2;
};