#include "CollisionSphere.h"
#include "CollisionBox.h"
#include "BVHLeafNode.h"
#include "BVHParentNode.h"
#include "World.h"
#include <cmath>
#include <ctime>

CollisionSphere::CollisionSphere(Vector centre_in, precision radius_in, RigidBody* obj)
    : centre(centre_in), radius(radius_in), Collision(SPHERE, obj)
{
    if(obj != NULL)
        myNode = World::getWorld()->insert(this);
    else
        myNode = NULL;
}

CollisionSphere::~CollisionSphere()
{
    if(myNode != NULL)
        myNode->remove();
    delete myNode;
}

bool CollisionSphere::Collides(const CollisionSphere &sphere)
{
     return ((centre - sphere.centre).magnitude() <= (radius + sphere.radius));
}

bool CollisionSphere::Collides(const CollisionPlane& plane)
{
	Vector normal = plane.normal;
	return (normal.scalarProduct(centre) - plane.offset) <= radius;
}

precision CollisionSphere::checkBoundingSphere(const CollisionSphere& sphere)
{
    Vector vectorBetween = centre - sphere.centre;
    return (vectorBetween.magnitude() + radius + sphere.radius)/2;
}

CollisionSphere* CollisionSphere::getBoundingSphere(const CollisionSphere& sphere)
{
	Vector sphereCentre = sphere.centre;
    Vector vectorBetween = sphereCentre - centre;
    precision newRadius = (vectorBetween.magnitude() + radius + sphere.radius)/2;
    vectorBetween.normalise();
    Vector newCentre = centre + vectorBetween*(newRadius - radius);
    return new CollisionSphere(newCentre, newRadius, NULL);
}

std::list<Contact*> CollisionSphere::getContacts(CollisionBox* box)
{
	std::list<Contact*> results;
	Vector closestPointOnBox(centre);

	Vector rel_xaxis = box->getAxis(0);
	Vector rel_yaxis = box->getAxis(1);
	Vector rel_zaxis = box->getAxis(2);

	//cout << rel_xaxis << rel_yaxis << rel_zaxis;

	Vector boxToSphere(centre - box->centre);

	//cout << "boxToSphere: " << boxToSphere << endl;

	precision xDist = rel_xaxis.scalarProduct(boxToSphere);
	precision yDist = rel_yaxis.scalarProduct(boxToSphere);
	precision zDist = rel_zaxis.scalarProduct(boxToSphere);

	//cout << "xDist " << xDist << endl;
	//cout << "yDist " << yDist << endl;
	//cout << "zDist " << zDist << endl;


	Vector relativeSphere(xDist, yDist, zDist);

	// Precheck (may return a false positive)
	if (std::abs(xDist)-radius > (box->halfLength.x) ||
		std::abs(yDist)-radius > (box->halfLength.y) || 
		std::abs(zDist)-radius > (box->halfLength.z))
	{
		return results;
	}

	// Find the closest point on the box (assuming a collision but check properly later)
	precision boxX = box->halfLength.x;
	if (xDist > (boxX)) closestPointOnBox.x = boxX;
	else if (xDist < -1*(boxX)) closestPointOnBox.x = -boxX;
	else closestPointOnBox.x=xDist;
	
	precision boxY = box->halfLength.y;
	if (yDist > (boxY)) closestPointOnBox.y = boxY;
	else if (yDist < -1*(boxY)) closestPointOnBox.y = -boxY;
	else closestPointOnBox.y=yDist;

	precision boxZ = box->halfLength.z;
	if (zDist > (boxZ)) closestPointOnBox.z = boxZ;
	else if (zDist < -1*(boxZ)) closestPointOnBox.z = -boxZ;
	else closestPointOnBox.z=zDist;

	//cout << "Closest point on box:" << closestPointOnBox;
	
	// See if closest point is inside sphere
	if ((relativeSphere-closestPointOnBox).magnitude() <= radius)
	{
		Vector worldContact = box->centre + rel_xaxis*closestPointOnBox.x + rel_yaxis*closestPointOnBox.y + rel_zaxis*closestPointOnBox.z;
		Vector normal = (centre-worldContact);
		normal.normalise();
		results.push_back(new Contact(closestPointOnBox + box->centre, radius - (relativeSphere-closestPointOnBox).magnitude(), normal, this, box));
	}
    return results;
}

std::list<Contact*> CollisionSphere::getContacts(CollisionSphere* sphere)
{
	std::list<Contact*> results;
	Vector vectorBetween = sphere->centre - centre;
	precision separatingDistance = (vectorBetween).magnitude();
	if (separatingDistance <= (precision)0.0 || separatingDistance >= (radius + sphere->radius))
		return results;

	Vector contactNormal = vectorBetween * ((precision)1.0/separatingDistance);

	results.push_back(new Contact(centre + (vectorBetween * 0.5),radius + sphere->radius - separatingDistance, contactNormal, this, sphere));
	return results;
}

std::list<Contact*> CollisionSphere::getContacts(CollisionPlane* plane)
{
	std::list<Contact*> results;
	precision separatingDistance = (plane->normal * centre) - radius - (plane->offset);
	if (separatingDistance>=0)
		return results;

	results.push_back(new Contact(centre - plane->normal *(separatingDistance + radius), -separatingDistance, (plane->normal * -1), this, plane));
	return results;
}

void CollisionSphere::resolveContacts(std::list<Contact*> conts)
{
    
}

void CollisionSphere::update(Vector position, Quaternion orientation)
{
    centre = position;
    /*if(std::rand() % 1000 == 0)
    {
        myNode->remove();
        World::getWorld()->insert(myNode);
    }*/
    myNode->buildBoundingSphere();
}