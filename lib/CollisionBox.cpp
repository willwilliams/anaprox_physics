#include "CollisionBox.h"
#include "BVHLeafNode.h"
#include "World.h"

CollisionBox::CollisionBox(RigidBody* obj, Vector centre_in, Vector halfLength_in, Quaternion orientation_in)
    : Collision(BOX, obj), centre(centre_in), halfLength(halfLength_in), q(orientation_in)
{
    myNode = World::getWorld()->insert(this);
}

CollisionBox::~CollisionBox()
{
    myNode->remove();
    delete myNode;
}

Vector CollisionBox::getAxis(int axisNumber)
{
	switch(axisNumber)
	{
		case 0:
			return q.rotateByThisQuaternion(Vector(1,0,0));
		case 1:
			return q.rotateByThisQuaternion(Vector(0,1,0));
		case 2:
			return q.rotateByThisQuaternion(Vector(0,0,1));
		default:
			return NULL;
	}
}

precision CollisionBox::projectToAxis(CollisionBox b, const Vector& axis)
{
	// Find all the component contributions in the direction of
	// the given axis
	return (b.halfLength.x * std::abs(b.getAxis(0).scalarProduct(axis))) +
		(b.halfLength.y * std::abs(b.getAxis(1).scalarProduct(axis)))+
		(b.halfLength.z * std::abs(b.getAxis(2).scalarProduct(axis)));
}

bool CollisionBox::overlaps(CollisionBox b1, CollisionBox b2, const Vector& axis)
{
	precision halfProjection1 = projectToAxis(b1, axis);
	precision halfProjection2 = projectToAxis(b2, axis);

	// Take the centres and project the third dimension onto axis
	precision p = std::abs((b2.centre - b1.centre).scalarProduct(axis));

	return (p < halfProjection1 + halfProjection2);
}

bool CollisionBox::earlyOut(CollisionBox b1, CollisionBox b2)
{
	Vector centre = b2.centre - b1.centre;
	return !(
		overlaps(b1, b2, b1.getAxis(0)) &&
		overlaps(b1, b2, b1.getAxis(1)) &&
		overlaps(b1, b2, b1.getAxis(2)) &&
		
		overlaps(b1, b2, b2.getAxis(0)) &&
		overlaps(b1, b2, b2.getAxis(1)) &&
		overlaps(b1, b2, b2.getAxis(2)) &&

		overlaps(b1, b2, b1.getAxis(0) % b2.getAxis(0)) &&
		overlaps(b1, b2, b1.getAxis(0) % b2.getAxis(1)) &&
		overlaps(b1, b2, b1.getAxis(0) % b2.getAxis(2)) &&
		overlaps(b1, b2, b1.getAxis(1) % b2.getAxis(0)) &&
		overlaps(b1, b2, b1.getAxis(1) % b2.getAxis(1)) &&
		overlaps(b1, b2, b1.getAxis(1) % b2.getAxis(2)) &&
		overlaps(b1, b2, b1.getAxis(2) % b2.getAxis(0)) &&
		overlaps(b1, b2, b1.getAxis(2) % b2.getAxis(1)) &&
		overlaps(b1, b2, b1.getAxis(2) % b2.getAxis(2))
		);
}

std::list<Contact*> CollisionBox::pointAndBox(CollisionBox* b1, Vector v, std::list<Contact*> conts)
{
	std::list<Contact*> result;
	Vector n;
	precision deepestPoint;

	// TODO: pack up the reused code
	Vector rel_xaxis = b1->getAxis(0);
	Vector rel_yaxis = b1->getAxis(1);
	Vector rel_zaxis = b1->getAxis(2);

	Vector boxToPoint(v - b1->centre);
	precision xDist = rel_xaxis.scalarProduct(boxToPoint);
	precision yDist = rel_yaxis.scalarProduct(boxToPoint);
	precision zDist = rel_zaxis.scalarProduct(boxToPoint);

	Vector relativePoint(xDist, yDist, zDist);

	// If the vertex is inside our box then a point-face 
	// calculation is needed. Take axis with shallowest
	// penetration

	// X-Axis
	precision x_depth = b1->halfLength.x - std::abs(relativePoint.x);
	if (x_depth<0) return result;
	else
	{
		deepestPoint = x_depth;
		n = rel_xaxis;
		if (relativePoint.x < 0) n = rel_xaxis * -1;
	}

	// Y-Axis
	precision y_depth = b1->halfLength.y - std::abs(relativePoint.y);
	if (y_depth<0) return result;
	else if (y_depth< deepestPoint)
	{
		deepestPoint = y_depth;
		n = rel_yaxis;
		if (relativePoint.y < 0) n = rel_yaxis * -1;
	}

	// Z-Axis
	precision z_depth = b1->halfLength.z - std::abs(relativePoint.z);
	if (z_depth<0) return result;
	else if (z_depth< deepestPoint)
	{
		deepestPoint = z_depth;
		n = rel_zaxis;
		if (relativePoint.z < 0) n = rel_zaxis * -1;
	}

	result.push_back(new Contact(v,	deepestPoint,	n, this, b1));

	return result;
}

CollisionSphere* CollisionBox::getBoundingSphere()
{
    return new CollisionSphere(centre, halfLength.magnitude());
}

std::list<Contact*> CollisionBox::getContacts(CollisionBox* box)
{
	std::list<Contact*> results;
	if (earlyOut(*this, *box)) return results;

	// for all verticies of A
	results =  pointAndBox(CollisionBox b1, const Vector& v, std::list<Contact*> conts);
	// for all verticies of B
	// retain the record with deepest penetration




    return results;
}

std::list<Contact*> CollisionBox::getContacts(CollisionPlane* plane)
{
	static precision vertexes[8][3] = {{1,1,1}, {-1,1,1}, {1,-1,1}, {1,1,-1}, {1,-1,-1}, {-1,1-1}, {-1,-1,1}, {-1,-1,-1}};

	std::list<Contact*> results;

	for (int i = 0; i < 8; i++)
	{
		// Find position of each vertex

		// Primitive declaration
		Vector vPos = Vector(vertexes[i][0], vertexes[i][1], vertexes[i][2]);
		// Scale to this box
		vPos.componentProductApply(halfLength);
		// Transform and offset
		vPos = q.rotateByThisQuaternion(vPos);
		vPos += centre;

		// Distance of Point in direction of the plane
		precision vDist = vPos.scalarProduct(plane->normal);

		// Find and return potential contact
		if (vDist <= (plane->offset))
		{
			Vector contactPoint = plane->normal;
			contactPoint *= ((plane->offset)-vDist)/2;
			contactPoint += vPos;

			results.push_back(new Contact(
				contactPoint,
				plane->offset - vDist,
				(plane->normal * -1), 
				this, 
				plane));
		}
	}
    return results;
}

std::list<Contact*> CollisionBox::getContacts(CollisionSphere* sphere)
{
	std::list<Contact*> results;
    return results;
}

void CollisionBox::resolveContacts(std::list<Contact*> conts)
{}

void CollisionBox::update(Vector position, Quaternion orientation)
{
    centre = position;
    /*if(std::rand() % 1000 == 0)
    {
        myNode->remove();
        World::getWorld()->insert(myNode);
    }*/
    myNode->buildBoundingSphere();
}