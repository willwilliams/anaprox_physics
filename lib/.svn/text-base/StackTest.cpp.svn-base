#include "CppUnitLite\TestHarness.h"
#include "Stack.h"
#include "Vector.h"
#include "Dataprecision.h"
#include "Matrix3.h"
#include "Quaternion.h"
#include "PointMass.h"
#include "RigidBody.h"
#include "CollisionSphere.h"
#include "CollisionPlane.h"
#include "BVHLeafNode.h"
#include "BVHParentNode.h"
#include "World.h"
#include <string>
#include <iostream>

SimpleString StringFrom(const std::string& value)
{
	return SimpleString(value.c_str());
}

TEST( Vector, headers)
{
  Vector v1;
  Vector v2(3,10,4);
  Vector v3(1,2,1);
  Vector v4(1,5,8);

  // Initialisation check
  CHECK(v1.x==0 && v1.y==0 && v1.z==0);

  // Magnitude check
  v2.normalise();
  precision a,b,c;
  a = 3/prec_sqrt(static_cast<precision>(125));
  b = 10.0/prec_sqrt(static_cast<precision>(125));
  c = 4/prec_sqrt(static_cast<precision>(125));

  CHECK(v2.x==a);
  CHECK(v2.y==b);
  CHECK(v2.z==c);

  // Scalar Product check
  CHECK(v3.scalarProduct(v4) == 19);
 
  // Vector product check
  Vector cross = v3.vectorProduct(v4);
  CHECK(cross.x==11 && cross.y==-7 && cross.z==3);

  // Component product check
  Vector component = v3.componentProduct(v4);
  CHECK(component.x==1);
  CHECK(component.y==10);
  CHECK(component.z==8);

  // Angle Between check
  precision angle = v3.angleBetween(v4);
  DOUBLES_EQUAL(angle, 0.6135141522, 6);

  SimpleString bc = "asa";
  CHECK_EQUAL("asa", bc);
}

TEST( Matrix3, headers)
{
	Matrix3 a;
	Matrix3 b(1,2,3,0,1,4,5,6,0);
	Matrix3 c(1,2,3,0,1,4,5,6,0);

	Matrix3 times1(1,2,-13,4,5,6,-7,8, 41);
	Matrix3 times2(7,2,1,8,9,1,2,-55,32);

	// Check correct construction
	CHECK(a.matrix[0]==0 && a.matrix[1]==0 && a.matrix[2]==0 && a.matrix[3]==0 && a.matrix[4]==0 &&
		a.matrix[5]==0 && a.matrix[6]==0 && a.matrix[7]==0 && a.matrix[8]==0);

	// Check correct invert
	b.invert();
	CHECK(b.matrix[0]==-24 && b.matrix[1]==18 && b.matrix[2]==5 && b.matrix[3]==20 && b.matrix[4]==-15 &&
		b.matrix[5]==-4 && b.matrix[6]==-5 && b.matrix[7]==4 && b.matrix[8]==1);

	// Check correct transpose
	Matrix3 d = c.getTranspose();
	CHECK(d.matrix[0]==1 && d.matrix[1]==0 && d.matrix[2]==5
		&& d.matrix[3]==2 && d.matrix[4]==1 && d.matrix[5]==6
		&& d.matrix[6]==3 && d.matrix[7]==4 && d.matrix[8]==0);

	// Verify overloaded * operator for matrix multiplication
	Matrix3 timesResult = times1*times2;
	CHECK(timesResult.matrix[0]==-3 && timesResult.matrix[1]==735  && timesResult.matrix[2]==-413);
	CHECK(timesResult.matrix[3]==80);
	CHECK(timesResult.matrix[4]==-277);
	CHECK(timesResult.matrix[5]==201 && timesResult.matrix[6]==97 && timesResult.matrix[7]==-2197
		&& timesResult.matrix[8]==1313);
}

TEST( Quaternion, headers)
{
	Quaternion q(13, 2, 1, 8);

	// Check normalised correctly
	precision divisor = prec_sqrt(static_cast<precision>(238));
	q.normalise();
	CHECK(q.r==static_cast<precision>(13/divisor));
	CHECK(q.i==static_cast<precision>(2/divisor));
	CHECK(q.j==static_cast<precision>(1/divisor));
	CHECK(q.k==static_cast<precision>(8/divisor));

	// Check quaternion multiplication
	Quaternion mult1(1,4,9,5);
	Quaternion mult2(10,2,14,1);
	mult1 *= mult2;

	CHECK(mult1.r==-129 && mult1.i==-19 && mult1.j==110 &&
		mult1.k==89);

	// Check rotation
	Vector rotationVector(2,4,8);
	Quaternion qr(3, 2, 1, 4);
	qr.rotate(rotationVector);

	CHECK(qr.r==-40 && qr.i==-2 && qr.j==4 && qr.k==30);
}

TEST( PointMass, headers)
{
	PointMass p1;

	// Check correct construction
	CHECK(p1.getPosition().x==0  && p1.getPosition().y==0 && 
		p1.getPosition().z==0);
	CHECK(p1.getVelocity().x==0 && p1.getVelocity().y==0 &&
		p1.getVelocity().z==0);
	CHECK(p1.getAcceleration().x==0 && p1.getAcceleration().y==-10 &&
		p1.getAcceleration().z==0);

	// Check impulse addition.
	// Impulse here is idealised to immediately change
	// the momentum of the object, i.e. a step change
	PointMass p2(Vector(), Vector(), Vector(), 0.25, 9.81);
	Vector impulse(7,16,2);
	p2.addImpulse(impulse);

	CHECK(p2.getVelocity().x==(7*0.25) && p2.getVelocity().y==4 &&
		p2.getVelocity().z==0.5);

	// Check force addition.
	Vector force(5,3,1);
	p2.addForce(force);

	CHECK(p2.getAcceleration().x==(5*0.25) && p2.getAcceleration().y==(-9.81+(3*0.25)) &&
		p2.getAcceleration().z==(0.25));
}

TEST( RigidBody, headers)
{
	// Check construction
	RigidBody rb(Vector(), Vector(), Vector(), 0.25);
	
	CHECK(rb.getPosition().x==0  && rb.getPosition().y==0 && 
		rb.getPosition().z==0);
	CHECK(rb.getVelocity().x==0 && rb.getVelocity().y==0 &&
		rb.getVelocity().z==0);
	CHECK(rb.getAcceleration().x==0 && rb.getAcceleration().y==-10 &&
		rb.getAcceleration().z==0);

	// Check force addition.
	Vector force(5,3,1);
	rb.addForce(force);

	CHECK(rb.getAcceleration().x==(5*0.25));
	CHECK(rb.getAcceleration().y==(-10+(3*0.25)));
	CHECK(rb.getAcceleration().z==(0.25));

	// Check angle addition
	rb.addAngVel(Vector(34,9,8));
	rb.addAngVel(Vector(-3,2,1));

	CHECK(rb.getAngVel().x==31 && rb.getAngVel().y==11 &&
		rb.getAngVel().z==9);

}

TEST( SphereToSphere, collisions)
{
	CollisionSphere* cs2= new CollisionSphere(Vector(15,0,0),5, NULL);
	CollisionSphere* cs1= new CollisionSphere(Vector(10.001,0,0),5, NULL);

	std::list<Contact*> results;
	results = cs1->getContacts(cs2);

	 list<Contact*>::iterator i;
	 i=results.begin();

	 // Check Contact Point with rounding errors
	 CHECK( (*i)->position.x<=12.5005001 && (*i)->position.x > 12.5004999);
	 CHECK( (*i)->position.y<=0.00001 && (*i)->position.y > -0.000001);
	 CHECK( (*i)->position.z<=0.00001 && (*i)->position.z > -0.000001);

	 // Check penetration depth
	 CHECK ( (*i)->penetrationDepth <= 5.001001  && (*i)->penetrationDepth > 5.00099999);

	 // Check normal
	 CHECK ( (*i)->contactNormal==Vector(1,0,0));

	 // *************** //

	CollisionSphere* cs4= new CollisionSphere(Vector(16,4,5),1, NULL);
	CollisionSphere* cs3= new CollisionSphere(Vector(10.001,0,0),1, NULL);

	std::list<Contact*> results2;
	results2 = cs3->getContacts(cs4);

	 list<Contact*>::iterator i2;
	 i2=results2.begin();

	 // Check that no contact was produced
	 CHECK(i2 == results2.end());

	 // ***************//

	 CollisionSphere* cs6= new CollisionSphere(Vector(0,0,0),4, NULL);
	 CollisionSphere* cs5= new CollisionSphere(Vector(4.01,4.01,4.01),4, NULL);

	 std::list<Contact*> results3;
	 results = cs5->getContacts(cs6);

	 list<Contact*>::iterator i3;
	 i3=results3.begin();

	 // Check that no contact was produced
	 CHECK(i3 == results3.end());

	 // *************** //

	 CollisionSphere* cs8= new CollisionSphere(Vector(0,0,0),4, NULL);
	 CollisionSphere* cs7= new CollisionSphere(Vector(3.9,3.9,3.9),4, NULL);

	 std::list<Contact*> results4;
	 results4 = cs7->getContacts(cs8);

	 list<Contact*>::iterator i4;
	 i4=results4.begin();

	 // Check Contact Point with rounding errors
	 CHECK( (*i4)->position.x<=1.950001 && (*i4)->position.x > 1.949999);
	 CHECK( (*i4)->position.y<=1.950001 && (*i4)->position.y > 1.949999);
	 CHECK( (*i4)->position.z<=1.950001 && (*i4)->position.z > 1.949999);

	 // Check penetration depth
	 CHECK ( (*i4)->penetrationDepth <= 1.246  && (*i4)->penetrationDepth > 1.244);

	 // Check normal
	 CHECK( (*i4)->contactNormal.x<=-0.57734 && (*i4)->contactNormal.x > -0.57736);
	 CHECK( (*i4)->contactNormal.y<=-0.57734 && (*i4)->contactNormal.y > -0.57736);
	 CHECK( (*i4)->contactNormal.z<=-0.57734 && (*i4)->contactNormal.z > -0.57736);
}

TEST( SphereToPlane, collisions)
{
	CollisionSphere* cs1= new CollisionSphere(Vector(0,1,0),2, NULL);
	CollisionPlane* cs2= new CollisionPlane(0, Vector(0,1,0));


	std::list<Contact*> results;
	results = cs1->getContacts(cs2);

	list<Contact*>::iterator i;
	i=results.begin();

	// Check contact point
	CHECK ((*i)->position==Vector(0,0,0));
	// Check penetration depth
	CHECK ((*i)->penetrationDepth == 1);
	// Check normal
	CHECK ((*i)->contactNormal==Vector(0,1,0));

	// *************** //

	CollisionSphere* cs3= new CollisionSphere(Vector(0,3,0),2, NULL);
	CollisionPlane* cs4= new CollisionPlane(0, Vector(0,1,0));


	std::list<Contact*> results2;
	results2 = cs3->getContacts(cs4);

	list<Contact*>::iterator i2;
	i2=results2.begin();

	CHECK (i2 == results2.end());

	// *************** //

	CollisionSphere* cs5= new CollisionSphere(Vector(0,0,0),2, NULL);
	CollisionPlane* cs6= new CollisionPlane(-1, Vector(-1,-1,-1));


	std::list<Contact*> results3;
	results3 = cs5->getContacts(cs6);

	list<Contact*>::iterator i3;
	i3=results3.begin();

	 // Check Contact Point with rounding errors
	 CHECK( (*i3)->position.x<=0.57736 && (*i3)->position.x > 0.57734);
	 CHECK( (*i3)->position.y<=0.57736 && (*i3)->position.y > 0.57734);
	 CHECK( (*i3)->position.z<=0.57736 && (*i3)->position.z > 0.57734);

	 // Check penetration depth
	 CHECK ( (*i3)->penetrationDepth <= 1.1  && (*i3)->penetrationDepth > 0.9);

	 // Check normal
	 CHECK( (*i3)->contactNormal.x<=-0.57734 && (*i3)->contactNormal.x > -0.57736);
	 CHECK( (*i3)->contactNormal.y<=-0.57734 && (*i3)->contactNormal.y > -0.57736);
	 CHECK( (*i3)->contactNormal.z<=-0.57734 && (*i3)->contactNormal.z > -0.57736);
}

TEST( BoxToPlane, collisions)
{
	RigidBody* rb = new RigidBody();
	CollisionBox* cb1 = new CollisionBox(rb, Vector(0,0,0), Vector(2.6, 2.6, 2.6), Quaternion(1, 0, 0, 0));
	CollisionPlane* cs1= new CollisionPlane(0, Vector(0,1,0));

	rb->setCollisionModel(cb1);

	std::list<Contact*> results;
	results = cb1->getContacts(cs1);

	list<Contact*>::iterator i;
	i=results.begin();

	// Check contact point
	CHECK ((*i)->position==Vector(2.6,-1.3,2.6));
	// Check penetration depth
	CHECK ((*i)->penetrationDepth == 2.6);
	// Check normal
	CHECK ((*i)->contactNormal==Vector(0,1,0));

	// *************** //

	RigidBody* rb2 = new RigidBody();
	CollisionBox* cb2 = new CollisionBox(rb2, Vector(0,0,0), Vector(2.6, 2.6, 2.6), Quaternion(0.92387995325, 0.3826834324, 0, 0));
	CollisionPlane* cs2= new CollisionPlane(0, Vector(0,1,0));

	rb2->setCollisionModel(cb2);

	std::list<Contact*> results2;
	results2 = cb2->getContacts(cs2);

	list<Contact*>::iterator i2;
	i2=results2.begin();

	// Check contact point
	CHECK (((*i2)->position.x > 2.59) && ((*i2)->position.x < 2.61));
	CHECK (((*i2)->position.y > -1.84) && ((*i2)->position.y < -1.83));
	CHECK (((*i2)->position.z > 0) && ((*i2)->position.z < 0.1));
	// Check penetration depth
	CHECK ((*i2)->penetrationDepth > 3.67 && (*i2)->penetrationDepth < 3.68);
	// Check normal
	CHECK ((*i2)->contactNormal==Vector(0,1,0));

	i2++;
	// Check contact point
	CHECK (((*i2)->position.x > 2.59) && ((*i2)->position.x < 2.61));
	CHECK (((*i2)->position.y > -0.01) && ((*i2)->position.y < 0.01));
	CHECK (((*i2)->position.z > -3.68) && ((*i2)->position.z < -3.67));
	// Check penetration depth
	CHECK ((*i2)->penetrationDepth > -0.01 && (*i2)->penetrationDepth < 0.01 );
	// Check normal
	CHECK ((*i2)->contactNormal==Vector(0,1,0));
}

TEST( SphereToBox, collisions)
{
	RigidBody* rb = new RigidBody();
	CollisionBox* cb1 = new CollisionBox(rb, Vector(0,0,0), Vector(1, 1, 1), Quaternion(1, 0, 0, 0));
	CollisionSphere* cs1= new CollisionSphere(Vector(0,1.1,0),0.2, NULL);

	std::list<Contact*> results;
	results = cs1->getContacts(cb1);

	list<Contact*>::iterator i;
	i=results.begin();

	// Check contact point
	CHECK ((*i)->position==Vector(0,1,0));
	// Check penetration depth
	CHECK ((*i)->penetrationDepth < 0.11 && (*i)->penetrationDepth > 0.09 );
	// Check normal
	CHECK ((*i)->contactNormal==Vector(0,1,0));

	// ************************** //
}

TEST( LeafTest, BoundingVolumeHierarchy)
{
	CollisionSphere* cs2= new CollisionSphere(Vector(15,0,0),5, NULL);
	CollisionSphere* cs1= new CollisionSphere(Vector(10.001,0,0),5, NULL);

	BVHLeafNode* ln1 = new BVHLeafNode(cs2, true, NULL);
	BVHLeafNode* ln2 = new BVHLeafNode(cs1, true, NULL);

	CHECK(ln1->getCollision(ln2));

	// ************* //

	CollisionSphere* cs4= new CollisionSphere(Vector(16,4,5),1, NULL);
	CollisionSphere* cs3= new CollisionSphere(Vector(10.001,0,0),1, NULL);

	BVHLeafNode* ln3 = new BVHLeafNode(cs4, true, NULL);
	BVHLeafNode* ln4 = new BVHLeafNode(cs3, true, NULL);

	// Check that no contact was produced
	CHECK(!ln3->getCollision(ln4));

	// ************* //

	CollisionSphere* cs6= new CollisionSphere(Vector(0,1,0),2, NULL);
	CollisionPlane* cs5= new CollisionPlane(0, Vector(0,1,0));

	BVHLeafNode* ln5 = new BVHLeafNode(cs6, true, NULL);
	BVHLeafNode* ln6 = new BVHLeafNode(cs5, true, NULL);

	// Check that no contact was produced
	CHECK(ln5->getCollision(ln6));

	// ************* //

	CollisionSphere* cs8= new CollisionSphere(Vector(0,3,0),2, NULL);
	CollisionPlane* cs7= new CollisionPlane(0, Vector(0,1,0));

	BVHLeafNode* ln7 = new BVHLeafNode(cs8, true, NULL);
	BVHLeafNode* ln8 = new BVHLeafNode(cs7, true, NULL);

	// Check that no contact was produced
	CHECK(!ln7->getCollision(ln8));
}

TEST(LeafInsert, BoundingVolumeHierarchy)
{
	CollisionSphere* cs2= new CollisionSphere(Vector(15,0,0),5, NULL);
	CollisionSphere* cs1= new CollisionSphere(Vector(10.001,0,0),5, NULL);

	BVHLeafNode* ln1 = new BVHLeafNode(cs2, true, NULL);
	BVHLeafNode* ln2 = new BVHLeafNode(cs1, true, NULL);

	ln1->insert(ln2);
	CHECK(ln1->parentNode != NULL);
	CHECK(ln2->parentNode != NULL);
	CHECK(ln1->parentNode == ln2->parentNode);

	new World();

	ln1->remove();
	CHECK(ln2->parentNode == NULL);
}

TEST(ParentCollision, BoundingVolumeHierarchy)
{
	CollisionSphere* cs2= new CollisionSphere(Vector(1,0,0),1, NULL);
	CollisionSphere* cs1= new CollisionSphere(Vector(3,0,0),1, NULL);
	CollisionSphere* cs3= new CollisionSphere(Vector(0,0,0),1, NULL);

	BVHLeafNode* ln1 = new BVHLeafNode(cs2, true, NULL);
	BVHLeafNode* ln2 = new BVHLeafNode(cs1, true, NULL);
	BVHLeafNode* ln3 = new BVHLeafNode(cs3, true, NULL);

	//Parent-Leaf collisions
	ln1->insert(ln2);
	BVHParentNode* pn1 = ln1->parentNode;

	CHECK(pn1->getCollision(ln3));

	CollisionSphere* cs4 = new CollisionSphere(Vector(2,2.5,0), 1, NULL);
	BVHLeafNode* ln4 = new BVHLeafNode(cs4, true, NULL);

	CHECK(pn1->getCollision(ln4));

	CollisionSphere* cs5 = new CollisionSphere(Vector(-1.5,0,0), 1, NULL);
	BVHLeafNode* ln5 = new BVHLeafNode(cs5, true, NULL);

	CHECK(!pn1->getCollision(ln5));

	//Parent-PlaneLeaf collision
	CollisionPlane* cp1= new CollisionPlane(0, Vector(0,1,0));
	BVHLeafNode* lp1 = new BVHLeafNode(cp1, true, NULL);

	CHECK(pn1->getCollision(lp1));

	CollisionPlane* cp2= new CollisionPlane(-3, Vector(0,1,0));
	BVHLeafNode* lp2 = new BVHLeafNode(cp2, true, NULL);

	CHECK(!pn1->getCollision(lp2));

	//Parent-Parent collision
	CollisionSphere* cs6 = new CollisionSphere(Vector(-1, 0, 0), 1, NULL);
	BVHLeafNode* ln6 = new BVHLeafNode(cs6, true, NULL);

	ln3->insert(ln6);
	BVHParentNode* pn2 = ln3->parentNode;

	CHECK(pn1->getCollision(pn2));

	CollisionSphere* cs7 = new CollisionSphere(Vector(-2, 0, 0), 1, NULL);
	BVHLeafNode* ln7 = new BVHLeafNode(cs7, true, NULL);

	ln5->insert(ln7);
	BVHParentNode* pn3 = ln5->parentNode;

	CHECK(!pn1->getCollision(pn3));
}