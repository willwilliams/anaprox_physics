#pragma once

#include "PointMass.h"
#include "Quaternion.h"
#include "Matrix3.h"

class Collision;

class RigidBody : public PointMass
{
public:
    RigidBody(Vector position_in = Vector(), Vector velocity_in = Vector(), Vector acceleration_in = Vector(),  precision invMass = 1, precision gravitationalForce = 10,
        Quaternion orientation_in = Quaternion(), Vector angVel_in = Vector(), Vector angAcc_in = Vector(), Matrix3 invInertiaTensor = Matrix3 (1, 0, 0, 0, 1, 0, 0, 0, 1), Vector centreOfMass = Vector());
    ~RigidBody();

    void update(precision timeFrame);
    Quaternion getOrientation();
    void addAngVel(Vector addedAngVel);
    void addForce(Vector addedForce);
    void addForce(Vector addedForce, Vector actingOn);
	Vector getAngVel();
    Matrix3 getInvTensor();
    void setCollisionModel(Collision* model);


private:
    //All in world space except the inverse inertia tensor
    Quaternion mOrientation;
    //In radians
    Vector mAngVel;
    Vector mAngAcc;
    Matrix3 mInvInertiaTensor;
    Vector mCentreOfMass;
    Collision* collisionModel;
};