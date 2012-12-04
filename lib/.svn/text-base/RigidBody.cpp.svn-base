#include "RigidBody.h"
#include "Collision.h"

RigidBody::RigidBody(Vector position_in, Vector velocity_in, Vector acceleration_in, precision invMass, precision gravitationalForce,
                     Quaternion orientation_in, Vector angVel_in, Vector angAcc_in, Matrix3 invInertiaTensor, Vector centreOfMass)
                     : PointMass(position_in, velocity_in, acceleration_in, invMass, gravitationalForce), mOrientation(orientation_in), mInvInertiaTensor(invInertiaTensor), mCentreOfMass(centreOfMass), collisionModel(NULL)
{
    if(mInvInertiaTensor.isZero())
    {
        mAngVel = Vector();
        mAngAcc = Vector();
    }
    else
    {
        mAngVel = angVel_in;
        mAngAcc = angAcc_in;
    }
}

RigidBody::~RigidBody()
{
    delete collisionModel;
}

void RigidBody::update(precision timeFrame)
{
    PointMass::update(timeFrame);

    mAngVel += (mAngAcc * timeFrame);

    Quaternion q(0, mAngVel.x*timeFrame, mAngVel.y*timeFrame, mAngVel.z*timeFrame);

    q *= mOrientation;
    mOrientation.r += (q.r * 0.5);
    mOrientation.i += (q.i * 0.5);
    mOrientation.j += (q.j * 0.5);
    mOrientation.k += (q.k * 0.5);

    if(collisionModel != NULL)
        collisionModel->update(mPosition, mOrientation);
}

Quaternion RigidBody::getOrientation()
{
    return mOrientation;
}

void RigidBody::addAngVel(Vector addedAngVel)
{
    mAngVel += addedAngVel;
}

void RigidBody::addForce(Vector addedForce)
{
    PointMass::addForce(addedForce);
}

void RigidBody::addForce(Vector addedForce, Vector actingOn)
{
    PointMass::addForce(addedForce);

    Vector forceAround = (actingOn - mCentreOfMass).vectorProduct(addedForce);
    mAngAcc += (mInvInertiaTensor * forceAround);
}

Vector RigidBody::getAngVel()
{
	return mAngVel;
}

void RigidBody::setCollisionModel(Collision* model)
{
    collisionModel = model;
}

Matrix3 RigidBody::getInvTensor()
{
    return mInvInertiaTensor;
}