#include "PointMass.h"


PointMass::PointMass(Vector position_in, Vector velocity_in, Vector acceleration_in,
                     precision inverseMass, precision gravitationalForce)
                     : mPosition(position_in), mInverseMass(inverseMass)
{
    if(mInverseMass == 0)
    {
        mVelocity = Vector();
        mAcceleration = Vector();
        mGravity = 0;
    }
    else
    {
        mVelocity = velocity_in;
        mGravity = gravitationalForce;
        mAcceleration = acceleration_in + (Vector(0, -1, 0) * mGravity);
    }
}

void PointMass::update(precision timeFrame)
{
    mVelocity += (getAcceleration()*timeFrame);
    // Update position
    mPosition += (getVelocity()*timeFrame);
}

Vector PointMass::getPosition()
{
    return mPosition;
}

void PointMass::addImpulse(Vector addedImpulse)
{
    mVelocity += (addedImpulse * mInverseMass);
}

void PointMass::addForce(Vector force)
{
    mAcceleration += (force * mInverseMass);
}

Vector PointMass::getVelocity()
{
	return mVelocity;
}

Vector PointMass::getAcceleration()
{
	return mAcceleration;
}

precision PointMass::getInvMass()
{
    return mInverseMass;
}

void PointMass::setPosition(Vector newPos)
{
    mPosition = newPos;
}