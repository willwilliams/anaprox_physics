/**
 * Simple particle physics definitions
 * Units: m/s
 */

#pragma once

#include "Vector.h"
//#include "Test.h"

class PointMass
{
public:

    PointMass(Vector position_in = Vector(), Vector velocity_in = Vector(), Vector acceleration_in = Vector(),
            precision invMass = 1, precision gravitationalForce = 30);

    // Unit to update positions over given timeframe
    void update(precision timeFrame);

    Vector getPosition();
	Vector getVelocity();
	Vector getAcceleration();
    void setPosition(Vector newPos);
    precision getInvMass();
    void addImpulse(Vector addedImpulse);
    void addForce(Vector force);

protected:
    // Gravitaional vector (unique for each particle)
    precision mGravity;

    // Inverse mass so that we can infinite mass will be just 0
    precision mInverseMass;

    // Position, first and second derivatives
    Vector mPosition, mVelocity, mAcceleration;
};