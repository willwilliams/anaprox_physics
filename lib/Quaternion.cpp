#include "Quaternion.h"

Quaternion::Quaternion(precision r_in, precision i_in, precision j_in, precision k_in) : r(r_in), i(i_in), j(j_in), k(k_in)
{}

void Quaternion::normalise()
{
    precision len = r*r + i*i + j*j + k*k;

    // If the quaternion is zero, apply non rotation
    if (len == 0)
    {
        r=1;
        return;
    }

    precision inverserootlen = ((precision)1.0)/ prec_sqrt(len);
    r*=inverserootlen;
    i*=inverserootlen;
    j*=inverserootlen;
    k*=inverserootlen;
}

// Quaternion Rotation
void Quaternion::rotate(const Vector& v)
{
    Quaternion rotationQuaternion(0, v.x, v.y, v.z);
    *this *= rotationQuaternion;
}

Vector Quaternion::rotateByThisQuaternion(const Vector& vectorToRotate)
{
	// v' = qr * v * qr^(-1)
	// qr = *this
	Quaternion v(0, vectorToRotate.x, vectorToRotate.y, vectorToRotate.z);
	Quaternion qr_inv(r, -i, -j, -k);

	Quaternion result =  *this * v * qr_inv;
	return Vector(result.i, result.j, result.k);
}