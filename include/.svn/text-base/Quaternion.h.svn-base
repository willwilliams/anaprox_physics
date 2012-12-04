/**
 * Quaternion definition
 */

#pragma once
#include "Dataprecision.h" 
#include "Vector.h"

class Quaternion {
public:

    precision r, i, j, k;

    // Construct a quaternion
    Quaternion(precision r_in =0 , precision i_in = 0, precision j_in = 0, precision k_in = 0);

    void normalise();
    void rotate(const Vector& v);
	Vector rotateByThisQuaternion(const Vector& vectorToRotate);

    // Quaternion Multiplication
    void operator *=(const Quaternion& q)
    {
        Quaternion qc = *this;

        r = qc.r*q.r - qc.i*q.i - qc.j*q.j - qc.k*q.k;
        i = qc.r*q.i + qc.i*q.r + qc.j*q.k - qc.k*q.j;
        j = qc.r*q.j - qc.i*q.k + qc.j*q.r + qc.k*q.i;
        k = qc.r*q.k + qc.i*q.j - qc.j*q.i + qc.k*q.r;
    }

    Quaternion operator *(const Quaternion& q)
    {
        Quaternion qc = *this;
		return Quaternion(qc.r*q.r - qc.i*q.i - qc.j*q.j - qc.k*q.k,
							qc.r*q.i + qc.i*q.r + qc.j*q.k - qc.k*q.j,
							qc.r*q.j - qc.i*q.k + qc.j*q.r + qc.k*q.i,
							qc.r*q.k + qc.i*q.j - qc.j*q.i + qc.k*q.r);
    }
};