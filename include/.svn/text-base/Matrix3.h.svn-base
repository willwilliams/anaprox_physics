/**
 * 3X3 Matrix definition
 */

#pragma once

#include "Vector.h"
#include <iostream>

class Matrix3
{
public:
    precision matrix[9];

    // Construct matrix
    Matrix3(precision m0 = 0,
        precision m1 = 0,
        precision m2 = 0,
        precision m3 = 0,
        precision m4 = 0,
        precision m5 = 0,
        precision m6 = 0,
        precision m7 = 0,
        precision m8 = 0
        );

    void invert();
    Matrix3 getInverse() const;
    void setInverse(Matrix3& m);
    Matrix3 getTranspose();
    bool isZero();

    // 3X3 Matrix multiplication
    Matrix3 operator*(const Matrix3& m)
    {
        return Matrix3( matrix[0]*m.matrix[0] + matrix[1]*m.matrix[3] + matrix[2]*m.matrix[6],
                        matrix[0]*m.matrix[1] + matrix[1]*m.matrix[4] + matrix[2]*m.matrix[7],
                        matrix[0]*m.matrix[2] + matrix[1]*m.matrix[5] + matrix[2]*m.matrix[8],

                        matrix[3]*m.matrix[0] + matrix[4]*m.matrix[3] + matrix[5]*m.matrix[6],
                        matrix[3]*m.matrix[1] + matrix[4]*m.matrix[4] + matrix[5]*m.matrix[7],
                        matrix[3]*m.matrix[2] + matrix[4]*m.matrix[5] + matrix[5]*m.matrix[8],

                        matrix[6]*m.matrix[0] + matrix[7]*m.matrix[3] + matrix[8]*m.matrix[6],
                        matrix[6]*m.matrix[1] + matrix[7]*m.matrix[4] + matrix[8]*m.matrix[7],
                        matrix[6]*m.matrix[2] + matrix[7]*m.matrix[5] + matrix[8]*m.matrix[8]
                        );
    }

    //Multiply Matrix with Vector
    Vector operator*(const Vector& v)
    {
        precision v_x = matrix[0]*v.x+ matrix[1]*v.y + matrix[2]*v.z;
        precision v_y = matrix[3]*v.x+ matrix[4]*v.y + matrix[5]*v.z;
        precision v_z = matrix[6]*v.x+ matrix[7]*v.y + matrix[8]*v.z;
        return Vector(v_x, v_y, v_z);
    }
};