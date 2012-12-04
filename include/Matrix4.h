/**
 * 3X4 Matrix definition
 * The fourth column is the additional translation
 * Otherwise, same as the 3X3 Matrix definition
 */

#pragma once

class Matrix4
{
public:

    // Holds the matrix transformation
    precision matrix[12];

    // Construct matrix

   Matrix43(precision m0 = 0,
        precision m1 = 0,
        precision m2 = 0,
        precision m3 = 0,
        precision m4 = 0,
        precision m5 = 0,
        precision m6 = 0,
        precision m7 = 0,
        precision m8 = 0,
        precision m9 = 0,
        precision m10 = 0,
        precision m11 = 0
        ) :
        matirx[0](m0),
        matirx[1](m1),
        matirx[2](m2),
        matirx[3](m3),
        matirx[4](m4),
        matirx[5](m5),
        matirx[6](m6),
        matrix[7](m7),
        matrix[8](m8),
        matirx[9](m9),
        matrix[10](m10),
        matrix[11](m11)
    {}

    // Methods
    precision getDeterminant() const;
    void invert();
    Matrix4 getInverse();
    void setInverse(Matrix4 m);

    // Transform by given this matrix.
    // Applies a translation after the rotation

    Vector operator*(const Vector& v)
    {
        return Vector(matrix[0]*v.x + matrix[1]*v.y + matrix[2]*v.z + matrix[3],
                        matrix[4]*v.x + matrix[5]*v.y + matrix[6]*v.z + matrix[7],
                        matrix[8]*v.x + matrix[9]*v.y + matrix[10]*v.z + matrix[11])
    }

    // Assuming bottom row is now 0,0,0,1 we have a 4X4
    // Perform multiplication on a given 3X4 with this 3X4

    Matrix4 operator*(const Matrix4& m)
    {
        return Matrix4(
            (matrix[0]*m.matrix[0] + matrix[1]*m.matrix[4] + matrix[2]*m.matrix[8]),
            (matrix[0]*m.matrix[1] + matrix[1]*m.matrix[5] + matrix[2]*m.matrix[9]),
            (matrix[0]*m.matrix[2] + matrix[1]*m.matrix[6] + matrix[2]*m.matrix[10]),
            (matrix[0]*m.matrix[3] + matrix[1]*m.matrix[7] + matrix[2]*m.matrix[11] + matrix [3]),

            (matrix[4]*m.matrix[0] + matrix[5]*m.matrix[4] + matrix[6]*m.matrix[8]),
            (matrix[4]*m.matrix[1] + matrix[5]*m.matrix[5] + matrix[6]*m.matrix[9]),
            (matrix[4]*m.matrix[2] + matrix[5]*m.matrix[6] + matrix[6]*m.matrix[10]),
            (matrix[4]*m.matrix[3] + matrix[5]*m.matrix[7] + matrix[6]*m.matrix[11] + matrix [7]),

            (matrix[8]*m.matrix[0] + matrix[9]*m.matrix[4] + matrix[10]*m.matrix[8]),
            (matrix[8]*m.matrix[1] + matrix[9]*m.matrix[5] + matrix[10]*m.matrix[9]),
            (matrix[8]*m.matrix[2] + matrix[9]*m.matrix[6] + matrix[10]*m.matrix[10]),
            (matrix[8]*m.matrix[3] + matrix[9]*m.matrix[7] + matrix[10]*m.matrix[11] + matrix [11])
            )
    }
}

precision Matrix4::getDeterminant() const
{
    return ( matrix[8]*matrix[5]*matrix[2] + 
             matrix[4]*matrix[9]*matrix[2] + 
             matrix[8]*matrix[1]*matrix[6] - 
             matrix[0]*matrix[9]*matrix[6] - 
             matrix[4]*matrix[1]*matrix[10] + 
             matrix[0]*matrix[5]*matrix[10]; 
}

//void Matrix4::invert() const
//{
//    // Here we pretent that the matrix has a bottom
//    // row with entries 0,0,0,1 to enable us to invert
//
//
//    precision det = getDeterminant();
//    // Inverse dne when determinant is zero
//    if (det == 0) return;
//    // Inverse determinant
//    precision idet = ((precision)1.0)/det;
//
//
//    precision matrix0 = idet * (matrix[5]*matrix[10] - matrix[9]*matrix[6]);
//    precision matrix1 = idet * (matrix[9]*matrix[2] - matrix[1]*matrix[10]);
//    precision matrix2 = idet * (matrix[1]*matrix[6]*matrix[15] - matrix[1]*matrix[10]);
//
//    precision matrix4 = idet * (matrix[8]*matrix[6] - matrix[4]*matrix[10]);
//}
