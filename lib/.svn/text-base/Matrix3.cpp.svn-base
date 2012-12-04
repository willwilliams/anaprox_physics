#include "Matrix3.h"

Matrix3::Matrix3(precision m0, precision m1, precision m2, precision m3, precision m4, precision m5, precision m6, precision m7, precision m8)
{
    matrix[0] = m0;
    matrix[1] = m1;
    matrix[2] = m2;
    matrix[3] = m3;
    matrix[4] = m4;
    matrix[5] = m5;
    matrix[6] = m6;
    matrix[7] = m7;
    matrix[8] = m8;
}

// Invert this matrix
void Matrix3::invert()
{
    // Calculate determinant
    precision det = matrix[0]*matrix[4]*matrix[8] + matrix[3]*matrix[7]*matrix[2] + matrix[6]*matrix[1]*matrix[5]
    - matrix[0]*matrix[7]*matrix[5] -matrix[6]*matrix[4]*matrix[2] - matrix[3]*matrix[1]*matrix[8];

    // Inverse determinant
    if (det == (precision)0.0f) return;
    precision idet = 1/det;

    precision matrix0= idet * ( matrix[4]* matrix[8] -  matrix[5]* matrix[7]);
    precision matrix1= idet * ( matrix[2]* matrix[7] - matrix[1]* matrix[8]);
    precision matrix2= idet * ( matrix[1]* matrix[5]-  matrix[2]* matrix[4]);
    precision matrix3= idet * ( matrix[5]* matrix[6]- matrix[3]* matrix[8]);
    precision matrix4= idet * ( matrix[0]* matrix[8]-  matrix[2]* matrix[6]);
    precision matrix5= idet * ( matrix[2]* matrix[3]-  matrix[0]* matrix[5]);
    precision matrix6= idet * ( matrix[3]* matrix[7] -  matrix[4]* matrix[6]);
    precision matrix7= idet * ( matrix[1]* matrix[6] -  matrix[0]* matrix[7]);
    precision matrix8= idet * ( matrix[0]* matrix[4] -  matrix[1]*  matrix[3]);

    matrix[0]=matrix0;
    matrix[1]=matrix1;
    matrix[2]=matrix2;
    matrix[3]=matrix3;
    matrix[4]=matrix4;
    matrix[5]=matrix5;
    matrix[6]=matrix6;
    matrix[7]=matrix7;
    matrix[8]=matrix8;
}

Matrix3 Matrix3::getInverse() const
{
    Matrix3 invertedMatrix;
    // Calculate determinant
    precision det = matrix[0]*matrix[4]*matrix[8] + matrix[3]*matrix[7]*matrix[2] + matrix[6]*matrix[1]*matrix[5]
    - matrix[0]*matrix[7]*matrix[5] -matrix[6]*matrix[4]*matrix[2] - matrix[3]*matrix[1]*matrix[8];

    // Inverse determinant
    if (det == (precision)0.0f)
        return Matrix3();
    precision idet = 1/det;

    invertedMatrix.matrix[0]= idet * ( matrix[4]* matrix[8] -  matrix[5]* matrix[7]);
    invertedMatrix.matrix[1]= idet * ( matrix[2]* matrix[7] - matrix[1]* matrix[8]);
    invertedMatrix.matrix[2]= idet * ( matrix[1]* matrix[5]-  matrix[2]* matrix[4]);
    invertedMatrix.matrix[3]= idet * ( matrix[5]* matrix[6]- matrix[3]* matrix[8]);
    invertedMatrix.matrix[4]= idet * ( matrix[0]* matrix[8]-  matrix[2]* matrix[6]);
    invertedMatrix.matrix[5]= idet * ( matrix[2]* matrix[3]-  matrix[0]* matrix[5]);
    invertedMatrix.matrix[6]= idet * ( matrix[3]* matrix[7] -  matrix[4]* matrix[6]);
    invertedMatrix.matrix[7]= idet * ( matrix[1]* matrix[6] -  matrix[0]* matrix[7]);
    invertedMatrix.matrix[8]= idet * ( matrix[0]* matrix[4] -  matrix[1]*  matrix[3]);

    return invertedMatrix;
}

// Set matrix to inverse of given matrix
void Matrix3::setInverse(Matrix3& m)
{
    Matrix3* temp = this;
    *this=m.getInverse();
    delete temp;
}

// Return the transpose of this matrix
Matrix3 Matrix3::getTranspose()
{
    return Matrix3(matrix[0],
        matrix[3],
        matrix[6],
        matrix[1],
        matrix[4],
        matrix[7],
        matrix[2],
        matrix[5],
        matrix[8]
    );
}

bool Matrix3::isZero()
{
    if(matrix[0] || matrix[1] || matrix[2] || matrix[3] || matrix[4] || matrix[5] || matrix[6] || matrix[7] || matrix[8])
        return false;
    else
        return true;
}

std::ostream& operator<<(std::ostream& os, const Matrix3& m)
{
    os << "[" << m.matrix[0] << " " << m.matrix[1] << " " << m.matrix[2] << "\n"
       << m.matrix[3] << " " << m.matrix[4] << " " << m.matrix[5] << "\n"
       << m.matrix[6] << " " << m.matrix[7] << " " << m.matrix[8] << "]\n";
    return os;
}