#include "Vector.h"

Vector::Vector(precision xi, precision yi, precision zi) :
x(xi), y(yi), z(zi) {
}

void Vector::negateVector()
{
    x = -x;
    y = -y;
    z = -z;
}

precision Vector::sqr_magnitude()
{
    return (x * x + y * y + z * z);
}

precision Vector::magnitude()
{
    return prec_sqrt(x * x + y * y + z * z);
}

void Vector::normalise()
{
	precision mag = magnitude();
    if (mag!= 0) {
        x = x / mag;
        y = y / mag;
        z = z / mag;
    }
}

Vector Vector::componentProduct(const Vector& vec)
{
    return Vector(x * vec.x, y * vec.y, z * vec.z);
}

void Vector::componentProductApply(const Vector& vec)
{
    x *= vec.x;
    y *= vec.y;
    z *= vec.z;
}

precision Vector::scalarProduct(const Vector& vec)
{
    return ((x * vec.x) + (y * vec.y) + (z * vec.z));
}

Vector Vector::vectorProduct(const Vector& vec)
{
    return Vector(y * vec.z - z * vec.y, z * vec.x - x * vec.z, x * vec.y
        - y * vec.x);
}

precision Vector::angleBetween(Vector& vec2)
{
    return acos(scalarProduct(vec2) / (magnitude()
        * vec2.magnitude()));
}