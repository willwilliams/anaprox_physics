/**
 * Contains all the mathematical definitions
 * and physics engine constructs
 */

#pragma once
#include "Dataprecision.h"
#include "Math.h"
#include <iostream>

using namespace std;

/**
 * Mathematical Vector in 3d
 */
class Vector {

public:
    Vector(precision xi = 0, precision yi = 0, precision zi = 0);

	//Values for the x, y and z axis.
	precision x;
	precision y;
	precision z;

private:
	//Padding for memory efficiency
	precision padding;

public:
	/**
	 * Vector functions
	 */

	void negateVector();
	precision sqr_magnitude();
    precision magnitude();
    void normalise();
    Vector componentProduct(const Vector& vec);
    void componentProductApply(const Vector& vec);
    precision scalarProduct(const Vector& vec);
    Vector vectorProduct(const Vector& vec);
    precision angleBetween(Vector& vec2);

	/**
	 * Overrides
	 */

	// *= CONSTANT INPUT - Times through all components
	void operator*=(const precision& multiple) {
		x *= multiple;
		y *= multiple;
		z *= multiple;
	}

	// * CONSTANT INPUT - Return new vector
	Vector operator*(const precision& multiple) {
		return Vector(multiple * x, multiple * y, multiple * z);
	}

	// * (Outer Product) VECTOR INPUT - Calculates scalar product
	precision operator*(const Vector& vec) {
		return x * vec.x + y * vec.y + z * vec.z;
	}

	// % (Inner product) VECTOR INPUT - Calculates vector product
	Vector operator%(const Vector& vec) const {
		return Vector(y * vec.z - z * vec.y, z * vec.x - x * vec.z, x * vec.y
				- y * vec.x);
	}

	// += VECTOR INPUT - Add to all components from another vector
	void operator+=(const Vector& vec) {
		x += vec.x;
		y += vec.y;
		z += vec.z;
	}

	// + VECTOR INPUT
	Vector operator+(const Vector& vec) {
		return Vector(vec.x + x, vec.y + y, vec.z + z);
	}

	// += CONSTANT INPUT
	void operator+=(const precision& val) {
		x += val;
		y += val;
		z += val;
	}

	// + CONSTANT INPUT
	Vector operator+(const precision& val) {
		return Vector(x + val, y + val, z + val);
	}

	// -= VECTOR INPUT - Add to all components from another vector
	void operator-=(const Vector& vec) {
		x -= vec.x;
		y -= vec.y;
		z -= vec.z;
	}

	// - VECTOR INPUT
	Vector operator-(const Vector& vec) {
		return Vector(x - vec.x, y - vec.y, z - vec.z);
	}

	// -= CONSTANT INPUT
	void operator-=(const precision& val) {
		x -= val;
		y -= val;
		z -= val;
	}

	// - CONSTANT INPUT
	Vector operator-(const precision& val) {
		return Vector(x - val, y - val, z - val);
	}

    bool operator==(const Vector& vec)
    {
        return (x == vec.x && y == vec.y && z == vec.z);
    }

	// Print out the vector's contents
	friend std::ostream& operator<<(std::ostream& os, const Vector& vec) {
		os << "Vector:-\n[" << vec.x << ", " << vec.y << ", " << vec.z << "]\n";
		return os;
	}
};