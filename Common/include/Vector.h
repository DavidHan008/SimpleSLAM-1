#ifndef _VECTOR_H_
#define _VECTOR_H_

#include <iostream>
#include <cmath>

using namespace std;

class Vector
{
    public:
        double x;
        double y;
        
    public:
        Vector();
        Vector(double, double);
        Vector(const Vector&);

        double getNorm() const;
        
        double calcDot(Vector &obj) const;
        double calcAngle(Vector &obj) const; // from 'this' to 'obj'
        double calcTheta(Vector &obj) const; 

        void setValueX(double);
        void setValueY(double);

        void normalize();
};

#endif // _VECTOR_H_