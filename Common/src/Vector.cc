#include "Vector.h"



Vector::Vector()
{
    this->x = 0;
    this->y = 0;
}

Vector::Vector(double inx, double iny)
{
    this->x = inx;
    this->y = iny;
}

Vector::Vector(const Vector& temp) 
{
    this->x = temp.x;
    this->y = temp.y;
}

double Vector::getNorm() const 
{
    return sqrt(x * x + y * y);
}

double Vector::calcDot(Vector &obj) const
{
    return x * obj.x + y * obj.y;
}

double Vector::calcAngle(Vector &obj) const
{
    double result_dot = calcDot(obj);
    double theta = fabs(acos(result_dot / (getNorm() * obj.getNorm())));

    double tempx = x * cos(theta) - y * sin(theta);
    double tempy = x * sin(theta) + y * cos(theta);

    if(fabs(tempx - obj.x) < 1e-6 && fabs(tempy - obj.y) < 1e-6)
    {
        return theta;
    }
    else
    {
        return -theta;
    }
}

double Vector::calcTheta(Vector &obj) const
{
    double result_dot = calcDot(obj);
    return fabs(acos(result_dot / (getNorm() * obj.getNorm())));
}

void Vector::setValueX(double inx)
{
    x = inx;
}

void Vector::setValueY(double iny)
{
    y = iny;
}

void Vector::normalize()
{
    double t_norm = getNorm();

    x /= t_norm;
    y /= t_norm;
}
