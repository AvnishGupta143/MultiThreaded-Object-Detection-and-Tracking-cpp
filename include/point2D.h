#ifndef POINT_2D_H
#define POINT_2D_H

#include <math.h>
#include <memory>
// Class providing point object with x and y fields
class Point2D
{
  public:
  
    float x;
    float y;

    // Operator Overloading "" operator
    Point2D operator + (Point2D const &obj) 
    { 
        Point2D p; 
        p.x = x + obj.x; 
        p.y = y + obj.y;
        return p;
    }

    // Operator Overloading "-" operator
	Point2D operator - (Point2D const &obj) 
    { 
        Point2D p; 
        p.x = x - obj.x; 
        p.y = y - obj.y;
        return p; 
    }

    // Operator Overloading "/" operator
   	Point2D operator / (float const &obj) 
    { 
        Point2D p;
        p.x = x /obj; 
        p.y = y /obj;
        return p;
    }

    // Operator Overloading "*" operator
    Point2D operator * (float const &obj) 
    { 
        Point2D p;
        p.x = x * obj; 
        p.y = y * obj;
        return p;
    }
	
    // Operator Overloading "=" operator
    Point2D operator = (Point2D const &obj) 
    { 
        Point2D p;
        p.x = obj.x; 
        p.y = obj.y;
        return p;
    }

    // Function to take norm of the point
    float norm()
    { 
        return sqrt(x*x+y*y);
    } 

};

#endif