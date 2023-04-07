#ifndef POLYGON_OPTIMIZER_H
#define POLYGON_OPTIMIZER_H

#include "shared.h"

class PolygonOptimizer
{
protected:
    Polygon_2& poly;  //constant pointer to the suboptimal polygon created.

public:
    PolygonOptimizer(Polygon_2& suboptimalPoly): poly(suboptimalPoly){};
    virtual Polygon_2 optimalPolygon() = 0;    //sub classes must implent this class
    virtual ~PolygonOptimizer(){};
};

#endif