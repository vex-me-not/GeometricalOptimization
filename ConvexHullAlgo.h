#ifndef CONVEX_HULL_ALGO
#define CONVEX_HULL_ALGO

#include "shared.h"
#include "PolygonGenerator.h"


class ConvexHullAlgo : public PolygonGenerator{
private:
    EdgeSelection method;
public:
    ConvexHullAlgo(PointList&, EdgeSelection);
    virtual Polygon_2 generatePolygon();
};

#endif