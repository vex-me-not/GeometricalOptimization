
#ifndef INCREMENTAL
#define INCREMENTAL


#include "PolygonGenerator.h"
#include "shared.h"
#include"Pick.h"


class IncAlgo : public PolygonGenerator{
private:
    Initialization initialization;
    EdgeSelection edgeSelection;
public:
    IncAlgo(PointList&, Initialization, EdgeSelection);
    virtual Polygon_2 generatePolygon();
};


#endif