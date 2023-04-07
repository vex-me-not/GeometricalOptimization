

#ifndef ANT_ALG
#define ANT_ALG
 

#include "PolygonOptimizer.h"
#include"Pick.h"


class Ant : public PolygonOptimizer{
private:
    AntParameters argFlags;
    PointList list;
public:
    Ant(AntParameters argFlags,PointList list,Polygon_2& poly);
    virtual Polygon_2 optimalPolygon();
};


#endif