#ifndef SIMULATED_ANNEALING_ALGO
#define SIMULATED_ANNEALING_ALGO

#include "shared.h"
#include "PolygonOptimizer.h"


class SimulatedAnnealing : public PolygonOptimizer{
private:
    OptimizationType optimizationType;
    int L;
    AnnealingType annealingType;
    int n;
    double chpArea;

public:

    double polygonArea();
    double minimizationEnergy();
    double maximizationEnergy();
    double getEnergy();
    void moveVertex(PointListIterator, PointListIterator, Polygon_2&);

    bool validityLocal(Point_2, Point_2, Point_2, Point_2, Tree&);
    bool validityGlobal(Point_2, Point_2, Point_2, Point_2, Point_2);

    Segment_2 getEdgeFromSource(Point_2);
    Segment_2 getEdgeFromTarget(Point_2);

    Polygon_2 localAnnealing();
    Polygon_2 globalAnnealing();
    Polygon_2 subdivisionAnnealing();
    

    SimulatedAnnealing(Polygon_2&, double, int, OptimizationType, AnnealingType);
    virtual Polygon_2 optimalPolygon();
};

#endif