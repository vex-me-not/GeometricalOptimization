#ifndef DEFAULT_HANDLER
#define DEFAULT_HANDLER

#include "shared.h"
#include "incr.h"
#include "ConvexHullAlgo.h"
#include "onion.h"
#include "ant.h"
#include "local.h"
#include "SimulatedAnnealing.h"
#include "AlgorithmHandler.h"


class DefaultHandler : public AlgorithmHandler{
private:
    
public:
    DefaultHandler(std::string filename): AlgorithmHandler(filename){};

    virtual double incrementalLocalSearch(OptimizationType type)
    {
        IncAlgo *generator = new IncAlgo(points, Initialization::a1, EdgeSelection::randomSelection);
        Polygon_2 initial = generator->generatePolygon();

        LocalAlgo *optimizer = new LocalAlgo(initial, convexHullArea, 0.10, type, 1);
        Polygon_2 optimal = (*optimizer).optimalPolygon();

        return abs(optimal.area()) / convexHullArea;
    }

    virtual double incrementalAnnealing(OptimizationType type)
    {
        IncAlgo *generator = new IncAlgo(points, Initialization::a1, EdgeSelection::randomSelection);
        Polygon_2 initial = generator->generatePolygon();

        SimulatedAnnealing *optimizer = new SimulatedAnnealing(initial, convexHullArea, 2500, type, AnnealingType::local);
        Polygon_2 optimal = (*optimizer).optimalPolygon();

        return abs(optimal.area()) / convexHullArea;
    }

    virtual double convexHullLocalSearch(OptimizationType type)
    {
        ConvexHullAlgo *generator = new ConvexHullAlgo(points, EdgeSelection::randomSelection);
        Polygon_2 initial = generator->generatePolygon();

        LocalAlgo *optimizer = new LocalAlgo(initial, convexHullArea, 0.10, type, 1);
        Polygon_2 optimal = (*optimizer).optimalPolygon();

        return abs(optimal.area()) / convexHullArea;
    }
     
    virtual double convexHullAnnealing(OptimizationType type)
    {
        ConvexHullAlgo *generator = new ConvexHullAlgo(points, EdgeSelection::randomSelection);
        Polygon_2 initial = generator->generatePolygon();

        SimulatedAnnealing *optimizer = new SimulatedAnnealing(initial, convexHullArea, 2500, type, AnnealingType::local);
        Polygon_2 optimal = (*optimizer).optimalPolygon();

        return abs(optimal.area()) / convexHullArea;
    }

    virtual double onionLocalSearch(OptimizationType type)
    {
        OnionAlgo *generator = new OnionAlgo(points, 3);
        Polygon_2 initial = generator->generatePolygon();

        LocalAlgo *optimizer = new LocalAlgo(initial, convexHullArea, 0.7, type, 5);
        Polygon_2 optimal = (*optimizer).optimalPolygon();

        return abs(optimal.area()) / convexHullArea;
    }

    virtual double onionAnnealing(OptimizationType type)
    {
        OnionAlgo *generator = new OnionAlgo(points, 1);
        Polygon_2 initial = generator->generatePolygon();

        SimulatedAnnealing *optimizer = new SimulatedAnnealing(initial, convexHullArea, 2500, type, AnnealingType::local);
        Polygon_2 optimal = (*optimizer).optimalPolygon();

        return abs(optimal.area()) / convexHullArea;
    }

    virtual double antColony(OptimizationType type)
    {
        AntParameters dummy;
        Polygon_2 dummyPoly;
        dummy.alpha=1;
        dummy.elitism=0;
        dummy.beta=3;
        dummy.L=2;
        dummy.optimizationType=type;
        dummy.ro=0.05;
        dummy.enable_breaks=0;
        dummy.divisor=2;

        
        Ant *optimizer = new Ant(dummy, points, dummyPoly);
        Polygon_2 optimal = (*optimizer).optimalPolygon();

        return abs(optimal.area()) / convexHullArea;
    }
};

#endif