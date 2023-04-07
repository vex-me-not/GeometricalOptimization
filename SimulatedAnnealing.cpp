#include "SimulatedAnnealing.h"
#include <random>
#include <algorithm>
#include <math.h>

using std::cout; using std::endl;

SimulatedAnnealing::SimulatedAnnealing(Polygon_2& initial, double cHullArea, int L, OptimizationType type, AnnealingType annType) : PolygonOptimizer(initial)
{
    this->optimizationType = type;
    this->L = L;
    this->annealingType = annType;
    this->n = initial.size();
    this->chpArea = cHullArea;
}

void initializeTree(Tree&, Polygon_2&);

Polygon_2 SimulatedAnnealing::optimalPolygon()
{
    std::srand(time(NULL));

    // this->poly.clear();

    Point_2 A(5,0), B(4,2), C(1,4), D(1, 1), E(2,3), F(3, 1.5), G(3, 0), H(0, 0), I(0.2, 2), J(0, 5), K(5,5);
    PointList points;
    points.push_back(A); points.push_back(B); points.push_back(C); points.push_back(D); points.push_back(E); points.push_back(F);  points.push_back(G);  points.push_back(H); points.push_back(I); points.push_back(J); points.push_back(K);

    /*for(PointListIterator iter = points.begin(); iter != points.end(); iter++)
        poly.push_back(*iter);

    std::ofstream dump("points.wkt"), dump2("initial.wkt");
    CGAL::IO::write_multi_point_WKT(
        dump,
        points
    );
    CGAL::IO::write_polygon_WKT(
        dump2,
        poly
    );*/

    switch (annealingType)
    {
    case local:
        return localAnnealing();
        break;
    
    case global:
        return globalAnnealing();
        break;
    
    case subdivision:
        return subdivisionAnnealing();
        break;
    default:
        return poly;
    }
}

double minX(Point_2 a, Point_2 b, Point_2 c, Point_2 d)
{
    double res = a[0];
    if(b[0] < res)res = b[0];
    if(c[0] < res)res = c[0];
    if(d[0] < res)res = d[0];
    return res;
}

double maxX(Point_2 a, Point_2 b, Point_2 c, Point_2 d)
{
    double res = a[0];
    if(b[0] > res)res = b[0];
    if(c[0] > res)res = c[0];
    if(d[0] > res)res = d[0];
    return res;
}

double minY(Point_2 a, Point_2 b, Point_2 c, Point_2 d)
{
    double res = a[1];
    if(b[1] < res)res = b[1];
    if(c[1] < res)res = c[1];
    if(d[1] < res)res = d[1];
    return res;
}

double maxY(Point_2 a, Point_2 b, Point_2 c, Point_2 d)
{
    double res = a[1];
    if(b[1] > res)res = b[1];
    if(c[1] > res)res = c[1];
    if(d[1] > res)res = d[1];
    return res;
}

Fuzzy_iso_box getQueryBox(Point_2 a, Point_2 b, Point_2 c, Point_2 d) 
{
    double xmin = minX(a, b, c, d);
    double xmax = maxX(a, b, c, d);
    double ymin = minY(a, b, c, d);
    double ymax = maxY(a, b, c, d);
    
    return Fuzzy_iso_box(Point_2(xmin, ymin), Point_2(xmax, ymax));
}

void getQueryResult(PointList& result, Fuzzy_iso_box query, Tree& tree)
{
    tree.search(std::back_inserter(result), query);
}

void initializeTree(Tree& tree, Polygon_2& poly)
{
    PointListIterator begin = poly.vertices_begin();
    PointListIterator end = poly.vertices_end();
    
    for(PointListIterator iter = begin; iter != end; ++iter)
    {
        tree.insert(*iter);
    }
}

Segment_2 nullSegment(){return Segment_2(Point_2(0,0), Point_2(0,0));}

Segment_2 SimulatedAnnealing::getEdgeFromSource(Point_2 source)
{
    PointListIterator begin = poly.vertices_begin();
    PointListIterator end = poly.vertices_end();
    
    Point_2 target;
    for(auto iter = begin; iter != end; iter++)
    {
        if(*iter == source)
        {
            if(iter+1 == end)
                target = *begin;
            else    
                target = *(iter+1);

            return Segment_2(source, target);
        }
    }
    return nullSegment();
}

Segment_2 SimulatedAnnealing::getEdgeFromTarget(Point_2 target)
{
    PointListIterator begin = poly.vertices_begin();
    PointListIterator end = poly.vertices_end();
    
    Point_2 source;
    for(auto iter = begin; iter != end; iter++)
    {
        if(*iter == target)
        {
            if(iter == begin)
                source = *(end - 1);
            else    
                source = *(iter - 1);

            return Segment_2(source, target);
        }
    }
    return nullSegment();
}

bool SimulatedAnnealing::validityLocal(Point_2 q, Point_2 r, Point_2 s, Point_2 p, Tree& tree)
{

    Segment_2 edgePR = Segment_2(p, r);
    Segment_2 edgeQS = Segment_2(q, s);

    //if new segments intersect each other, return false
    if(CGAL::do_intersect(edgePR, edgeQS))
        return false;
    
    //check if the new segments intersect another edge with the tree
    
    Fuzzy_iso_box query = getQueryBox(p, q, r, s);

    //get points in box
    PointList result;
    getQueryResult(
        result,
        query,
        tree
    );

    //for every edge that has a point in the list, check for intersection
    for(PointListIterator iter = result.begin(); iter != result.end(); ++iter)
    {
        Point_2 testPoint = *iter;
        Segment_2 s1 = getEdgeFromSource(testPoint);
        Segment_2 s2 = getEdgeFromTarget(testPoint);

        Point_2 s1Source = s1.source();
        Point_2 s1Target = s1.target();

        Point_2 s2Source = s2.source();
        Point_2 s2Target = s2.target();

        //s1 and pr
        if
        (
            CGAL::do_intersect(s1, edgePR) &&
            s1Source != p && s1Source != r &&
            s1Target != p && s1Target != r
        )
        {
            return false;
        }
            
        //s1 and qs
        if
        (
            CGAL::do_intersect(s1, edgeQS) &&
            s1Source != q && s1Source != s &&
            s1Target != q && s1Target != s
        )
        {
            return false;
        }

        //s2 and pr
        if
        (
            CGAL::do_intersect(s2, edgePR) &&
            s2Source != p && s2Source != r &&
            s2Target != p && s2Target != r
        )
        {
            return false;    
        }

        //s2 and qs
        if
        (
            CGAL::do_intersect(s2, edgeQS) &&
            s2Source != q && s2Source != s &&
            s2Target != q && s2Target != s
        )
        {
            return false;
        }

    }

    return true;

}

bool SimulatedAnnealing::validityGlobal(Point_2 q, Point_2 r, Point_2 s, Point_2 p, Point_2 t)
{
    Segment_2 edgePR(p, r);
    Segment_2 edgeSQ(s, q);
    Segment_2 edgeQT(q, t);
    
    if
    (
        CGAL::do_intersect(edgePR, edgeSQ) ||
        CGAL::do_intersect(edgePR, edgeQT)
    )
    {
        return false;
    }

    PointListIterator begin = poly.vertices_begin();
    PointListIterator end = poly.vertices_end();
    PointListIterator iter;
    Point_2 source, target;
    Segment_2 currEdge;
    
    for(iter = begin; iter != end; ++iter)
    {
        source = *iter;
        target = (iter == end - 1) ? *begin : *(iter+1);
        currEdge = Segment_2(source, target);

        if
        (
            CGAL::do_intersect(edgePR, currEdge) &&
            (source != p && source != r) &&
            (target != p && target != r)
        )
        {
            return false;
        }
        if
        (
            CGAL::do_intersect(edgeSQ, currEdge) &&
            (source != s && source != q) &&
            (target != s && target != q)
        )
        {
            return false;
        }
        if
        (
            CGAL::do_intersect(edgeQT, currEdge) &&
            (source != q && source != t) &&
            (target != q && target != t)
        )
        {
            return false;
        }

        
    }

    return true;

}

Polygon_2 SimulatedAnnealing::localAnnealing()
{
    Tree tree;
    initializeTree(tree, poly);

    double T = 1;
    Point_2 q, r, s, p;
    PointListIterator qIndex, rIndex, sIndex, pIndex;

    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(0.0,1.0);

    int iteration = 1;
    while(T > 0)
    {
        double energyInitial = getEnergy();
        PointListIterator begin = poly.vertices_begin();
        PointListIterator end = poly.vertices_end();

        int selection;

        //get random valid transition
        do
        {
            selection = rand()%n;
            qIndex = begin + selection;
            rIndex = qIndex + 1; if(rIndex == end) rIndex = begin;
            sIndex = rIndex + 1; if(sIndex == end) sIndex = begin;
            pIndex = (selection == 0) ? end - 1 : qIndex - 1;

            q = *(qIndex);
            r = *(rIndex);
            s = *(sIndex);
            p = *(pIndex);

            selection = (selection + 1) % n;
        }while(!validityLocal(q, r, s, p, tree));

        //make transition
        *rIndex = q;
        *qIndex = r;

        double energyFinal = getEnergy();
        double DE = energyFinal - energyInitial;

        //if energy increased, and Metropolis criterion doesn't hold revert change
        if(DE >= 0)
        {
            if(exp(-(DE/T)) < distribution(generator))
            {
                *rIndex = r;
                *qIndex = q;
            }
                
        }
        
        T = T - (1 / (double)L);    
    }
    return poly;
}

Polygon_2 SimulatedAnnealing::globalAnnealing()
{
    double T = 1;

    Point_2 q, r, s, p, t;
    PointListIterator qIndex, rIndex, sIndex, pIndex, tIndex;
    int selectionQ, selectionS;

    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(0.0,1.0);

    while(T > 0)
    {
        double energyInitial = getEnergy();
        PointListIterator begin = poly.vertices_begin();
        PointListIterator end = poly.vertices_end();

        //get random valid transition
        do
        {
            //select random q and s
            selectionQ = rand()%n;
            do{selectionS = rand()%n;}while(selectionS == selectionQ);
            
            //get q, p, r, s and t points
            qIndex = begin + selectionQ;
            rIndex = qIndex + 1; if(rIndex == end) rIndex = begin;
            pIndex = (selectionQ == 0) ? end - 1 : qIndex - 1;
            
            sIndex = begin + selectionS;
            tIndex = sIndex + 1; if(tIndex == end) tIndex = begin;
            
            q = *(qIndex);
            r = *(rIndex);
            s = *(sIndex);
            p = *(pIndex);
            t = *(tIndex);

        }while(!validityGlobal(q, r, s, p, t));

        Polygon_2 temp = this->poly;
        moveVertex(qIndex, tIndex, this->poly);

        double energyFinal = getEnergy();
        double DE = energyFinal - energyInitial;

        //if energy increased, and Metropolis criterion doesn't hold revert change
        if(DE >= 0)
        {
            if(exp(-(DE/T)) < distribution(generator))
            {
                moveVertex(tIndex, qIndex, this->poly);
            }
        }

        T = T - (1 / (double) L);
    }
    return poly;
}

Polygon_2 SimulatedAnnealing::subdivisionAnnealing()
{
    for(int annealingIteration = 0; annealingIteration < L; annealingIteration++)
    {
        cout << "subdivision: "  << annealingIteration << endl;
    }
    return poly;
}

void SimulatedAnnealing::moveVertex(PointListIterator from, PointListIterator to, Polygon_2& poly)
{
    if(to == poly.vertices_end())
        to = poly.vertices_begin();
    
    Point_2 p = *(from);
    poly.erase(from);
    poly.insert(to, p);
    return;
}

double SimulatedAnnealing::getEnergy()
{
    if(this->optimizationType == maximization)
        return maximizationEnergy();
    else
        return minimizationEnergy();
}

double SimulatedAnnealing::minimizationEnergy()
{
    return this->n * (polygonArea() / this->chpArea);
}

double SimulatedAnnealing::maximizationEnergy()
{
    return this->n * (1 - (polygonArea() / this->chpArea));
}

double SimulatedAnnealing::polygonArea()
{
    return abs(this->poly.area());
}