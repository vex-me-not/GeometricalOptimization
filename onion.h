#ifndef ONION
#define ONION

#include "shared.h"
#include "PolygonGenerator.h"

#include <CGAL/Convex_hull_traits_adapter_2.h>
#include <CGAL/property_map.h>

#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Polygon_2_algorithms.h>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/intersections.h>
#include <CGAL/Polygon_set_2.h>
#include <CGAL/Boolean_set_operations_2.h>

#include <boost/optional/optional_io.hpp>

#include <vector>
#include <numeric>
#include <cstdlib>


#define ENDL std::endl 
#define COUT std::cout


typedef CGAL::Polygon_set_2<Kernel, std::vector<Kernel::Point_2>> Polygon_set_2;
typedef CGAL::Polygon_with_holes_2<Kernel> Polygon_with_holes_2;

typedef CGAL::Convex_hull_traits_adapter_2<Kernel,
          CGAL::Pointer_property_map<Point_2>::type > Convex_hull_traits_2;

class OnionAlgo : public PolygonGenerator{
private:
    int option; // the initialization option
public:
    OnionAlgo(PointList&, int);
    virtual Polygon_2 generatePolygon();

};

bool isVisible(Segment_2& initialEdge, Polygon_2& poly);
bool pointInPolygon(Point_2& point,Polygon_2& poly);

Point_2 getClosestK(Point_2& pointM,int& indexClosestK ,Polygon_2& poly);
Polygon_2::Vertex_iterator getVertexIt(Point_2& vertex,Polygon_2& poly);


int nextIndex(int&,Polygon_2&);
int previousIndex(int&,Polygon_2&);

#endif