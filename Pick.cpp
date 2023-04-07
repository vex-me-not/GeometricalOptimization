#include "Pick.h"
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point;
typedef CGAL::Polygon_2<K> Polygon_2;
 
#include <CGAL/IO/WKT.h>
struct points{


  double a=0;
  double b=0;
};
//See if a given point is inside/outside/on the boundary of the polygon
points check_inside(Point pt, Point *pgn_begin, Point *pgn_end, K traits)
{ points temp;
  switch(CGAL::bounded_side_2(pgn_begin, pgn_end,pt, traits)) {
    case CGAL::ON_BOUNDED_SIDE :
      temp.a=1;

      break;
    case CGAL::ON_BOUNDARY:
      temp.b=1;
      break;
    case CGAL::ON_UNBOUNDED_SIDE:
      break;
  }

  return temp;
}

//Return the area using the formula
double pick(double a,double b){
  return a+ (b/2) -1;
}
//Calculate the points inside and on the boundary of the polygon by checking all the points in a square.A lot of computing time but couldnt find something
//faster.
double Pick(Polygon_2 poly){
  double a=0;
  double b=0;
  points add;

  int j=0;
  Point points[poly.size()] ;
  //store the vertices in a list to give to the algorithm
  for (auto vi = poly.vertices_begin(); vi != poly.vertices_end(); ++vi,++j)
    {
       points[j]=vi[0];
     }
  int xmin=CGAL::to_double(poly.left_vertex()[0][0]);
  int xmax=CGAL::to_double(poly.right_vertex()[0][0]);
  int ymin=CGAL::to_double(poly.bottom_vertex()[0][1]);
  int ymax=CGAL::to_double(poly.top_vertex()[0][1]);
//find the max and min values of x,y to make the square
  for(int i=xmin;i<=xmax;++i){
  for(int k=ymin;k<=ymax;++k)
    {add=check_inside(Point(i,k), points, points+poly.size(), K());
    a+=add.a;
    b+=add.b;
  }
 
}

  double temp=pick(a,b);
  return temp;
}
