#include"incr.h"
#include <climits>

IncAlgo::IncAlgo(PointList& list, Initialization initialization, EdgeSelection edgeSelection) : PolygonGenerator(list)
{
  this->initialization = initialization;
  this->edgeSelection = edgeSelection;
};

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_2                                          Point;

int inter(CGAL::Segment_2<Kernel>, CGAL::Segment_2<Kernel> , CGAL::Point_2<Kernel> );
bool isReplaceable(Point_2, Segment_2, Polygon_2&);

std::vector<Point> SortPoints(std::string ,std::vector<Point>);
//struct to give the std::sort function so that it sorts based on y instead.
struct {
      bool operator()(Point a, Point b) const { return a.y() < b.y(); }
    } customLess;

struct {
      bool operator()(Point a, Point b) const { return a.y() > b.y(); }
    } customMore;

struct PurpleEdges{

  Point x;
  Point y;

};


std::vector<Segment_2> CheckPol(Polygon_2 ,Point ,int ,PurpleEdges);

PurpleEdges CheckHull(Polygon_2 ,Point ,int );

//function that finds the position of a visible edge and returns it.

int Edgeselection(Polygon_2 poly,Point p,std::vector<Segment_2> segs,int mode){
  int i=0;
  int pos=0;
  Polygon_2 triangle;
  Segment_2 seg;
  srand ( time(NULL) );
  int temp;
  if(segs.size()>2)
    temp=rand()%segs.size();
  else
    temp=0;
  if(mode==0)
    for(auto v2=poly.edges_begin();v2!=poly.edges_end();++v2,++i){

      seg=segs[temp];


      if (seg==v2[0])
        { pos=i+1;
        return pos;
       }


    }
  i=0;
  if(mode==2){
    int max=-1;
    for(auto v2=segs.begin();v2!=segs.end();++v2){

    triangle.push_back(p);
    triangle.push_back(v2[0][0]);
    triangle.push_back(v2[0][1]);
    if(max<=abs(triangle.area())){
      max=abs(triangle.area());
      seg=v2[0];
    }   
    }
    for(auto v2=poly.edges_begin();v2!=poly.edges_end();++v2,++i){
      if (seg==v2[0])
      { pos=i+1;
      return pos;
     }


    } 

  }
  i=0;
  if(mode==1){
    int min=INT_MAX;
    for(auto v2=segs.begin();v2!=segs.end();++v2){

    triangle.push_back(p);
    triangle.push_back(v2[0][0]);
    triangle.push_back(v2[0][1]);
    if(min>=abs(triangle.area())){
      min=abs(triangle.area());
      seg=v2[0];
    }
    }

  for(auto v2=poly.edges_begin();v2!=poly.edges_end();++v2,++i){
    if (seg==v2[0])
      { pos=i+1;

        return pos;
     }


  }


  }

  return -1;
}
Polygon_2 IncAlgo::generatePolygon(){

  std::vector <Point> vec;
  std::ostream_iterator< Point>  out( std::cout, "\n" );
  std::ofstream os("test.wkt");
  std::ofstream os1("test1.wkt");
  std::ofstream os2("test12.wkt");
  PurpleEdges edges;
  std::string mode;
  if(initialization==0)
    mode="1a";
  else if(initialization==1)
    mode="2a";
else   if(initialization==2)
   mode="1b";
else   if(initialization==3)
   mode="2b";
  
  vec=SortPoints(mode,list);

  Polygon_2 poly;
  Polygon_2 hull;

  poly.push_back(vec[0]);
  poly.push_back(vec[1]);
  poly.push_back(vec[2]);
 //insert the first 3 points in the polygon.

  if(!poly.is_simple()){
  
    std::cout<<"Not simple anymore error occured!!!"<<std::endl;

  }
  int i=0;
  int j=0;
  int count=0;
  int select=0;
  std::vector<Segment_2> points;
  int pos=1;
  //for every other point ,find the purple edges,then the visible edges in the polygon between them and insert one to the polygon based on the algo.
  for(auto v1=vec.begin()+3;v1!=vec.end();++v1,++i){
    CGAL::convex_hull_2( poly.begin(), poly.end() ,std::back_inserter(hull));
    edges=CheckHull(hull,v1[0],pos);

    points=CheckPol(poly,v1[0],pos,edges);
    i=0;

    j++;
    if(!points.empty()){
      pos=Edgeselection(poly,v1[0],points,edgeSelection);
      poly.insert(poly.vertices_begin()+pos,v1[0]);
      pos=pos-1;
    }
    }
  hull.clear();

  if(!poly.is_simple()){
  
    std::cout<<"Not simple anymore error occured!!!"<<std::endl;

  }

  
  
  CGAL::IO::write_polygon_WKT(os2,poly);
  return poly;



}
//intersection between a segment of a polygon and the line ,although its not used because its slower than isReplaceable.
int inter(CGAL::Segment_2<Kernel> seg, CGAL::Segment_2<Kernel> line, CGAL::Point_2<Kernel> p)
{
  int count=0;
  Point point;
  CGAL::Object result = CGAL::intersection(seg, line);
  if(seg[0]==p || seg[1]==p)
    return 0;
    
  if (const CGAL::Point_2<Kernel> *ipoint = CGAL::object_cast<CGAL::Point_2<Kernel> >(&result)) {
    
    if(*ipoint!=p)
      count=1;

  }

  return count;
}



//sort the points
std::vector<Point> SortPoints(std::string mode,std::vector<Point> v){

if(mode=="2a")
  std::sort(v.begin(),v.end());
else if(mode=="1a")
  std::sort(v.begin(), v.end(), std::greater<Point>());
else if(mode=="2b")
  std::sort(v.begin(), v.end(), customLess);
else if (mode =="1b")
  std::sort(v.begin(), v.end(), customMore);

return v;
}

//find the purple edges of a polygon by checking the convex hull.Return them.
PurpleEdges CheckHull(Polygon_2 hull,Point p,int pos){


  
  PurpleEdges res;
  int x;
  int y;

  bool test;

  for (auto vi = hull.edges_begin()+pos+1; vi != hull.edges_begin(); --vi){


   
    test=isReplaceable(p,vi[0],hull);
    
    if(test==false){
      res.y=vi[0][0];
    break;

  }

  }

  for (auto vi = hull.edges_begin()+pos; vi != hull.edges_end(); ++vi){

  
    test=isReplaceable(p,vi[0],hull);
    
    if(test==false){
      res.x=vi[0][0];
  break;

  }
  }
    return res;
}

//Check between the purple edges and return a list of all the visible edges. 
std::vector<Segment_2> CheckPol(Polygon_2 poly,Point p,int pos,PurpleEdges edges){

  
  std::vector<Segment_2> res;
  double x=0;
  double y=0;
  bool test;

  for (auto vi = poly.edges_begin()+pos; vi != poly.edges_begin(); --vi){


    
    test=isReplaceable(p,vi[0],poly);
    
    if(test==true){
 
      res.push_back(vi[0]);
    }
    if(vi[0][1]==edges.x)
      break;
    }

  for (auto vi = poly.edges_begin()+pos+1; vi != poly.edges_end(); ++vi){



 test=isReplaceable(p,vi[0],poly);
    
    if(test==true){
      res.push_back(vi[0]);
    }

    if(vi[0][1]==edges.y)
    break;
  }

  return res;
}

