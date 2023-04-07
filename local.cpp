# include "local.h"

// Constructor 
LocalAlgo::LocalAlgo(Polygon_2& suboptimal,long convexHullArea ,double threshold, OptimizationType type, int length):PolygonOptimizer(suboptimal){
  this->convexHullArea=convexHullArea;
  this->threshold=threshold;
  this->type=type;
  this->length=length;
}


// Optimizer
Polygon_2 LocalAlgo::optimalPolygon(){
  Polygon_2 finalPoly=this->poly;
  
  long area=abs(finalPoly.area());
  long oldArea=area;

  int sizeBefore=finalPoly.size();

  // COUT<<"NOW STARTS OPTIMAL "<<ENDL;
  
  int length=this->length;

  if (length<=0 || length>=sizeBefore || length>10){
    if(length <=0 || length>10){    
      // COUT<<"L must range from 1 to 10 "<<ENDL;
    }

    if(length>=sizeBefore){
      // COUT<<"L must range from 1 to "<< finalPoly.size() -1<<ENDL;      
    }

    return finalPoly;
  }

  OptimizationType type=this->type; // the type of the improvement, min or max


  double score = (double)oldArea/(double)(this->convexHullArea); // the score of our polygon, as described in the paper provided

  double thres; // the threshold given by the command line
                          //Consider turning threshold into score +- 0.10

  if(this->type==minimization){

    if(sizeBefore<100){
      if(score<0.40){
        thres=0.20;
      }else{
        thres=score/2;
      }

    }else{
      thres = score - this->threshold;
    }

    if (thres<0.20){
      thres=0.20;
    }

  }else{

    if(sizeBefore<100){
      thres=0.84;
    }else{
      thres = score + this->threshold;
    }

    if(thres>0.84){
      thres=0.84;
    }


  }


  // COUT<<"THRESHOLD is "<<thres<<ENDL;
  // COUT<<"INITIAL SCORE IS "<<score<<ENDL;

  std::list<areaChange> possibleChanges; // The list of the changes to be applied at the suboptimal polygon IN OR OUT?
  
  // while the improvement between the old and the new polygon is not negligable
  while(checkThreshold(thres,score,type)){
    

    // We iterate over the edges of the polygon
    for (auto eit=finalPoly.edges_begin();eit!=finalPoly.edges_end();eit++){
      
      auto eitAfter=eit; // the edge after 
      auto eitBefore=eit; // the edge before
      
      getNextEdge(eitAfter,finalPoly);
      getPreviousEdge(eitBefore,finalPoly);
      
      Segment_2 edgy=*eit;
      Segment_2 edgyAfter=*eitAfter;
      Segment_2 edgyBefore=*eitBefore;

      int len=1;

      //We are going to iterate until we reach the given length
      while (len <= length){

        // For every vertex we are going to create chains of length len
        for(auto veit=finalPoly.vertices_begin();veit!=finalPoly.vertices_end();veit++){
          Polygon_2 candPoly=finalPoly; // A potential form that our polygon may turn into
          
          auto vveit=veit;
          std::vector<Point_2> vChain; // the chain with the vertices,its size is length
          
          // We get the vertices of the Chain
          for(int i=1;i<=len;i++){
            getNextIter(vveit,finalPoly);
            vChain.push_back(*vveit);
          }


          // if the chain we created does not have parts of it in the edge we are checking nor in its neighbours, we apply the changes 
          if(!chainInEdge(vChain,edgy) && !chainInEdge(vChain,edgyAfter) && !chainInEdge(vChain,edgyBefore)){

            applyChanges(candPoly,vChain,eit);  // we apply the change
            long ar=abs(candPoly.area());

            // we check for validity and improvement
            if(finalPoly.size()==candPoly.size() && areaImproves(ar,area,type) && candPoly.is_simple()){

             
              changePair ev; // we create a change pair to reprent the tuple (e,V)
              ev.e=eit;   //This is the edge we need to break
              ev.V=vChain; // This is the chain that it will be rerouted

              areaChange alteration; // In the list we need to save the area as well, in order to know which change is the best
              alteration.change=ev;
              alteration.area=ar;

              possibleChanges.push_back(alteration); // we save the change in the list of changes

            }else{

            }
          }
        }

        len++; // we move on to the next length
      }
    }

    // There is a chance that we found no elligalbe changes. We need to exit
    if(possibleChanges.empty()){
      score=thres;
    }

    // We sort the list depending on what type of optimization we want
    if(type==maximization){
      possibleChanges.sort(compareAlterMax);
    }else{
      possibleChanges.sort(compareAlterMin);
    }


    bool improved=false; // There is a chance that none of our changes our elligable.In this case our polygon may not improve


    
    //We iterate over the list of the potential changes we found before
    for(auto it=possibleChanges.begin();it!=possibleChanges.end();it++){
      Polygon_2 polyOnRoids=finalPoly;
      long areaEx=abs(finalPoly.area());

      Segment_2 edgy=*(it->change.e);

      // We check whether the edge we need to break is still in the polygon. If it's not we will not apply the change
      // Also we check whether any part of the chain is in the edge we need to break.If there is such part we won't apply the change
      // Furthermore, we check whether we reached the section with the already applied changes
      if((type==maximization && it->area==-1) || (type==minimization && it->area==this->convexHullArea) 
          || chainInEdge(it->change.V,edgy) || !findEdgeInPoly(finalPoly,edgy)){
        
        if(type==maximization && it->area==-1){
          break;
        }

        if(type==minimization && it->area==this->convexHullArea){
          break;
        }
      }else{
        
        applyChanges(polyOnRoids,it->change.V,it->change.e); // we apply the change
        long ar=abs(polyOnRoids.area());

        // And we check for validity and improvement
        if(sizeBefore==polyOnRoids.size() && areaImproves(ar,areaEx,type) && polyOnRoids.is_simple()){
          
          improved=true; // we actually improved our polygon
          
          score=(double)ar/(double)(this->convexHullArea); // the new score
          
          if(type==maximization){ // We mark the changes we applied, based on what kind of improvement we want
            it->area=-1;
          }else{
            it->area=this->convexHullArea;
          }
          
          finalPoly=polyOnRoids; // Our polygon becomes the new and improved one

          if(!checkThreshold(thres,score,type)){
            // it=possibleChanges.end();
            break;
          }
        }
      }

    }

    // If we haven't improved in this iteration, there is no need to keep looking
    if(!improved){
      score=thres;
    }else{ // If we have improved, we need to reorganize our list so that the applied changes are in the end
      if(type==maximization){
        possibleChanges.sort(compareAlterMax);
      }else{
        possibleChanges.sort(compareAlterMin);
      }      
    }
  }
  

  if(sizeBefore==finalPoly.size() && finalPoly.is_simple()){
    // COUT<<"DONE IMPROVING"<<ENDL;
    // COUT<<"FINAL SCORE IS "<< abs((double) finalPoly.area()/this->convexHullArea)<<ENDL;
  }else{
    
    if(sizeBefore!=finalPoly.size()){
      // COUT<<"POINTS MISSING: "<<sizeBefore-finalPoly.size()<<ENDL<<ENDL;
    }
  
    if(!finalPoly.is_simple()){
      // COUT<<"NEW POLYGON IS NOT SIMPLE"<<ENDL<<ENDL;
    }

  }

  return finalPoly;
}

// We check, based on what kind of optimization we want, whether we have surpassed our threshold
  bool checkThreshold(double& threshold, double& score, OptimizationType type){
    if(type==maximization){
      return (score<threshold);
    }else{
      return (score>threshold);
    }
  }

// Based on what kind of improvement we desire(min or max), we check whether the area has actually improved
  bool areaImproves(long& areaNew, long& areaOld, OptimizationType type){
    if(type==maximization){
      return (areaNew>areaOld);
    }else{
      return (areaNew<areaOld);
    }
  }

// We determine whether <vertex> belongs to <poly>
  bool findVertexInPoly(Polygon_2& poly, Point_2& vertex){
    for(auto veit=poly.vertices_begin();veit!=poly.vertices_end();veit++){
      if(*veit==vertex){
        return true;
      }
    }

    return false;
  }

// We determine whether <edge> belongs to <poly>
  bool findEdgeInPoly(Polygon_2& poly, Segment_2& edge){
    for(auto eit=poly.edges_begin();eit!=poly.edges_end();eit++){
      if(*eit==edge){
        return true;
      }
    }
    return false;
  }

//Function used to compare two areaChanges, when we minimize the polygon
  bool compareAlterMin(const areaChange& alter1, const areaChange& alter2){
    if(alter1.area!=alter2.area){
      return (alter1.area<alter2.area);
    }else{
      return !(alter1.change.V.size()<alter2.change.V.size());
    }
  }

//Function used to compare two areaChanges, when we maximize the polygon
  bool compareAlterMax(const areaChange& alter1, const areaChange& alter2){
    if(alter1.area!=alter2.area){
      return !(alter1.area<alter2.area);
    }else{
      return (alter1.change.V.size()<alter2.change.V.size());
    }
  }

// Checks if parts of vChain belong in edge edge
  bool chainInEdge(std::vector<Point_2>& vChain,Segment_2& edge){

    for(int i=0;i<vChain.size();i++){
      if(pointInEdge(vChain[i],edge)){
        return true;
      }
    }

    return false;
  }

// The function that applies the changes to the polygon
  void applyChanges(Polygon_2& poly, std::vector<Point_2>& vChain,Polygon_2::Edge_const_iterator& edge){

    Segment_2 edgy=*edge;
    Point_2 u2=edgy[1];

    // First we have to remove the points of the chain, in order for the detour to be created
    for(int i=0;i<vChain.size();i++){
      if(!pointInEdge(vChain[i],edgy)){
        auto veit=getVertexIt(vChain[i],poly);
        poly.erase(veit);      
      }

    }

    // We find the edge we need to break. Practically, we find its end, so the points may be added behind (thus "breaking" the edge)
    auto v2=getVertexIt(u2,poly);

    // We insert the points of the chain
    for(int i=vChain.size()-1;i>=0;i--){
      poly.insert(v2,vChain[i]);
    }

  }

// We check whether <vertex> belong to <edge>
  bool pointInEdge(Point_2& vertex,Segment_2& edge){
    return ( vertex==edge[0] || vertex==edge[1]);
  }

// We get the edge after eit in Poly. Returns an iterator
  void getNextEdge(Polygon_2::Edge_const_iterator& eit, Polygon_2& poly){

    if(eit!= poly.edges_end()-1 && eit!=poly.edges_end()){
      eit++;
    }else{
      eit=poly.edges_begin();
    }

  }

// We get the edge before eit in Poly. Returns an iterator
  void getPreviousEdge(Polygon_2::Edge_const_iterator& eit, Polygon_2& poly){

    if(eit!= poly.edges_begin()){
      eit--;
    }else{
      eit=poly.edges_end()-1;
    }

  }

// We get the vertex after veit in Poly. Returns an iterator
  void getNextIter(Polygon_2::Vertex_iterator& veit, Polygon_2& poly){

    if(veit!= poly.vertices_end()-1 && veit!=poly.vertices_end()){
      veit++;
    }else{
      veit=poly.vertices_begin();
    }

  }

// We get the vertex before veit in Poly. Returns an iterator
  void getPreviousIter(Polygon_2::Vertex_iterator& veit, Polygon_2& poly){

    if(veit!= poly.vertices_begin()){
      veit--;
    }else{
      veit=poly.vertices_end()-1;
    }

  }