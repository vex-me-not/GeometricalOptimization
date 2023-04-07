
#include "onion.h"
#include <time.h>


// Constructor
OnionAlgo::OnionAlgo(PointList& list, int option) : PolygonGenerator(list){this->option=option;};

// Function that actually finds the polygon
Polygon_2 OnionAlgo::generatePolygon(){

  srand(time(0));

  std::vector<Point_2> points=list;
  std::vector<Polygon_2> allPolys;

  // We iterate over the points,creating convex Hulls, until there are less than 3  points left
  while(points.size()>2){
    if(points.size()==3 && CGAL::collinear(points[0],points[1],points[2])){
      std::vector<Point_2>::iterator it = points.begin();
      points.erase(it+1); //Check for potential trouble
    }else{

      // the part that finds the convexHull of point set was taken verbatim from CGAL's user's manual about convexHulls
      std::vector<std::size_t> indices(points.size()), out;
      std::iota(indices.begin(), indices.end(),0);
      CGAL::convex_hull_2(indices.begin(), indices.end(), std::back_inserter(out),
                        Convex_hull_traits_2(CGAL::make_property_map(points)));
    

      Polygon_2 poly;
      std::vector<Point_2> pointsLeftAct;

    // loop that indicates the convexHull points
      for( std::size_t i : out){

        poly.push_back(points[i]); // push the point to a polygon
        points[i]=Point_2(-99,-99);   // mark the convex hull points
  
      }
      allPolys.push_back(poly);

    // we find the points that do not belong to the convexHull, whilst checking if a point belongs in 2 hulls
      for(int i =0; i<points.size();i++){
        if(points[i]!=Point_2(-99,-99) && !pointInPolygon(points[i],poly)){ // if a point is left unmarked this means that it does not belong to the convexHull
          pointsLeftAct.push_back(points[i]); // push it to the pointsLeftAct
        }
      }   
      points=pointsLeftAct;
    }

  }
  
  int criterion=this->option; // criterion that defines the value of m
  int m=0;

  if(criterion==1){
    m= rand()% allPolys[0].size();  // random m among all the available vertices of the first convex hull
  }else if(criterion==2){
    while(allPolys[0].vertex(m)!= *allPolys[0].left_vertex()){ //m is the vertex with the lowest x
      m++;
    }  
  }else if(criterion==3){
    while(allPolys[0].vertex(m)!= *allPolys[0].right_vertex()){ // m is the vertex with the highest x
      m++;
    }
  }else if(criterion==4){
    while(allPolys[0].vertex(m)!= *allPolys[0].bottom_vertex()){ // m is the vertex with the lowest y
      m++;
    }
  }else if(criterion==5){
    while(allPolys[0].vertex(m)!= *allPolys[0].top_vertex()){  // m is the vertex with the highest y
      m++;
    }
  }else{
    m=allPolys[0].size()/2; // Personal recommendation
  }

  int initialM=m;

  int mPlus=nextIndex(m,allPolys[0]);
  int mMinus=previousIndex(m,allPolys[0]);

  Polygon_2 finalPoly=allPolys[0];

  // we iterate over the available convex Hulls
  for(int i =0;i<allPolys.size();i++){

    Point_2 mVertex;
    Point_2 mVertexPlus;
    Point_2 mVertexMinus;

    mVertex = finalPoly.vertex(m);
    mVertexPlus=finalPoly.vertex(mPlus);
    mVertexMinus=finalPoly.vertex(mMinus);

    // if we are not at the lowest level
    if(i+1!=allPolys.size()){
      int indexClosestK=-1;
      Point_2 closestK=getClosestK(mVertex,indexClosestK,allPolys[i+1]);

      int indexLamda=nextIndex(indexClosestK,allPolys[i+1]);
      int indexBeforeK=previousIndex(indexClosestK,allPolys[i+1]);      
      
      Point_2 lamda=allPolys[i+1].vertex(indexLamda);  
      Point_2 BeforeKVertex=allPolys[i+1].vertex(indexBeforeK);  

      Segment_2 mToK(mVertex,closestK);
      Segment_2 mPlusToLamda(mVertexPlus,lamda);
      Segment_2 mMinusToKMinus(mVertexMinus,BeforeKVertex);

      // we have to ensure that k is visible from m
      // If not, we will have to choose a different m and therefore a different k(along with a different mPlus and lamda)
      while(!isVisible(mToK,allPolys[i+1])){
        mMinus=m;
        mVertexMinus=mVertex;

        m=nextIndex(m,finalPoly);
        mVertex=finalPoly.vertex(m);
        mPlus=nextIndex(m,finalPoly);
        mVertexPlus=finalPoly.vertex(mPlus);
        
        
        closestK=getClosestK(mVertex,indexClosestK,allPolys[i+1]);
        indexLamda=nextIndex(indexClosestK,allPolys[i+1]);
        indexBeforeK=previousIndex(indexClosestK,allPolys[i+1]);

        lamda=allPolys[i+1].vertex(indexLamda);
        BeforeKVertex=allPolys[i+1].vertex(indexBeforeK);

        mToK=Segment_2(mVertex,closestK);
        mPlusToLamda=Segment_2(mVertexPlus,lamda);
        mMinusToKMinus=Segment_2(mVertexMinus,BeforeKVertex);
      }

      
      Segment_2 edgeInPoly(mVertexPlus,lamda);
 
      // we check whether lamda is visible from m+1
      if(isVisible(edgeInPoly,allPolys[i+1])){
      }else{ // if not, based on the relation between m and mPlus(which is "infront") we have to either increase or decrease lamda
        int initM=m;
        int initMPlus=mPlus;       
        
        if((m<mPlus && m!=finalPoly.size()-1) ||(m>mPlus && m==finalPoly.size()-1) ){
          mPlus=previousIndex(m,finalPoly);

        }
        else{
          mPlus=nextIndex(m,finalPoly);     
        }
        mVertexPlus=finalPoly.vertex(mPlus);        
        
        Segment_2 newMPlusLamda(mVertexPlus,lamda);

        if((initMPlus<initM && !isVisible(newMPlusLamda,allPolys[i+1])) ||
          (initMPlus>initM && !isVisible(newMPlusLamda,allPolys[i+1]))){
            indexLamda=previousIndex(indexClosestK,allPolys[i+1]);
        }

        lamda=allPolys[i+1].vertex(indexLamda);

        int indexKMinus=previousIndex(indexClosestK,allPolys[i+1]);
        Point_2 kMinusVertex=allPolys[i+1].vertex(indexKMinus);
        Segment_2 kMinusMplus(mVertexPlus,kMinusVertex);

        newMPlusLamda=Segment_2(mVertexPlus,lamda);
       
        // There is a chance that the new lamda is not visble from neither m-1 nor m+1.
        //Thus we have to find a new m and repeat the above process
        while(!isVisible(newMPlusLamda,allPolys[i+1])){

          m=nextIndex(m,finalPoly);
          mPlus=nextIndex(m,finalPoly);
          
          mVertex=finalPoly.vertex(m);
          mVertexPlus=finalPoly.vertex(mPlus);
          
          closestK=getClosestK(mVertex,indexClosestK,allPolys[i+1]);
          indexLamda=nextIndex(indexClosestK,allPolys[i+1]);
          lamda=allPolys[i+1].vertex(indexLamda);

          newMPlusLamda=Segment_2(mVertexPlus,lamda);
          
          if(!isVisible(newMPlusLamda,allPolys[i+1])){
            int initM2=m;
            int initMPlus2=mPlus;
             
            if((m<mPlus && m!=finalPoly.size()-1) ||(m>mPlus && m==finalPoly.size()-1) ){
              mPlus=previousIndex(m,finalPoly);

            }else{
              mPlus=nextIndex(m,finalPoly);     
            }
            mVertexPlus=finalPoly.vertex(mPlus);
            newMPlusLamda=Segment_2(mVertexPlus,lamda); 
            
            if((initMPlus2<initM2 && !isVisible(newMPlusLamda,allPolys[i+1])) ||
              (initMPlus2>initM2 && !isVisible(newMPlusLamda,allPolys[i+1]))){
                indexLamda=previousIndex(indexClosestK,allPolys[i+1]);
            }

            lamda=allPolys[i+1].vertex(indexLamda);
            newMPlusLamda=Segment_2(mVertexPlus,lamda);
          }
        }
      }

      Segment_2 mToClosestKEdge(mVertex,closestK);
      Segment_2 mPlusToLamdaEdge(mVertexPlus,lamda);

      // We check whether the potential edges m-k and (m+1)-lamda intersect with one another
      if(CGAL::do_intersect(mToClosestKEdge,mPlusToLamdaEdge)){
        std::swap(m,mPlus);
        mVertex = finalPoly.vertex(m);
        mVertexPlus=finalPoly.vertex(mPlus);       
      }

      if(finalPoly.size()==allPolys[0].size()){
        
        // we find where we are going to place the new points(aka before which vertex, m or mPlus)
        auto veit=finalPoly.vertices_begin();
        
        if(mPlus>m || (mPlus==0 && m==finalPoly.size()-1)){
          veit=getVertexIt(mVertexPlus,finalPoly);
        }else{
          veit=getVertexIt(mVertex,finalPoly);          
        }
        
        std::vector<Point_2> toBeAdded; // the vector that contains the points to be added

        // The order which the new points are being placed varies based on m,mPlus,k,lamda
        if(m<mPlus && mPlus!=0 && m!=0){
          if(indexClosestK==allPolys[i+1].size()-1){
            for(int ind=indexClosestK;ind>=0;ind--){
              toBeAdded.push_back(allPolys[i+1].vertex(ind));
            }  
          }else{
            for(int ind=indexClosestK;ind>=0;ind--){
              toBeAdded.push_back(allPolys[i+1].vertex(ind));
            }
            for(int ind=allPolys[i+1].size()-1;ind>=indexLamda;ind--){
              toBeAdded.push_back(allPolys[i+1].vertex(ind));
            }
          }
        }else if(m<mPlus && mPlus==finalPoly.size()-1 && m==0){
          if((indexClosestK<indexLamda ) || 
             ( indexClosestK==allPolys[i+1].size()-1 && indexLamda==0)){          
            
            for(int ind=indexClosestK;ind>=0;ind--){
              toBeAdded.push_back(allPolys[i+1].vertex(ind));
            }
            for(int ind=allPolys[i+1].size()-1;ind>=indexLamda;ind--){
              toBeAdded.push_back(allPolys[i+1].vertex(ind));
            }              
          }else if((indexClosestK>indexLamda ) || 
                   (indexClosestK==allPolys[i+1].size()-1 && indexLamda==0) ){
            for(int ind=indexLamda;ind>=0;ind--){
              toBeAdded.push_back(allPolys[i+1].vertex(ind));
            }
            for(int ind=allPolys[i+1].size()-1;ind>=indexClosestK;ind--){
              toBeAdded.push_back(allPolys[i+1].vertex(ind));
            }
          }
        }else if(m<mPlus && mPlus!=0 && mPlus==finalPoly.size()-1 && m==0){
          if(indexClosestK==allPolys[i+1].size()-1 && indexLamda==0){
            for(int ind=indexClosestK;ind>=0;ind--){
              toBeAdded.push_back(allPolys[i+1].vertex(ind));
            }            
          }else if((indexClosestK>indexLamda ) ){
            for(int ind=indexLamda;ind>=0;ind--){
              toBeAdded.push_back(allPolys[i+1].vertex(ind));
            }
            for(int ind=allPolys[i+1].size()-1;ind>=indexClosestK;ind--){
              toBeAdded.push_back(allPolys[i+1].vertex(ind));
            }
          }else{
            for(int ind=indexClosestK;ind>=0;ind--){
              toBeAdded.push_back(allPolys[i+1].vertex(ind));
            }
            for(int ind=allPolys[i+1].size()-1;ind>=indexLamda;ind--){
              toBeAdded.push_back(allPolys[i+1].vertex(ind));
            }            
          }
        }else if(m>mPlus && mPlus!=0 && m!=0){
          if(indexClosestK==0){
            for(int ind=indexLamda;ind>=0;ind--){
              toBeAdded.push_back(allPolys[i+1].vertex(ind));
            }            
          }
          if((indexClosestK!=0 && indexLamda!=allPolys[i+1].size()-1) || (indexClosestK!=allPolys[i+1].size()-1 && indexLamda!=0)){
            if(indexClosestK>indexLamda && indexClosestK!=allPolys[i+1].size()-1 && indexLamda!=0){
              for(int ind=indexLamda;ind>=0;ind--){
                toBeAdded.push_back(allPolys[i+1].vertex(ind));
              }
              for(int ind=allPolys[i+1].size()-1;ind>=indexClosestK;ind--){
                toBeAdded.push_back(allPolys[i+1].vertex(ind));
              }             
            }else if(indexClosestK>indexLamda && indexClosestK!=allPolys[i+1].size()-1 && indexLamda==0){
              for(int ind=indexLamda;ind>=0;ind--){
                toBeAdded.push_back(allPolys[i+1].vertex(ind));
              }
              for(int ind=allPolys[i+1].size()-1;ind>=indexClosestK;ind--){
                toBeAdded.push_back(allPolys[i+1].vertex(ind));
              }               
            }else if(indexClosestK>indexLamda && indexClosestK==allPolys[i+1].size()-1 && indexLamda==0){
              for(int ind=indexLamda;ind<=indexClosestK;ind++){
                toBeAdded.push_back(allPolys[i+1].vertex(ind));
              }              
            }else if(indexClosestK>indexLamda && indexClosestK==allPolys[i+1].size()-1 && indexLamda!=0){
              for(int ind=indexLamda;ind>=0;ind--){
                toBeAdded.push_back(allPolys[i+1].vertex(ind));
              }
              for(int ind=allPolys[i+1].size()-1;ind>=indexClosestK;ind--){
                toBeAdded.push_back(allPolys[i+1].vertex(ind));
              }                
            }
          }
        }else if(m>mPlus && mPlus==0){
          if(indexLamda==0 && indexClosestK==allPolys[i+1].size()-1){
            for(int ind=indexClosestK;ind>=0;ind--){
              toBeAdded.push_back(allPolys[i+1].vertex(ind));              
            }
          }else if(indexLamda==0 && indexClosestK!=allPolys[i+1].size()-1){
             for(int ind=indexLamda;ind>=0;ind--){
              toBeAdded.push_back(allPolys[i+1].vertex(ind));              
            }
            for(int ind=allPolys[i+1].size()-1;ind>=indexClosestK;ind--){
              toBeAdded.push_back(allPolys[i+1].vertex(ind));              
            }            
          }else if(indexClosestK>indexLamda && indexLamda!=0){
            for(int ind=indexLamda;ind>=0;ind--){
              toBeAdded.push_back(allPolys[i+1].vertex(ind));
            }
            for(int ind=allPolys[i+1].size()-1;ind>=indexClosestK;ind--){
              toBeAdded.push_back(allPolys[i+1].vertex(ind));
            }
          }else{
            for(int ind=indexClosestK;ind>=0;ind--){
              toBeAdded.push_back(allPolys[i+1].vertex(ind));              
            }
            for(int ind=allPolys[i+1].size()-1;ind>=indexLamda;ind--){
              toBeAdded.push_back(allPolys[i+1].vertex(ind));              
            }
          }
        }else{
          if(indexLamda<indexClosestK && indexLamda!=0 && indexClosestK!=allPolys[i+1].size()-1){
            for(int ind=indexLamda;ind>=0;ind--){
              toBeAdded.push_back(allPolys[i+1].vertex(ind));
            }
            for(int ind=allPolys[i+1].size()-1;ind>=indexClosestK;ind--){
              toBeAdded.push_back(allPolys[i+1].vertex(ind));
            }            
          }else if(indexLamda<indexClosestK && indexLamda==0 && indexClosestK==allPolys[i+1].size()-1){
            for(int ind=indexClosestK;ind>=0;ind--){
              toBeAdded.push_back(allPolys[i+1].vertex(ind));
            } 
          }else{
            for(int ind=indexClosestK;ind>=0;ind--){
              toBeAdded.push_back(allPolys[i+1].vertex(ind));
            }
            for(int ind=allPolys[i+1].size()-1;ind>=indexLamda;ind--){
              toBeAdded.push_back(allPolys[i+1].vertex(ind));
            }
          }
        }

        // we insert the points
        if(mPlus==0){
          finalPoly.insert(veit,toBeAdded.begin(),toBeAdded.end());
        }else{
          finalPoly.insert(veit,toBeAdded.begin(),toBeAdded.end());
        }


      }else{
        auto veit=finalPoly.vertices_begin();

        std::vector<Point_2> toBeAdded; // the vector that contains the points to be added
        
        // we find where we are going to place the new points(aka before which vertex, m or mPlus)
        if(mPlus>m){
          veit=getVertexIt(mVertexPlus,finalPoly);
        }else{
          veit=getVertexIt(mVertex,finalPoly);
        }    
        
        // The order which the new points are being placed varies based on m,mPlus,k,lamda
        if(m<mPlus){
          if(indexClosestK>indexLamda && indexLamda!=0){
            for(int ind=indexClosestK;ind<allPolys[i+1].size();ind++){
              toBeAdded.push_back(allPolys[i+1].vertex(ind));
            }
            for(int ind=0;ind<=indexLamda;ind++){
              toBeAdded.push_back(allPolys[i+1].vertex(ind));
            } 
          }else if(indexClosestK>indexLamda && indexLamda==0 && indexClosestK==allPolys[i+1].size()-1){
            for(int ind=indexClosestK;ind>=0;ind--){
              toBeAdded.push_back(allPolys[i+1].vertex(ind));
            }
          }else if(indexClosestK>indexLamda && indexLamda==0 && indexClosestK!=allPolys[i+1].size()-1){
            for(int ind=indexClosestK;ind<allPolys[i+1].size();ind++){
              toBeAdded.push_back(allPolys[i+1].vertex(ind));
            }
            for(int ind=0;ind<=indexLamda;ind++){
              toBeAdded.push_back(allPolys[i+1].vertex(ind));
            }
          }else if(indexClosestK<indexLamda && indexLamda==allPolys[i+1].size()-1 && indexClosestK==0){
            for(int ind=0;ind<=indexLamda;ind++){
              toBeAdded.push_back(allPolys[i+1].vertex(ind));
            }
          }else if(indexClosestK<indexLamda && indexLamda==allPolys[i+1].size()-1 && indexClosestK!=0){
            for(int ind=indexClosestK;ind>=0;ind--){
              toBeAdded.push_back(allPolys[i+1].vertex(ind));
            }
            for(int ind=indexLamda;ind<allPolys[i+1].size();ind++){
              toBeAdded.push_back(allPolys[i+1].vertex(ind));
            }             
          }else if(indexClosestK<indexLamda && indexLamda!=allPolys[i+1].size()-1){
            for(int ind=indexClosestK;ind>=0;ind--){
              toBeAdded.push_back(allPolys[i+1].vertex(ind));
            }
            for(int ind=allPolys[i+1].size()-1;ind>=indexLamda;ind--){
              toBeAdded.push_back(allPolys[i+1].vertex(ind));
            }           
          }else{          
            for(int ind=indexClosestK;ind>=0;ind--){
              toBeAdded.push_back(allPolys[i+1].vertex(ind));
            }
            for(int ind=allPolys[i+1].size()-1;ind>=indexLamda;ind--){
              toBeAdded.push_back(allPolys[i+1].vertex(ind));
            }
          }
        }else{
          if(indexClosestK>indexLamda && indexLamda!=0){
            for(int ind=indexLamda;ind>=0;ind--){
              toBeAdded.push_back(allPolys[i+1].vertex(ind));
            }
            for(int ind=allPolys[i+1].size()-1;ind>=indexClosestK;ind--){
              toBeAdded.push_back(allPolys[i+1].vertex(ind));
            }
          }else if(indexLamda==0){
            if(indexClosestK!=allPolys[i+1].size()-1){
              for(int ind=indexLamda;ind>=0;ind--){
                toBeAdded.push_back(allPolys[i+1].vertex(ind));
              }
              for(int ind=allPolys[i+1].size()-1;ind>=indexClosestK;ind--){
                toBeAdded.push_back(allPolys[i+1].vertex(ind));
              }                        
            }else{
              for(int ind=indexLamda;ind<allPolys[i+1].size();ind++){
                toBeAdded.push_back(allPolys[i+1].vertex(ind));              
              }
            }
          }else if(indexClosestK<indexLamda && indexClosestK==0 && indexLamda==allPolys[i+1].size()-1){
            for(int ind=indexLamda;ind>=0;ind--){
              toBeAdded.push_back(allPolys[i+1].vertex(ind));              
            }
          }else if(indexClosestK<indexLamda && indexClosestK==0 && indexLamda!=allPolys[i+1].size()-1){
            for(int ind=indexLamda;ind<allPolys[i+1].size();ind++){
              toBeAdded.push_back(allPolys[i+1].vertex(ind));  
            }
            for(int ind=0;ind<=indexClosestK;ind++){
              toBeAdded.push_back(allPolys[i+1].vertex(ind)); 
            }                        
          }else{
            for(int ind=indexLamda;ind<allPolys[i+1].size();ind++){
              toBeAdded.push_back(allPolys[i+1].vertex(ind));
            }
            for(int ind=0;ind<=indexClosestK;ind++){
                toBeAdded.push_back(allPolys[i+1].vertex(ind));
            } 
          }  
        }

        // we insert the points           
        finalPoly.insert(veit,toBeAdded.begin(),toBeAdded.end());
      }

      int initK=indexClosestK;
      int initLam=indexLamda;

      // Now we have to search for the new edge m-(m+1) which has to be different from the edge k-lamda that we used
      // whilst being neighbours
      do{

        indexLamda=nextIndex(indexLamda,allPolys[i+1]);
        indexClosestK=previousIndex(indexClosestK,allPolys[i+1]);
 
        // If a polygon's size at depth i+1 is an odd number, there is a chance that k and lamda will meet at the same vertex
        // We force one of them to wait
        if((allPolys[i+1].size()) %2 && indexLamda==indexClosestK){
          indexLamda=nextIndex(indexLamda,allPolys[i+1]);
        }

      }while((indexLamda-indexClosestK!=1 && indexLamda-indexClosestK!=-1 && indexLamda-indexClosestK!=allPolys[i+1].size()-1 
        && indexLamda-indexClosestK!=-(allPolys[i+1].size()-1)) || (indexClosestK==initK && indexLamda==initLam) ||
        (indexLamda==initK && indexClosestK==initLam) );

      // if lamda is before k, we swap them to make our lives easier
      if( indexLamda-indexClosestK==-1 ||indexLamda-indexClosestK==allPolys[i+1].size()-1){
        std::swap(indexClosestK,indexLamda);
      }


      // We find where exactly in our "merged" Polygon the new edges are
      int kInPoly=0;
      int lamInPoly=0;

      Point_2 findK=allPolys[i+1].vertex(indexClosestK);
      Point_2 findLam=allPolys[i+1].vertex(indexLamda);

      int help=0;
      while(finalPoly.vertex(help)!=findK){
        help++;
      }
      kInPoly=help;
      help=0;

      while(finalPoly.vertex(help)!=findLam){
        help++;
      }
      lamInPoly=help;

      // we update our m and mPlus
      m=kInPoly;
      mPlus=lamInPoly;

    }else{ // Here we are in the final depth of the convex Hulls, so the only thing left to do is add the points that were left out of
           //of the convex hulls

      if(finalPoly.is_empty()){
        finalPoly=allPolys[i];
      }

      // we iterate over the left over points
      for(int j=0;j<points.size();j++){
        int indexClosePoint=-1;
        Point_2 closePoint=getClosestK(points[j],indexClosePoint,finalPoly); // we find the closest vertex of our "merged" polygon

        auto veit=finalPoly.vertices_begin();
        veit=getVertexIt(closePoint,finalPoly);

        Segment_2 pointLine(*(veit+1),points[j]);
        // if the point is visible from the next point of the above closest point
        if(isVisible(pointLine,finalPoly)){
          Segment_2 pointLine2(*veit,points[j]);

          // And if it's visible from the closest point
          while(!isVisible(pointLine2,finalPoly)){
            if(veit!=finalPoly.vertices_end()){
              veit++;
            }else{
              veit=finalPoly.vertices_begin();
            }
            pointLine2=Segment_2(*veit,points[j]);
          }
          finalPoly.insert(veit+1,points[j]); // we place it after the closest point we found
        }else{
          Segment_2 lineFinalPoly(*(veit),points[j]);
          // if the point is visible from its closest point but not visible from closest point+1, 
          //we will try to place it before the closest point if we can          
         
          if(isVisible(lineFinalPoly,finalPoly)){
            lineFinalPoly=Segment_2(*(veit-1),points[j]);
            
            // Closest point -1 should be visible, if not we find a different closest
            // The smart choice is to look for the points that belong both in finalPoly and in the last ConvexHull
            while(!isVisible(lineFinalPoly,finalPoly)){
              closePoint=getClosestK(points[j],indexClosePoint,allPolys[i]);
              veit=getVertexIt(closePoint,finalPoly);
              lineFinalPoly=Segment_2(*(veit-1),points[j]);
            }

            // we insert the closest points
            finalPoly.insert(veit,points[j]);
          }else{
            // we have to find the closest point in the last convex hull,which is certainly visible
            closePoint=getClosestK(points[j],indexClosePoint,allPolys[i]);
            
            auto veit2=finalPoly.vertices_begin();
            veit2=getVertexIt(closePoint,finalPoly);

            Segment_2 pointLine2(*(veit2+1),points[j]);
            
            // if the next from the closest is visible, place it after the closest
            if(isVisible(pointLine2,finalPoly)){
              finalPoly.insert(veit2+1,points[j]); 
            }else{; // place it before the closest
              finalPoly.insert(veit2,points[j]);
            }            
          }
        }
      }
    }
  }
  
  return finalPoly;
}

// Practically checks whether <initialEdge> intersects with <poly> in more than 2 spots since initialEdge[1] is a vertex of <poly>
bool isVisible(Segment_2& initialEdge, Polygon_2& poly){
    int timesInter=0;
    for(auto eit=poly.edges_begin();eit!=poly.edges_end();eit++){
      const auto res=CGAL::intersection(initialEdge,*eit);
      if(res){
        timesInter++;
        if(timesInter>2){
          return false;
        }
      }      
    }
    if(timesInter==2){
      return true;
    }else{
      return false;
    }

}


// Finds the closest Point to PointM in poly. Returns its position in indexClosestK
Point_2 getClosestK(Point_2& pointM, int& indexClosestK ,Polygon_2& poly){
    
  Point_2 closestK=Point_2(-1111,-1111);
  int indexK=-1;
  double dist=INFINITY;

  // Find closest Point K
  for(int j=0;j<poly.size();j++){
    Point_2 vert=poly.vertex(j);
    double vertDist=sqrt(((vert[0]-pointM[0])*(vert[0]-pointM[0]))+((vert[1]-pointM[1])*(vert[1]-pointM[1])));
            
    if(vertDist<dist){
      dist=vertDist;
      closestK=vert;
      indexK=j;
    }
  }
  
  indexClosestK=indexK;
  return closestK;
}

// Checks whether a Point_2 <point> lies on the boundary of Polygon_2 <poly>
bool pointInPolygon(Point_2& point,Polygon_2& poly){
  for(auto eit=poly.edges_begin();eit!=poly.edges_end() ; eit++){
    Segment_2 edge=*eit;
    if(CGAL::collinear(edge[0],point,edge[1]) ){
      return true;
    }
  }
  return false;
}


// For a given Point_2 <vertex> returns its position in Polygon_2 <poly> in a iterator
Polygon_2::Vertex_iterator getVertexIt(Point_2& vertex,Polygon_2& poly){
  auto veit=poly.vertices_begin();
    
  while(*veit!=vertex){
    if(veit+1!=poly.vertices_end()){
      veit++;
    }else{
      veit=poly.vertices_begin();
    }
  }

  return veit;
}

// returns the index after <index> in Polygon_2 <poly>
int nextIndex(int& index, Polygon_2& poly){
  int next=0;
  
  if(index!=poly.size()-1){
    next = index+1;
  }else{
    next=0;
  }

  return next;
}

// returns the index before <index> in Polygon_2 <poly>
int previousIndex(int& index, Polygon_2& poly){
  int previous=0;
  
  if(index!=0){
    previous = index-1;
  }else{
    previous=poly.size()-1;
  }

  return previous;
}