#ifndef ALGORITHM_HANDLER_H
#define ALGORITHM_HANDLER_H

#include "shared.h"

void getPointsFromFile(std::string filepath, int& size, PointList& points, long& convexHullArea)
{
    //get number of lines
    std::ifstream infile(filepath);
    int linesCount = 0;
    std::string line;

    while(getline(infile, line))linesCount++;
    int pointsCount = linesCount - 2;   //files always have 2 comments and then just points
    size = pointsCount;

    
    //reset reading cursor
    infile.clear();
    infile.seekg(0);

    //ignore first 2 lines
    getline(infile, line);
    getline(infile,line);

    //get convex hull area from second line
    convexHullArea = stol(   // we use stol instead of stoi because stoi return an integer and stol return a long
        line.substr(
            line.find("{\"area\": \"") + 10, 
            line.find("\"}"))
    );

    //read points
    int x,y;
    for(int i = 0; getline(infile, line); i++)
    {
        int ignore;
        std::istringstream iss(line);
        iss >> ignore >> x >> y;
        points.push_back(Point_2(x,y));
    }

    infile.close();
    return;
}

class AlgorithmHandler
{
protected:
    std::string filename;
    PointList points;  
    int size;
    long convexHullArea;

public:
    AlgorithmHandler(std::string name): filename(name){getPointsFromFile(name, size, points, convexHullArea);};
    virtual ~AlgorithmHandler(){};

    virtual double incrementalLocalSearch(OptimizationType type) = 0; 
    virtual double incrementalAnnealing(OptimizationType type) = 0; 
    virtual double convexHullLocalSearch(OptimizationType type) = 0; 
    virtual double convexHullAnnealing(OptimizationType type) = 0; 
    virtual double onionLocalSearch(OptimizationType type) = 0; 
    virtual double onionAnnealing(OptimizationType type) = 0;  
    virtual double antColony(OptimizationType type) = 0;

    virtual void printFields()
    {
        std::cout << "filename: " << filename << std::endl;
        std::cout << "size: " << size << std::endl;
        std::cout << "convex hull area: " << convexHullArea << std::endl;
    }

    int getSize(){return size;}
    long getCHullArea(){return convexHullArea;}

    void resetFile(std::string newFile)
    {
        filename = newFile;
        points.clear();
        getPointsFromFile(newFile, size, points, convexHullArea);
    }
    
};

#endif