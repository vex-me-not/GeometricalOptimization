#ifndef SHARED_H
#define SHARED_H

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/IO/WKT.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/Kd_tree.h>
#include <CGAL/Fuzzy_iso_box.h>
#include <CGAL/Search_traits_2.h>
#include <iostream>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Polygon_2<Kernel> Polygon_2;
typedef CGAL::Point_2<Kernel> Point_2;
typedef std::vector<Point_2> PointList;
typedef std::vector<Point_2>::iterator PointListIterator;
typedef Kernel::Segment_2 Segment_2;
typedef Polygon_2::Edge_const_iterator EdgeIterator;
typedef std::pair<Point_2, Point_2> PointPair;
typedef std::vector<PointPair> PointPairList;
typedef std::vector<PointPair>::iterator PointPairListIterator;

typedef CGAL::Search_traits_2<Kernel> Traits;
typedef CGAL::Kd_tree<Traits> Tree;
typedef CGAL::Fuzzy_iso_box<Traits> Fuzzy_iso_box;

enum GenerationAlgorithm {incremental, convex_hull, onion};
enum EdgeSelection {randomSelection, min, max};
enum Initialization {a1, a2, b1, b2};

enum OptimazationAlgorithm {local_search, simulated_annealing, ant_colony};
enum AnnealingType {local, global, subdivision};
enum OptimizationType {maximization, minimization};

struct AntParameters{
    
    int L;
    OptimizationType optimizationType;
    double threshold;
    AnnealingType annealingType;
    double alpha;
    double beta;
    double ro;
    int elitism;
    int enable_breaks;
    int divisor;

};

struct ArgumentFlags{
    std::string inputDirectory;
    std::string outputFile;
    std::string preprocess;

    bool error;
    bool useAnt;
    std::string errorMessage;
};

struct testResults{
    double min_score;
    double max_score;
};

#endif