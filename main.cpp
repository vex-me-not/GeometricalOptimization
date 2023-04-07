#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <cstdlib>
#include <string.h>
#include <filesystem>

#include "shared.h"

#include "PolygonGenerator.h"
#include "PolygonOptimizer.h"

#include "ConvexHullAlgo.h"
#include "onion.h"
#include "incr.h"

#include "local.h"
#include "SimulatedAnnealing.h"
#include "ant.h"

#include "AlgorithmHandler.h"
#include "DefaultHandler.h"
#include "ResultLogger.h"
#include "SmartHandler.h"
  
using std::cout;
using std::endl;
using std::string;
using std::ifstream;

void handleArgs(ArgumentFlags& argFlags, int& argc, char**& argv);
void printArguments(ArgumentFlags& argFlags);

double handleAlgorithm(AlgorithmHandler& handler, Combination combo, OptimizationType type)
{
    switch (combo)
    {
    case incrLocal:
        return handler.incrementalLocalSearch(type);
    case incrAnn:
        return handler.incrementalAnnealing(type);
    case chullLocal:
        return handler.convexHullLocalSearch(type);
    case chullAnn:
        return handler.convexHullAnnealing(type);
    case onionLocal:
        return handler.onionLocalSearch(type);
    case onionAnn:
        return handler.onionAnnealing(type);
    case antColony:
        return handler.antColony(type);
    }

    return 0.0;
}

int main(int argc, char **argv)
{
    ArgumentFlags argFlags;
    handleArgs(argFlags, argc, argv);
    
    //check for input error
    if(argFlags.error)
    {
        cout << argFlags.errorMessage << endl;
        cout << "./evaluate -i <point set path> -o <output file> -preprocess <optional>" << endl;
        return -1;
    }

    ResultLogger logger;
    AlgorithmHandler *handler;
    bool starting = true;

    int comboLimit = (argFlags.useAnt) ? 7 : 6;

    for (const auto & entry : std::filesystem::directory_iterator(argFlags.inputDirectory))
    {
        cout << "Working on file " << entry.path() << "..." << endl;
        if(starting)
        {   
            if(argFlags.preprocess == "smart")
            {
                handler = new SmartHandler(entry.path());
            }
            else
            {
                handler = new DefaultHandler(entry.path());
            }
                
            starting = false;
        } 
        else
            handler->resetFile(entry.path());

        int size = handler->getSize();

        for(int i = 0; i < comboLimit; i++)
        {
            cout << "Combination: " << combinationShortName((Combination) i) << "..." << endl;

            auto start = std::chrono::high_resolution_clock::now();
            double minScore =  handleAlgorithm(*handler, (Combination) i, minimization);
            auto stop = std::chrono::high_resolution_clock::now();
            double maxScore =  handleAlgorithm(*handler, (Combination) i, maximization);
            auto stop2 = std::chrono::high_resolution_clock::now();

            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
            auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(stop2 - stop);

            minScore = (duration.count() < 500*size) ? minScore : 1;
            maxScore = (duration2.count() < 500*size) ? maxScore : 0;

            // cout << "duration1: " << duration.count() << endl << "  duration2: " << duration2.count() << endl;

            // if (minScore == 1 || maxScore == 0)
            //     cout << "hit cuttof" << endl;

            logger.updateEntry(size, (Combination) i, minScore, maxScore);
        }
        cout << endl;
    }

    delete handler;

    // cout << "Done with files" << endl;

    logger.printLogger(argFlags.outputFile);

    auto start = std::chrono::high_resolution_clock::now();

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    
    return 0;
}

void handleArgs(ArgumentFlags& argFlags, int& argc, char**& argv)
{
    int waitingForArg = 0;
    bool waitingInput = true;
    bool waitingOutput = true;

    argFlags.preprocess = "default";
    argFlags.useAnt = false;

    for (int i = 1; i < argc; i++)
    {
        char* arg = argv[i];
        switch (waitingForArg)
        {
            case 0:
                if (!strcmp(arg, "-i"))
                    waitingForArg = 1;
                else if (!strcmp(arg, "-o"))
                    waitingForArg = 2;
                else if (!strcmp(arg, "-preprocess"))
                    waitingForArg = 3;
                else if (!strcmp(arg, "-useAnt"))
                    argFlags.useAnt = true;
                break;
            case 1:
                argFlags.inputDirectory = string(arg);
                waitingInput = false;
                waitingForArg = 0;
                break;
            case 2:
                argFlags.outputFile = string(arg);
                waitingOutput = false;
                waitingForArg = 0;
                break;
            case 3:
                argFlags.preprocess = string(arg);
                waitingForArg = 0;
                break;
        }
    }

    if(waitingInput){
        argFlags.error = true;
        argFlags.errorMessage = string("Must select input file");
        return;
    }
    if(waitingOutput){
        argFlags.error = true;
        argFlags.errorMessage = string("Must select output file");
        return;
    }

    argFlags.error = false;
    return;
}



void printArguments(ArgumentFlags& argFlags)
{
    cout << "Input file: " << argFlags.inputDirectory << endl;
    cout << "Output file: " << argFlags.outputFile << endl;
    cout << "preprocess: " << argFlags.preprocess << endl;
}