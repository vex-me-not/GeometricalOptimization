#ifndef RESULT_LOGGER_H
#define RESULT_LOGGER_H

#include <string>
#include <map>
#include <iostream>
#include <fstream>
#include <memory>
#include <stdexcept>

template<typename ... Args>
std::string string_format( const std::string& format, Args ... args )
{
    int size_s = std::snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
    if( size_s <= 0 ){ throw std::runtime_error( "Error during formatting." ); }
    auto size = static_cast<size_t>( size_s );
    std::unique_ptr<char[]> buf( new char[ size ] );
    std::snprintf( buf.get(), size, format.c_str(), args ... );
    return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}



struct ResultEntry
{
    double min_score;
    double max_score;
    double min_bound;
    double max_bound;
};

enum Combination
{
    incrLocal,
    incrAnn,
    chullLocal,
    chullAnn,
    onionLocal,
    onionAnn,
    antColony
};

std::string combinationName(Combination combo)
{
    switch (combo)
    {
    case incrLocal:
        return "\t\t\t\t\tIncremental & Local Search\t\t\t\t\t";
    case incrAnn:
        return "\t\t\t\tIncremental & Simulated Annealing\t\t\t\t";
    case chullLocal:
        return "\t\t\t\t\tConvex Hull & Local Search\t\t\t\t\t";
    case chullAnn:
        return "\t\t\t\tConvex Hull & Simulated Annealing\t\t\t\t";
    case onionLocal:
        return "\t\t\t\t\tOnion & Local Search\t\t\t\t\t\t";
    case onionAnn:
        return "\t\t\t\t\tOnion & Simulated Annealing\t\t\t\t\t";
    case antColony:
        return "\t\t\t\t\t\t\tAnt Colony\t\t\t\t\t\t\t";
    default:
        return "Unknown combination";
    }
}

std::string combinationShortName(Combination combo)
{
    switch (combo)
    {
    case incrLocal:
        return "Incremental & Local Search";
    case incrAnn:
        return "Incremental & Simulated Annealing";
    case chullLocal:
        return "Convex Hull & Local Search";
    case chullAnn:
        return "Convex Hull & Simulated Annealing";
    case onionLocal:
        return "Onion & Local Search";
    case onionAnn:
        return "Onion & Simulated Annealing";
    case antColony:
        return "Ant Colony";
    default:
        return "Unknown combination";
    }
}


typedef std::map<int, ResultEntry*> Dictionary;

class ResultLogger
{
private:
    Dictionary log; 
public:
    ResultLogger();
    ~ResultLogger();

    void updateEntry(int, Combination, double, double);
    void updateMinEntry(int, Combination, double);
    void updateMaxEntry(int, Combination, double);
    void printLogger(std::string);
};

ResultLogger::ResultLogger(){}
ResultLogger::~ResultLogger()
{
    for(auto it = log.begin(); it != log.end(); it++)
        delete [] log[it->first];
}

void ResultLogger::updateEntry(int key, Combination combination, double minScore, double maxScore)
{
    if(log.find(key) == log.end())
    {
        //key does not exist
        log[key] = new ResultEntry[7];
        
        for(int i = 0; i < 7; i++)
        {
            log[key][i].min_score = 0;
            log[key][i].max_score = 0;
            log[key][i].min_bound = -1;
            log[key][i].max_bound = 2;
        }
    }

    updateMinEntry(key, combination, minScore);
    updateMaxEntry(key, combination, maxScore);
}

void ResultLogger::updateMinEntry(int key, Combination combination, double minScore)
{
    log[key][combination].min_score += minScore;

    double prevMinBound = log[key][combination].min_bound;
    log[key][combination].min_bound = std::max(prevMinBound, minScore);
}

void ResultLogger::updateMaxEntry(int key, Combination combination, double maxScore)
{
    log[key][combination].max_score += maxScore;

    double prevMaxBound = log[key][combination].max_bound;
    log[key][combination].max_bound = std::min(prevMaxBound, maxScore);
}

void ResultLogger::printLogger(std::string streamName)
{
    
    std::ofstream outputStream(streamName);

    //first row

    outputStream << "\t\t||";
    for(int i = 0; i < 7; i++) outputStream << combinationName((Combination) i) << "||";
    outputStream << std::endl;

    //second row

    outputStream << "Size\t||\t";
    for(int i = 0; i < 7; i++) outputStream << "min score\t||\t" << "max score\t||\t" << "min bound\t||\t" << "max bound\t||\t"; 
    outputStream << std::endl;

    //for all keys (aka sizes of data files)

    for(auto iter = log.begin(); iter != log.end(); iter++)
    {
        int key = iter->first;
        ResultEntry *logNode = log[key];

        outputStream << string_format("%-8d||", key);
        for(int i = 0; i < 7; i++)
        {
            outputStream << string_format("%14.2f||", logNode[i].min_score);
            outputStream << string_format("%14.2f||", logNode[i].max_score);
            outputStream << ((logNode[i].min_bound != -1) ? string_format("%14.2f||", logNode[i].min_bound) : "\t\t?\t\t||");
            outputStream << ((logNode[i].max_bound != 2) ? string_format("%14.2f||", logNode[i].max_bound) : "\t\t?\t\t||"); 
        } 
        outputStream << std::endl;

    }


}

#endif