#include "heuristic.hpp"
#include <map>
#include <cmath>

Heuristic::Heuristic()
{
    allModels["dijkstra"] = Heuristic::HeuristicModel::Dijkstra;
    allModels["l1"] = Heuristic::HeuristicModel::L1;
    allModels["l2"] = Heuristic::HeuristicModel::L2;
    allModels["diagonal"] = Heuristic::HeuristicModel::Diagonal;
    allModels["l1-altitude"] = Heuristic::HeuristicModel::L1Altitude;
    allModels["l2-altitude"] = Heuristic::HeuristicModel::L2Altitude;
    allModels["diagonal-altitude"] = Heuristic::HeuristicModel::DiagonalAltitude;
    // allModels["theta-star"] = Heuristic::HeuristicModel::ThetaStar;

    // {
    //     std::make_pair( "dijkstra", Heuristic::HeuristicModel::Dijkstra ),
    //     std::make_pair( "l1", Heuristic::HeuristicModel::L1 ),
    //     std::make_pair( "l2", Heuristic::HeuristicModel::L2 ),
    //     std::make_pair( "diagonal", Heuristic::HeuristicModel::Diagonal ),
    //     std::make_pair( "l1-altitude", Heuristic::HeuristicModel::L1Altitude ),
    //     std::make_pair( "l2-altitude", Heuristic::HeuristicModel::L2Altitude ),
    //     std::make_pair( "diagonal-altitude", Heuristic::HeuristicModel::DiagonalAltitude ),
    //     std::make_pair( "theta-star", Heuristic::HeuristicModel::ThetaStar )
    // };
}

double Heuristic::computeHeuristic(const CellLocation& source, 
                                    const CellLocation& target, 
                                    const HeuristicModel& model,
                                    const double aveAltitude)
{
    switch ( model )
    {
        case HeuristicModel::Dijkstra:
            return computeDijkstra( source, target, model, aveAltitude );

        case HeuristicModel::L1:
            return computeL1( source, target, model, aveAltitude );

        case HeuristicModel::L2:
            return computeL2( source, target, model, aveAltitude );

        case HeuristicModel::Diagonal:
            return computeDiagonal( source, target, model, aveAltitude );

        case HeuristicModel::L1Altitude:
            return computeL1( source, target, model, aveAltitude );

        case HeuristicModel::L2Altitude:
            return computeL2( source, target, model, aveAltitude );

        case HeuristicModel::DiagonalAltitude:
            return computeDiagonal( source, target, model, aveAltitude );

        // case HeuristicModel::ThetaStar:
            // return computeThetaStar( source, target, model, aveAltitude );
    }
}

double Heuristic::computeDijkstra( const CellLocation& source, 
                        const CellLocation& target,
                        const HeuristicModel& model,
                        const double aveAltitude )
{
    return 0.0;
}

double Heuristic::computeL1( const CellLocation& source, 
                    const CellLocation& target,
                    const HeuristicModel& model,
                    const double aveAltitude )
{
    const auto dx = abs(target.X() - source.X());
    const auto dy = abs(target.Y() - source.Y());
    return (dx + dy) * aveAltitude; 
}

double Heuristic::computeL2( const CellLocation& source, 
                    const CellLocation& target,
                    const HeuristicModel& model,
                    const double aveAltitude )
{
    const auto dx = abs(target.X() - source.X());
    const auto dy = abs(target.Y() - source.Y());
    return std::sqrt(dx*dx + dy*dy) * aveAltitude;
}

double Heuristic::computeDiagonal( const CellLocation& source, 
                    const CellLocation& target,
                    const HeuristicModel& model,
                    const double aveAltitude )
{
    const auto dx = abs(target.X() - source.X());
    const auto dy = abs(target.Y() - source.Y());
    return (std::max(dx, dy) + (std::sqrt(2.0) - 1) * std::min(dx, dy)) * aveAltitude;
}