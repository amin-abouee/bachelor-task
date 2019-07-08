#include "cost.hpp"
#include "cmath"
// #include "map"


std::map< std::string, Cost::CostModel > Cost::allModels
{
  std::make_pair( "octile", Cost::CostModel::Octile ),
  std::make_pair( "pick", Cost::CostModel::Pick ),
  std::make_pair( "mean-pick", Cost::CostModel::MeanPick ),
  std::make_pair( "l2", Cost::CostModel::L2 ),
  std::make_pair( "l1", Cost::CostModel::L1 ),
  std::make_pair( "linf", Cost::CostModel::LInf ),
  std::make_pair( "angle", Cost::CostModel::Angle ),
  std::make_pair( "dificulty-level", Cost::CostModel::DifficultyLevel )
};

double Cost::computeCost(const CellLocation& source, 
                            const uint8_t elevationSource, 
                            const CellLocation& target, 
                            const uint8_t elevationTarget,
                            const CostModel& model)
{
    switch ( model )
    {
        case CostModel::Octile:
            return computeOCtile( source, elevationSource, target, elevationTarget );
            // break;
        case CostModel::Pick:
            return computePick( source, elevationSource, target, elevationTarget );

        case CostModel::MeanPick:
            return computeMeanPick( source, elevationSource, target, elevationTarget );
            // break;
        case CostModel::L2:
            return computeL2( source, elevationSource, target, elevationTarget );
            // break;
        case CostModel::L1:
            return computeL1( source, elevationSource, target, elevationTarget );
            // break;
        case CostModel::LInf:
            return computeLInf( source, elevationSource, target, elevationTarget );
            // break;
        case CostModel::Angle:
            return computeAngle( source, elevationSource, target, elevationTarget );
            // break;
        case CostModel::DifficultyLevel:
            return computeDifficultyLevel( source, elevationSource, target, elevationTarget );
            // break;
    }
}

double Cost::computeOCtile( const CellLocation& source, 
                            const uint8_t elevationSource, 
                            const CellLocation& target, 
                            const uint8_t elevationTarget )
{
    double distance = std::abs(source.x - target.x) + std::abs(source.y - target.y);
    if (distance == 1.0)
        return 1.0;
    else
        return std::sqrt(2.0);
}

double Cost::computePick( const CellLocation& source, 
                            const uint8_t elevationSource, 
                            const CellLocation& target, 
                            const uint8_t elevationTarget )
{
    return std::max(elevationSource, elevationTarget);
}

double Cost::computeMeanPick( const CellLocation& source, 
                            const uint8_t elevationSource, 
                            const CellLocation& target, 
                            const uint8_t elevationTarget )
{
    return std::min(elevationSource, elevationTarget) + (std::abs(elevationTarget - elevationSource) / 2.0);
}