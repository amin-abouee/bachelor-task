#include "cost.hpp"
#include "cmath"
// #include "map"


std::map< std::string, Cost::CostModel > Cost::allModels
{
  std::make_pair( "octile", Cost::CostModel::Octile ),
  std::make_pair( "peak", Cost::CostModel::Peak ),
  std::make_pair( "mean-peak", Cost::CostModel::MeanPeak ),
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
            return computeOctile( source, elevationSource, target, elevationTarget );
            // break;
        case CostModel::Peak:
            return computePeak( source, elevationSource, target, elevationTarget );

        case CostModel::MeanPeak:
            return computeMeanPeak( source, elevationSource, target, elevationTarget );
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

double Cost::computeOctile( const CellLocation& source, 
                            const uint8_t elevationSource, 
                            const CellLocation& target, 
                            const uint8_t elevationTarget )
{
    //  manhattan distance between source and target
    const auto distance = std::abs(source.X() - target.X()) + std::abs(source.Y() - target.Y());
    if (distance == 1)
        return 1.0;
    else
        return std::sqrt(2.0);
}

double Cost::computePeak( const CellLocation& source, 
                            const uint8_t elevationSource, 
                            const CellLocation& target, 
                            const uint8_t elevationTarget )
{
    return std::max(elevationSource, elevationTarget);
}

double Cost::computeMeanPeak( const CellLocation& source, 
                            const uint8_t elevationSource, 
                            const CellLocation& target, 
                            const uint8_t elevationTarget )
{
    return (elevationSource + elevationTarget) / 2.0;
}