#include "up_hill_cost.hpp"
#include "cmath"
#include "algorithm"

UpHillCost::UpHillCost() : Cost()
{
    
}

double UpHillCost::computeL2( const CellLocation& source, 
                            const uint8_t elevationSource, 
                            const CellLocation& target, 
                            const uint8_t elevationTarget )
{
    const double dx = target.X() - source.X();
    const double dy = target.Y() - source.Y();
    const double dz = elevationTarget - elevationSource;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

double UpHillCost::computeL1( const CellLocation& source, 
                            const uint8_t elevationSource, 
                            const CellLocation& target, 
                            const uint8_t elevationTarget )

{
    const double dx = std::abs(target.X() - source.X());
    const double dy = std::abs(target.Y() - source.Y());
    const double dz = std::abs(elevationTarget - elevationSource);
    return (dx + dy + dz);
}

double UpHillCost::computeLInf( const CellLocation& source, 
                            const uint8_t elevationSource, 
                            const CellLocation& target, 
                            const uint8_t elevationTarget )
{
    return (double)std::abs(elevationTarget- elevationSource);
}

double UpHillCost::computeAngle( const CellLocation& source, 
                            const uint8_t elevationSource, 
                            const CellLocation& target, 
                            const uint8_t elevationTarget )
{
    constexpr double pi = double(3.1415926535897932385);
    const double level = 15;

    double distance = std::abs(source.X() - target.X()) + std::abs(source.Y() - target.Y());
    double dxdy = 1.0;
    if (distance == 2.0)
        double dxdy = std::sqrt(2.0);
    const double dz = std::abs(elevationTarget - elevationSource);
    return ((std::atan2(dz, dxdy) * 180 / pi) / level);
} 
                            
double UpHillCost::computeDifficultyLevel( const CellLocation& source, 
                            const uint8_t elevationSource, 
                            const CellLocation& target, 
                            const uint8_t elevationTarget )
{
    const double pi = double(3.1415926535897932385);
    const double level = 15;
    const double dx = std::abs(target.X() - source.X());
    const double dy = std::abs(target.Y() - source.Y());
    double dxdy = 1.0;
    if (dx + dy == 2.0)
        double dxdy = std::sqrt(2.0);
    const double dz = std::abs(elevationTarget - elevationSource);
    
    const double angleInDegree = (std::atan2(dz, dxdy) * 180 / pi);
    const double norm2 = std::sqrt(dx * dx + dy*dy + dz * dz);
    return norm2 * static_cast<int>(angleInDegree/level);
}