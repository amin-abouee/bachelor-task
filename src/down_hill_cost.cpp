#include "down_hill_cost.hpp"
#include "cmath"

DownHillCost::DownHillCost() : Cost()
{
    
}

double DownHillCost::computeL2( const CellLocation& source, 
                            const uint8_t elevationSource, 
                            const CellLocation& target, 
                            const uint8_t elevationTarget )
{
    /// kinetic friction for down hill movement. 
    const double KineticFriction = 1.2;
    const auto dx = target.X() - source.X();
    const auto dy = target.Y() - source.Y();
    const auto dz = elevationTarget - elevationSource;
    return (KineticFriction / std::sqrt(dx*dx + dy*dy + dz*dz));
}

double DownHillCost::computeL1( const CellLocation& source, 
                            const uint8_t elevationSource, 
                            const CellLocation& target, 
                            const uint8_t elevationTarget )

{
    /// kinetic friction for down hill movement. 
    const double KineticFriction = 2.0;
    const auto dx = std::abs(target.X() - source.X());
    const auto dy = std::abs(target.Y() - source.Y());
    const auto dz = std::abs(elevationTarget - elevationSource);
    return  (KineticFriction / (dx + dy + dz));
}

double DownHillCost::computeLInf( const CellLocation& source, 
                            const uint8_t elevationSource, 
                            const CellLocation& target, 
                            const uint8_t elevationTarget )
{
    return (1 / (double)std::abs(elevationTarget- elevationSource));
}

double DownHillCost::computeAngle( const CellLocation& source, 
                            const uint8_t elevationSource, 
                            const CellLocation& target, 
                            const uint8_t elevationTarget )
{
    const double pi = double(3.1415926535897932385);
    const double level = 15.0;
    const double KineticFriction = 0.25;

    const auto distance = std::abs(source.X() - target.X()) + std::abs(source.Y() - target.Y());
    double dxdy = 1.0;
    if (distance == 2)
        dxdy = std::sqrt(2.0);
    const double dz = std::abs(elevationTarget - elevationSource);
    return ((std::atan2(dz, dxdy) * 180 / pi) / level) * KineticFriction;
} 
                            
double DownHillCost::computeDifficultyLevel( const CellLocation& source, 
                            const uint8_t elevationSource, 
                            const CellLocation& target, 
                            const uint8_t elevationTarget )
{
    const double pi = double(3.1415926535897932385);
    const double level = 15.0;
    const double KineticFriction = 0.25;

    const auto dx = std::abs(target.X() - source.X());
    const auto dy = std::abs(target.Y() - source.Y());
    double dxdy = 1.0;
    if (dx + dy == 2)
        dxdy = std::sqrt(2.0);
    const double dz = std::abs(elevationTarget - elevationSource);
    const double angleInDegree = (std::atan2(dz, dxdy) * 180 / pi);
    const double norm2 = std::sqrt(dx * dx + dy*dy + dz * dz);
    return (norm2 * (angleInDegree/level) * KineticFriction);
}

double DownHillCost::computeL2Trimm( const CellLocation& source, 
                            const uint8_t elevationSource, 
                            const CellLocation& target, 
                            const uint8_t elevationTarget )
{
    const double pi = double(3.1415926535897932385);
    const double KineticFriction = 1.2;
    const auto dx = std::abs(target.X() - source.X());
    const auto dy = std::abs(target.Y() - source.Y());
    double dxdy = 1.0;
    if (dx + dy == 2)
        dxdy = std::sqrt(2.0);
    const auto dz = std::abs(elevationTarget - elevationSource);
    const double angleInDegree = (std::atan2(dz, dxdy) * 180 / pi);
    const double norm2 = std::sqrt(dx * dx + dy*dy + dz * dz);
    
    if (angleInDegree > 60.0)
        return std::numeric_limits<double>::max() ;
    return KineticFriction/norm2;
}