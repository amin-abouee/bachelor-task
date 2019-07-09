/**
* @file up_hill_cost,.hpp
* @brief computed cost for down
*
* @date  08.07.2019
* @author Amin Abouee
*
* @section DESCRIPTION
* Compute cost function for down hill movements for all defined method in Cost class
*
*/
#ifndef __DOWN_HILL_COST_H__
#define __DOWN_HILL_COST_H__

#include <iostream>
#include "cost.hpp"

/**
 * @brief Compute cost function for down hill movements for all defined method in Cost class (inheritance of Cost)
 * 
 */
class DownHillCost final : public Cost
{
public:
    /// C'tor
    explicit DownHillCost();

    /// Copy C'tor
    DownHillCost(const DownHillCost& rhs) = default;

    /// Move C'tor
    DownHillCost(DownHillCost&& rhs) = default;

    /// Copy assignment operator
    DownHillCost& operator=(const DownHillCost& rhs) = default;

    /// Move assignment operator
    DownHillCost& operator=(DownHillCost&& rhs) = default;

    /// D'tor
    ~DownHillCost() = default;

    /**
     * @brief Compute L2 norm from source to target in 3D with effect of experimental constant kinetic friction 
     * 
     * @param source 
     * @param elevationSource 
     * @param target 
     * @param elevationTarget 
     * @return double kinetic_friction / sqrt(dx^2 + dy^2 + dz^2)
     */
    double computeL2( const CellLocation& source, 
                                const uint8_t elevationSource, 
                                const CellLocation& target, 
                                const uint8_t elevationTarget ); 

    /**
     * @brief Compute L1 norm from source to target in 3D with effect of experimental constant kinetic friction (Manhattan Distance)
     * 
     * @param source 
     * @param elevationSource 
     * @param target 
     * @param elevationTarget 
     * @return double kinetic_friction / abs(dx) + abs(dy) + abs(dz)
     */
    double computeL1( const CellLocation& source, 
                                const uint8_t elevationSource, 
                                const CellLocation& target, 
                                const uint8_t elevationTarget ); 

    /**
     * @brief Compute infinity norm from source to target in 3D with effect of experimental constant kinetic friction
     * 
     * @param source 
     * @param elevationSource 
     * @param target 
     * @param elevationTarget 
     * @return double kinetic_friction / maximum(dx, dy, dz)
     */
    double computeLInf( const CellLocation& source, 
                                const uint8_t elevationSource, 
                                const CellLocation& target, 
                                const uint8_t elevationTarget ); 

    /**
     * @brief instead of euclidean distance, apply a ratio of angle between source and target with effect of experimental constant kinetic friction
     * 
     * @param source 
     * @param elevationSource 
     * @param target 
     * @param elevationTarget 
     * @return double  (angle(source, target) / 15) * kinetic_friction
     */
    double computeAngle( const CellLocation& source, 
                                const uint8_t elevationSource, 
                                const CellLocation& target, 
                                const uint8_t elevationTarget ); 
                                
    /**
     * @brief consider the effect of angle and euclidean distance between source and target with effect of experimental constant kinetic friction
     * 
     * @param source 
     * @param elevationSource 
     * @param target 
     * @param elevationTarget 
     * @return double (angle(source, target) / 15) * kinetic_friction * norm L2
     */
    double computeDifficultyLevel( const CellLocation& source, 
                                const uint8_t elevationSource, 
                                const CellLocation& target, 
                                const uint8_t elevationTarget ); 

 };

 #endif /* __DOWN_HILL_COST_H__ */