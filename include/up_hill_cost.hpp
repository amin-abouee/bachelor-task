/**
* @file up_hill_cost,.hpp
* @brief computed cost for uphill
*
* @date 08.07.2019
* @author Amin Abouee
*
* @section DESCRIPTION
*
*/
#ifndef __UP_HILL_COST_H__
#define __UP_HILL_COST_H__

#include <iostream>
#include "cost.hpp"

/**
 * @brief Compute cost function for up hill movements for all defined method in Cost class (inheritance of Cost)
 * 
 */
class UpHillCost final : public Cost
{
public:
    /// C'tor
    explicit UpHillCost();

    /// Copy C'tor
    UpHillCost(const UpHillCost& rhs) = default;

    /// Move C'tor
    UpHillCost(UpHillCost&& rhs) = default;

    /// Copy assignment operator
    UpHillCost& operator=(const UpHillCost& rhs) = default;

    /// Move assignment operator
    UpHillCost& operator=(UpHillCost&& rhs) = default;

    ///D'tor
    ~UpHillCost() = default;

    /**
     * @brief Compute L2 norm from source to target in 3D 
     * 
     * @param source 
     * @param elevationSource 
     * @param target 
     * @param elevationTarget 
     * @return double sqrt(dx^2 + dy^2 + dz^2)
     */
    double computeL2( const CellLocation& source, 
                                const uint8_t elevationSource, 
                                const CellLocation& target, 
                                const uint8_t elevationTarget ); 
    /**
     * @brief Compute L1 norm from source to target in 3D (Manhattan Distance)
     * 
     * @param source 
     * @param elevationSource 
     * @param target 
     * @param elevationTarget 
     * @return double abs(dx) + abs(dy) + abs(dz)
     */
    double computeL1( const CellLocation& source, 
                                const uint8_t elevationSource, 
                                const CellLocation& target, 
                                const uint8_t elevationTarget ); 

    /**
     * @brief Compute infinity norm from source to target in 3D
     * 
     * @param source 
     * @param elevationSource 
     * @param target 
     * @param elevationTarget 
     * @return double maximum(dx, dy, dz)
     */
    double computeLInf( const CellLocation& source, 
                                const uint8_t elevationSource, 
                                const CellLocation& target, 
                                const uint8_t elevationTarget ); 
    
    /**
     * @brief instead of euclidean distance, apply a ratio of angle between source and target
     * 
     * @param source 
     * @param elevationSource 
     * @param target 
     * @param elevationTarget 
     * @return double angle(source, target) / 15
     */
    double computeAngle( const CellLocation& source, 
                                const uint8_t elevationSource, 
                                const CellLocation& target, 
                                const uint8_t elevationTarget ); 

    /**
     * @brief consider the effect of angle and euclidean distance between source and target
     * 
     * @param source 
     * @param elevationSource 
     * @param target 
     * @param elevationTarget 
     * @return double angle(source, target) / 15 * norm L2
     */                     
    double computeDifficultyLevel( const CellLocation& source, 
                                const uint8_t elevationSource, 
                                const CellLocation& target, 
                                const uint8_t elevationTarget ); 

    /**
     * @brief Compute L2 norm from source to target if angle is less than 60
     * 
     * @param source 
     * @param elevationSource 
     * @param target 
     * @param elevationTarget 
     * @return double norm2 if angle is less than 60
     */
    double computeL2Trimm( const CellLocation& source, 
                            const uint8_t elevationSource, 
                            const CellLocation& target, 
                            const uint8_t elevationTarget );

 };

 #endif /* __UP_HILL_COST_H__ */