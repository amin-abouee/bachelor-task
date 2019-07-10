/**
* @file cost.hpp
* @brief compute cost for altitude uphill and downhill movement
*
* @date 07.07.2019
* @author Amin Abouee
*
* @section DESCRIPTION
*/
#ifndef __COST_H__
#define __COST_H__

#include <iostream>
#include <map>
#include "cell_info.hpp"

/**
 * @brief Compute cost for up hill and down hill movements \n
 *   reference: http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html#heuristics-for-grid-maps \n
 *   reference: http://theory.stanford.edu/~amitp/GameProgramming/MovementCosts.html \n
 *   reference: http://www.adammil.net/blog/v125_Roguelike_Vision_Algorithms.html
 * 
 */
class Cost
{
public:

    /**
     * @brief enum class for all supporting models
     * 
     */
    enum class CostModel : unsigned int
    {
        Octile       = 0,
        Peak         = 1,
        MeanPeak     = 2,
        L2           = 3,
        L1           = 4,
        LInf         = 5,
        Angle        = 6,
        DifficultyLevel = 7
    };

    ///C'tor
    explicit Cost() = default;

    /// Copy C'tor
    Cost(const Cost& rhs) = default;

    /// Move C'tor
    Cost(Cost&& rhs) = default;

    /// Copy assignment operator
    Cost& operator=(const Cost& rhs) = default;

    /// Move assignment operator
    Cost& operator=(Cost&& rhs) = default;

    //D'tor
    virtual ~Cost() = default;

    /// Mapping from string to enum class, used for outside initialization
    static std::map< std::string, CostModel > allModels;

/**
 * @brief estimate cost from source to target cell based on model
 * 
 * @param source 
 * @param elevationSource 
 * @param target 
 * @param elevationTarget 
 * @param model
 * @return double 
 */
double computeCost(const CellLocation& source, 
                            const uint8_t elevationSource, 
                            const CellLocation& target, 
                            const uint8_t elevationTarget,
                            const CostModel& model);

/**
 * @brief Octile movement when source and target have the same elvation
 * 1 for up, down, left, right
 * sqrt(2) for up right, up left, down right, down left
 * 
 * @param source 
 * @param elevationSource 
 * @param target 
 * @param elevationTarget 
 * @return double 
 */
double computeOctile( const CellLocation& source, 
                            const uint8_t elevationSource, 
                            const CellLocation& target, 
                            const uint8_t elevationTarget ); 

/**
 * @brief Select the maximum between source and target peaks
 * 
 * @param source 
 * @param elevationSource 
 * @param target 
 * @param elevationTarget 
 * @return double maximum(elevationSource, elevationTarget)
 */
double computePeak( const CellLocation& source, 
                            const uint8_t elevationSource, 
                            const CellLocation& target, 
                            const uint8_t elevationTarget );

/**
 * @brief Average between peaks
 * 
 * @param source 
 * @param elevationSource 
 * @param target 
 * @param elevationTarget 
 * @return double mean(elevationSource, elevationTarget)
 */
double computeMeanPeak( const CellLocation& source, 
                            const uint8_t elevationSource, 
                            const CellLocation& target, 
                            const uint8_t elevationTarget ); 

/**
 * @brief Compute L2 norm from source to target in 3D 
 * 
 * @param source 
 * @param elevationSource 
 * @param target 
 * @param elevationTarget 
 * @return double sqrt(dx^2 + dy^2 + dz^2)
 */
virtual double computeL2( const CellLocation& source, 
                            const uint8_t elevationSource, 
                            const CellLocation& target, 
                            const uint8_t elevationTarget ) = 0; 

/**
 * @brief Compute L1 norm from source to target in 3D (Manhattan Distance)
 * 
 * @param source 
 * @param elevationSource 
 * @param target 
 * @param elevationTarget 
 * @return double abs(dx) + abs(dy) + abs(dz)
 */
virtual double computeL1( const CellLocation& source, 
                            const uint8_t elevationSource, 
                            const CellLocation& target, 
                            const uint8_t elevationTarget ) = 0; 

/**
 * @brief Compute infinity norm from source to target in 3D
 * 
 * @param source 
 * @param elevationSource 
 * @param target 
 * @param elevationTarget 
 * @return double maximum(dx, dy, dz)
 */
virtual double computeLInf( const CellLocation& source, 
                            const uint8_t elevationSource, 
                            const CellLocation& target, 
                            const uint8_t elevationTarget ) = 0; 

/**
 * @brief instead of euclidean distance, apply a ratio of angle between source and target
 * 
 * @param source 
 * @param elevationSource 
 * @param target 
 * @param elevationTarget 
 * @return double angle(source, target) / 15
 */
virtual double computeAngle( const CellLocation& source, 
                            const uint8_t elevationSource, 
                            const CellLocation& target, 
                            const uint8_t elevationTarget ) = 0; 

/**
 * @brief consider the effect of angle and euclidean distance between source and target
 * 
 * @param source 
 * @param elevationSource 
 * @param target 
 * @param elevationTarget 
 * @return double angle(source, target) / 15 * norm L2
 */
virtual double computeDifficultyLevel( const CellLocation& source, 
                            const uint8_t elevationSource, 
                            const CellLocation& target, 
                            const uint8_t elevationTarget ) = 0; 


 };

 #endif /* __COST_H__ */