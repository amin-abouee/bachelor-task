/**
* @file heuristic.hpp
* @brief compute cost for altitude uphill and downhill movement
*
* @date 08.07.2019
* @author Amin Abouee
*
* @section DESCRIPTION
*/
#ifndef __HEURISTIC_H__
#define __HEURISTIC_H__

#include <iostream>
#include <map>
#include "cell_info.hpp"
#include "matrix.hpp"

/**
 * @brief Compute heuristic estimation for A* algorithm \n
 *   reference: http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html#heuristics-for-grid-maps \n
 *   reference: http://aigamedev.com/open/tutorials/theta-star-any-angle-paths/
 */

class Heuristic
{
public:

    /**
     * @brief enum class for all supporting heuristic models
     * 
     */
    enum class HeuristicModel : unsigned int
    {
        Dijkstra           = 0,
        L1                 = 1,
        L2                 = 2,
        Diagonal           = 3,
        L1Altitude         = 4,
        L2Altitude         = 5,
        DiagonalAltitude   = 6
    };

    ///C'tor
    explicit Heuristic();

    /// Copy C'tor
    Heuristic(const Heuristic& rhs) = default;

    /// Move C'tor
    Heuristic(Heuristic&& rhs) = default;

    /// Copy assignment operator
    Heuristic& operator=(const Heuristic& rhs) = default;

    /// Move assignment operator
    Heuristic& operator=(Heuristic&& rhs) = default;

    //D'tor
    ~Heuristic() = default;

    /// Mapping from string to enum class, used for outside initialization
    std::map< std::string, HeuristicModel > allModels;

    /**
     * @brief Estimate heuristic from source to target cell based on selected model
     * 
     * @param source 
     * @param elevationSource 
     * @param target 
     * @param elevationTarget 
     * @param model
     * @return double 
     */
    double computeHeuristic(const CellLocation& source, 
                                const CellLocation& target, 
                                const HeuristicModel& model,
                                const double aveAltitude );

    /**
     * @brief Compute dijkstra algorithm (brute force search)
     * 
     * @param source 
     * @param target 
     * @param model 
     * @param aveAltitude 
     * @return 0
     */
    double computeDijkstra( const CellLocation& source, 
                        const CellLocation& target, 
                        const HeuristicModel& model,
                        const double aveAltitude );
    
    /**
     * @brief Compute manhattan distance between source and target
     * 
     * @param source 
     * @param target 
     * @param model 
     * @param aveAltitude 
     * @return abs(dx) + abs(dy)
     */
    double computeL1( const CellLocation& source, 
                        const CellLocation& target, 
                        const HeuristicModel& model ,
                        const double aveAltitude); 

    /**
     * @brief Compute euclidean distance between source and target
     * 
     * @param source 
     * @param target 
     * @param model 
     * @param aveAltitude 
     * @return sqrt( dx^2 + dy^2 + dz^2)
     */
    double computeL2( const CellLocation& source, 
                        const CellLocation& target, 
                        const HeuristicModel& model ,
                        const double aveAltitude); 

    /**
     * @brief Compute euclidean distance between source and target
     * 
     * @param source 
     * @param target 
     * @param model 
     * @param aveAltitude 
     * @return max(dx, dy) + (sqrt(2)-1) * min(dx, dy)
     */
    double computeDiagonal( const CellLocation& source, 
                        const CellLocation& target, 
                        const HeuristicModel& model ,
                        const double aveAltitude);

private:
};

 #endif /* __HEURISTIC_H__ */