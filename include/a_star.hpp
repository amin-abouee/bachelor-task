/** 
 * @file a-star.hpp
 * @author  Amin Abouee
 * @date 05.06.2019
 *
 * @section DESCRIPTION
 */

#ifndef __A_STAR_H__
#define __A_STAR_H__

#include <iostream>
#include <vector>
#include "square_grid_graph.hpp"
#include "shortest_path.hpp"
#include "heuristic.hpp"

/**
 * @brief A* implementation that works with any arbitrary heuristic and cost function models \n
 * reference: http://aigamedev.com/open/tutorials/theta-star-any-angle-paths/ \n
 * reference: https://www.redblobgames.com/pathfinding/ \n
 * reference: http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html \n
 * 
 * @tparam T data contain value
 * @tparam P type of movements
 */
template <typename  T, typename P>
class AStar final : public ShortestPath<T,P>
{
public:
    /// C'tor
    explicit AStar();
    
    /// C'tor
    explicit AStar(const std::string& downHillCostModel, const std::string& upHillCostModel, const std::string& heuristicModel);

    /// D'tor
    virtual ~AStar() = default;

    /// Copy C'tor
    AStar(const AStar & rhs) = default;

    /// Move C'tor
    AStar(AStar && rhs) = default;

    /// Copy assignment operator
    AStar &operator=(const AStar & rhs) = default;

    /// Move assignment operator
    AStar &operator=(AStar && rhs) = default;

    /**
     * @brief Override function of shortest path class that computes the shortest path inside a grid graph from source cell to target cell
     * 
     * @param graph input graph
     * @param elevation altitude of grid graph
     * @param overrides the eligible and ineligible cells in grid
     * @param source start position in graph
     * @param target target position in graph
     */
    void findShortestPath(SquareGridGraph<T, P>& graph, 
                            const Matrix<uint8_t>& elevation, 
                            const Matrix<uint8_t>& overrides, 
                            const P& source, 
                            const P& target) override;

private:
    /**
     * @brief Update weight and parent a cell if the new computed weight is less than current one
     * reference: Introduction to algorithms, CLRS, chapter 24, page 649
     * 
     * @param current value and location of current cell
     * @param next value and location of next cell
     * @param cost computed cost from current to next
     * @return true edge relaxed
     * @return false no relaxation 
     */
    bool relax(T& current, T& next, double cost) const;

    /**
     * @brief Find path from source cell to target cell
     * 
     * @param graph square grid graph
     * @param source input cell content
     * @param target target cell content
     */
    void updatePath (SquareGridGraph<T, P>& graph, const P& source, const P& target);

    /**
     * @brief Print summary information from all paths
     * 
     * @param source 
     * @param target 
     */
    void printSummary (const P& source, const P& target, const double cost);

    /**
     * @brief Check all paths and visited path with overrides and elevation (check possibility of error)
     * 
     * @param graph 
     * @param overrides 
     * @param elevation 
     */
    // void checkPath (SquareGridGraph<T, P>& graph, const Matrix<uint8_t>& overrides);

    /**
     * @brief Approximate average altitude based on the max and min from source to target
     * 
     * @param source 
     * @param target 
     * @param elevation 
     * @param overrides 
     * @return const double 
     */
    const double computeAverageAltitudeInPath (const P& source, 
                                                const P& target, 
                                                const Matrix<uint8_t>& elevation, 
                                                const Matrix<uint8_t>& overrides);

    Heuristic m_heuristicEstimator;
    Heuristic::HeuristicModel m_heuristicModel;
};

#endif /* __A_STAR_H__ */