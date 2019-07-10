/** 
 * @file bidirectional.hpp
 * @author  Amin Abouee
 * @date 09.06.2019
 *
 * @section LICENSE
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details at
 * https://www.gnu.org/copyleft/gpl.html
 *
 * @section DESCRIPTION
 */

#ifndef __BIDIRECTIONAL_SEARCH_H__
#define __BIDIRECTIONAL_SEARCH_H__

#include <iostream>
#include <vector>
#include <set>
#include <queue>
#include "square_grid_graph.hpp"
#include "shortest_path.hpp"
#include "heuristic.hpp"

/**
 * @brief A* implementation for different heuristic and cost function model \n
 * reference: http://aigamedev.com/open/tutorials/theta-star-any-angle-paths/ \n
 * reference: https://www.redblobgames.com/pathfinding/ \n
 * reference: http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html \n
 * 
 * @tparam T data contain value
 * @tparam P type of movements
 */
template <typename  T, typename P>
class BidirectionalSearch final : public ShortestPath<T,P>
{
public:
    /// C'tor
    explicit BidirectionalSearch();
    
    /// C'tor
    explicit BidirectionalSearch(const std::string& downHillCostModel, const std::string& upHillCostModel, const std::string& heuristicModel);

    /// D'tor
    virtual ~BidirectionalSearch() = default;

    /// Copy C'tor
    BidirectionalSearch(const BidirectionalSearch & rhs) = default;

    /// Move C'tor
    BidirectionalSearch(BidirectionalSearch && rhs) = default;

    /// Copy assignment operator
    BidirectionalSearch &operator=(const BidirectionalSearch & rhs) = default;

    /// Move assignment operator
    BidirectionalSearch &operator=(BidirectionalSearch && rhs) = default;

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

    using PQCell = std::pair<P, double>;
    // auto compare = [](const PQCell& lhs, const PQCell& rhs)
    //             {
    //                 return lhs.second > rhs.second; 
    //             };

    struct compare 
    {
        bool operator() (const PQCell& lhs, const PQCell& rhs) const
        {
            // if (lhs.first.X() != rhs.first.X() || lhs.first.Y() != rhs.first.Y())
            //     return true;
            // else
            //     return lhs.second < rhs.second;
            // return lhs.second < rhs.second || (lhs.second == rhs.second && lhs.first != rhs.first );
            // return lhs.first != rhs.first || (lhs.first == rhs.first && lhs.second < rhs.second );
            return (lhs.first == rhs.first && lhs.second < rhs.second );
        }
    };

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
    bool relax(const T& current, T& next, double cost) const;

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
    void checkPath (SquareGridGraph<T, P>& graph, const Matrix<uint8_t>& overrides);

    const double computeAverageAltitudeInPath (const P& source, 
                                                const P& target, 
                                                const Matrix<uint8_t>& elevation, 
                                                const Matrix<uint8_t>& overrides);

    void traverseDirectional (SquareGridGraph<T, P>& graph, 
                                std::priority_queue<PQCell, std::vector<PQCell>, compare>& frontier, 
                                T& currentCell,
                                const P& target,
                                const Matrix<uint8_t>&override, 
                                const Matrix<uint8_t>&elevation);

    Heuristic m_heuristicEstimator;
    Heuristic::HeuristicModel m_heuristicModel;
};

#endif /* __BIDIRECTIONAL_H__SEARCH_ */