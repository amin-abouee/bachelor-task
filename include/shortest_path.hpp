/** 
 * @file shortest_path.hpp
 * @author  Amin Abouee
 * @date 05.06.2019
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
 * This is a abstract class for finding the shortest path inside a grid graph
 *
 */

#ifndef __SHORTEST_PATH_H__
#define __SHORTEST_PATH_H__

#include <iostream>
#include <vector>
#include "square_grid_graph.hpp"
#include "cost.hpp"

/**
 * @brief Abstract class for finding the shortest path inside a grid graph
 * 
 * @tparam T Template type name for cell content
 * @tparam P How to move in this grid graph
 */
template <typename  T, typename P>
class ShortestPath
{
public:
    ///C'tor
    explicit ShortestPath();

    /**
     * @brief Construct a new Shortest Path object
     * 
     * @param downHillCostModel represents the desire model for computing the cost in up hill movements (see CostModel enum in cost.hpp class)
     * @param upHillCostModel represents the desire model for computing the cost in down hill movements (see CostModel enum in cost.hpp class)
     */
    explicit ShortestPath(const std::string& downHillCostModel, const std::string& upHillCostModel);
    
    /// D'tor
    virtual ~ShortestPath() = default;

    /// Copy C'tor
    ShortestPath(const ShortestPath & rhs) = default;

    /// Move C'tor
    ShortestPath(ShortestPath && rhs) = default;

    /// Copy assignment operator
    ShortestPath &operator=(const ShortestPath & rhs) = default;

    /// Move assignment operator
    ShortestPath &operator=(ShortestPath && rhs) = default;

    /**
     * @brief main function that computes the shortest path inside a grid graph from source cell to target cell
     * 
     * @param graph input graph
     * @param elevation altitude of grid graph
     * @param overrides the eligible and ineligible cells in grid
     * @param source start position in graph
     * @param target target position in graph
     */
    virtual void findShortestPath(SquareGridGraph<T, P>& graph, 
                            const Matrix<uint8_t>& elevation, 
                            const Matrix<uint8_t>& overrides, 
                            const P& source, 
                            const P& target) = 0;

protected:
    /// The number of cells that expanded by shortest path algorithms (good for evaluation and summery)
    std::uint32_t m_cntExpandedCells;
    /// The number of cells which was found by shortest path algorithms as path from source to target (good for evaluation and summery)
    std::uint32_t m_cntTotalPath;
    /// The number of path movements that were straight, means up, down, left, right  (good for evaluation and summery)
    std::uint32_t m_cntTotalStraightPath;
    /// The number of path movements that were diagonal, means up-left, up-right, down-left, down-right  (good for evaluation and summery)
    std::uint32_t m_cntTotalDiagonalPath;
    /// The cost object for up hill movements
    std::unique_ptr<Cost> m_upHillCostEstimator;
    /// The cost object for down hill movements
    std::unique_ptr<Cost> m_downHillCostEstimator;
    /// The cost model for up hill movements
    Cost::CostModel m_upHillCostModel;
    /// The cost model for down hill movements
    Cost::CostModel m_downHillCostModel;
};

#endif /* __SHORTEST_PATH_H__ */