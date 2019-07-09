/** 
 * @file a-star.hpp
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
 * A* implementation for different heuristic and cost function model
 * reference: http://aigamedev.com/open/tutorials/theta-star-any-angle-paths/
 * reference: https://www.redblobgames.com/pathfinding/
 * reference: http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html
 */

#ifndef __A_STAR_H__
#define __A_STAR_H__

#include <iostream>
#include <vector>
#include "square_grid_graph.hpp"
#include "shortest_path.hpp"

/**
 * @brief A* shortest path class
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
    explicit AStar(const std::string& downHillCostModel, const std::string& upHillCostModel);

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
     * @brief override function of shortest path class that computes the shortest path inside a grid graph from source cell to target cell
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
     * @brief update weight and parent a cell if the new computed weight is less than current one
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
     * @brief find path from source cell to target cell
     * 
     * @param graph square grid graph
     * @param source input cell content
     * @param target target cell content
     */
    void updatePath (SquareGridGraph<T, P>& graph, const P& source, const P& target);
};

#endif /* __A_STAR_H__ */