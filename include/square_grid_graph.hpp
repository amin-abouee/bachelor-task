/** 
 * @file grid-graph.hpp
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
 */

#ifndef __SQUARE_GRID_GRAPH_H__
#define __SQUARE_GRID_GRAPH_H__

#include <iostream>
#include <vector>
#include <memory>
#include "matrix.hpp"
#include "cell_info.hpp"

/**
 * @brief Graph class representation for square grid graph. This representation is specialized to find the shortest path \n
 * reference: Design Patterns for the Implementation of Graph Algorithms (http://www.dietmar-kuehl.de/generic-graph-algorithms.pdf) \n
 * reference: https://www.redblobgames.com/pathfinding/a-star/implementation.html#cplusplus \n
 * reference: https://www.redblobgames.com/pathfinding/a-star/making-of.html
 * 
 * @tparam T type of grid cell value
 * @tparam P how we can traverse (move) in this graph (for example 2D, 3D or which kind of movements are possible)
 */
template <typename  T, typename P>
class SquareGridGraph final
{
public:
    ///C'tor
    explicit SquareGridGraph(const uint32_t gridSize, const uint8_t possibleMovements);

    //D'tor
    virtual ~SquareGridGraph() = default;

    //Copy C'tor
    SquareGridGraph(const SquareGridGraph & rhs) = default;

    //move C'tor
    SquareGridGraph(SquareGridGraph && rhs) = default;

    //Copy assignment operator
    SquareGridGraph &operator=(const SquareGridGraph & rhs) = default;

    //move assignment operator
    SquareGridGraph &operator=(SquareGridGraph && rhs) = default;

    /**
     * @brief overload operator ()
     * 
     * @param row y axis
     * @param column x axis
     * @return const T& 
     */
    const T & operator()(const uint32_t row, const uint32_t column) const;

    /**
     * @brief overload operator ()
     * 
     * @param row y axis
     * @param column x axis
     * @return T& 
     */
    T & operator()(const uint32_t row, const uint32_t column);

    /**
     * @brief overload operator ()
     * 
     * @param location contain x and y position
     * @return const T& 
     */
    const T & operator()(const P& location) const;

    /**
     * @brief overload operator ()
     * 
     * @param location contain x and y position
     * @return T& 
     */
    T & operator()(const P& location);


    /**
     * @brief initialize all variable of template T to default value
     * 
     */
    void initializeAllCells();

    /**
     * @brief find all available and possible neighbours of input cell in graph
     * 
     * @param source input cell
     * @param overrides matrix contains the eligible and ineligible cells
     * @param neighbours vector of cells
     */
    void findNeighbours(const P& source, const Matrix<uint8_t>& overrides, std::vector<P>& neighbours);


    int32_t getGridSize () const;

    // friend std::ostream& operator>>( std::istream& os, SquareGridGraph& graph);

private:
    /**
     * @brief check the boundary of grid graph for input value
     * 
     * @param location input value
     * @return true if input is inside the grid
     * @return false if input is not inside the grid
     */
    bool inBounds(const P& location);

    bool isAccessible (const P& cell, const Matrix<uint8_t>& overrides);

    /// size of grid (rows * cols)
    int32_t m_gridSize;
    /// number of possible movements that you can move in this grid
    int8_t m_possibleMovements;
    /// graph data 
    std::unique_ptr< Matrix<T> > m_graphData;
    /// all possible movements that you can move in this grid
    std::vector<P> movements;
};

#endif /* __SQUARE_GRID_GRAPH_H__ */