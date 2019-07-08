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
 * reference: Design Patterns for the Implementation of Graph Algorithms (http://www.dietmar-kuehl.de/generic-graph-algorithms.pdf)
 *
 */

#ifndef __SQUARE_GRID_GRAPH_H__
#define __SQUARE_GRID_GRAPH_H__

#include <iostream>
#include <vector>
#include <memory>
#include "matrix.hpp"
#include "cell_info.hpp"

template <typename  T, typename P>
class SquareGridGraph final
{
public:
    //C'tor
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

    const T & operator()(const uint32_t row, const uint32_t column) const;

    T & operator()(const uint32_t row, const uint32_t column);

    const T & operator()(const P& location) const;

    T & operator()(const P& location);

    void initializeAllCells();

    void findNeighbours(const P& source, const Matrix<uint8_t>& overrides, std::vector<P>& neighbours);

    // friend std::ostream& operator>>( std::istream& os, SquareGridGraph& graph);

private:
    bool inBounds(const P& location);
    int32_t m_gridSize;
    int8_t m_possibleMovements;
    std::unique_ptr<Matrix<T>> m_graphData;
    std::vector<P> movements;
};

#endif /* __SQUARE_GRID_GRAPH_H__ */