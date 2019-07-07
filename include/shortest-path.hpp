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
 * reference: Design Patterns for the Implementation of Graph Algorithms (http://www.dietmar-kuehl.de/generic-graph-algorithms.pdf)
 *
 */

#ifndef __SHORTEST_PATH_H__
#define __SHORTEST_PATH_H__

#include <iostream>
#include <vector>
#include "square-grid-graph.hpp"

template <typename  T, typename P>
class ShortestPath
{
public:
    //C'tor
    explicit ShortestPath();
    //D'tor
    virtual ~ShortestPath() = default;

    //Copy C'tor
    ShortestPath(const ShortestPath & rhs) = default;
    //move C'tor
    ShortestPath(ShortestPath && rhs) = default;
    //Copy assignment operator
    ShortestPath &operator=(const ShortestPath & rhs) = default;
    //move assignment operator
    ShortestPath &operator=(ShortestPath && rhs) = default;

    virtual void findShortestPath(SquareGridGraph<T, P>& graph, 
                            const Matrix<uint8_t>& elevation, 
                            const Matrix<uint8_t>& overrides, 
                            const P& source, 
                            const P& target) = 0;

protected:
    std::uint32_t m_cntExploredCells;
};

#endif /* __SHORTEST_PATH_H__ */