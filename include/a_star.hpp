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
 * reference: Design Patterns for the Implementation of Graph Algorithms (http://www.dietmar-kuehl.de/generic-graph-algorithms.pdf)
 *
 */

#ifndef __A_STAR_H__
#define __A_STAR_H__

#include <iostream>
#include <vector>
#include "square_grid_graph.hpp"
#include "shortest_path.hpp"

template <typename  T, typename P>
class AStar final : public ShortestPath<T,P>
{
public:
    //C'tor
    explicit AStar();
    //D'tor
    virtual ~AStar() = default;

    //Copy C'tor
    AStar(const AStar & rhs) = default;
    //move C'tor
    AStar(AStar && rhs) = default;
    //Copy assignment operator
    AStar &operator=(const AStar & rhs) = default;
    //move assignment operator
    AStar &operator=(AStar && rhs) = default;

    void findShortestPath(SquareGridGraph<T, P>& graph, 
                            const Matrix<uint8_t>& elevation, 
                            const Matrix<uint8_t>& overrides, 
                            const P& source, 
                            const P& target) override;

private:
    bool relax(T& current, T& next, double weight) const;
    void updatePath (SquareGridGraph<T, P>& graph, const P& source, const P& target);
};

#endif /* __A_STAR_H__ */