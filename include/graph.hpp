/** 
 * @file graph.hpp
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

#ifndef __GRAPH_H__
#define __GRAPH_H__

// template <typename  T>
class Graph
{
public:
    //C'tor
    explicit Graph(const uint32_t nVertices, const uint32_t nEdges);
    //D'tor
    virtual ~Graph() = default;

    //Copy C'tor
    Graph(const Graph & rhs) = default;
    //move C'tor
    Graph(Graph && rhs) = default;
    //Copy assignment operator
    Graph &operator=(const Graph & rhs) = default;
    //move assignment operator
    Graph &operator=(Graph && rhs) = default;

    virtual const T & operator()(const uint32_t row, const uint32_t column) const = 0;

    virtual T & operator()(const uint32_t row, const uint32_t column) = 0;

    virtual bool inBounds(const uint32_t row, const uint32_t column) = 0;

    virtual void makeGraph() = 0;

    virtual friend std::ostream& operator>>( std::istream& os, Graph& graph) =;

private:
    uint32_t m_nVertices;
    uint32_t m_nEdges;
};

#endif /* __GRAPH_H__ */