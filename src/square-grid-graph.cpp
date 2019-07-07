#include "square-grid-graph.hpp"
#include <iostream>
#include <numeric>

template <typename  T, typename P>
SquareGridGraph<T,P>::SquareGridGraph(const uint32_t gridSize, const uint8_t possibleMovements): m_gridSize(gridSize), m_graphData(std::make_unique<Matrix<T>>(m_gridSize, m_gridSize)), m_possibleMovements(possibleMovements)
{
    for(int32_t i(0); i< m_gridSize; i++)
    {
        for(int32_t j(0); j< m_gridSize; j++)
        {
            auto& currentCell = m_graphData->operator()(i,j);
            currentCell.setLoc(j ,i);
        }
    }

    // octile movement
    // draw in code 
    if (m_possibleMovements == 8)
    {
        movements.emplace_back(P{-1, 0});
        movements.emplace_back(P{-1, -1});
        movements.emplace_back(P{0, -1});
        movements.emplace_back(P{1, -1});
        movements.emplace_back(P{1, 0});
        movements.emplace_back(P{1, 1});
        movements.emplace_back(P{0, 1});
        movements.emplace_back(P{-1, 1});
    }
}

template <typename  T, typename P>
const T & SquareGridGraph<T,P>::operator()(const uint32_t row, const uint32_t col) const
{
    return m_graphData->operator()(row, col);
}

template <typename  T, typename P>
T & SquareGridGraph<T,P>::operator()(const uint32_t row, const uint32_t col)
{
    return m_graphData->operator()(row, col);
}

template <typename  T, typename P>
const T & SquareGridGraph<T,P>::operator()(const P& location) const
{
    return m_graphData->operator()(location.y, location.x);
}

template <typename  T, typename P>
T & SquareGridGraph<T,P>::operator()(const P& location)
{
    return m_graphData->operator()(location.y, location.x);
}

template <typename  T, typename P>
void SquareGridGraph<T,P>::findNeighbours(const P& source, const Matrix<uint8_t>& overrides,  std::vector<P>& neighbours)
{
    for (auto& dir : movements) 
    {
        P next{source.x + dir.x, source.y + dir.y};
        if (inBounds(next.y, next.x) && overrides(next.y, next.x) == 0) 
        {
            neighbours.push_back(next);
        }
    }
}

template <typename  T, typename P>
void SquareGridGraph<T,P>::initializeAllCells()
{
    for(int32_t i(0); i< m_gridSize; i++)
    {
        for(int32_t j(0); j< m_gridSize; j++)
        {
            auto& currentCell = m_graphData->operator()(i,j);
            currentCell.initialize();
        }
    }
}

template <typename  T, typename P>
bool SquareGridGraph<T,P>::inBounds(const uint32_t row, const uint32_t col)
{
    return 0 <= col && col < m_gridSize
        && 0 <= row && row < m_gridSize;
}

// template <typename  T, typename P>
// std::ostream& operator>>( std::istream& out, SquareGridGraph<T, P>& graph)
// {
//     out << graph;
//     return out;
// }


template class SquareGridGraph<GridWithWeights, GridLocation>;