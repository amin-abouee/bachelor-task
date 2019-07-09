#include "square_grid_graph.hpp"
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
    // Generating all the 8 successor of this cell 
    // N.W   N   N.E 
    //   
    //     
    // W----Cell----E 
    //      
    //    
    // S.W    S   S.E 
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
    return m_graphData->operator()(location.Y(), location.X());
}

template <typename  T, typename P>
T & SquareGridGraph<T,P>::operator()(const P& location)
{
    return m_graphData->operator()(location.Y(), location.X());
}

template <typename  T, typename P>
void SquareGridGraph<T,P>::findNeighbours(const P& source, const Matrix<uint8_t>& overrides,  std::vector<P>& neighbours)
{
    for (auto& dir : movements) 
    {
        P next{source.X() + dir.X(), source.Y() + dir.Y()};
        if (inBounds(next) && isAccessible(next, overrides) )
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
bool SquareGridGraph<T,P>::inBounds(const P& location)
{
    return 0 <= location.X() && location.X() < m_gridSize
        && 0 <= location.Y() && location.Y() < m_gridSize;
}

template <typename  T, typename P>
bool SquareGridGraph<T,P>::isAccessible (const P& cell, const Matrix<uint8_t>& overrides)
{
    if (overrides(cell.Y(), cell.X()) == 0 && m_graphData->operator()(cell.Y(), cell.X()).getVisited() == false)
        return true;
    else
        return false;
}


template <typename  T, typename P>
int32_t SquareGridGraph<T,P>::getGridSize () const
{
    return m_gridSize;
}


// template <typename  T, typename P>
// std::ostream& operator>>( std::istream& out, SquareGridGraph<T, P>& graph)
// {
//     out << graph;
//     return out;
// }


template class SquareGridGraph<CellData, CellLocation>;