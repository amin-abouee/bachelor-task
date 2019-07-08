#include "shortest_path.hpp"

template <typename  T, typename P>
ShortestPath<T,P>::ShortestPath(): m_cntExploredCells(0)
{

}

template class ShortestPath<CellData, CellLocation>;

