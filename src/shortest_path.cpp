#include "shortest_path.hpp"
#include "up_hill_cost.hpp"
#include "down_hill_cost.hpp"

template <typename  T, typename P>
ShortestPath<T,P>::ShortestPath(): m_cntExpandedCells(0), 
                                    m_cntTotalPath(0), 
                                    m_cntTotalStraightPath(0), 
                                    m_cntTotalDiagonalPath(0),
                                    m_upHillCostEstimator(std::make_unique<UpHillCost>()),
                                    m_downHillCostEstimator(std::make_unique<DownHillCost>())

{

}

template class ShortestPath<CellData, CellLocation>;

