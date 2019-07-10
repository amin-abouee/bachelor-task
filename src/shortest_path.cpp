#include "shortest_path.hpp"
// #include "cost.hpp"
#include "up_hill_cost.hpp"
#include "down_hill_cost.hpp"

template <typename  T, typename P>
ShortestPath<T,P>::ShortestPath(): m_cntExpandedCells(0), 
                                    m_cntTotalPath(0), 
                                    m_cntTotalStraightPath(0), 
                                    m_cntTotalDiagonalPath(0),
                                    m_upHillCostEstimator(std::make_unique<UpHillCost>()),
                                    m_downHillCostEstimator(std::make_unique<DownHillCost>()),
                                    m_upHillCostModel(Cost::CostModel::L2),
                                    m_downHillCostModel(Cost::CostModel::L2)
{

}

template <typename  T, typename P>
ShortestPath<T,P>::ShortestPath(const std::string& downHillCostModel, const std::string& upHillCostModel): m_cntExpandedCells(0), 
                                    m_cntTotalPath(0), 
                                    m_cntTotalStraightPath(0), 
                                    m_cntTotalDiagonalPath(0),
                                    m_upHillCostEstimator(std::make_unique<UpHillCost>()),
                                    m_downHillCostEstimator(std::make_unique<DownHillCost>())
{
    auto it = Cost::allModels.find(downHillCostModel);
    if (it != Cost::allModels.end())
    {
        m_downHillCostModel = Cost::allModels[downHillCostModel];
    }
    else
    {
        throw std::runtime_error("doesn't support your mode");
    }

    it = Cost::allModels.find(upHillCostModel);
    if (it != Cost::allModels.end())
    {
        m_upHillCostModel = Cost::allModels[upHillCostModel];
    }
    else
    {
        throw std::runtime_error("doesn't support your mode");
    }
}

template <typename  T, typename P>
void ShortestPath<T,P>::resetMemberVariables()
{
    m_cntExpandedCells = 0;
    m_cntTotalPath = 0;
    m_cntTotalStraightPath = 0;
    m_cntTotalDiagonalPath = 0;
}


template class ShortestPath<CellData, CellLocation>;

