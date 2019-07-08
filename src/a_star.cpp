#include "a_star.hpp"
#include "matrix.hpp"
#include "cell_info.hpp"

#include <queue>
#include <iostream>
#include <cmath>

template <typename  T, typename P>
AStar<T,P>::AStar() : ShortestPath<T,P>() 
{

}

template <typename  T, typename P>
void AStar<T,P>::findShortestPath(SquareGridGraph<T, P>& graph, 
                            const Matrix<uint8_t>& elevation, 
                            const Matrix<uint8_t>& overrides, 
                            const P& source, 
                            const P& target)
{
    Cost::CostModel downHillModel = Cost::CostModel::L2;
    Cost::CostModel upHillModel = Cost::CostModel::L2;
    using PQNode = std::pair<P, double>;
    auto compare = [](const PQNode& lhs, const PQNode& rhs)
                {
                    return lhs.second > rhs.second; 
                };
    // std::priority_queue<T, std::vector<T>, decltype(compare)> frontier(compare);

    std::priority_queue<PQNode, std::vector<PQNode>, decltype(compare)> frontier(compare);

    graph.initializeAllCells();
    auto& sourceCell = graph(source.y, source.x);
    sourceCell.weight = 0;
    sourceCell.parent = std::addressof(sourceCell.loc);

    frontier.push(std::make_pair(sourceCell.loc, 0));
    while (!frontier.empty()) 
    {
        P current = frontier.top().first;
        frontier.pop();
        auto& currentCell = graph(current);
        ShortestPath<T, P>::m_cntExpandedCells++;

        if (currentCell.visited == true)
        {
            continue;
            std::cout << "WARNING " << std::endl;
        }

        // std::cout << "X: " << current.loc.x << " Y: " << current.loc.y << std::endl;
        if (currentCell.loc == target) 
        {
            currentCell.visited = true;
            std::cout << "Location: " << currentCell.loc.x << " " << currentCell.loc.y << std::endl;
            std::cout << "Weight: " << currentCell.weight << std::endl;
            std::cout << "Expanded Cells: " << ShortestPath<T,P>::m_cntExpandedCells << std::endl;
            break;
        }

        std::vector<P> neighbours;
        neighbours.reserve(8);
        graph.findNeighbours(currentCell.loc, overrides, neighbours);
        for (const auto& next : neighbours) 
        {
            auto& nextNode = graph(next);
            // if (nextNode.visited == false)
            // {
                // const double dx = next.x - current.x;
                // const double dy = next.y - current.y;

                const double dz = elevation(next.y , next.x) - elevation(current.y, current.x);
                // double cost = std::sqrt(dx * dx + dy * dy + dz * dz);
                // if (dz < 0)
                //     cost = 1/cost;
                double cost = 0;
                if (dz > 0)
                {
                    cost = ShortestPath<T,P>::m_upHillCostEstimator->computeCost(current, elevation(current.y, current.x), next, elevation(next.y , next.x), upHillModel);
                }
                else if ( dz < 0)
                {
                    cost = ShortestPath<T,P>::m_downHillCostEstimator->computeCost(current, elevation(current.y, current.x), next, elevation(next.y , next.x), downHillModel);
                }
                else
                {
                    cost = ShortestPath<T,P>::m_upHillCostEstimator->computeCost(current, elevation(current.y, current.x), next, elevation(next.y , next.x), Cost::CostModel::Octile);/* code */
                }
                

                // const double dxBig = target.x - current.x;
                // const double dyBig = target.y - current.y;
                // cost += std::sqrt(dxBig * dxBig + dyBig * dyBig);
                const double priority = cost;
                // const double priority = cost + std::abs(dxBig) + std::abs(dyBig);
                // const double priority = cost + std::sqrt(dxBig * dxBig + dyBig * dyBig);
                if (relax(currentCell, nextNode, cost))
                    frontier.push(std::make_pair(nextNode.loc, nextNode.weight + priority));
            // }
        }
        currentCell.visited = true;
    }
    updatePath(graph, source, target);
}

template <typename  T, typename P>
bool AStar<T,P>::relax(T& current, T& next, double weight) const
{
    if (next.weight > current.weight + weight)
    {
        next.weight = current.weight + weight;
        next.parent = &(current.loc);
        return true;
    }
    return false;
}

template <typename  T, typename P>
void AStar<T,P>::updatePath(SquareGridGraph<T, P>& graph, const P& source, const P& target)
{
    T& current = graph(target);
    P prevoiusLocation {current.loc};
    while(!(current.loc == source))
    {
        graph(current.loc).path = true;
        ShortestPath<T, P>::m_cntTotalPath++;
        current = graph(*current.parent);
        int distance = std::abs(current.loc.x - prevoiusLocation.x) + std::abs(current.loc.y - prevoiusLocation.y);
        if (distance == 1)
            ShortestPath<T, P>::m_cntTotalStraightPath++;
        else
            ShortestPath<T, P>::m_cntTotalDiagonalPath++;
        prevoiusLocation = current.loc;
    }
    std::cout << "Total Path: " << ShortestPath<T,P>::m_cntTotalPath << std::endl;
    std::cout << "Straight Cells: " << ShortestPath<T,P>::m_cntTotalStraightPath << std::endl;
    std::cout << "Diagonal Cells: " << ShortestPath<T,P>::m_cntTotalDiagonalPath << std::endl;

}

template class AStar<CellData, CellLocation>;
