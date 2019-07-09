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
AStar<T,P>::AStar(const std::string& downHillCostModel, const std::string& upHillCostModel): ShortestPath<T,P>(downHillCostModel, upHillCostModel)
{

}

template <typename  T, typename P>
void AStar<T,P>::findShortestPath(SquareGridGraph<T, P>& graph, 
                            const Matrix<uint8_t>& elevation, 
                            const Matrix<uint8_t>& overrides, 
                            const P& source, 
                            const P& target)
{
    using PQCell = std::pair<P, double>;
    auto compare = [](const PQCell& lhs, const PQCell& rhs)
                {
                    return lhs.second > rhs.second; 
                };

    // create a priority queue (min heap) to have the smallest path for each moment
    // for this one, we create a priority queue that contains the cell location as well as the minimum cost to reach this cell from source
    std::priority_queue<PQCell, std::vector<PQCell>, decltype(compare)> frontier(compare);

    /// initialize all cells
    graph.initializeAllCells();

    // get the source cell and put in PQ as start point
    auto& sourceCell = graph(source.Y(), source.X());
    sourceCell.setWeight(0.0);
    // for this part, we need to have the const reference or reference of each node
    sourceCell.setParent(std::addressof(sourceCell.getLoc()));
    frontier.push(std::make_pair(sourceCell.getLoc(), 0));

    while (!frontier.empty()) 
    {
        // location of top PQ
        P current = frontier.top().first;
        frontier.pop();
        // get the content of current position in grid graph in type T
        auto& currentCell = graph(current);
        // increase expended nodes by 1
        ShortestPath<T, P>::m_cntExpandedCells++;

        if (currentCell.getVisited() == true)
        {
            continue;
            std::cout << "WARNING " << std::endl;
        }

        // if current expantion node == target, we find the shortest path. ENTRY EXIT!
        if (currentCell.getLoc() == target) 
        {
            currentCell.setVisited(true);
            break;
        }

        std::vector<P> neighbours;
        neighbours.reserve(8);
        graph.findNeighbours(currentCell.getLoc(), overrides, neighbours);
        for (const auto& next : neighbours) 
        {
            auto& nextNode = graph(next);
            const double dz = elevation(next.Y() , next.X()) - elevation(current.Y(), current.X());
            double cost = 0;
            if (dz > 0)
            {
                cost = ShortestPath<T,P>::m_upHillCostEstimator->computeCost(current, elevation(current.Y(), current.X()), next, elevation(next.Y() , next.X()),  ShortestPath<T,P>::m_upHillCostModel);
            }
            else if ( dz < 0)
            {
                cost = ShortestPath<T,P>::m_downHillCostEstimator->computeCost(current, elevation(current.Y(), current.X()), next, elevation(next.Y() , next.X()), ShortestPath<T,P>::m_downHillCostModel);
            }
            else
            {
                cost = ShortestPath<T,P>::m_upHillCostEstimator->computeCost(current, elevation(current.Y(), current.X()), next, elevation(next.Y() , next.X()), Cost::CostModel::Octile);
            }
            const double dxBig = target.X() - current.Y();
            const double dyBig = target.X() - current.Y();
            // cost += std::sqrt(dxBig * dxBig + dyBig * dyBig);
            const double priority = 0.0;
            // const double priority = std::abs(dxBig) + std::abs(dyBig);
            // const double priority = std::sqrt(dxBig * dxBig + dyBig * dyBig);
            if (relax(currentCell, nextNode, cost))
                frontier.push(std::make_pair(nextNode.getLoc(), nextNode.getWeight() + priority));
        }
        currentCell.setVisited(true);
    }
    updatePath(graph, source, target);
    printSummary(source, target);
}

template <typename  T, typename P>
bool AStar<T,P>::relax(T& current, T& next, double cost) const
{
    if (next.getWeight() > current.getWeight() + cost)
    {
        next.setWeight (current.getWeight() + cost);
        next.setParent (&(current.getLoc()));
        return true;
    }
    return false;
}

template <typename  T, typename P>
void AStar<T,P>::updatePath(SquareGridGraph<T, P>& graph, const P& source, const P& target)
{
    // traverse from target to source by calling parent 
    P current{target};
    P prevoiusLocation{target};
    while(!(current == source))
    {
        // set flag of path
        graph(current).setPath(true);
        ShortestPath<T, P>::m_cntTotalPath++;
        // get the parent from curerent
        current = *graph(current).getParent();
        int distance = std::abs(current.X() - prevoiusLocation.X()) + std::abs(current.Y() - prevoiusLocation.Y());
        // straight path
        if (distance == 1)
            ShortestPath<T, P>::m_cntTotalStraightPath++;
        // diagonal path
        else
            ShortestPath<T, P>::m_cntTotalDiagonalPath++;
        prevoiusLocation = current;
    }
    // std::cout << "Total Path: " << ShortestPath<T,P>::m_cntTotalPath << std::endl;
    // std::cout << "Straight Cells: " << ShortestPath<T,P>::m_cntTotalStraightPath << std::endl;
    // std::cout << "Diagonal Cells: " << ShortestPath<T,P>::m_cntTotalDiagonalPath << std::endl;
}

template <typename  T, typename P>
void AStar<T,P>::printSummary (const P& source, const P& target)
{
    std::cout << "{" << std::endl;
    std::cout << "\t Path from Source: [" << source.X() << " , " << source.Y() << "] ";
    std::cout << "-> Target: [" << target.X() << " , " << target.Y() << "]" << std::endl;
    // std::cout << "Total Weight: " << currentCell.getWeight() << std::endl;
    std::cout << "\t Expanded Cells: " << ShortestPath<T,P>::m_cntExpandedCells << std::endl;
    std::cout << "\t Total Path Length: " << ShortestPath<T,P>::m_cntTotalPath << std::endl;
    std::cout << "\t Straight Path: " << ShortestPath<T,P>::m_cntTotalStraightPath << std::endl;
    std::cout << "\t Diagonal Path: " << ShortestPath<T,P>::m_cntTotalDiagonalPath << std::endl;
    std::cout << "}" << std::endl;
    std::cout << std::endl;
}

template class AStar<CellData, CellLocation>;
