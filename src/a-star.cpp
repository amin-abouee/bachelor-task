#include "a-star.hpp"
#include "matrix.hpp"

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
        ShortestPath<T, P>::m_cntExploredCells++;

        if (currentCell.visited == true)
            continue;

        // std::cout << "X: " << current.loc.x << " Y: " << current.loc.y << std::endl;
        if (currentCell.loc == target) 
        {
            std::cout << "Location: " << currentCell.loc.x << " " << currentCell.loc.y << std::endl;
            std::cout << "Weight: " << currentCell.weight << std::endl;
            std::cout << "Explored Cells: " << ShortestPath<T,P>::m_cntExploredCells << std::endl;
            break;
        }

        std::vector<GridLocation> neighbours;
        neighbours.reserve(8);
        graph.findNeighbours(currentCell.loc, overrides, neighbours);
        for (const auto& next : neighbours) 
        {
            auto& nextNode = graph(next);
            // if (nextNode.visited == false)
            // {
                const double dx = next.x - current.x;
                const double dy = next.y - current.y;
                const double dz = elevation(next.y, next.x) - elevation(current.y, current.x);
                double cost = std::sqrt(dx * dx + dy * dy + dz * dz);
                if (dz < 0)
                    cost = 1/cost;
                const double dxBig = target.x - current.x;
                const double dyBig = target.y - current.y;
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
    while(!(current.loc == *current.parent))
    {
        graph(current.loc).path = true;
        current = graph(*current.parent);
    }
}

template class AStar<GridWithWeights, GridLocation>;
