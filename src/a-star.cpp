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
    auto compare = [](const T first, const T second)
                {
                    return first.weight > second.weight; 
                };
    std::priority_queue<T, std::vector<T>, decltype(compare)> frontier(compare);

    graph.initializeAllCells();
    auto& sourceCell = graph(source.y, source.x);
    sourceCell.weight = 0;
    sourceCell.parent = std::addressof(sourceCell.loc);

    frontier.push(sourceCell);
    while (!frontier.empty()) 
    {
        T current = frontier.top();
        frontier.pop();
        auto& currentCell = graph(current.loc);
        ShortestPath<T, P>::m_cntExploredCells++;

        if (currentCell.visited == true)
            continue;

        // std::cout << "X: " << current.loc.x << " Y: " << current.loc.y << std::endl;
        if (current.loc == target) 
        {
            std::cout << "Location: " << current.loc.x << " " << current.loc.y << std::endl;
            std::cout << "Weight: " << current.weight << std::endl;
            std::cout << "Explored Cells: " << ShortestPath<T,P>::m_cntExploredCells << std::endl;
            break;
        }

        std::vector<GridLocation> neighbours;
        neighbours.reserve(8);
        graph.findNeighbours(current.loc, overrides, neighbours);
        for (const auto& next : neighbours) 
        {
            auto& nextNode = graph(next);
            if (nextNode.visited == false)
            {
                const double dx = next.x - current.loc.x;
                const double dy = next.y - current.loc.y;
                const double dz = elevation(next.y, next.x) - elevation(current.loc.y, current.loc.x);
                double cost = std::sqrt(dx * dx + dy * dy + dz * dz);
                if (dz < 0)
                    cost = 1/cost;
                const double dxBig = target.x - current.loc.x;
                const double dyBig = target.y - current.loc.y;
                relax(currentCell, nextNode, cost);
                frontier.push(nextNode);
            }
        }
        currentCell.visited = true;
    }
    updatePath(graph, source, target);
}

template <typename  T, typename P>
void AStar<T,P>::relax(T& current, T& next, double weight) const
{
    if (next.weight > current.weight + weight)
    {
        next.weight = current.weight + weight;
        next.parent = &(current.loc);
    }
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
