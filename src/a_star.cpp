#include "a_star.hpp"
#include "matrix.hpp"
#include "cell_info.hpp"

#include <queue>
#include <iostream>
#include <cmath>

template <typename  T, typename P>
AStar<T,P>::AStar() : ShortestPath<T,P>(), m_heuristicEstimator(), m_heuristicModel(Heuristic::HeuristicModel::Diagonal)
{

}

template <typename  T, typename P>
AStar<T,P>::AStar(const std::string& downHillCostModel, const std::string& upHillCostModel, const std::string& heuristicModel): 
    ShortestPath<T,P>(downHillCostModel, upHillCostModel), m_heuristicEstimator()
{
    auto it = m_heuristicEstimator.allModels.find(heuristicModel);
    if (it != m_heuristicEstimator.allModels.end())
    {
        m_heuristicModel = m_heuristicEstimator.allModels[heuristicModel];
    }
    else
    {
        throw std::runtime_error("Your model does not define. Possible option: dijkstra, l1, l2, diagonal, l1-altitude, l2-altitude, diagonal-altitude");
    }
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

    // initialize all cells
    graph.initializeAllCells();
    // reset member variables for a new path 
    ShortestPath<T,P>::resetMemberVariables();

    // compute average of altitude based on the heuristic model
    const double aveAltitude = computeAverageAltitudeInPath(source, target, elevation, overrides);
    // std::cout << "ave altitude: " << aveAltitude << std::endl;

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

        // if (currentCell.getVisited() == true)
        // {
        //     continue;
        //     std::cout << "WARNING " << std::endl;
        // }

        // if current expantion node == target, we find the shortest path. ENTRY EXIT!
        if (currentCell.getLoc() == target) 
        {
            currentCell.setVisited(true);
            break;
        }

        std::vector<P> neighbours;
        neighbours.reserve(8);
        graph.findNeighbours(currentCell.getLoc(), overrides, elevation, neighbours);
        for (const auto& next : neighbours) 
        {
            auto& nextNode = graph(next);
            // decide based on different of altitude
            const uint8_t dz = elevation(next.Y() , next.X()) - elevation(current.Y(), current.X());
            double cost = 0;
            if (dz > 0)
            {
                // up hill movement
                cost = ShortestPath<T,P>::m_upHillCostEstimator->computeCost(current, elevation(current.Y(), current.X()), next, elevation(next.Y() , next.X()),  ShortestPath<T,P>::m_upHillCostModel);
            }
            else if ( dz < 0)
            {
                // downhill movement
                cost = ShortestPath<T,P>::m_downHillCostEstimator->computeCost(current, elevation(current.Y(), current.X()), next, elevation(next.Y() , next.X()), ShortestPath<T,P>::m_downHillCostModel);
            }
            else
            {
                // octile movement
                cost = ShortestPath<T,P>::m_upHillCostEstimator->computeCost(current, elevation(current.Y(), current.X()), next, elevation(next.Y() , next.X()), Cost::CostModel::Octile);
            }

            const double priority = m_heuristicEstimator.computeHeuristic(current, target, m_heuristicModel, aveAltitude);
            // std::cout << "priority: " << priority << std::endl;
            if (relax(currentCell, nextNode, cost))
                frontier.push(std::make_pair(nextNode.getLoc(), nextNode.getWeight() + priority));
        }
        currentCell.setVisited(true);
    }
    updatePath(graph, source, target);
    printSummary(source, target, graph(target).getWeight());
    // checkPath(graph, overrides);
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
        int32_t distance = std::abs(current.X() - prevoiusLocation.X()) + std::abs(current.Y() - prevoiusLocation.Y());
        // straight path
        if (distance == 1)
            ShortestPath<T, P>::m_cntTotalStraightPath++;
        // diagonal path
        else
            ShortestPath<T, P>::m_cntTotalDiagonalPath++;
        prevoiusLocation = current;
    }
}

template <typename  T, typename P>
void AStar<T,P>::printSummary (const P& source, const P& target, const double cost)
{
    std::cout << "{" << std::endl;
    std::cout << "\t Path from Source: [" << source.X() << " , " << source.Y() << "] ";
    std::cout << "-> Target: [" << target.X() << " , " << target.Y() << "]" << std::endl;
    std::cout << "\t Total Cost: " << cost << std::endl;
    std::cout << "\t Expanded Cells: " << ShortestPath<T,P>::m_cntExpandedCells << std::endl;
    std::cout << "\t Total Path Length: " << ShortestPath<T,P>::m_cntTotalPath << std::endl;
    std::cout << "\t Straight Path: " << ShortestPath<T,P>::m_cntTotalStraightPath << std::endl;
    std::cout << "\t Diagonal Path: " << ShortestPath<T,P>::m_cntTotalDiagonalPath << std::endl;
    std::cout << "}" << std::endl;
    std::cout << std::endl;
}

// template <typename  T, typename P>
// void AStar<T,P>::checkPath (SquareGridGraph<T, P>& graph, const Matrix<uint8_t>& overrides)
// {
//     for (int i=0; i< graph.getGridSize(); i++)
//     {
//         for (int j=0; j<graph.getGridSize(); j++)
//         {
//             P temp {j , i};
//             if (graph(temp).getVisited() == true && overrides(temp.Y(), temp.X()) > 0)
//             {
//                 std::cout << "X: " << temp.X() << " y: " << temp.Y() << std::endl;
//                 throw std::runtime_error("Find PATH in INELIGIBLE Point");
//             }
//         }
//     }
// }

template <typename  T, typename P>
const double AStar<T,P>::computeAverageAltitudeInPath (const P& source, 
                                                const P& target, 
                                                const Matrix<uint8_t>& elevation, 
                                                const Matrix<uint8_t>& overrides)
{

    if (m_heuristicModel == Heuristic::HeuristicModel::L1 ||  m_heuristicModel == Heuristic::HeuristicModel::L2 || m_heuristicModel == Heuristic::HeuristicModel::Diagonal)
        return 1.0;
    else
    {
        // Find min between width and height from source to target
        const double step = std::min(std::abs(target.Y() - source.Y()), std::abs(target.X() - source.X()));

        // find row step and col step 
        const int32_t minRow = target.Y() - source.Y();
        const double rowStep = minRow / step;

        const int32_t minCol = target.X() - source.X();
        const double colStep = minCol / step;


        // find minimun and maximum elevation from source and target
        // create a line of slight from source to target
        uint32_t cnt = 0;
        int32_t maximum = 0;
        int32_t minimum = 300;
        for(int i(0); i<=step; i++)
        {
            int32_t idy = source.Y() + rowStep * i;
            int32_t idx = source.X() + colStep * i;
            if (overrides(idy, idx) == 0)
            {
                maximum = std::max(maximum, (int)elevation(idy, idx));
                minimum = std::min(minimum, (int)elevation(idy, idx));
                cnt ++;
            }
        }   
        const double minmax = maximum - minimum;
        // average of altitude for uphill and down hill movements
        return  (minmax * 2 / cnt) + (cnt / (minmax * 2));
    }
}


template class AStar<CellData, CellLocation>;
