#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <exception>
#include <unordered_set>
#include <bitset>
#include <queue>

#include <nlohmann/json.hpp>

#include "visualizer.h"
#include "matrix.hpp"
#include "square-grid-graph.hpp"

#ifdef _MSC_VER
static const char* PATH_SEP = "\\";
#else
static const char* PATH_SEP = "/";
#endif

// Bits used in the overrides image bytes
enum OverrideFlags
{
    OF_RIVER_MARSH = 0x10,
    OF_INLAND = 0x20,
    OF_WATER_BASIN = 0x40
};

// Some constants
// enum
// {
//     IMAGE_DIM = 2048, // Width and height of the elevation and overrides image
    
//     ROVER_X = 159,
//     ROVER_Y = 1520,
//     BACHELOR_X = 1303,
//     BACHELOR_Y = 85,
//     WEDDING_X = 1577,
//     WEDDING_Y = 1294
// };

std::ifstream::pos_type fileSize(const std::string& filename)
{
    std::ifstream in(filename, std::ifstream::ate | std::ifstream::binary);
    if (!in.good())
    {
        throw std::exception();
    }
    return in.tellg();
}

// std::vector<uint8_t> loadFile(const std::string& filename, const uint32_t expectedFileSize)
// {
//     // std::cout << "filename: " << filename << std::endl;
//     // std::cout << "expected File Size: " << expectedFileSize << std::endl;
//     const std::size_t fsize = fileSize(filename);
//     if (fsize != expectedFileSize)
//     {
//         throw std::exception();
//     }
//     std::vector<uint8_t> data(fsize, 0);
//     std::ifstream ifile(filename, std::ifstream::binary);
//     if (!ifile.good())
//     {
//         throw std::exception();
//     }
//     ifile.read((char*)&data[0], fsize);
//     return data;
// }

void loadFile(const std::string& filename, Matrix<uint8_t>& mat)
{
    const uint32_t expectedFileSize = mat.getTotalSize();
    const std::size_t fsize = fileSize(filename);
    if (fsize != expectedFileSize)
    {
        throw std::exception();
    }
    std::ifstream ifile(filename, std::ifstream::binary);
    if (!ifile.good())
    {
        throw std::exception();
    }
    ifile.read((char*)mat.data(), fsize);
    ifile.close();
}

bool donut(int x, int y, int x1, int y1)
{
    int dx = x - x1;
    int dy = y - y1;
    int r2 = dx * dx + dy * dy;
    return r2 >= 150 && r2 <= 400;
}

// bool checkPath (int x, int y, )

void relax(GridWithWeights& current, GridWithWeights& next, double weight)
{
    if (next.weight > current.weight + weight)
    {
        next.weight = current.weight + weight;
        next.parent = &(current.loc);
    }
}

bool in_bounds(const GridLocation& id, const uint32_t imageDimension)
{
    return 0 <= id.x && id.x < imageDimension
        && 0 <= id.y && id.y < imageDimension;
}

bool passable(const GridLocation& id, const Matrix<uint8_t>& overrides)
{
    return overrides(id.y, id.x) == 0;
}

std::vector<GridLocation> neighbors(const GridWithWeights& s, const Matrix<uint8_t>& overrides, const uint32_t imageDimension) 
{
    std::array<GridLocation, 8> DIRS {{GridLocation{1, 0}, GridLocation{0, -1}, GridLocation{-1, 0}, GridLocation{0, 1}, 
                                    GridLocation{1, 1}, GridLocation{1, -1}, GridLocation{-1, 1}, GridLocation{-1, -1}}};
    std::vector<GridLocation> results;

    const auto& current = s.loc;
    for (GridLocation dir : DIRS) 
    {
        GridLocation next{current.x + dir.x, current.y + dir.y};
        if (in_bounds(next, imageDimension) && passable(next, overrides)) 
        {
            results.push_back(next);
        }
    }

    // if ((id.x + id.y) % 2 == 0) {
    //   // aesthetic improvement on square grids
    //   std::reverse(results.begin(), results.end());
    // }

    return results;
}

// template <typename T>
std::vector<GridLocation> dijkstraSearch(const uint32_t imageDimension, Matrix<GridWithWeights>& grid, const Matrix<uint8_t>& elevation, const Matrix<uint8_t>& overrides, const std::pair<uint32_t, uint32_t>& source, const std::pair<uint32_t, uint32_t>& target)
{
    // Matrix<GridWithWeights> grid{imageDimension, imageDimension};

    // struct myComparator 
    // { 
    //     bool operator() (const GridWithWeights* first, const GridWithWeights* second)
    //     {
    //         return first->weight > second->weight; 
    //     };
    // }; 

    auto compare = [](const GridWithWeights first, const GridWithWeights second)
                {
                    return first.weight > second.weight; 
                };


    std::priority_queue<GridWithWeights, std::vector<GridWithWeights>, decltype(compare)> minQueue(compare);

    for(std::size_t i(0); i< imageDimension; i++)
    {
        for(std::size_t j(0); j<imageDimension; j++)
        {
            auto& currentCell = grid(i,j);
            currentCell.loc.x = j;
            currentCell.loc.y = i;
        }
    }


    auto& cellSource = grid(source.second, source.first);
    cellSource.weight = 0;
    cellSource.parent = std::addressof(cellSource.loc);
    // cellSource.visited = true;


    // std::cout << grid(source.second, source.first).weight << std::endl;


    // for(std::size_t i(0); i< imageDimension; i++)
    // {
    //     for(std::size_t j(0); j<imageDimension; j++)
    //     {
    //         auto& currentCell = grid(i,j);
    //         if (overrides(i,j) == 0)
    //             minQueue.push(currentCell);
    //     }
    // }

    minQueue.push(cellSource);
    while (!minQueue.empty()) 
    {
        GridWithWeights current = minQueue.top();
        // current.visited = true;
        minQueue.pop();
        auto& currentNode = grid(current.loc.y, current.loc.x);
        if (currentNode.visited == true)
            continue;

        // std::cout << "X: " << current.loc.x << " Y: " << current.loc.y << std::endl;
        if (current.loc.x == target.first && current.loc.y == target.second) 
        {
            std::cout << "Location: " << current.loc.x << " " << current.loc.y << std::endl;
            std::cout << "Weight: " << current.weight << std::endl;
            break;
        }

        std::vector<GridLocation> NG = neighbors(current, overrides, imageDimension);
        for (const auto& next : NG) 
        {
            // double new_cost = cost_so_far[current] + graph.cost(current, next);
            // if (cost_so_far.find(next) == cost_so_far.end()
            //     || new_cost < cost_so_far[next]) 
            // {
            //     cost_so_far[next] = new_cost;
            //     came_from[next] = current;
            //     frontier.put(next, new_cost);
            // }
            auto& nextNode = grid(next.y, next.x);
            if (nextNode.visited == false)
            {
                const double dx = next.x - current.loc.x;
                const double dy = next.y - current.loc.y;
                const double dz = elevation(next.y, next.x) - elevation(current.loc.y, current.loc.x);
                double cost = std::sqrt(dx * dx + dy * dy + dz * dz);
                if (dz < 0)
                    cost = 1/cost;
                relax(currentNode, nextNode, cost);
                    // nextNode.visited = true;
                minQueue.push(nextNode);
            }
        }
        currentNode.visited = true;
    }

    std::vector <GridLocation> result;
    auto& pp = grid(target.second, target.first );
    // std::cout << "X: " << pp.loc.x << " Y: " << pp.loc.y << std::endl;

    while(!(pp.loc.x == pp.parent->x && pp.loc.y == pp.parent->y))
    {
        // std::cout << "X: " << pp.loc.x << " Y: " << pp.loc.y  << "  " << (long) std::addressof(pp.loc) << "  " << (long)pp.parent << std::endl;
        // pp.path = true;
        result.emplace_back(GridLocation(pp.loc.x, pp.loc.y));
        pp = grid(pp.parent->y, pp.parent->x );
    }
    // pp.path = true;
    result.emplace_back(GridLocation(pp.loc.x, pp.loc.y));
    return result;
}


int main(int argc, char** argv)
{

    std::ifstream fileReader( argv[1] );
    const nlohmann::json configFile = nlohmann::json::parse( fileReader );
    // nlohmann::json configFile;
    // fileReader >> configFile;
    const nlohmann::json& filePathsJsonNode = configFile[ "file_paths" ];

    const auto elevationFilepath = filePathsJsonNode[ "elevation_filepath" ].get< std::string >();
    const auto overridesFilepath = filePathsJsonNode[ "overrides_filepath" ].get< std::string >();
    // std::cout << elevationFilepath << std::endl;
    // std::cout << overridesFilepath << std::endl;

    const nlohmann::json& constraintsJsonNode = configFile[ "constraints" ];
    const uint32_t imageDimension = constraintsJsonNode["image_dimension"].get<uint32_t>();
    std::cout << "image dimension: " << imageDimension << std::endl;
    const auto roverLoc = constraintsJsonNode["rover_loc"].get<std::pair<uint32_t, uint32_t>>();
    const auto bachelorLoc = constraintsJsonNode["bachelor_loc"].get<std::pair<uint32_t, uint32_t>>();
    const auto weddingLoc = constraintsJsonNode["wedding_loc"].get<std::pair<uint32_t, uint32_t>>();
    std::cout << roverLoc.first << " " << roverLoc.second << std::endl;
    std::cout << bachelorLoc.first << " " << bachelorLoc.second << std::endl;
    std::cout << weddingLoc.first << " " << weddingLoc.second << std::endl;

    Matrix<uint8_t> elevation{imageDimension, imageDimension};
    Matrix<uint8_t> overrides{imageDimension, imageDimension};
    // uint8_t* ptr = a.data();
    // std::cout << "ptr: " << (long)ptr << std::endl;


    const uint32_t expectedFileSize = imageDimension * imageDimension;
    // Address assets relative to application location
    // std::string anchor = std::string(".") + PATH_SEP;
    // std::string pname = argv[0];
    // auto lastpos = pname.find_last_of("/\\");
    // if (lastpos != std::string::npos)
    // {
    //     anchor = pname.substr(0, lastpos) + PATH_SEP;
    // }
    // std::cout << "anchor: " << anchor << std::endl;
    // auto elevation = loadFile(elevationFilepath, expectedFileSize);
    // auto overrides = loadFile(overridesFilepath, expectedFileSize);

    loadFile(elevationFilepath, elevation);
    loadFile(overridesFilepath, overrides);
    // std::cout << a << std::endl;
    // auto overrides2 = loadFile2(overridesFilepath, imageDimension);

    // for (int i(0); i< 50; i++)
    // {
    //     for(int j(0); j<50; j++)
    //     {
    //         std::cout << (int)elevation(i, j) << " ";
    //     }
    //     std::cout << std::endl;
    // }

    // for (int i(0); i< 50; i++)
    // {
    //     for(int j(0); j<50; j++)
    //     {
    //         std::cout << (int)overrides(i, j) << " ";
    //     }
    //     std::cout << std::endl;
    // }



    // for (int i = 1250; i < 1300; i++)
    // {
    //     for (int j = 1550; j < 1600; j++)
    //     {
    //         int idx = i * IMAGE_DIM + j;
    //         std::cout << (int)overrides[idx] << " ";
    //     }
    //     std::cout << std::endl;
    // }

    // std::unordered_set<uint8_t> set;
    // for (const int &i: overrides.getMatrix()) 
    // {
    //     set.insert(i);
    // }

    // for (const int &i: set) {
    //     std::string binary = std::bitset<8>(i).to_string();
    //     std::cout << i << " " << binary << std::endl;
    // }

///
    // {

    SquareGridGraph<GridWithWeights, GridLocation> newGraph(imageDimension, 8);
    newGraph.initializeAllCells();

    std::cout << newGraph(0, 0).loc.x << " " << newGraph(0, 0).loc.y << " " << newGraph(0, 0).weight << std::endl;

    Matrix<GridWithWeights> grid {imageDimension, imageDimension};    
    auto result = dijkstraSearch(imageDimension, grid, elevation, overrides, roverLoc, bachelorLoc);

    for(const auto& ll : result)
        grid(ll.y, ll.x).path = true;

    // std::cout << (int)grid(roverLoc.second, roverLoc.first).path << std::endl;
    // std::cout << (int)grid(bachelorLoc.second, bachelorLoc.first).path << std::endl;
    // std::cout << "X: " << 
    // }

    // for (int i = 0; i < imageDimension; i++)
    // {
    //     for (int j = 0; j < imageDimension; j++)
    //     {
    //         if (grid(i, j).path == true)
    //         {
    //             std::cout << "X: " << j << " Y: " << i << std::endl; 
    //         }
    //     }
    //     // std::cout << std::endl;
    // }


    std::ofstream of("pic.bmp", std::ofstream::binary);

    auto pixelFilter = [&grid, &overrides, &imageDimension, &roverLoc, &bachelorLoc, &weddingLoc] (size_t x, size_t y, uint8_t elevation) 
    {
        // Marks interesting positions on the map
        if (donut(x, y, roverLoc.first, roverLoc.second) ||
            donut(x, y, bachelorLoc.first, bachelorLoc.second) ||
            donut(x, y, weddingLoc.first, weddingLoc.second))
        {
            return uint8_t(visualizer::IPV_PATH);
        }

        if (grid(y,x).path == true)
        {
            // std::cout << "X: " << x << " Y: " << y << std::endl;
            return uint8_t(visualizer::IPV_PATH);
        }
        
        // Signifies water
        if ((overrides(y, x) & (OF_WATER_BASIN | OF_RIVER_MARSH)) ||
            elevation == 0)
        {
            return uint8_t(visualizer::IPV_WATER);
        }
        
        // Signifies normal ground color
        if (elevation < visualizer::IPV_ELEVATION_BEGIN)
        {
            elevation = visualizer::IPV_ELEVATION_BEGIN;
        }
        return elevation;
    };
    
    visualizer::writeBMP(
        of,
        elevation.data(),
        imageDimension,
        imageDimension,
        pixelFilter);
    of.flush();
#if __APPLE__
    auto res = system("open pic.bmp");
    (void)res;
#endif
    return 0;
}

