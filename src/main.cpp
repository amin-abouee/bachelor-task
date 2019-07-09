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
#include "cell_info.hpp"
#include "square_grid_graph.hpp"
#include "shortest_path.hpp"
#include "a_star.hpp"

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
    const uint32_t imageDimension = constraintsJsonNode["image_dimension"].get<int32_t>();
    std::cout << "image dimension: " << imageDimension << std::endl;
    // CellLocation temp {0, 0};
    const auto roverLoc = constraintsJsonNode["rover_loc"].get<std::pair<int32_t, int32_t>>();
    // const auto [temp.x ,temp.y] = constraintsJsonNode["rover_loc"].get<std::pair<int32_t, int32_t>>();
    const auto bachelorLoc = constraintsJsonNode["bachelor_loc"].get<std::pair<int32_t, int32_t>>();
    const auto weddingLoc = constraintsJsonNode["wedding_loc"].get<std::pair<int32_t, int32_t>>();
    std::cout << roverLoc.first << " " << roverLoc.second << std::endl;
    // std::cout << temp.x << " " << temp.y << std::endl;
    std::cout << bachelorLoc.first << " " << bachelorLoc.second << std::endl;
    std::cout << weddingLoc.first << " " << weddingLoc.second << std::endl;



    Matrix<uint8_t> elevation{imageDimension, imageDimension};
    Matrix<uint8_t> overrides{imageDimension, imageDimension};


    // const uint32_t expectedFileSize = imageDimension * imageDimension;
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

    const nlohmann::json& SPPJsonNode = configFile[ "shortest_path_parameters" ];
    const auto upHillCostModel = SPPJsonNode["up_hill_cost_model"].get<std::string>();
    const auto downHillCostModel = SPPJsonNode["down_hill_cost_model"].get<std::string>();
    std::cout << "Up Hill Model Name: " << upHillCostModel << std::endl;
    std::cout << "Down Hill Model Name: " << downHillCostModel << std::endl;


    SquareGridGraph<CellData, CellLocation> graph(imageDimension, 8);
    std::unique_ptr<ShortestPath<CellData, CellLocation>> shortestPath = std::make_unique<AStar<CellData, CellLocation>>(downHillCostModel, upHillCostModel);
    CellLocation roverplace{static_cast<int32_t>(roverLoc.first), static_cast<int32_t>(roverLoc.second)};
    CellLocation bachelorplace{static_cast<int32_t>(bachelorLoc.first), static_cast<int32_t>(bachelorLoc.second)};
    shortestPath->findShortestPath(graph, elevation, overrides, roverplace, bachelorplace);

    // std::cout << newGraph(0, 0).loc.x << " " << newGraph(0, 0).loc.y << " " << newGraph(0, 0).weight << std::endl;

    // Matrix<GridWithWeights> grid {imageDimension, imageDimension};    
    // auto result = dijkstraSearch(imageDimension, grid, elevation, overrides, roverLoc, bachelorLoc);
    // auto result = dijkstraSearch2(graph, elevation, overrides, roverLoc, bachelorLoc);

    // for(const auto& ll : result)
        // graph(ll.y, ll.x).path = true;

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
    // std::cout << "HERE: " << std::endl;

    auto pixelFilter = [&graph, &overrides, &imageDimension, &roverLoc, &bachelorLoc, &weddingLoc] (size_t x, size_t y, uint8_t elevation) 
    {
        // Marks interesting positions on the map
        if (donut(x, y, roverLoc.first, roverLoc.second) ||
            donut(x, y, bachelorLoc.first, bachelorLoc.second) ||
            donut(x, y, weddingLoc.first, weddingLoc.second))
        {
            return uint8_t(visualizer::IPV_PATH);
        }

        if (graph(y,x).getPath() == true)
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
    std::cout << "FINISH: " << std::endl;
    return 0;
}

