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
    // const nlohmann::json configFile = nlohmann::json::parse( fileReader );
    nlohmann::json configFile;
    fileReader >> configFile;
    const nlohmann::json& filePathsJsonNode = configFile[ "file_paths" ];

    const auto elevationFilepath = filePathsJsonNode[ "elevation_filepath" ].get< std::string >();
    const auto overridesFilepath = filePathsJsonNode[ "overrides_filepath" ].get< std::string >();

    const nlohmann::json& constraintsJsonNode = configFile[ "constraints" ];
    const uint32_t imageDimension = constraintsJsonNode["image_dimension"].get<int32_t>();
    std::cout << "Image Dimension: " << imageDimension << std::endl;
    const auto roverLocPair = constraintsJsonNode["rover_loc"].get<std::pair<int32_t, int32_t>>();
    const auto bachelorLocPair = constraintsJsonNode["bachelor_loc"].get<std::pair<int32_t, int32_t>>();
    const auto weddingLocPair = constraintsJsonNode["wedding_loc"].get<std::pair<int32_t, int32_t>>();
    CellLocation roverLoc {roverLocPair.first, roverLocPair.second};
    CellLocation bachelorLoc {bachelorLocPair.first, bachelorLocPair.second};
    CellLocation weddingLoc {weddingLocPair.first, weddingLocPair.second};

    std::cout << "Rover Location: [ "<< roverLoc.X() << " , " << roverLoc.Y() << "]" << std::endl;
    std::cout << "Bachelor Location: [ " << bachelorLoc.X() << " , " << bachelorLoc.Y() << "]" << std::endl;
    std::cout << "Wedding Location: [ " << weddingLoc.X() << " , " << weddingLoc.Y() << "]" << std::endl;

    Matrix<uint8_t> elevation{imageDimension, imageDimension};
    Matrix<uint8_t> overrides{imageDimension, imageDimension};

    loadFile(elevationFilepath, elevation);
    loadFile(overridesFilepath, overrides);


    const nlohmann::json& SPPJsonNode = configFile[ "shortest_path_parameters" ];
    const auto upHillCostModel = SPPJsonNode["up_hill_cost_model"].get<std::string>();
    const auto downHillCostModel = SPPJsonNode["down_hill_cost_model"].get<std::string>();
    const auto heuristicModel = SPPJsonNode["heuristic_model"].get<std::string>();
    std::cout << "Up Hill Model: " << upHillCostModel << std::endl;
    std::cout << "Down Hill Model: " << downHillCostModel << std::endl;
    std::cout << "Heuristic Model: " << heuristicModel << std::endl;


    SquareGridGraph<CellData, CellLocation> graph(imageDimension, 8);
    std::unique_ptr<ShortestPath<CellData, CellLocation>> shortestPath = std::make_unique<AStar<CellData, CellLocation>>(downHillCostModel, upHillCostModel, heuristicModel);
    shortestPath->findShortestPath(graph, elevation, overrides, roverLoc, bachelorLoc);
    shortestPath->findShortestPath(graph, elevation, overrides, bachelorLoc, weddingLoc);

    std::ofstream of("pic.bmp", std::ofstream::binary);

    auto pixelFilter = [&graph, &overrides, &imageDimension, &roverLoc, &bachelorLoc, &weddingLoc] (size_t x, size_t y, uint8_t elevation) 
    {
        // Marks interesting positions on the map
        if (donut(x, y, roverLoc.X(), roverLoc.Y()) ||
            donut(x, y, bachelorLoc.X(), bachelorLoc.Y()) ||
            donut(x, y, weddingLoc.X(), weddingLoc.Y()))
        {
            return uint8_t(visualizer::IPV_PATH);
        }

        if (graph(y, x).getPath() == true)
        {
            return uint8_t(visualizer::IPV_PATH);
        }
        
        // Signifies water
        if ((overrides(y, x) & (OF_WATER_BASIN | OF_RIVER_MARSH)) || elevation == 0)
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
    std::cout << "FINISH" << std::endl;
    return 0;
}

