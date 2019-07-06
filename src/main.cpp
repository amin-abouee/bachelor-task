#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <exception>
#include <unordered_set>
#include <bitset>

#include <nlohmann/json.hpp>

#include "visualizer.h"
#include "matrix.hpp"

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

std::vector<uint8_t> loadFile(const std::string& filename, const uint32_t expectedFileSize)
{
    // std::cout << "filename: " << filename << std::endl;
    // std::cout << "expected File Size: " << expectedFileSize << std::endl;
    const std::size_t fsize = fileSize(filename);
    if (fsize != expectedFileSize)
    {
        throw std::exception();
    }
    std::vector<uint8_t> data(fsize, 0);
    std::ifstream ifile(filename, std::ifstream::binary);
    if (!ifile.good())
    {
        throw std::exception();
    }
    ifile.read((char*)&data[0], fsize);
    return data;
}

std::vector<std::vector<uint8_t> > loadFile2(const std::string& filename, const uint32_t imageDimension)
{
    const uint32_t expectedFileSize = imageDimension * imageDimension;
    const std::size_t fsize = fileSize(filename);
    if (fsize != expectedFileSize)
    {
        throw std::exception();
    }
    // std::vector<uint8_t> data(fsize, 0);
    std::vector<std::vector<uint8_t> > data( imageDimension, std::vector<uint8_t>(imageDimension, 0) ) ;
    std::ifstream ifile(filename, std::ifstream::binary);
    if (!ifile.good())
    {
        throw std::exception();
    }
    ifile.read((char*)&(data[0][0]), fsize);
    ifile.close();
    return data;
}

bool donut(int x, int y, int x1, int y1)
{
    int dx = x - x1;
    int dy = y - y1;
    int r2 = dx * dx + dy * dy;
    return r2 >= 150 && r2 <= 400;
}

struct Point 
{
    double x, y;
    Point( double x, double y) :  x( x) , y( y) { }
};

void dijkstra_search()
{

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

    Matrix<uint8_t> a{imageDimension, imageDimension};


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
    auto elevation = loadFile(elevationFilepath, expectedFileSize);
    auto overrides = loadFile(overridesFilepath, expectedFileSize);

    // auto elevation2 = loadFile2(elevationFilepath, imageDimension);
    // auto overrides2 = loadFile2(overridesFilepath, imageDimension);

    // for (int i(0); i< 50; i++)
    // {
    //     for(int j(0); j<50; j++)
    //     {
    //         std::cout << (int)overrides2[i][j] << " ";
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

    std::unordered_set<uint8_t> set;
    for (const int &i: overrides) {
        set.insert(i);
    }

    for (const int &i: set) {
        std::string binary = std::bitset<8>(i).to_string();
        std::cout << i << " " << binary << std::endl;
    }


    std::ofstream of("pic.bmp", std::ofstream::binary);

    auto pixelFilter = [&overrides, &imageDimension, &roverLoc, &bachelorLoc, &weddingLoc] (size_t x, size_t y, uint8_t elevation) 
    {
            // Marks interesting positions on the map
            if (donut(x, y, roverLoc.first, roverLoc.second) ||
                donut(x, y, bachelorLoc.first, bachelorLoc.second) ||
                donut(x, y, weddingLoc.first, weddingLoc.second))
            {
                return uint8_t(visualizer::IPV_PATH);
            }
            
            // Signifies water
            if ((overrides[y * imageDimension + x] & (OF_WATER_BASIN | OF_RIVER_MARSH)) ||
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
        &elevation[0],
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

