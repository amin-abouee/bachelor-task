#include "visualizer.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <exception>
#include <unordered_set>
#include <bitset>

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
enum
{
    IMAGE_DIM = 2048, // Width and height of the elevation and overrides image
    
    ROVER_X = 159,
    ROVER_Y = 1520,
    BACHELOR_X = 1303,
    BACHELOR_Y = 85,
    WEDDING_X = 1577,
    WEDDING_Y = 1294
};

std::ifstream::pos_type fileSize(const std::string& filename)
{
    std::ifstream in(filename, std::ifstream::ate | std::ifstream::binary);
    if (!in.good())
    {
        throw std::exception();
    }
    return in.tellg(); 
}

std::vector<uint8_t> loadFile(const std::string& filename, size_t expectedFileSize)
{
    std::cout << "filename: " << filename << std::endl;
    std::cout << "expected File Size: " << expectedFileSize << std::endl;
    size_t fsize = fileSize(filename);
    if (fsize != expectedFileSize)
    {
        throw std::exception();
    }
    std::vector<uint8_t> data(fsize);
    std::ifstream ifile(filename, std::ifstream::binary);
    if (!ifile.good())
    {
        throw std::exception();
    }
    ifile.read((char*)&data[0], fsize);
    return data;
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
    constexpr size_t expectedFileSize = IMAGE_DIM * IMAGE_DIM;
    // Address assets relative to application location
    std::string anchor = std::string(".") + PATH_SEP;
    std::string pname = argv[0];
    auto lastpos = pname.find_last_of("/\\");
    if (lastpos != std::string::npos)
    {
        anchor = pname.substr(0, lastpos) + PATH_SEP;
    }
    // std::cout << "anchor: " << anchor << std::endl;
    auto elevation = loadFile("../assets/elevation.data", expectedFileSize);
    auto overrides = loadFile("../assets/overrides.data", expectedFileSize);

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
    
    visualizer::writeBMP(
        of,
        &elevation[0],
        IMAGE_DIM,
        IMAGE_DIM,
        [&] (size_t x, size_t y, uint8_t elevation) {
        
            // Marks interesting positions on the map
            if (donut(x, y, ROVER_X, ROVER_Y) ||
                donut(x, y, BACHELOR_X, BACHELOR_Y) ||
                donut(x, y, WEDDING_X, WEDDING_Y))
            {
                return uint8_t(visualizer::IPV_PATH);
            }
            
            // Signifies water
            if ((overrides[y * IMAGE_DIM + x] & (OF_WATER_BASIN | OF_RIVER_MARSH)) ||
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
    });
    of.flush();
#if __APPLE__
    auto res = system("open pic.bmp");
    (void)res;
#endif
    return 0;
}
