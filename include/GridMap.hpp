#ifndef GRIDMAP_HPP
#define GRIDMAP_HPP

#include <vector>
#include <string>
#include <filesystem>

namespace fs = std::filesystem;

struct LoadMapParameters
{
    std::string mapFile;
    uint8_t resolution;
    std::vector<double> origin{0,0,0};
    double occupancy_thresh;
    double free_thresh;
};

struct OccupancyGrid
{
    uint8_t resolution; // one cell represents resolution x resolution pixels | default = 1
    uint32_t width; // number of cells in the x direction
    uint32_t height; // number of cells in the y direction
    std::vector<double> origin; // [x, y, theta] of the lower-left corner of the map
    std::vector<int8_t> data; // 2D grid representation of the map (size = width * height)
};

class GridMap 
{
public:
    // Constructor to load map and parameters required for initialization
    GridMap();

    // Load map from PNG image and set parameters
    // This method handles loading map filename, set parameters
    // Return true if successful, false otherwise
    bool load_map(fs::path img_dir, const std::string & mapFile,  
        std::vector<double> origin, 
        double occupancy_thresh, 
        double free_thresh,
        uint8_t resolution = 1,
        bool decomposition = false);

    OccupancyGrid grid_; // 2D grid representation of the map

    ~GridMap();

private:

    LoadMapParameters map_params;
    // Convert image to occupancy grid, given that parameters are already set
    /*
        - load an image (PNG format) from the specified file path using ImageMagick
        - convert it into a grayscale matrix with pixel intensities -> occupancy values, using thresholds
        - store the resulting occupancy grid in the grid_ member variable
    */
    void convert_map(fs::path img_dir, bool decomposition = false);

};

#endif //GRIDMAP_HPP