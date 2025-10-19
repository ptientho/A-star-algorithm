#include <iostream>
#include <ranges>
#include <vector>
#include "AstarSearch/AstarSearch.hpp"
#include "Map/GridMap.hpp"
#include <filesystem>
#include <argparse/argparse.hpp>

namespace fs = std::filesystem;

// Helper function to calculate 1D index from 2D coordinates
uint32_t idx(uint32_t x, uint32_t y, uint32_t width) {
    return y * width + x;
}

int main(int argc, char** argv) {
    // Declare arguments
    argparse::ArgumentParser program("astar_search", "1.0");
    std::cout << "Starting: " << std::endl;
    program.add_argument("map_to_process").help("map image name to process occupancy grid (e.g. test1.png)");
    program.add_argument("map_to_draw").help("map image name to draw the path on (e.g. test1.png). If scaling is applied, use resized map");
    std::cout << "L1: " << std::endl;
    program.add_argument("-s","--start_position")
        .required()
        .help("start position in the map (e.g. 10 10)")
        .nargs(2)
        .scan<'i', uint32_t>();
    std::cout << "L2: " << std::endl;
    program.add_argument("-g","--goal_position")
        .required().help("goal position in the map (e.g. 20 20)")
        .nargs(2)
        .scan<'i', uint32_t>();
    std::cout << "L3: " << std::endl;
    int resolution;
    program.add_argument("--scaling_factor").help("map scaling factor. Default is 1")
        .default_value(1)
        .scan<'i', int>()
        .store_into(resolution);
    std::cout << "L4: " << std::endl;
    bool apply_scaling;
    program.add_argument("--apply_scaling").help("whether to use scaling factor or not")
        .default_value(false)
        .store_into(apply_scaling);
    std::cout << "L5: " << std::endl;
    std::string outFile;
    program.add_argument("--output_map").help("Output map name with shortest path")
        .default_value("output.png")
        .store_into(outFile);
    std::cout << "L6: " << std::endl;
    try{

        program.parse_args(argc, argv);
        std::cout << program << '\n';

    }catch (const std::exception& err){
        std::cerr << err.what() << std::endl;
        std::cerr << program;
        return 1;
    }

    fs::path img_dir = fs::path(PROJECT_SOURCE_DIR) / fs::path("images");
    
    std::string mapFile = program.get<std::string>("map_to_process");
    fs::path out = img_dir / fs::path(mapFile);

    // Assign map parameters
    GridMap gm;
    std::vector<double> origin = {0.0, 0.0, 0.0};
    double occ_thresh = 0.65;
    double free_thresh = 0.196;

    // Load map and convert to occupancy grid
    gm.load_map(img_dir, mapFile, origin, occ_thresh, free_thresh, static_cast<uint8_t>(resolution), apply_scaling);

    // Define start and goal locations (pixels in the map)
    auto start = program.get<std::vector<uint32_t>>("--start_position");
    auto goal = program.get<std::vector<uint32_t>>("--goal_position");
    size_t start_idx = idx(start[0] / static_cast<uint8_t>(resolution), start[1] / static_cast<uint8_t>(resolution), gm.grid_.width); // Starting at (10,10)
    size_t goal_idx = idx(goal[0] / static_cast<uint8_t>(resolution), goal[1] / static_cast<uint8_t>(resolution), gm.grid_.width); // Goal at (49, 23)

    std::vector<size_t> shortest_path;

    bool path_found = astar_search(gm.grid_, start_idx, goal_idx, shortest_path);

    if (path_found) {
        std::cout << "Path found!" << "\n";
        
        // Save image output of map and path
        mapFile = program.get<std::string>("map_to_draw"); // Declare as argument map_to_draw | if scaling is applied, use resized map
        create_map_path(img_dir, mapFile, shortest_path, outFile);
        std::cout << "\n";

    } else {
        std::cout << "No path found.\n";
    }
    
    return 0;

}