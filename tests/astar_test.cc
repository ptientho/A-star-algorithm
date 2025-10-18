#include <gtest/gtest.h>
#include "Map/GridMap.hpp"
#include "AstarSearch/AstarSearch.hpp"
#include <filesystem>
#include <iostream>

namespace fs = std::filesystem;

class AstarSearchTest : public testing::Test 
{
    protected:
        fs::path img_dir;

        // Helper function to calculate 1D index from 2D coordinates
        uint32_t idx(uint32_t x, uint32_t y, uint32_t width) {
            return y * width + x;
        }

        void SetUp() override {
            // Path to images folder in the project directory
            img_dir = fs::path(PROJECT_SOURCE_DIR) / fs::path("images");
            std::cout << "Image directory: " << img_dir << std::endl;

        }

};
// Test 1
TEST_F(AstarSearchTest, CheckPathFoundReturned) {
    std::string mapFile = "test1.png";
    fs::path out = img_dir / fs::path(mapFile);

    GridMap gm;
    std::vector<double> origin = {0.0, 0.0, 0.0};
    double occ_thresh = 0.65;
    double free_thresh = 0.196;
    uint8_t resolution = 4; // scale down by 4

    ASSERT_TRUE(gm.load_map(img_dir, mapFile, origin, occ_thresh, free_thresh, resolution, true));

    ASSERT_EQ(gm.grid_.resolution, resolution);

    ASSERT_EQ(gm.grid_.width, 50); // 200 / 4
    ASSERT_EQ(gm.grid_.height, 25); // 100 / 4

    size_t start_idx = idx(10, 10, gm.grid_.width); // Starting at (10,10)
    size_t goal_idx = idx(49, 23, gm.grid_.width); // Goal at (49, 23)

    std::vector<size_t> shortest_path;

    bool path_found = astar_search(gm.grid_, start_idx, goal_idx, shortest_path);

    EXPECT_TRUE(path_found);
    EXPECT_FALSE(shortest_path.empty());

}

// Test 2
TEST_F(AstarSearchTest, PrintMapPathMapScaling) {
    std::string mapFile = "test1.png";
    fs::path out = img_dir / fs::path(mapFile);

    GridMap gm;
    std::vector<double> origin = {0.0, 0.0, 0.0};
    double occ_thresh = 0.65;
    double free_thresh = 0.196;
    uint8_t resolution = 4; // scale down by 4

    ASSERT_TRUE(gm.load_map(img_dir, mapFile, origin, occ_thresh, free_thresh, resolution, true));

    size_t start_idx = idx(10, 10, gm.grid_.width); // Starting at (10,10)
    size_t goal_idx = idx(49, 23, gm.grid_.width); // Goal at (49, 23)

    std::cout << "Grid width: " << gm.grid_.width << ", height: " << gm.grid_.height << "\n";
    
    std::vector<size_t> shortest_path;

    bool path_found = astar_search(gm.grid_, start_idx, goal_idx, shortest_path);

    ASSERT_TRUE(path_found);
    ASSERT_FALSE(shortest_path.empty());

    // Print 10 first indices of the path
    std::cout << "Size of shortest path: " << shortest_path.size() << "\n";
    for (size_t i = 0; i < std::min(shortest_path.size(), static_cast<size_t>(50)); ++i) {
        // convert 1D index to 2D coordinates
        size_t x = shortest_path[i] % gm.grid_.width;
        size_t y = shortest_path[i] / gm.grid_.width;
        std::cout << "(" << x << "," << y << ") ";
    }
    std::cout << "\n";

    // Create map path image
    mapFile = "test1_resize.png";
    std::string outFile = "output.png";
    create_map_path(img_dir, mapFile, shortest_path, outFile, 2);

}

// Test 3
TEST_F(AstarSearchTest, PrintMapPathMapNoScale) {
    std::string mapFile = "test1.png";
    fs::path out = img_dir / fs::path(mapFile);

    GridMap gm;
    std::vector<double> origin = {0.0, 0.0, 0.0};
    double occ_thresh = 0.65;
    double free_thresh = 0.196;
    uint8_t resolution = 1; // scale down by 4

    ASSERT_TRUE(gm.load_map(img_dir, mapFile, origin, occ_thresh, free_thresh, resolution));

    size_t start_idx = idx(10, 10, gm.grid_.width); // Starting at (10,10)
    size_t goal_idx = idx(49, 23, gm.grid_.width); // Goal at (49, 23)

    std::cout << "Grid width: " << gm.grid_.width << ", height: " << gm.grid_.height << "\n";
    
    std::vector<size_t> shortest_path;

    bool path_found = astar_search(gm.grid_, start_idx, goal_idx, shortest_path);

    ASSERT_TRUE(path_found);
    ASSERT_FALSE(shortest_path.empty());

    // Print 10 first indices of the path
    std::cout << "Size of shortest path: " << shortest_path.size() << "\n";
    for (size_t i = 0; i < std::min(shortest_path.size(), static_cast<size_t>(50)); ++i) {
        // convert 1D index to 2D coordinates
        size_t x = shortest_path[i] % gm.grid_.width;
        size_t y = shortest_path[i] / gm.grid_.width;
        std::cout << "(" << x << "," << y << ") ";
    }
    std::cout << "\n";

    // Create map path image
    mapFile = "test1.png";
    std::string outFile = "output_path2.png";
    create_map_path(img_dir, mapFile, shortest_path, outFile, 2);
}

// Test 4
TEST_F(AstarSearchTest, NoPathFound) {
    std::string mapFile = "test2.png";
    fs::path out = img_dir / fs::path(mapFile);

    GridMap gm;
    std::vector<double> origin = {0.0, 0.0, 0.0};
    double occ_thresh = 0.65;
    double free_thresh = 0.196;
    uint8_t resolution = 1; // scale down by 4

    ASSERT_TRUE(gm.load_map(img_dir, mapFile, origin, occ_thresh, free_thresh, resolution));

    size_t start_idx = idx(10, 10, gm.grid_.width); // Starting at (10,10)
    size_t goal_idx = idx(49, 23, gm.grid_.width); // Goal at (49, 23)

    std::cout << "Grid width: " << gm.grid_.width << ", height: " << gm.grid_.height << "\n";
    
    std::vector<size_t> shortest_path;

    bool path_found = astar_search(gm.grid_, start_idx, goal_idx, shortest_path);

    ASSERT_FALSE(path_found);
    ASSERT_FALSE(shortest_path.empty());

}