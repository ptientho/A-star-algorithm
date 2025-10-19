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

    size_t start_idx = idx(10, 10, gm.grid_.width);
    size_t goal_idx = idx(49, 23, gm.grid_.width);

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
    std::string outFile = "output_path1.png";
    create_map_path(img_dir, mapFile, shortest_path, outFile);

}

// Test 3
TEST_F(AstarSearchTest, PrintMapPathMapNoScale) {
    std::string mapFile = "test1.png";
    fs::path out = img_dir / fs::path(mapFile);

    GridMap gm;
    std::vector<double> origin = {0.0, 0.0, 0.0};
    double occ_thresh = 0.65;
    double free_thresh = 0.196;
    uint8_t resolution = 1;

    ASSERT_TRUE(gm.load_map(img_dir, mapFile, origin, occ_thresh, free_thresh, resolution));

    size_t start_idx = idx(10, 10, gm.grid_.width);
    size_t goal_idx = idx(49, 23, gm.grid_.width);

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
    create_map_path(img_dir, mapFile, shortest_path, outFile);
}

// Test 4
TEST_F(AstarSearchTest, NoPathFound) {
    std::string mapFile = "test2.png";
    fs::path out = img_dir / fs::path(mapFile);

    GridMap gm;
    std::vector<double> origin = {0.0, 0.0, 0.0};
    double occ_thresh = 0.65;
    double free_thresh = 0.196;
    uint8_t resolution = 1;

    ASSERT_TRUE(gm.load_map(img_dir, mapFile, origin, occ_thresh, free_thresh, resolution));

    size_t start_idx = idx(10, 10, gm.grid_.width);
    size_t goal_idx = idx(49, 23, gm.grid_.width);

    std::cout << "Grid width: " << gm.grid_.width << ", height: " << gm.grid_.height << "\n";
    
    std::vector<size_t> shortest_path;

    bool path_found = astar_search(gm.grid_, start_idx, goal_idx, shortest_path);

    ASSERT_FALSE(path_found);
    ASSERT_TRUE(shortest_path.empty());

}

// Test 5
TEST_F(AstarSearchTest, PathWithObstacles) {
    std::string mapFile = "test3.png";
    fs::path out = img_dir / fs::path(mapFile);

    GridMap gm;
    std::vector<double> origin = {0.0, 0.0, 0.0};
    double occ_thresh = 0.65;
    double free_thresh = 0.196;
    uint8_t resolution = 1;

    ASSERT_TRUE(gm.load_map(img_dir, mapFile, origin, occ_thresh, free_thresh, resolution));

    size_t start_idx = idx(31, 31, gm.grid_.width); 
    size_t goal_idx = idx(179, 178, gm.grid_.width);

    std::cout << "Grid width: " << gm.grid_.width << ", height: " << gm.grid_.height << "\n";
    
    std::vector<size_t> shortest_path;

    bool path_found = astar_search(gm.grid_, start_idx, goal_idx, shortest_path);

    ASSERT_TRUE(path_found);
    ASSERT_FALSE(shortest_path.empty());

    // Create map path image
    mapFile = "test3.png";
    std::string outFile = "output_path3.png";
    create_map_path(img_dir, mapFile, shortest_path, outFile);

}

// Test 6
TEST_F(AstarSearchTest, PathScalingWithObstacles) {
    std::string mapFile = "test3.png";
    fs::path out = img_dir / fs::path(mapFile);

    GridMap gm;
    std::vector<double> origin = {0.0, 0.0, 0.0};
    double occ_thresh = 0.65;
    double free_thresh = 0.196;
    uint8_t resolution = 4; // scale down by 4

    ASSERT_TRUE(gm.load_map(img_dir, mapFile, origin, occ_thresh, free_thresh, resolution, true));

    size_t start_idx = idx(7, 7, gm.grid_.width);
    size_t goal_idx = idx(43, 43, gm.grid_.width); 

    std::cout << "Grid width: " << gm.grid_.width << ", height: " << gm.grid_.height << "\n";
    
    std::vector<size_t> shortest_path;

    bool path_found = astar_search(gm.grid_, start_idx, goal_idx, shortest_path);

    ASSERT_TRUE(path_found);
    ASSERT_FALSE(shortest_path.empty());

    // Create map path image
    mapFile = "test3_resize.png";
    std::string outFile = "output_path4.png";
    create_map_path(img_dir, mapFile, shortest_path, outFile);

}

// Test 7
TEST_F(AstarSearchTest, GradientMap) {
    std::string mapFile = "test4.png";
    fs::path out = img_dir / fs::path(mapFile);

    GridMap gm;
    std::vector<double> origin = {0.0, 0.0, 0.0};
    double occ_thresh = 0.65;
    double free_thresh = 0.196;
    uint8_t resolution = 1;

    ASSERT_TRUE(gm.load_map(img_dir, mapFile, origin, occ_thresh, free_thresh, resolution, false));

    size_t start_idx = idx(188, 8, gm.grid_.width); 
    size_t goal_idx = idx(196, 84, gm.grid_.width); 

    std::cout << "Grid width: " << gm.grid_.width << ", height: " << gm.grid_.height << "\n";
    
    std::vector<size_t> shortest_path;

    bool path_found = astar_search(gm.grid_, start_idx, goal_idx, shortest_path);

    ASSERT_TRUE(path_found);
    ASSERT_FALSE(shortest_path.empty());

    // Create map path image
    mapFile = "test4.png";
    std::string outFile = "output_path5.png";
    create_map_path(img_dir, mapFile, shortest_path, outFile);

}

// Test 8
TEST_F(AstarSearchTest, RealMap) {
    std::string mapFile = "test9.png";
    fs::path out = img_dir / fs::path(mapFile);

    GridMap gm;
    std::vector<double> origin = {0.0, 0.0, 0.0};
    double occ_thresh = 0.65;
    double free_thresh = 0.196;
    uint8_t resolution = 1;

    ASSERT_TRUE(gm.load_map(img_dir, mapFile, origin, occ_thresh, free_thresh, resolution, false));

    size_t start_idx = idx(65, 41, gm.grid_.width);
    size_t goal_idx = idx(367, 187, gm.grid_.width);

    std::cout << "Grid width: " << gm.grid_.width << ", height: " << gm.grid_.height << "\n";
    
    std::vector<size_t> shortest_path;

    bool path_found = astar_search(gm.grid_, start_idx, goal_idx, shortest_path);

    ASSERT_TRUE(path_found);
    ASSERT_FALSE(shortest_path.empty());

    // Create map path image
    mapFile = "test9.png";
    std::string outFile = "output_path6.png";
    create_map_path(img_dir, mapFile, shortest_path, outFile);

}

// Test 9
TEST_F(AstarSearchTest, RealMapScaling) {
    std::string mapFile = "test9.png";
    fs::path out = img_dir / fs::path(mapFile);

    GridMap gm;
    std::vector<double> origin = {0.0, 0.0, 0.0};
    double occ_thresh = 0.65;
    double free_thresh = 0.196;
    uint8_t resolution = 2;

    ASSERT_TRUE(gm.load_map(img_dir, mapFile, origin, occ_thresh, free_thresh, resolution, true));

    size_t start_idx = idx(30, 26, gm.grid_.width);
    size_t goal_idx = idx(185, 94, gm.grid_.width);

    std::cout << "Grid width: " << gm.grid_.width << ", height: " << gm.grid_.height << "\n";
    
    std::vector<size_t> shortest_path;

    bool path_found = astar_search(gm.grid_, start_idx, goal_idx, shortest_path);

    ASSERT_TRUE(path_found);
    ASSERT_FALSE(shortest_path.empty());

    // Create map path image
    mapFile = "test9_resize.png";
    std::string outFile = "output_path7.png";
    create_map_path(img_dir, mapFile, shortest_path, outFile);

}

// Test 10
TEST_F(AstarSearchTest, RealMapGoalBlocked) {
    std::string mapFile = "test9.png";
    fs::path out = img_dir / fs::path(mapFile);

    GridMap gm;
    std::vector<double> origin = {0.0, 0.0, 0.0};
    double occ_thresh = 0.65;
    double free_thresh = 0.196;
    uint8_t resolution = 2;

    ASSERT_TRUE(gm.load_map(img_dir, mapFile, origin, occ_thresh, free_thresh, resolution, true));

    size_t start_idx = idx(41, 27, gm.grid_.width);
    size_t goal_idx = idx(246, 46, gm.grid_.width); // the goal is located inside an obstacle

    std::cout << "Grid width: " << gm.grid_.width << ", height: " << gm.grid_.height << "\n";
    
    std::vector<size_t> shortest_path;

    bool path_found = astar_search(gm.grid_, start_idx, goal_idx, shortest_path);

    EXPECT_FALSE(path_found);
    EXPECT_TRUE(shortest_path.empty());

}

// Test 11
TEST_F(AstarSearchTest, NarrowCorridors) {
    std::string mapFile = "andino_office.png";
    fs::path out = img_dir / fs::path(mapFile);

    GridMap gm;
    std::vector<double> origin = {0.0, 0.0, 0.0};
    double occ_thresh = 0.65;
    double free_thresh = 0.196;
    uint8_t resolution = 2;

    ASSERT_TRUE(gm.load_map(img_dir, mapFile, origin, occ_thresh, free_thresh, resolution, true));

    size_t start_idx = idx(389, 265, gm.grid_.width);
    size_t goal_idx = idx(118, 231, gm.grid_.width);

    std::cout << "Grid width: " << gm.grid_.width << ", height: " << gm.grid_.height << "\n";
    
    std::vector<size_t> shortest_path;

    bool path_found = astar_search(gm.grid_, start_idx, goal_idx, shortest_path);

    ASSERT_TRUE(path_found);
    ASSERT_FALSE(shortest_path.empty());

    // Create map path image
    mapFile = "andino_office_resize.png";
    std::string outFile = "output_path8.png";
    create_map_path(img_dir, mapFile, shortest_path, outFile);

}

// Test 12
TEST_F(AstarSearchTest, NarrowCorridorsScalingUp1) {
    std::string mapFile = "andino_office.png";
    fs::path out = img_dir / fs::path(mapFile);

    GridMap gm;
    std::vector<double> origin = {0.0, 0.0, 0.0};
    double occ_thresh = 0.65;
    double free_thresh = 0.196;
    uint8_t resolution = 3;

    ASSERT_TRUE(gm.load_map(img_dir, mapFile, origin, occ_thresh, free_thresh, resolution, true));

    size_t start_idx = idx(264, 174, gm.grid_.width);
    size_t goal_idx = idx(80, 155, gm.grid_.width);

    std::cout << "Grid width: " << gm.grid_.width << ", height: " << gm.grid_.height << "\n";
    
    std::vector<size_t> shortest_path;

    bool path_found = astar_search(gm.grid_, start_idx, goal_idx, shortest_path);

    ASSERT_TRUE(path_found);
    ASSERT_FALSE(shortest_path.empty());

    // Create map path image
    mapFile = "andino_office_resize2.png";
    std::string outFile = "output_path9.png";
    create_map_path(img_dir, mapFile, shortest_path, outFile);

}

// Test 13
TEST_F(AstarSearchTest, NarrowCorridorsScalingUp2) {
    std::string mapFile = "andino_office.png";
    fs::path out = img_dir / fs::path(mapFile);

    GridMap gm;
    std::vector<double> origin = {0.0, 0.0, 0.0};
    double occ_thresh = 0.65;
    double free_thresh = 0.196;
    uint8_t resolution = 4;

    ASSERT_TRUE(gm.load_map(img_dir, mapFile, origin, occ_thresh, free_thresh, resolution, true));

    size_t start_idx = idx(199, 132, gm.grid_.width);
    size_t goal_idx = idx(62, 117, gm.grid_.width);

    std::cout << "Grid width: " << gm.grid_.width << ", height: " << gm.grid_.height << "\n";
    
    std::vector<size_t> shortest_path;

    bool path_found = astar_search(gm.grid_, start_idx, goal_idx, shortest_path);

    ASSERT_TRUE(path_found);
    ASSERT_FALSE(shortest_path.empty());

    // Create map path image
    mapFile = "andino_office_resize3.png";
    std::string outFile = "output_path10.png";
    create_map_path(img_dir, mapFile, shortest_path, outFile);

}