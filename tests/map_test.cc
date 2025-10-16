#include <gtest/gtest.h>
#include "Map/GridMap.hpp"
#include <filesystem>
#include <iostream>

namespace fs = std::filesystem;

class GridMapTest : public testing::Test 
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

};;
// Test 1
TEST_F(GridMapTest, AllFreeCells) {
    std::string mapFile = "test1.png"; // All free cells

    GridMap gm;
    std::vector<double> origin{0.0, 0.0, 0.0};
    double occ_thresh = 0.65;
    double free_thresh = 0.196;
    uint8_t resolution = 1;

    ASSERT_TRUE(gm.load_map(this->img_dir, mapFile, origin, occ_thresh, free_thresh, resolution));

    ASSERT_EQ(gm.grid_.resolution, resolution);

    EXPECT_EQ(gm.get_depth(), 16);

    // All cells should be free
    auto total = gm.grid_.width * gm.grid_.height;
    uint32_t free_count = 0;
    std::cout << "First grid cell value: " << static_cast<int>(gm.grid_.data[0]) << std::endl;
    for (auto i = 0; i < total; i++) {
        if (static_cast<int>(gm.grid_.data[i]) == 0) {
            free_count++;
        }
    }
    EXPECT_EQ(free_count, total);
}

// Test 2
TEST_F(GridMapTest, AllOccupiedCells) {
    std::string mapFile = "test2.png"; // All occupied cells
    fs::path out = img_dir / fs::path(mapFile);

    GridMap gm;
    std::vector<double> origin = {0.0, 0.0, 0.0};
    double occ_thresh = 0.65;
    double free_thresh = 0.196;
    uint8_t resolution = 1;

    ASSERT_TRUE(gm.load_map(img_dir, mapFile, origin, occ_thresh, free_thresh, resolution));

    ASSERT_EQ(gm.grid_.resolution, resolution);

    EXPECT_EQ(gm.get_depth(), 1);

    // All cells should be occupied
    auto total = gm.grid_.width * gm.grid_.height;
    uint32_t occ_count = 0;
    for (uint32_t i = 0; i < total; i++) {
        if (static_cast<int>(gm.grid_.data[i]) == 100) {
            occ_count++;
        }
    }
    EXPECT_EQ(occ_count, total);
}

// Test 3
TEST_F(GridMapTest, MixedCells) {
    std::string mapFile = "test3.png"; // Mixed cells
    fs::path out = img_dir / fs::path(mapFile);

    GridMap gm;
    std::vector<double> origin = {0.0, 0.0, 0.0};
    double occ_thresh = 0.65;
    double free_thresh = 0.196;
    uint8_t resolution = 1;

    ASSERT_TRUE(gm.load_map(img_dir, mapFile, origin, occ_thresh, free_thresh, resolution));

    ASSERT_EQ(gm.grid_.resolution, resolution);

    EXPECT_EQ(gm.get_depth(), 1);

    // In this test image, the top-left corner of the rectangle is 50x50
    // the bottom-right corner is 150x150
    // Check some inside points in rectangular area. should be occupied (100)
    EXPECT_EQ(gm.grid_.data[idx(50, 50, gm.grid_.width)], 100);
    EXPECT_EQ(gm.grid_.data[idx(75, 75, gm.grid_.width)], 100);
    EXPECT_EQ(gm.grid_.data[idx(100, 100, gm.grid_.width)], 100);

    // Check some inside points in rectangular area. should be free (0)
    EXPECT_EQ(gm.grid_.data[idx(10, 20, gm.grid_.width)], 0);
    EXPECT_EQ(gm.grid_.data[idx(20, 30, gm.grid_.width)], 0);
    EXPECT_EQ(gm.grid_.data[idx(160, 170, gm.grid_.width)], 0);
}

// Test 4
TEST_F(GridMapTest, CellsWithUnknown) {
    std::string mapFile = "test4.png"; // Grayscale image
    fs::path out = img_dir / fs::path(mapFile);

    GridMap gm;
    std::vector<double> origin = {0.0, 0.0, 0.0};
    double occ_thresh = 0.65;
    double free_thresh = 0.196;
    uint8_t resolution = 1;

    ASSERT_TRUE(gm.load_map(img_dir, mapFile, origin, occ_thresh, free_thresh, resolution));

    ASSERT_EQ(gm.grid_.resolution, resolution);

    EXPECT_EQ(gm.get_depth(), 16); // Grayscale image normally has depth of 16
    
    // Get the average of the first row
    int sum_first_row = 0;
    int cols = gm.grid_.width;
    for (uint32_t x = 0; x < gm.grid_.width; x++) {
        sum_first_row += static_cast<int>(gm.grid_.data[idx(x, 0, gm.grid_.width)]);
    }
    double avg_first_row = static_cast<double>(sum_first_row) / cols;
    
    // to pass: not equal to 100 or 0 -> contains unknown area
    EXPECT_NE(avg_first_row, 100);
    EXPECT_NE(avg_first_row, 0);

}

// Test 5
TEST_F(GridMapTest, CellsWithAlpha) {
    std::string mapFile = "test5.png"; // Image with alpha
    fs::path out = img_dir / fs::path(mapFile);

    GridMap gm;
    std::vector<double> origin = {0.0, 0.0, 0.0};
    double occ_thresh = 0.65;
    double free_thresh = 0.196;
    uint8_t resolution = 1;

    ASSERT_TRUE(gm.load_map(img_dir, mapFile, origin, occ_thresh, free_thresh, resolution));

    ASSERT_EQ(gm.grid_.resolution, resolution);

    EXPECT_EQ(gm.get_depth(), 16);

    // Expect all cell unknowns
    auto total = gm.grid_.width * gm.grid_.height;
    uint32_t unknown_count = 0;
    for (auto i = 0; i < total; i++) {
        if (static_cast<int>(gm.grid_.data[i]) == -1) {
            unknown_count++;
        }
    }
    EXPECT_EQ(unknown_count, total);

}

// Test 6
TEST_F(GridMapTest, PartialAlpha) {
    std::string mapFile = "test6.png"; // Image with partial alpha
    fs::path out = img_dir / fs::path(mapFile);

    GridMap gm;
    std::vector<double> origin = {0.0, 0.0, 0.0};
    double occ_thresh = 0.65;
    double free_thresh = 0.196;
    uint8_t resolution = 1;

    ASSERT_TRUE(gm.load_map(img_dir, mapFile, origin, occ_thresh, free_thresh, resolution));

    ASSERT_EQ(gm.grid_.resolution, resolution);

    EXPECT_EQ(gm.get_depth(), 16);

    // Expect all cell unknowns
    auto total = gm.grid_.width * gm.grid_.height;
    uint32_t unknown_count = 0;
    for (auto i = 0; i < total; i++) {
        if (static_cast<int>(gm.grid_.data[i]) == -1) {
            unknown_count++;
        }
    }
    EXPECT_NE(unknown_count, total);

}

// Test 7
TEST_F(GridMapTest, DecomposeImage) {
    std::string mapFile = "test1.png";
    fs::path out = img_dir / fs::path(mapFile);

    GridMap gm;
    std::vector<double> origin = {0.0, 0.0, 0.0};
    double occ_thresh = 0.65;
    double free_thresh = 0.196;
    uint8_t resolution = 4; // scale down by 4

    ASSERT_TRUE(gm.load_map(img_dir, mapFile, origin, occ_thresh, free_thresh, resolution, true));

    ASSERT_EQ(gm.grid_.resolution, resolution);

    EXPECT_EQ(gm.get_depth(), 16);

    // Expect that total cells is less than original (200x100=20000)
    auto total = gm.grid_.data.size();
    
    // Get the number of
    EXPECT_NE(total, 20000);

}

// Test 8
TEST_F(GridMapTest, DecomposeObstacledImage) {
    std::string mapFile = "test3.png";
    fs::path out = img_dir / fs::path(mapFile);

    GridMap gm;
    std::vector<double> origin = {0.0, 0.0, 0.0};
    double occ_thresh = 0.65;
    double free_thresh = 0.196;
    uint8_t resolution = 4; // scale down by 4

    ASSERT_TRUE(gm.load_map(img_dir, mapFile, origin, occ_thresh, free_thresh, resolution, true));

    ASSERT_EQ(gm.grid_.resolution, resolution);

    EXPECT_EQ(gm.get_depth(), 1);

    // Expect that total cells is less than original (200x100=20000)
    auto total = gm.grid_.data.size();
    
    // Get the number of
    EXPECT_NE(total, 20000);

}

// Test 9
TEST_F(GridMapTest, DecomposeMapImage) {
    std::string mapFile = "test9.png";
    fs::path out = img_dir / fs::path(mapFile);

    GridMap gm;
    std::vector<double> origin = {0.0, 0.0, 0.0};
    double occ_thresh = 0.65;
    double free_thresh = 0.196;
    uint8_t resolution = 4; // scale down by 4

    ASSERT_TRUE(gm.load_map(img_dir, mapFile, origin, occ_thresh, free_thresh, resolution, true));

    ASSERT_EQ(gm.grid_.resolution, resolution);

    EXPECT_EQ(gm.get_depth(), 8); // This image has depth of 8

    // Expect that total cells is less than original (200x100=20000)
    auto total = gm.grid_.data.size();
    auto img_size = 624 * 530;
    // Get the number of
    EXPECT_NE(total, img_size);

}

// Test 10
TEST_F(GridMapTest, CheckObstacleMapImage) {
    std::string mapFile = "test9.png";
    fs::path out = img_dir / fs::path(mapFile);

    GridMap gm;
    std::vector<double> origin = {0.0, 0.0, 0.0};
    double occ_thresh = 0.65;
    double free_thresh = 0.196;
    uint8_t resolution = 1; // scale down by 4

    ASSERT_TRUE(gm.load_map(img_dir, mapFile, origin, occ_thresh, free_thresh, resolution));

    ASSERT_EQ(gm.grid_.resolution, resolution);

    EXPECT_EQ(gm.get_depth(), 8); // This image has depth of 8

    // Expect that obstacle or free is located in this defined pixel location
    EXPECT_EQ(gm.grid_.data[idx(158, 95, gm.grid_.width)], 100);
    EXPECT_EQ(gm.grid_.data[idx(127, 82, gm.grid_.width)], 0);

}

// Test 11
TEST_F(GridMapTest, CheckObstacleMapImageWithDecomposition) {
    std::string mapFile = "test9.png";
    fs::path out = img_dir / fs::path(mapFile);

    GridMap gm;
    std::vector<double> origin = {0.0, 0.0, 0.0};
    double occ_thresh = 0.65;
    double free_thresh = 0.196;
    uint8_t resolution = 4; // scale down by 4

    ASSERT_TRUE(gm.load_map(img_dir, mapFile, origin, occ_thresh, free_thresh, resolution, true));

    ASSERT_EQ(gm.grid_.resolution, resolution);

    EXPECT_EQ(gm.get_depth(), 8); // This image has depth of 8

    // Expect that obstacle or free is located in this defined pixel location
    EXPECT_NE(gm.grid_.data[idx(158, 95, gm.grid_.width)], 100);
    EXPECT_EQ(gm.grid_.data[idx(127, 82, gm.grid_.width)], 0);

}
