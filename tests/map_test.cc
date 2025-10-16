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

    // All cells should be free
    auto total = gm.grid_.width * gm.grid_.height;
    uint32_t free_count = 0;
    std::cout << "First grid cell value: " << static_cast<int>(gm.grid_.data[0]) << std::endl;
    for (auto i = 0; i < total; i++) {
        if (gm.grid_.data[i] == 0) {
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

    // All cells should be occupied
    auto total = gm.grid_.width * gm.grid_.height;
    uint32_t occ_count = 0;
    for (uint32_t i = 0; i < total; i++) {
        if (gm.grid_.data[i] == 100) {
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
    std::string mapFile = "test3.png"; // Grayscale image
    fs::path out = img_dir / fs::path(mapFile);

    GridMap gm;
    std::vector<double> origin = {0.0, 0.0, 0.0};
    double occ_thresh = 0.65;
    double free_thresh = 0.196;
    uint8_t resolution = 1;

    ASSERT_TRUE(gm.load_map(img_dir, mapFile, origin, occ_thresh, free_thresh, resolution));

    ASSERT_EQ(gm.grid_.resolution, resolution);
    // In this test image, the top-left corner of the rectangle is 50x50
    // the bottom-right corner is 150x150

    // normalized >= occupancy_thresh -> occupied (100)
    EXPECT_EQ(gm.grid_.data[idx(0, 0, gm.grid_.width)], 100);
    EXPECT_EQ(gm.grid_.data[idx(200, 100, gm.grid_.width)], 0);
    EXPECT_EQ(gm.grid_.data[idx(50, 50, gm.grid_.width)], -1);

//     Expected equality of these values:
//   gm.grid_.data[idx(0, 0, gm.grid_.width)]
//     Which is: '\0'
//   100
// /home/peera/A-star-algorithm/tests/map_test.cc:126: Failure
// Expected equality of these values:
//   gm.grid_.data[idx(50, 50, gm.grid_.width)]
//     Which is: 'd' (100, 0x64)
//   -1
    // get the average of the first row
    // to pass: not equal to 100 or 0 -> contains unknown area
}

// Test 5
TEST_F(GridMapTest, CellsWithAlpha) {

}