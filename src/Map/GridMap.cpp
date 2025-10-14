#include "GridMap.hpp"
#include <iostream>
#include "Magick++.h"
#include "eigen3/Eigen/Dense"

GridMap::GridMap() {}

GridMap::~GridMap() {}

bool GridMap::load_map(const std::string & mapFile,  
    std::vector<double> origin, 
    double occupancy_thresh, 
    double free_thresh,
    uint8_t resolution = 1) 
{
    if (mapFile.empty()){

        std::cerr << "Error: Map file name is empty." << std::endl;
        return false;
    }
    // Set map parameters
    std::cout << "Loading map from file: " << mapFile << std::endl;
    this->map_params.mapFile = mapFile;
    this->map_params.resolution = resolution;
    this->map_params.origin = origin;
    this->map_params.occupancy_thresh = occupancy_thresh;
    this->map_params.free_thresh = free_thresh;

    // Load map from file
    try {
        convert_map();
    } catch (const std::exception & e) {
        std::cerr << "Error loading map: " << e.what() << std::endl;
        return false;
    }

    return true;
}


/*
    - load an image (PNG format) from the specified file path using ImageMagick
    - convert it into a grayscale matrix with pixel intensities -> occupancy values, using thresholds
    - store the resulting occupancy grid in the grid_ member variable
*/
void GridMap::convert_map(bool decomposition = false) {
    Magick::InitializeMagick(nullptr);
    
    std::cout << "Loading image file" << this->map_params.mapFile << std::endl;
    Magick::Image image(this->map_params.mapFile);

    // Populate map metadata
    this->grid_.width = image.size().width();
    this->grid_.height = image.size().height();
    this->grid_.resolution = this->map_params.resolution;
    this->grid_.origin[0] = this->map_params.origin[0]; // x
    this->grid_.origin[1] = this->map_params.origin[1]; // y
    this->grid_.origin[2] = this->map_params.origin[2]; // theta around z axis
    
    // Allocate matrix
    this->grid_.data.resize(this->grid_.width * this->grid_.height);

    // Convert to grayscale and extract raw bytes
    Magick::Image gray_img = image;
    gray_img.type(Magick::GrayscaleType);

    size_t width = gray_img.columns();
    size_t height = gray_img.rows();
    std::vector<uint8_t> gray_buffer(width * height);

    gray_img.write(0,0,width,height,"I",Magick::CharPixel,gray_buffer.data());

    // Map grayscale buffer into Eigen matrix
    Eigen::Map<Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> 
    gray_matrix(gray_buffer.data(), height, width);
    
    // Detect alpha channel
    bool has_alpha_channel = image.matte();

    // Convert grayscale to float in range [0,1]
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
    normalized = gray_matrix.cast<float>() / 255.0f;

    if (!decomposition) {
        std::cout << "No matrix decomposition applied." << std::endl;
        // Prepare result matrix
        Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> result_matrix(height, width);
    
        // Extract binary masks for occupied, free, unknown
        Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
        occupied_cells = (normalized.array() >= this->map_params.occupancy_thresh).cast<uint8_t>();
    
        // Free cells
        Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
        free_cells = (normalized.array() <= this->map_params.free_thresh).cast<uint8_t>();

        // Initialize result with unknown value (-1)
        result_matrix.setConstant(-1);

        // Scale occupancy values to [0,100]
        result_matrix = (occupied_cells.array() == 1).select(100, result_matrix);
        result_matrix = (free_cells.array() == 1).select(0, result_matrix);

        // Alpha handling (unknown areas)
        if (has_alpha_channel) {
            std::vector<uint8_t> alpha_buffer(width * height);
            image.write(0,0,width,height,"A",Magick::CharPixel,alpha_buffer.data());
            
            Eigen::Map<Eigen::Array<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> 
            alpha_array(alpha_buffer.data(), height, width);

            // Set cells with alpha = 0 to unknown (-1)
            result_matrix = (alpha_array < 255).select(-1, result_matrix);
        }

        // Convert result matrix to row-major 1D vector
        std::memcpy(this->grid_.data.data(), result_matrix.data(), width * height);

    } 
    else {
        std::cout << "Applying matrix decomposition with resolution: " << 
        static_cast<int>(this->grid_.resolution) << std::endl;

        // Compute width and height for coarse matrix (matrix decomposition)
        auto c_width = (this->grid_.width + this->grid_.resolution - 1) / this->grid_.resolution;
        auto c_height = (this->grid_.height + this->grid_.resolution - 1) / this->grid_.resolution;

        // Prepare coarse matrix
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> result_matrix(c_height, c_width);
        
        // For each coarse cell rectangle compute sums via integral image and apply threshold rules in 
        for (size_t i = 0; i < c_height; ++i) {
            for (size_t j = 0; j < c_width; ++j) {
                // Define the block boundaries
                size_t row_start = i * this->grid_.resolution;
                size_t row_end = std::min(row_start + this->grid_.resolution, height);
                size_t col_start = j * this->grid_.resolution;
                size_t col_end = std::min(col_start + this->grid_.resolution, width);

                // Extract the block and compute the mean
                auto block = normalized.block(row_start, col_start, row_end - row_start, col_end - col_start);
                result_matrix(i, j) = block.mean();
            }
        }

        // Extract binary masks for occupied, free, unknown
        Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
        occupied_cells = (result_matrix.array() >= this->map_params.occupancy_thresh).cast<uint8_t>();

        // Free cells
        Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
        free_cells = (result_matrix.array() <= this->map_params.free_thresh).cast<uint8_t>();

        // Unknown cells
        Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
        unknown_cells = ((result_matrix.array() < this->map_params.occupancy_thresh) && 
                         (result_matrix.array() > this->map_params.free_thresh)).cast<uint8_t>();

        // Scale occupancy values to [0,100]
        result_matrix = (occupied_cells.array() == 1).select(100, result_matrix);
        result_matrix = (free_cells.array() == 1).select(0, result_matrix);
        result_matrix = (unknown_cells.array() == 1).select(-1, result_matrix);

        // Convert result matrix to row-major 1D vector
        std::memcpy(this->grid_.data.data(), result_matrix.data(), width * height);
    }

    
}