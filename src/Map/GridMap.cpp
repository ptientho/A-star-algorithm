#include "GridMap.hpp"
#include <iostream>

using MagickCore::Quantum;

GridMap::GridMap() {}

GridMap::~GridMap() {}

bool GridMap::load_map(fs::path img_dir, const std::string & mapFile,  
    std::vector<double> origin, 
    double occupancy_thresh, 
    double free_thresh,
    uint8_t resolution,
    bool decomposition) 
{
    if (mapFile.empty()){
        std::cerr << "Error: Map file name is empty." << std::endl;
        return false;
    }
    
    fs::path out = img_dir / fs::path(mapFile);
    if (!fs::exists(out)) {
        std::cerr << "Error: Map file does not exist: " << out << std::endl;
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
        convert_map(img_dir, decomposition);
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
void GridMap::convert_map(fs::path img_dir, bool decomposition) {
    Magick::InitializeMagick(nullptr);
    
    std::cout << "Loading image file " << this->map_params.mapFile << std::endl;
    fs::path out = img_dir / fs::path(this->map_params.mapFile);
    Magick::Image image;
    try {
        image.read(out.string());
    } catch (const Magick::Exception &e) {
        std::cerr << "Error loading image file: " << e.what() << std::endl;
        throw;
    }

    if (image.columns() == 0 || image.rows() == 0) {
        throw std::runtime_error("Loaded image has zero width/height: " + this->map_params.mapFile);
    }

    // Populate map metadata
    this->grid_.width = static_cast<uint32_t>(image.columns());
    this->grid_.height = static_cast<uint32_t>(image.rows());
    this->grid_.resolution = static_cast<uint8_t>(this->map_params.resolution);
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

    // Check image depth
    depth = gray_img.depth(); // bits per channel (1, 8, 16, ...)
    
    // Choose appropriate pixel type based on depth
    std::variant<
        Eigen::Array<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>,
        Eigen::Array<uint16_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> alpha_array;

    if (depth <= 8) {
        alpha_array = construct_alpha_array<uint8_t>(depth);
    } else if (depth == 16) {
        alpha_array = construct_alpha_array<uint16_t>(depth);
    } else {
        throw std::invalid_argument("Unsupported image depth: " + std::to_string(depth));
    }
    
    // Prepare normalized matrix
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> normalized(height, width);
    
    // Detect alpha channel if present
    #if defined(MAGICKCORE_QUANTUM_DEPTH) || defined(MAGICKCORE_HDRI_ENABLE)
        // likely ImageMagick7 / modern Magick++: use alphaChannel() if available
        // note: some versions provide hasAlphaChannel() or alpha(); check your Magick++ reference if compile fails.
        bool has_alpha_channel = image.alpha();
    #else
        // older API
        bool has_alpha_channel = image.matte();
    #endif

    // Normalize pixels within a range (0,1) | WHITE=0.0, BLACK=1.0
    this->normalize_pixel(normalized, gray_img, width, height, depth);
    
    // Handle alpha channel if present
    if (has_alpha_channel) {
        this->fill_alpha_array(alpha_array, image, width, height, depth);
    }
    

    if (has_alpha_channel) {
        std::cout << "Image has alpha channel." << std::endl;
    } else {
        std::cout << "Image does not have alpha channel." << std::endl;
    }

    std::cout << "First ten elements of normalized matrix:" << std::endl;
    for (int i = 0; i < std::min(20, static_cast<int>(normalized.size())); ++i) {
        std::cout << normalized.data()[i] << " ";
    }
    std::cout << std::endl;

    if (!decomposition) {
        std::cout << "No matrix decomposition applied." << std::endl;
        
        // Prepare result matrix
        Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> result_matrix(height, width);

        // Extract binary masks for occupied, free, unknown
        Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
        occupied_cells = (normalized.array() >= this->map_params.occupancy_thresh).cast<uint8_t>(); // array of 0s and 1s
        
        // Free cells
        Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
        free_cells = (normalized.array() <= this->map_params.free_thresh).cast<uint8_t>(); // array of 0s and 1s

        // Initialize result with unknown value (-1)
        result_matrix.setConstant(-1); // array of -1s

        // print first ten elements of -1s
        std::cout << "First ten elements of initialized result matrix:" << std::endl;
        for (int i = 0; i < std::min(20, static_cast<int>(result_matrix.size())); ++i) {
            std::cout << static_cast<int>(result_matrix.data()[i]) << " ";
        }
        // Scale occupancy values to [0,100]
        result_matrix = (occupied_cells.array() > 0).select(100, result_matrix);
        result_matrix = (free_cells.array() > 0).select(0, result_matrix);

        // Select unknown cells where alpha < 255 if alpha channel exists
        if (has_alpha_channel) {
            if (depth <= 8){
                // Set cells with alpha = 0 to unknown (-1)
                result_matrix = (std::get<Eigen::Array<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(alpha_array) < 255).select(-1, result_matrix);
            } else {
                // Set cells with alpha = 0 to unknown (-1)
                result_matrix = (std::get<Eigen::Array<uint16_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(alpha_array) < 65535).select(-1, result_matrix);
            }
        }

        std::cout << "First ten elements of result matrix:" << std::endl;
        for (int i = 0; i < std::min(20, static_cast<int>(result_matrix.size())); ++i) {
            std::cout << static_cast<int>(result_matrix.data()[i]) << " ";
        }
        std::cout << std::endl;

        // Convert result matrix to row-major 1D vector
        std::memcpy(this->grid_.data.data(), result_matrix.data(), width * height);
        // Print a random member of grid data
        if (!this->grid_.data.empty()) {
            size_t random_index = std::rand() % this->grid_.data.size();
            std::cout << "Random grid data value: " << static_cast<int>(this->grid_.data[random_index]) << std::endl;
        } else {
            std::cerr << "Grid data is empty." << std::endl;
        }
    } 
    else {
        std::cout << "Applying matrix decomposition with resolution: " << 
        static_cast<int>(this->grid_.resolution) << std::endl;

        // Compute width and height for coarse matrix (matrix decomposition)
        auto c_width = (this->grid_.width + this->grid_.resolution - 1) / this->grid_.resolution;
        auto c_height = (this->grid_.height + this->grid_.resolution - 1) / this->grid_.resolution;

        // Resize grid data to coarse size
        this->grid_.width = static_cast<uint32_t>(c_width);
        this->grid_.height = static_cast<uint32_t>(c_height);
        this->grid_.data.resize(c_width * c_height);

        // Print width and height
        std::cout << "Coarse matrix dimensions: " << c_width << " x " << c_height << std::endl;

        // Prepare coarse matrix
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> result_matrix(c_height, c_width);
        
        // For each coarse cell rectangle compute sums via integral image and apply threshold rules in 
        for (size_t i = 0; i < c_height; ++i) {
            for (size_t j = 0; j < c_width; ++j) {
                // Define the block boundaries
                size_t row_start = i * this->grid_.resolution;
                size_t row_end = std::min(row_start + this->grid_.resolution, height);
                size_t col_start = j * this->grid_.resolution;
                size_t col_end = std::min(col_start + this->grid_.resolution, width);

                // Compute block sizes
                const Eigen::Index block_h = static_cast<Eigen::Index>(row_end - row_start);
                const Eigen::Index block_w = static_cast<Eigen::Index>(col_end - col_start);

                // Extract the block and compute the mean
                Eigen::Block<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
                    block = normalized.block(static_cast<Eigen::Index>(row_start), static_cast<Eigen::Index>(col_start), block_h, block_w);

                const double sum = block.sum();
                const double mean = sum / (static_cast<double>(block_h) * static_cast<double>(block_w));
                result_matrix(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j)) = mean;
            }
        }

        Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> result_int8(
        result_matrix.rows(), result_matrix.cols());

        result_int8.setConstant(-1);

        // Extract binary masks for occupied, free, unknown
        Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
        occupied_cells = (result_matrix.array() >= this->map_params.occupancy_thresh).cast<uint8_t>();

        // Free cells
        Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
        free_cells = (result_matrix.array() <= this->map_params.free_thresh).cast<uint8_t>();

    
        result_int8 = (occupied_cells.array() > 0).select(static_cast<int8_t>(100), result_int8);
        result_int8 = (free_cells.array() > 0).select(static_cast<int8_t>(0), result_int8);

        // Select unknown cells where alpha < 255 if alpha channel exists
        // if (has_alpha_channel) {
        //     if (depth <= 8){
        //         // Set cells with alpha = 0 to unknown (-1)
        //         result_matrix = (std::get<Eigen::Array<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(alpha_array) < 255).select(-1, result_matrix);
        //     } else {
        //         // Set cells with alpha = 0 to unknown (-1)
        //         result_matrix = (std::get<Eigen::Array<uint16_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(alpha_array) < 65535).select(-1, result_matrix);
        //     }
        // }
        //result_matrix = (unknown_cells.array() == 1).select(-1, result_matrix);
        
        
        // Convert result matrix to row-major 1D vector
        std::memcpy(this->grid_.data.data(), result_int8.data(), c_width * c_height);
        // Print a random member of grid data
        if (!this->grid_.data.empty()) {
            size_t random_index = std::rand() % this->grid_.data.size();
            std::cout << "Random grid data value: " << static_cast<int>(this->grid_.data[random_index]) << std::endl;
        } else {
            std::cerr << "Grid data is empty." << std::endl;
        }
    }

    
}

void GridMap::normalize_pixel(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> & norm_mat,
        Magick::Image & img, size_t width, size_t height, unsigned int depth){
        if (depth == 1) {
            // 1-bit image
            std::vector<uint8_t> buf(static_cast<size_t>(width * height));
            
            img.write(0, 0, static_cast<unsigned long>(width), static_cast<unsigned long>(height),
                    "I", Magick::CharPixel, buf.data());

            for (Eigen::Index r = 0; r < height; ++r) {
                for (Eigen::Index c = 0; c < width; ++c) {
                    uint8_t v = buf[static_cast<size_t>(r * width + c)];
                    norm_mat(r, c) = (v > 0) ? 0.0 : 1.0;
                }
            }
        } else if (depth == 8) {
            // Typical 8-bit image (0..255)
            std::vector<uint8_t> buf(static_cast<size_t>(width * height));
            
            img.write(0, 0, static_cast<unsigned long>(width), static_cast<unsigned long>(height),
                    "I", Magick::CharPixel, buf.data());

            const double denom = 255.0;
            // Vectorize via Eigen::Map if desired, but loop is clear and portable:
            for (Eigen::Index r = 0; r < height; ++r) {
                for (Eigen::Index c = 0; c < width; ++c) {
                    norm_mat(r, c) = 1.0 - static_cast<double>(buf[static_cast<size_t>(r * width + c)]) / denom;
                }
            }
        } else if (depth == 16) {
            // depth > 8 (e.g., 16-bit) -> read as ShortPixel (uint16_t)
            std::vector<uint16_t> buf(static_cast<size_t>(width * height));
            
            img.write(0, 0, static_cast<unsigned long>(width), static_cast<unsigned long>(height),
                    "I", Magick::ShortPixel, buf.data());

            // denom = (2^depth - 1) but often depth==16 => denom=65535
            const double denom = static_cast<double>((1u << depth) - 1u);
            for (Eigen::Index r = 0; r < height; ++r) {
                for (Eigen::Index c = 0; c < width; ++c) {
                    norm_mat(r, c) = 1.0 - static_cast<double>(buf[static_cast<size_t>(r * width + c)]) / denom;
                }
            }
        } else {
            throw std::invalid_argument("Unsupported image depth: " + std::to_string(depth));
        }

}

void GridMap::fill_alpha_array(AlphaArray & alpha_array,
        Magick::Image & img, size_t width, size_t height, unsigned int depth) {

     std::visit([&](auto&& arr) {
        // Determine the type of arr at runtime for assignment using with variant
        using ArrayScalar = typename std::decay_t<decltype(arr)>::Scalar;
        
        // Get a pointer to the raw image data
        const auto* img_pixels = img.getConstPixels(0,0,width,height);
        
        // Cast the pixel data to the correct type and create Eigen::Map
        if (depth <= 8) {
                // 8-bit or less depth
                using MagickScalar = uint8_t;
                const auto* typed_pixels = reinterpret_cast<const MagickScalar*>(img_pixels);
                Eigen::Map<const Eigen::Array<MagickScalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
                    magick_map(typed_pixels, height, width);
                
                // Assign with an explicit cast
                arr = magick_map.template cast<ArrayScalar>();

            } else if (depth == 16) {
                // For 16-bit depth, get a 16-bit view
                using MagickScalar = uint16_t;
                const auto* typed_pixels = reinterpret_cast<const MagickScalar*>(img_pixels);
                Eigen::Map<const Eigen::Array<MagickScalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
                    magick_map(typed_pixels, height, width);
                
                // Assign with an explicit cast using .template cast()
                arr = magick_map.template cast<ArrayScalar>();
            } else {
                throw std::invalid_argument("Unsupported image depth: " + std::to_string(depth));
            }
     }, alpha_array);  
}

void create_map_path(fs::path img_dir, const std::string & mapFile, const std::vector<size_t> & shortest_path, const std::string & output_filename) {
    
    Magick::InitializeMagick(nullptr);
    // Read image file
    if (mapFile.empty()){
        std::cerr << "Error: Map file name is empty." << std::endl;
        return;
    }

    fs::path out = img_dir / fs::path(mapFile);
    if (!fs::exists(out)) {
        std::cerr << "Error: Map file does not exist: " << out << std::endl;
        return;
    }

    Magick::Image image;
    try {
        image.read(out.string());
    } catch (const Magick::Exception &e) {
        std::cerr << "Error loading image file: " << e.what() << std::endl;
        return;
    }

    const auto img_w = static_cast<size_t>(image.columns());
    const auto img_h = static_cast<size_t>(image.rows());

    if (img_w == 0 || img_h == 0) {
        std::cerr << "Error: image has zero width or height." << std::endl;
        return;
    }

    
    
    // Helper to draw a small square centered at (x,y)
    // auto draw_point = [&](size_t x, size_t y) {
    //     // signed coords for Magick API safety
    //     long cx = static_cast<long>(x);
    //     long cy = static_cast<long>(y);

    //     // draw a small square of size (2*point_radius + 1)
    //     for (int dy = -point_radius; dy <= point_radius; ++dy) {
    //         long yy = cy + dy;
    //         if (yy < 0 || static_cast<size_t>(yy) >= img_h) continue;
    //         for (int dx = -point_radius; dx <= point_radius; ++dx) {
    //             long xx = cx + dx;
    //             if (xx < 0 || static_cast<size_t>(xx) >= img_w) continue;
    //             try {
    //                 image.pixelColor(static_cast<size_t>(xx), static_cast<size_t>(yy), blue_rgb);
    //                 //std::cout << "Set pixel at (" << xx << "," << yy << ") to blue." << "\n";
    //             } catch (const Magick::Exception &e) {
    //                 // Shouldn't normally happen, but guard anyway
    //                 std::cerr << "Warning: pixelColor failed at (" << xx << "," << yy << "): " << e.what() << std::endl;
    //             }
    //         }
    //     }
    // };

    // Draw each path point
    const size_t total_pixels = img_w * img_h;
    for (size_t idx = 0; idx < shortest_path.size(); ++idx) {
        size_t index = shortest_path[idx];

        // Bounds check
        if (index >= total_pixels) {
            std::cerr << "Warning: path index " << index << " is out of image bounds (skip)" << std::endl;
            continue;
        }

        ssize_t gx = static_cast<ssize_t>(index % img_w);
        ssize_t gy = static_cast<ssize_t>(index / img_w);

        // std::cout << "Drawing path point " << idx << " at (" << gx << ", " << gy << ")" << std::endl;
        // Draw one pixel
        draw_pixel(image, gx, gy);
    } 
    
    // Save the modified image back to the directory with the name "output.png"
    fs::path output_path = img_dir / output_filename;
    try {
        image.write(output_path.string());
        std::cout << "Path image saved to: " << output_path << std::endl;
    } catch (const Magick::Exception &e) {
        std::cerr << "Error saving image file: " << e.what() << std::endl;
    }
    
}

void draw_pixel(Magick::Image & img, size_t x, size_t y)
{

    // Prepare to modify image pixels
    try
    {
        // Set the image type to TrueColor DirectClass representation
        img.type(Magick::TrueColorType);
        // Ensure that there is only one reference to underlying image 
        // If this is not done, then image pixels will not be modified
        img.modifyImage();
        // Allocate pixel view
        Magick::Pixels view(img);

        // std::cout << "Drawing path on image." << std::endl;
        // std::cout << "Image size: " << img.columns() * img.rows() << std::endl;
 
        Magick::ColorRGB blue_rgb(0.0, 0.0, 1.0); 
        
        Magick::Quantum qR = static_cast<Magick::Quantum>(QuantumRange * blue_rgb.red());
        Magick::Quantum qG = static_cast<Magick::Quantum>(QuantumRange * blue_rgb.green());
        Magick::Quantum qB = static_cast<Magick::Quantum>(QuantumRange * blue_rgb.blue());

        ssize_t px = static_cast<ssize_t>(x);
        ssize_t py = static_cast<ssize_t>(y);
         
        Magick::Quantum *pixels = view.get(px,py,1,1);
        if (!pixels) {
            std::cerr << "Error: Failed to authenticate pixels." << std::endl;
            return;
        }
        // for (ssize_t row = 0; row < rows; ++row) 
        //     for (ssize_t column = 0; column < columns; ++column) 
        //     {
        //         // pixels->red = MaxRGB * green.redQuantum();
        //         // pixels->green = MaxRGB * green.greenQuantum();
        //         // pixels->blue = MaxRGB * green.blueQuantum();
        //         // ++pixels;
        //         *pixels++ = MagickCore::QuantumRange * green.quantumRed();
        //         *pixels++ = MagickCore::QuantumRange * green.quantumGreen();
        //         *pixels++ = MagickCore::QuantumRange * green.quantumBlue();
        //     }
        // // Save changes to image.
        int channels = img.channels(); // typically 3 (RGB) or 4 (RGBA)
        // Example: set one pixel at location (px, py)
        if (pixels != nullptr) {
            // Write interleaved values for one pixel:
            // Many ImageMagick builds use RGB[A] ordering.
            // We'll write R,G,B and if there's an alpha channel we preserve it by skipping one Quantum
            if (channels == 3) {
                pixels[0] = qR;
                pixels[1] = qG;
                pixels[2] = qB;
            } else if (channels == 4) {
                pixels[0] = qR;
                pixels[1] = qG;
                pixels[2] = qB;
                // keep alpha as fully opaque
                pixels[3] = QuantumRange; // optional: set alpha to max (opaque)
            } else {
                // fallback: write first 3 channels
                pixels[0] = qR;
                pixels[1] = qG;
                pixels[2] = qB;
            }
            view.sync(); // commit region back to image
        }
        
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return;
    }

}
