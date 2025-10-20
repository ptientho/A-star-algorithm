find_package(Eigen3 REQUIRED NO_MODULE)
find_package(ImageMagick REQUIRED COMPONENTS Magick++ MagickCore)

# print them for debugging:
message(STATUS "ImageMagick include dirs: ${ImageMagick_INCLUDE_DIRS}")
message(STATUS "ImageMagick libraries: ${ImageMagick_LIBRARIES}")

# Create a library target with its source files
add_library(Map GridMap.cpp)

target_compile_definitions(Map PUBLIC
  MAGICKCORE_HDRI_ENABLE=1
  MAGICKCORE_QUANTUM_DEPTH=16
)

include_directories(${ImageMagick_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
target_link_libraries(Map PUBLIC Eigen3::Eigen ${ImageMagick_LIBRARIES})

target_include_directories(Map
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src
)

install(TARGETS Map DESTINATION lib)

install(FILES GridMap.hpp DESTINATION include)
