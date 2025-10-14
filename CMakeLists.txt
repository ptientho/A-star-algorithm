cmake_minimum_required(VERSION 3.7)
project(A-star-algorithm)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_executable(astar_search src/main.cpp)

# Add subdirectories for libraries
add_subdirectory(src/AstarSearch)
add_subdirectory(src/Map)

# Include headers
target_include_directories(astar_search PUBLIC
                           "${PROJECT_BINARY_DIR}"
                           "${PROJECT_SOURCE_DIR}/src/AstarSearch"
                           "${PROJECT_SOURCE_DIR}/src/Map"
                            )

# Link library to the executable
target_link_libraries(astar_search PUBLIC AstarSearch Map)

install(TARGETS astar_search DESTINATION bin)