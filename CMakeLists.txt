cmake_minimum_required(VERSION 3.7)
project(A-star-algorithm)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)


add_executable(astar_search src/main.cpp)

# Add subdirectories for libraries
add_subdirectory(src/AstarSearch)
add_subdirectory(src/Map)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)

# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Include headers
target_include_directories(astar_search PUBLIC
                           "${PROJECT_BINARY_DIR}"
                           "${PROJECT_SOURCE_DIR}/src/AstarSearch"
                           "${PROJECT_SOURCE_DIR}/src/Map"
                            )

# Link library to the executable
target_link_libraries(astar_search PUBLIC AstarSearch Map)

install(TARGETS astar_search DESTINATION bin)

enable_testing()

add_executable(
    map_test tests/map_test.cc 
)
target_link_libraries(
    map_test
    GTest::gtest_main
    Map
)
add_executable(
    astar_test tests/astar_test.cc 
)
target_link_libraries(
    astar_test
    GTest::gtest_main
    AstarSearch
    Map
)
target_compile_definitions(map_test PRIVATE
    PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)
target_compile_definitions(astar_test PRIVATE
    PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)

include(GoogleTest)
gtest_discover_tests(map_test)
gtest_discover_tests(astar_test)
