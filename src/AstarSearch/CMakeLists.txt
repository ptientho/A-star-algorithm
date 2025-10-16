# Create a library target with its source files
add_library(AstarSearch AstarSearch.cpp)

target_link_libraries(AstarSearch PRIVATE Map)

target_include_directories(AstarSearch
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src
)

install(TARGETS AstarSearch DESTINATION lib)

install(FILES AstarSearch.hpp DESTINATION include)