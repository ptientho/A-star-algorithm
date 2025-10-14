# Create a library target with its source files
add_library(AstarSearch a.cpp b.cpp)

install(TARGETS AstarSearch DESTINATION lib)

install(FILES AstarSearch.hpp DESTINATION include)