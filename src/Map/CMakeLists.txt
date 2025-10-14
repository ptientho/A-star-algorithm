# Create a library target with its source files
add_library(Map c.cpp)

install(TARGETS Map DESTINATION lib)

install(FILES Map.hpp DESTINATION include)