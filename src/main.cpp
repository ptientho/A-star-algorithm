#include <iostream>
#include <ranges>
#include <vector>
#include "AstarSearch.hpp"

int main() {
    std::vector<int> numbers = {1, 2, 3, 4, 5};
    
    // Use C++20 ranges to filter and transform the numbers
    auto filtered = numbers | std::ranges::views::filter([](int n) { return n % 2 == 0; })
                             | std::ranges::views::transform([](int n) { return n * n; });

    std::cout << "Processed numbers: ";
    for (int n : filtered) {
        std::cout << n << " ";
    }
    std::cout << std::endl;

    return 0;
}