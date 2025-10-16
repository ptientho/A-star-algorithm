#include "AstarSearch.hpp"
#include <iostream>
#include <unordered_set>
#include <vector>

std::vector<std::vector<uint32_t>> reconstruct_path(Node * current_node) {
    
}

bool astar_search(const OccupancyGrid & grid,
    const size_t & start_idx,
    const size_t & goal_idx, std::vector<std::vector<size_t>> & shortest_path) 
{

    // boolean flag to keep track if path is found
    bool path_found = false;

    // Container to store nodes associated with g_cost
    std::vector<std::shared_ptr<Node>> open_list;

    // Container to store visited nodes
    std::unordered_set<size_t> closed_list;

    // Current node with all costs set to 0.0
    std::shared_ptr<Node> current_node = std::make_shared<Node>(nullptr, start_idx);

    // Put start node into open_list
    open_list.push_back(current_node);

    std::cout << "A* Search Initialized" << "\n";

    while (!open_list.empty())
    {
        // Select node with the lowest f_cost as a current node
        std::sort(open_list.begin(), open_list.end(), [](std::shared_ptr<Node> a, std::shared_ptr<Node> b) {
            return a->get_f_cost() < b->get_f_cost();
        });

        current_node = open_list.front();
        open_list.erase(open_list.begin());

        // Add the current node to closed list as a visited node
        closed_list.insert(current_node->get_index());

        // Break the loop when current node = goal node
        if (current_node->get_index() == goal_idx) {
            path_found = true;
            break;
        }

        // Find neighbors of the current node
        std::vector<std::shared_ptr<Node>> neighbors = find_neighbors(current_node, grid);

        // Iterate over each neighbors
        for (const auto& neighbor : neighbors)
        {
            // Get neighbor index and step cost
            size_t neighbor_idx = neighbor->get_index();
            double step_cost = neighbor->get_g_cost();

            // Check if neighbor is in closed list
            auto closed_idx = closed_list.find(neighbor_idx);

            if (closed_idx != closed_list.end())
            {
                // Neighbor is already evaluated, skip to next neighbor
                continue;
            }

            // Update cost. g_cost is already set in find_neighbors function
            // Update only f and h values
            double h_cost = calculate_heuristic(neighbor_idx, goal_idx, grid.width);
            neighbor->set_h_cost(h_cost);

            double f_cost = neighbor->get_g_cost() + neighbor->get_h_cost();
            neighbor->set_f_cost(f_cost);

            // Check if neighbor is in open_list
            auto open_list_idx = std::find_if(open_list.begin(), open_list.end(),
                [neighbor_idx](const std::shared_ptr<Node>& node) {
                    return node->get_index() == neighbor_idx;
                });

            if (open_list_idx != open_list.end())
            {
                // Update existing neighbor in open_list if new f_cost is lower
            } else {
                // Add new neighbor to open list
                open_list.push_back(neighbor);
            }
        }


    }

    // A* doen traversing nodes in open_list
    // Reconstruct path if found
    if (path_found) {

    } else {
        std::cout << "No path found from start to goal." << std::endl;
    }

    return path_found;
}

size_t gridCellToIndex(const size_t & x, const size_t & y, const size_t & width)
{

}

void indexToGridCell(const size_t & index, const size_t & width, size_t & x, size_t & y)
{

}

std::vector<std::shared_ptr<Node>>
find_neighbors(const std::shared_ptr<Node>& current_node, const OccupancyGrid & grid)
{
    // Given current node and map, find its 8-connected neighbors
    std::vector<std::shared_ptr<Node>> neighbors;

    double diag_cost = 1.41421;
    double step_cost = 1.0;
    int8_t obstacle = 100;
    int8_t unknown = -1;

    // Get index value above the current node
    size_t upper = current_node->get_index() - grid.width;

    // Get index value to the left of the current node
    size_t left = current_node->get_index() - 1;

    // Get index value to the upper left of the current node
    size_t upper_left = current_node->get_index() - grid.width - 1;

    // Get index value to the upper right of the current node
    size_t upper_right = current_node->get_index() - grid.width + 1;

    // Get index value to the right of the current node
    size_t right = current_node->get_index() + 1;

    // Get index value to the lower left of the current node
    size_t lower_left = current_node->get_index() + grid.width - 1;

    // Get index value to below the current node
    size_t lower = current_node->get_index() + grid.width;

    // Get index value to the lower right of the current node
    size_t lower_right = current_node->get_index() + grid.width + 1;

    // Check if each neighbor is within bounds and not an obstacle or unknown
    if (upper >= 0)
    {
        if (grid.data[upper] != obstacle && grid.data[upper] != unknown)
        {
            neighbors.push_back(std::make_shared<Node>(current_node, upper));
            neighbors.back()->set_g_cost(current_node->get_g_cost() + step_cost);
        }
    }

    // Exclude left neighbor that are outside left boundary
    // Exclude left neighbor that are on the right most column
    if (left % static_cast<size_t>(grid.width) > 0 && left % static_cast<size_t>(grid.width) != static_cast<size_t>(grid.width) - 1)
    {
        if (grid.data[left] != obstacle && grid.data[left] != unknown)
        {
            neighbors.push_back(std::make_shared<Node>(current_node, left));
            neighbors.back()->set_g_cost(current_node->get_g_cost() + step_cost);
        }
    }

    // Exclude cell outside the map
    // Exclude upper left neighbor that are on the right most column
    if (upper_left >= 0 && left % static_cast<size_t>(grid.width) != static_cast<size_t>(grid.width) - 1)
    {
        if (grid.data[upper_left] != obstacle && grid.data[upper_left] != unknown)
        {
            neighbors.push_back(std::make_shared<Node>(current_node, upper_left));
            neighbors.back()->set_g_cost(current_node->get_g_cost() + diag_cost);
        }
    }

    // Exclude cell outside the map
    // Exclude upper right neighbor that are on the left most column
    if (upper_right >= 0 && upper_right % static_cast<size_t>(grid.width) != 0)
    {
        if (grid.data[upper_right] != obstacle && grid.data[upper_right] != unknown)
        {
            neighbors.push_back(std::make_shared<Node>(current_node, upper_right));
            neighbors.back()->set_g_cost(current_node->get_g_cost() + diag_cost);
        }
    }

    // Exclude right neightbor in the first column
    // Exclude it if it is outside the map
    if (right % static_cast<size_t>(grid.width) != 0 && right >= static_cast<size_t>(grid.width * grid.height))
    {
        if (grid.data[right] != obstacle && grid.data[right] != unknown)
        {
            neighbors.push_back(std::make_shared<Node>(current_node, right));
            neighbors.back()->set_g_cost(current_node->get_g_cost() + step_cost);
        }
    }

    // Exclude lower left neighbor if it exceeds the max costmap size
    // Exclude lower left neighbor that are on the right most column
    if (lower_left < static_cast<size_t>(grid.width * grid.height) && lower_left % static_cast<size_t>(grid.width) != static_cast<size_t>(grid.width) - 1)
    {
        if (grid.data[lower_left] != obstacle && grid.data[lower_left] != unknown)
        {
            neighbors.push_back(std::make_shared<Node>(current_node, lower_left));
            neighbors.back()->set_g_cost(current_node->get_g_cost() + diag_cost);
        }
    }

    // Exclude lower neighbor if it exceeds the max costmap size
    if (lower < static_cast<size_t>(grid.width * grid.height))
    {
        if (grid.data[lower] != obstacle && grid.data[lower] != unknown)
        {
            neighbors.push_back(std::make_shared<Node>(current_node, lower));
            neighbors.back()->set_g_cost(current_node->get_g_cost() + step_cost);
        }
    }

    // Exclude the lower right neighbor if it exceeds the max costmap size
    // Exclude lower right neighbor that are on the left most column
    if (lower_right < static_cast<size_t>(grid.width * grid.height) && lower_right % static_cast<size_t>(grid.width) != 0)
    {
        if (grid.data[lower_right] != obstacle && grid.data[lower_right] != unknown)
        {
            neighbors.push_back(std::make_shared<Node>(current_node, lower_right));
            neighbors.back()->set_g_cost(current_node->get_g_cost() + diag_cost);
        }
    }


    return neighbors;
}

