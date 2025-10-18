#include "AstarSearch.hpp"
#include <iostream>
#include <unordered_set>
#include <vector>

void reconstruct_path(std::vector<size_t> & shortest_path, std::shared_ptr<Node> current_node, size_t start_index, size_t goal_index)
{
    
    if (current_node->get_index() != goal_index) {
        std::cerr << "Error: Current node is not the goal node." << std::endl;
        return;
    }

    std::shared_ptr<Node> node = current_node;
    
    // validate that node is goal_index
    if (node->get_index() != goal_index) {
        std::cerr << "Error: Node index does not match goal index." << std::endl;
        return;
    }

    while (node)
    {
       shortest_path.push_back(node->get_index());
       if (node->get_index() == start_index) break;
       node = node->get_parent();
    }
    

    std::reverse(shortest_path.begin(), shortest_path.end());
    
}

bool astar_search(const OccupancyGrid & grid,
    const size_t & start_idx,
    const size_t & goal_idx, std::vector<size_t> & shortest_path) 
{

    size_t total = static_cast<size_t>(grid.width * grid.height);
    if (start_idx >= total || goal_idx >= total) {
        std::cerr << "Start/goal index out of range\n";
        return false;
    }

    if (grid.data[start_idx] == 100 || grid.data[start_idx] == -1) {
        std::cerr << "Start cell is not free\n"; 
        return false;
    }
    if (grid.data[goal_idx] == 100 || grid.data[goal_idx] == -1) {
        std::cerr << "Goal cell is not free\n"; 
        return false;
    }

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
        std::unordered_map<size_t, double> neighbors = find_neighbors(current_node, grid);

        // Iterate over each neighbors
        for (const auto& neighbor : neighbors)
        {
            // Get neighbor index and step cost
            size_t neighbor_idx = neighbor.first;
            double step_cost = neighbor.second;

            // Check if neighbor is in closed list
            auto closed_idx = closed_list.find(neighbor_idx);

            if (closed_idx != closed_list.end())
            {
                // Neighbor is already evaluated, skip to next neighbor
                continue;
            }

            // Update costs
            // Update only f and h values
            double g_cost = current_node->get_g_cost() + step_cost;
            double h_cost = calculate_heuristic(neighbor_idx, goal_idx, grid.width);

            double f_cost = g_cost + h_cost;

            // Check if neighbor is in open_list
            auto open_list_idx = std::find_if(open_list.begin(), open_list.end(),
                [neighbor_idx](const std::shared_ptr<Node>& node) {
                    return node->get_index() == neighbor_idx;
                });

            if (open_list_idx != open_list.end())
            {
                // Update existing neighbor in open_list if new f_cost is lower
                if (f_cost < (*open_list_idx)->get_f_cost())
                {   
                    
                    (*open_list_idx)->set_g_cost(g_cost);
                    (*open_list_idx)->set_h_cost(h_cost);
                    (*open_list_idx)->set_f_cost(f_cost);
                    (*open_list_idx)->set_parent(current_node);
                } else if (f_cost == (*open_list_idx)->get_f_cost())
                {
                    // Tie breaking: if f_cost is the same, prefer the one with lower h_cost
                    if (h_cost < (*open_list_idx)->get_h_cost())
                    {
                        (*open_list_idx)->set_g_cost(g_cost);
                        (*open_list_idx)->set_h_cost(h_cost);
                        (*open_list_idx)->set_f_cost(f_cost);
                        (*open_list_idx)->set_parent(current_node);
                    }
                }
            } else {
                // Add new neighbor to open list
                open_list.push_back(std::make_shared<Node>(current_node, neighbor_idx));
                open_list.back()->set_g_cost(g_cost);
                open_list.back()->set_h_cost(h_cost);
                open_list.back()->set_f_cost(f_cost);
            }
        }


    }

    // A* doen traversing nodes in open_list
    // Reconstruct path if found
    if (path_found) {
        reconstruct_path(shortest_path, current_node, start_idx, goal_idx);
    } else {
        std::cout << "No path found from start to goal." << std::endl;
    }

    return path_found;
}

size_t gridCellToIndex(const size_t & x, const size_t & y, const size_t & width)
{
    return y * width + x;
}

void indexToGridCell(const size_t & index, const size_t & width, size_t & x, size_t & y)
{
    x = index % width;
    y = index / width;
}

std::unordered_map<size_t, double>
find_neighbors(const std::shared_ptr<Node>& current_node, const OccupancyGrid & grid)
{
    // Given current node and map, find its 8-connected neighbors
    std::unordered_map<size_t, double> neighbors;

    const double diag_cost = 1.41421;
    const double step_cost = 1.0;
    const int8_t OCC = 100;
    const int8_t UNKNOWN = -1;

    size_t curr_idx = current_node->get_index();
    size_t gx, gy;
    size_t width = static_cast<size_t>(grid.width);
    size_t height = static_cast<size_t>(grid.height);
    indexToGridCell(curr_idx, width, gx, gy);
    
    // 8-connected offsets
    constexpr int dx[8] = {-1,  0, +1, -1, +1, -1,  0, +1};
    constexpr int dy[8] = {-1, -1, -1,  0,  0, +1, +1, +1};

    for (int k = 0; k < 8; ++k) 
    {
        int64_t nx = static_cast<int64_t>(gx) + dx[k];
        int64_t ny = static_cast<int64_t>(gy) + dy[k];

        // bounds check
        if (nx < 0 || ny < 0) continue;
        if (static_cast<size_t>(nx) >= width || static_cast<size_t>(ny) >= height) continue;

        size_t nindex = gridCellToIndex(static_cast<size_t>(nx), static_cast<size_t>(ny), width);
        
        // skip obstacle or unknown
        if (grid.data[nindex] == OCC || grid.data[nindex] == UNKNOWN) continue;

        double cost = (dx[k] != 0 && dy[k] != 0) ? diag_cost : step_cost;
        neighbors.emplace(nindex, cost);
    }

    return neighbors;

    // // Get index value above the current node
    // size_t upper = current_node->get_index() - grid.width;

    // // Get index value to the left of the current node
    // size_t left = current_node->get_index() - 1;

    // // Get index value to the upper left of the current node
    // size_t upper_left = current_node->get_index() - grid.width - 1;

    // // Get index value to the upper right of the current node
    // size_t upper_right = current_node->get_index() - grid.width + 1;

    // // Get index value to the right of the current node
    // size_t right = current_node->get_index() + 1;

    // // Get index value to the lower left of the current node
    // size_t lower_left = current_node->get_index() + grid.width - 1;

    // // Get index value to below the current node
    // size_t lower = current_node->get_index() + grid.width;

    // // Get index value to the lower right of the current node
    // size_t lower_right = current_node->get_index() + grid.width + 1;

    // // Check if each neighbor is within bounds and not an obstacle or unknown
    // if (upper >= 0)
    // {
    //     if (grid.data[upper] != obstacle && grid.data[upper] != unknown)
    //     {
    //         neighbors.insert({upper, step_cost});
    //     }
    // }

    // // Exclude left neighbor that are outside left boundary
    // // Exclude left neighbor that are on the right most column
    // if (left % static_cast<size_t>(grid.width) > 0 && left % static_cast<size_t>(grid.width) != static_cast<size_t>(grid.width) - 1)
    // {
    //     if (grid.data[left] != obstacle && grid.data[left] != unknown)
    //     {
    //         neighbors.insert({left, step_cost});
    //     }
    // }

    // // Exclude cell outside the map
    // // Exclude upper left neighbor that are on the right most column
    // if (upper_left >= 0 && left % static_cast<size_t>(grid.width) != static_cast<size_t>(grid.width) - 1)
    // {
    //     if (grid.data[upper_left] != obstacle && grid.data[upper_left] != unknown)
    //     {
    //         neighbors.insert({upper_left, diag_cost});
    //     }
    // }

    // // Exclude cell outside the map
    // // Exclude upper right neighbor that are on the left most column
    // if (upper_right >= 0 && upper_right % static_cast<size_t>(grid.width) != 0)
    // {
    //     if (grid.data[upper_right] != obstacle && grid.data[upper_right] != unknown)
    //     {
    //         neighbors.insert({upper_right, diag_cost});
    //     }
    // }

    // // Exclude right neightbor in the first column
    // // Exclude it if it is outside the map
    // if (right % static_cast<size_t>(grid.width) != 0 && right >= static_cast<size_t>(grid.width * grid.height))
    // {
    //     if (grid.data[right] != obstacle && grid.data[right] != unknown)
    //     {
    //         neighbors.insert({right, step_cost});
    //     }
    // }

    // // Exclude lower left neighbor if it exceeds the max costmap size
    // // Exclude lower left neighbor that are on the right most column
    // if (lower_left < static_cast<size_t>(grid.width * grid.height) && lower_left % static_cast<size_t>(grid.width) != static_cast<size_t>(grid.width) - 1)
    // {
    //     if (grid.data[lower_left] != obstacle && grid.data[lower_left] != unknown)
    //     {
    //         neighbors.insert({lower_left, diag_cost});
    //     }
    // }

    // // Exclude lower neighbor if it exceeds the max costmap size
    // if (lower < static_cast<size_t>(grid.width * grid.height))
    // {
    //     if (grid.data[lower] != obstacle && grid.data[lower] != unknown)
    //     {
    //         neighbors.insert({lower, step_cost});
    //     }
    // }

    // // Exclude the lower right neighbor if it exceeds the max costmap size
    // // Exclude lower right neighbor that are on the left most column
    // if (lower_right < static_cast<size_t>(grid.width * grid.height) && lower_right % static_cast<size_t>(grid.width) != 0)
    // {
    //     if (grid.data[lower_right] != obstacle && grid.data[lower_right] != unknown)
    //     {
    //         neighbors.insert({lower_right, diag_cost});
    //     }
    // }


    // return neighbors;

}

double calculate_heuristic(size_t current_index, size_t goal_index, size_t grid_width)
{
    size_t current_x;
    size_t current_y;
    size_t goal_x;
    size_t goal_y;

    indexToGridCell(current_index, grid_width, current_x, current_y);
    indexToGridCell(goal_index, grid_width, goal_x, goal_y);

    double heuristic = std::sqrt(std::pow(static_cast<double>(goal_x) - static_cast<double>(current_x), 2) +
        std::pow(static_cast<double>(goal_y) - static_cast<double>(current_y), 2));

    return heuristic;
}

