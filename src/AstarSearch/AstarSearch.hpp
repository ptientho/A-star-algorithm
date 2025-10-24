#ifndef ASTARSEARCH_HPP
#define ASTARSEARCH_HPP

#include "Map/GridMap.hpp"
#include <iostream>

class Node
{
public:
    Node(std::shared_ptr<Node> parent, const size_t &current_position_index = 0)
        : parent_(parent), curr_position_idx_(current_position_index) {}

    ~Node() {}

    // get parent
    std::shared_ptr<Node> &get_parent() { return parent_; }

    // get current position
    size_t get_index() { return curr_position_idx_; }

    // get g_cost
    double get_g_cost() { return g_cost; }

    // get h_cost
    double get_h_cost() { return h_cost; }

    // get f_cost
    double get_f_cost() { return f_cost; }

    // set g_cost
    void set_g_cost(double g) { g_cost = g; }

    // set h_cost
    void set_h_cost(double h) { h_cost = h; }

    // set f_cost
    void set_f_cost(double f) { f_cost = f; }

    // set parent
    void set_parent(std::shared_ptr<Node> parent) { parent_ = parent; }

private:
    std::shared_ptr<Node> parent_;
    size_t curr_position_idx_;
    double g_cost = 0.0; // cost from start to current node
    double h_cost = 0.0; // heuristic cost from current to goal
    double f_cost = 0.0; // total cost
};

/*
    Path reconstruction function
    Given a current node, construct path backward to start node
*/
void reconstruct_path(std::vector<size_t> &shortest_path, std::shared_ptr<Node> current_node, size_t start_index, size_t goal_index);

/*
    A* Search algorithm implementation
    Requirements:
    8-connected grid
    Movement cost: 1 for axial, sqrt(2) for diagonal
    Output: collision free path
*/
bool astar_search(const OccupancyGrid &grid,
                  const size_t &start_idx,
                  const size_t &goal_idx, std::vector<size_t> &shortest_path);

/*
    Convert (x,y) coordinate to linear index value
    x, y: coordinate values
    width: width of the grid
    return: linear index, specifying a pixel in 1-D array
*/
size_t gridCellToIndex(const size_t &x, const size_t &y, const size_t &width);

/*
    Convert a linear index to (x,y) coordinate value
    index: linear index, specifying a pixel in 1-D array
    width: width of the grid
    x, y: output coordinate values
*/
void indexToGridCell(const size_t &index, const size_t &width, size_t &x, size_t &y);

/*
    Find neighbor nodes inspecting 8 adjacent nodes
    current_node: current node index value of grid cell
    return: hash table of neighbor node indices and their movement costs
*/
std::unordered_map<size_t, double>
find_neighbors(const std::shared_ptr<Node> &current_node, const OccupancyGrid &grid);

/*
    Calculate heuristic cost using Euclidean distance
    The distance between current position and goal position
*/
double calculate_heuristic(size_t current_index, size_t goal_index, size_t grid_width);

#endif // ASTARSEARCH_HPP