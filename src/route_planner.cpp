#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


// Calculate h-value for a given node 
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


// AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (auto n : current_node->neighbors)
    {
        n->parent = current_node;
        n->h_value = CalculateHValue(n);
        n->g_value = current_node->g_value + current_node->distance(*n);
        open_list.push_back(n);
        n->visited = true;

    }
}


// NextNode method to sort the open list and return the next node.
RouteModel::Node *RoutePlanner::NextNode() {
    // Sort the open_list according to the sum of the h valueand g value.
    std::sort(open_list.begin(), open_list.end(), [](const auto &node_1, const auto &node_2)
        {
            return (node_1->h_value + node_1->g_value) < (node_2->h_value + node_2->g_value);
        });

    RouteModel::Node *lowest_node = open_list.front();
    // Remove the node with lowest sum of h and g from the open_list
    open_list.erase(open_list.begin());
    return lowest_node;
}


// ConstructFinalPath method to return the final path found from A* search.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    RouteModel::Node parent;

    while (current_node->parent != nullptr) {

        path_found.push_back(*current_node);
        parent = *(current_node->parent);
        distance += current_node->distance(parent);
        current_node = current_node->parent;
    }
    path_found.push_back(*current_node);

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    std::reverse(path_found.begin(), path_found.end());
    return path_found;

}


// A* Search algorithm.
void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    
    start_node->visited = true;
    open_list.push_back(start_node);
    
    while (open_list.size() > 0) {
        current_node = NextNode();
        if (current_node->distance(*end_node) == 0) {
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        AddNeighbors(current_node);
    }
}