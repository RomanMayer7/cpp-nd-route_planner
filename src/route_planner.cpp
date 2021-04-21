#include "route_planner.h"
#include <algorithm>
using std::sort;

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Using FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Storing the nodes we find in the RoutePlanner's start_node and end_node attributes.

    start_node = &model.FindClosestNode(start_x,start_y);
    end_node   = &model.FindClosestNode(end_x,end_y);


}

//Implementing the CalculateHValue method.
// - Using distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);

}

// AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  current_node->FindNeighbors();//populate current_node.neighbors vector with all the neighbors.
  for(auto neighbour:current_node->neighbors)
  {
      neighbour->parent=current_node; //setting the parent
      neighbour->g_value=current_node->g_value + current_node->distance(*neighbour);//Calculate g_value 
      neighbour->h_value=CalculateHValue(neighbour);//Using CalculateHValue below to implement the h-Value calculation.
      neighbour->visited=true;//Setting the node's visited attribute to true
      open_list.push_back(neighbour);
  }
}

//Compare two Nodes by  the sum of the h value and g value.
 bool RoutePlanner::CompareNodes(const RouteModel::Node *a,const RouteModel::Node *b)
{
    const float f1 = a->g_value + a->h_value;
    const float f2 = b->g_value + b->h_value;

    return f1>f2;
}

// Completing the NextNode method to sort the open list and return the next node.

// - Sorting the open_list according to the sum of the h value and g value.
// - Creating a pointer to the node in the list with the lowest sum.
// - Removing that node from the open_list.
// - Returning the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
    if(open_list.size()>0)
    {
      sort(open_list.begin(),open_list.end(),CompareNodes);
      RouteModel::Node * lowestValueNode = open_list.back();
      open_list.pop_back();

      return lowestValueNode;
    }
    else return nullptr;
}

// ConstructFinalPath method to return the final path found from our A* search.

// - This method takes the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, we add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;


    while(!((current_node->x==start_node->x)&&(current_node->y==start_node->y)))
    {
        auto previous_node=current_node->parent;

         path_found.insert(path_found.begin(), *current_node);
         distance+=current_node->distance(*previous_node);
         current_node=current_node->parent;

    }
    path_found.insert(path_found.begin(), *start_node);
    

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


//   A* Search algorithm .
// - Using the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Using the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, we use the ConstructFinalPath method to return the final path that was found.
// - Storing the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    current_node=start_node;
    current_node->visited=true;

     //Add First Node to Open List
      open_list.push_back(current_node);
      while(!((current_node->x==end_node->x)&&(current_node->y==end_node->y)))
    {
        
          AddNeighbors(current_node);
          current_node=NextNode();
    }  

     m_Model.path=ConstructFinalPath(current_node);
     std::cout<<"m_Model.path.size():"<<m_Model.path.size()<<std::endl;


}