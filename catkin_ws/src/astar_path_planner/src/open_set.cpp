#include "astar_path_planner/open_set.h"

#include <ros/ros.h>

namespace astar_path_planner
{
void OpenSet::push(const Node& n)
{
  nodes_.push_back(n);
}
void waitForKey()
{
  ROS_INFO("Paused, press enter to continue...");
  std::cin.get();
}
Node OpenSet::pop(double heuristic_cost_weight)
{
  int index = 0;

  // Find the best node in "nodes_"
  // Save the index into "index" and it will be removed from "nodes_" and returned
  // You need to compare combined costs: the cost of the node + (heuristic cost * weight)
  // Use "heuristic_cost_weight" to calculate the combined cost so it can be modified later
  
  // Initialise current_lowest weight with useful value, should prevent errors
  double current_lowest = nodes_[index].cost + (nodes_[index].heuristic_cost * heuristic_cost_weight);

  // Iterate through nodes and calculate total weight
  for (int i = 1; i < nodes_.size(); i++)
  {
    double new_low = nodes_[i].cost + (nodes_[i].heuristic_cost * heuristic_cost_weight);

    // If weight is less than previous lowest, update index of lowest node and new lowest cost
    if (new_low < current_lowest)
    {
      current_lowest = new_low;
      index = i;
    }
  }

  // YOU DON'T NEED TO MODIFY ANYTHING AFTER THIS LINE

  // Copy the node
  Node n = nodes_[index];

  // Overwrite the node with the node at the end of the vector
  nodes_[index] = nodes_.back();

  // Delete the node at the end of the vector
  nodes_.pop_back();

  // Return the copy
  return n;
}

bool OpenSet::contains(int id)
{
  // Returns true if the node is in nodes_
  for (const auto& n : nodes_)
  {
    if (n.id == id)
    {
      return true;
    }
  }

  return false;
}

void OpenSet::update(const Node& n)
{
  // Find node "n" in "nodes_"
  // If the cost of node "n" is less than the cost of the node already in the open set, replace it

  // Iterate through nodes to find specified node
  for (auto& oldnode : nodes_)
  {
    // If node is found and cost is lower, update the new cost
    // I.e. if node is now easier to get to, reflect that change in cost value
    if (n.id == oldnode.id)
    {
      if (n.cost <= oldnode.cost)
      {
        // oldnode.cost = n.cost;
        oldnode = n;
      }
    }
  }

}

bool OpenSet::empty()
{
  return nodes_.empty();
}

const std::vector<Node>& OpenSet::getNodes()
{
  return nodes_;
}

std::ostream& operator<<(std::ostream& os, const OpenSet& open_set)
{
  os << "\n\nOpen set:" << std::endl;

  for (const auto& n : open_set.nodes_)
  {
    os << n;
  }

  return os;
}

}  // namespace astar_path_planner
