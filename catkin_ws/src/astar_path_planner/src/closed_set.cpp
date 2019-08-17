#include "astar_path_planner/closed_set.h"

#include <algorithm>
#include <ros/ros.h>

namespace astar_path_planner
{
size_t ClosedSet::size()
{
  return nodes_.size();
}

void ClosedSet::push(const Node& n)
{
  nodes_.push_back(n);
}

bool ClosedSet::contains(int id)
{
  // Return true if the node is in nodes_
  for (const auto& n : nodes_)
  {
    if (n.id == id)
    {
      return true;
    }
  }

  return false;
}

std::vector<int> ClosedSet::getPath(int start_id, int goal_id)
{
  // Return a path of IDs from the goal to the start
  // Find the goal node in "nodes_", then find its parent
  // Keep finding parents until you get to the start node
  // You should also reverse the path before returning it

  std::vector<int> path{};
  int current_id = goal_id;
  while (current_id != start_id)
  {
    for (auto pathnode : nodes_)
    {
      if (pathnode.id == current_id)
      {
        path.push_back(current_id);
        current_id = pathnode.parent_id;      
      }
    }
  }
  path.push_back(current_id);

  ROS_INFO("Reversing\n");
  std::reverse(path.begin(),path.end());
  return path;
}

const std::vector<Node>& ClosedSet::getNodes()
{
  return nodes_;
}

std::ostream& operator<<(std::ostream& os, const ClosedSet& closed_set)
{
  os << "\n\nClosed set:" << std::endl;

  for (const auto& n : closed_set.nodes_)
  {
    os << n;
  }

  return os;
}

}  // namespace astar_path_planner
