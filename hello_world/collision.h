#pragma once

#include <vector>

class SelfCollisionCache
{
public:
  /// Maximum number of collisions allowed per particle.
  int max_collision_per_vertex;
  std::vector<int> indices;
  std::vector<int> count_scan;

  SelfCollisionCache(int max_collisions, int num_vertices)
    : max_collision_per_vertex(max_collisions),
    indices(num_vertices* max_collisions), 
    count_scan(num_vertices + 1)
  {
  }
};