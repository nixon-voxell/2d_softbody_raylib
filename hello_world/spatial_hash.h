#pragma once
#include <vector>
#include "raylib.h"

unsigned int hash_coord(Vector2 i32_coord_vec, unsigned int table_size);
void inclusive_sum_scan_in_place(std::vector<unsigned int>& vec);
Vector2 get_i32_coord(Vector2 coord, float spacing);

class SpatialHash
{
public:
  SpatialHash(unsigned int table_size, int cell_len);

  // Hashes coordinates (Vector2) into cells, directly taking spacing.
  void hash_particles(const std::vector<Particle>& particles, float spacing);

  // Hashes a single coordinate (Vector2, representing grid cell index)
  unsigned int hash_coord(Vector2 i32_coord_vec) const;

  // Get entries within the hash cell.
  // Returns a pointer to the start of the entries for the given hash.
  const unsigned int* get_cell_entries(unsigned int hash) const;

  // Returns the number of entries in the hash cell.
  int get_cell_entries_count(unsigned int hash) const;

private:
  unsigned int table_size;
  /*
  Stores the start index of each hash cell at `cell_bounds[hash]`.
  Also stores the end index of each hash cell at `cell_bounds[hash + 1]`.

  The length of the vector will be `table_size + 1` with
  additional 1 as the guard for the end index of the last cell.
  */
  std::vector<unsigned int> cell_bounds;
  // Stores the index of the hashed entries,
  // arranged in chunks determined by `cell_starts`.
  std::vector<unsigned int> cell_entries;
};