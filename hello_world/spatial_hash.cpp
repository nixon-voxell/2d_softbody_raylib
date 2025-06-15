#include "particle.h"
#include "spatial_hash.h"
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <numeric>
#include <cassert>
#include <iostream>

Vector2 get_i32_coord(Vector2 coord, float spacing) {
  return {
      floorf(coord.x / spacing),
      floorf(coord.y / spacing)
  };
}

unsigned int hash_coord(Vector2 floored_coord, unsigned int table_size) {
  unsigned int x = static_cast<unsigned int>(floored_coord.x);
  unsigned int y = static_cast<unsigned int>(floored_coord.y);

  unsigned int hash = (x * 73856093) ^ (y * 19349663);
  return hash % table_size;
}

void inclusive_sum_scan_in_place(std::vector<unsigned int>& vec) {
  if (vec.empty()) {
    return;
  }
  std::partial_sum(vec.begin(), vec.end(), vec.begin());
}

SpatialHash::SpatialHash(unsigned int table_size, int cell_len)
  : table_size(table_size)
  , cell_bounds(table_size + 1)
  , cell_entries(cell_len)
{
}

void SpatialHash::hash_particles(const std::vector<Particle>& particles, float spacing)
{
  assert(spacing > FLT_EPSILON && "Spacing for spatial hash must be > FLT_EPSILON");
  assert(particles.size() == this->cell_entries.size() && "Coords and cell_entries must have the same size");

  std::fill(this->cell_bounds.begin(), this->cell_bounds.end(), 0);

  for (const Particle& particle : particles) {
    Vector2 i32_coord = get_i32_coord(particle.position, spacing);
    unsigned int hash = this->hash_coord(i32_coord);
    this->cell_bounds[hash] += 1;
  }

  inclusive_sum_scan_in_place(cell_bounds);

  for (int i = 0; i < particles.size(); ++i) {
    Vector2 i32_coord_vec = get_i32_coord(particles[i].position, spacing);
    unsigned int hash = this->hash_coord(i32_coord_vec);

    this->cell_bounds[hash] -= 1;
    this->cell_entries[cell_bounds[hash]] = static_cast<unsigned int>(i);
  }
}

unsigned int SpatialHash::hash_coord(Vector2 i32_coord_vec) const
{
  return ::hash_coord(i32_coord_vec, this->table_size);
}

const unsigned int* SpatialHash::get_cell_entries(unsigned int hash) const
{
  assert(hash < this->table_size && "Hash value out of bounds for get_cell_entries");
  int start_index = this->cell_bounds[hash] % this->cell_entries.size();
  return &this->cell_entries[start_index];
}

int SpatialHash::get_cell_entries_count(unsigned int hash) const
{
  assert(hash < this->table_size && "Hash value out of bounds for get_cell_entries_count");
  int start_index = this->cell_bounds[hash];
  int end_index = this->cell_bounds[1 + hash];
  return end_index - start_index;
}