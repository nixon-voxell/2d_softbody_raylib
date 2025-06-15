#pragma once
#include "raylib.h"

struct Particle {
public:
  Vector2 position;
  Vector2 prev_position;
  float inv_mass;
  Vector2 velocity;

  Particle(const Vector2 position, const float inv_mass);
  void draw(const float radius);
  void apply_gravity(const Vector2 gravity, const float dt);
  void apply_velocity(const float dt);
  void set_velocity(const float dt);
};

struct Edge {
public:
  int p0;
  int p1;
  float distance;
};
