#include "particle.h"
#include "raymath.h"
#include <iostream>

Particle::Particle(const Vector2 position, const float inv_mass)
  : position(position)
  , prev_position(position)
  , inv_mass(inv_mass)
  , velocity(Vector2Zero())
{
}

void Particle::draw(const float radius) {
  DrawCircleV(this->position, radius, RED);
}

void Particle::apply_gravity(const Vector2 gravity, const float dt) {
  if (this->inv_mass < 0.0001) { return; }
  this->velocity += gravity * dt;
}

void Particle::apply_velocity(const float dt) {
  if (this->inv_mass < 0.0001) { return; }
  this->position += this->velocity * dt;
}

void Particle::set_velocity(const float dt) {
  Vector2 diff = this->position - this->prev_position;
  this->velocity = diff / dt;
  this->prev_position = this->position;
}
