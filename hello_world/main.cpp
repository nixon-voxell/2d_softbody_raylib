#include "raylib.h"
#include "raymath.h"
#include <vector>
#include <array>
#include <iostream>
#include <random>
#include <chrono>
#include "particle.h"
#include "spatial_hash.h"

static std::mt19937_64 rng(std::chrono::system_clock::now().time_since_epoch().count());
static std::uniform_real_distribution<float> dist_angle(0.0f, 360.0f);

//------------------------------------------------------------------------------------
// Program main entry point
//------------------------------------------------------------------------------------
int main(void)
{
  // Initialization
  //--------------------------------------------------------------------------------------
  const int screenWidth = 1280;
  const int screenHeight = 720;

  InitWindow(screenWidth, screenHeight, "raylib [core] example - 2d camera");

  Camera2D camera = { 0 };
  camera.target = Vector2 { 0.0f, 0.0f };
  camera.offset = Vector2 { screenWidth / 2.0f, screenHeight / 2.0f };
  camera.rotation = 0.0f;
  camera.zoom = 1.0f;

  SetTargetFPS(60);
  //--------------------------------------------------------------------------------------
  const float gravity_magnitude = 981.0f;
  Vector2 gravity = Vector2 { 0.0, gravity_magnitude };

  // Create cloth particles.
  const int count = 20;
  const int particle_count = count * count;
  const float particle_dist = 20.0f;
  const float particle_dist_diag = sqrtf(particle_dist * particle_dist * 2);
  const float particle_radius = 10.0f;
  const float particle_diameter = particle_radius * 2.0f;
  const int substep_count = 6;
  const float stiffness = 0.1;

  std::vector<Particle> particles;
  std::vector <Edge> edges;
  SpatialHash spatial_hash = SpatialHash(5 * particle_count, particle_count);

  Vector2 offset = Vector2{ -200.0f, -250.0f };

  for (int y = 0; y < count; y++) {
    for (int x = 0; x < count; x++) {
      // Spawn particle.
      Particle particle = Particle(Vector2{ (float)x, (float)y } * particle_dist + offset, 1.0f);
      particles.push_back(particle);
    }
  }
  particles[count / 2].inv_mass = 0.0f;
  //particles[count - 4].inv_mass = 0.0f;

  // Horizontal + Vertical.
  for (int y = 0; y < count - 1; y++) {
    for (int x = 0; x < count - 1; x++) {
        int start = x + (count * y);
      edges.push_back(Edge { start, start + 1, particle_dist });
      edges.push_back(Edge { start, x + count * (y + 1), particle_dist });
    }
  }

  // Last row.
  for (int x = 0; x < count - 1; x++) {
    int start = x + (count * (count - 1));
    edges.push_back(Edge { start, start + 1, particle_dist });
  }

  // Last column.
  for (int y = 0; y < count - 1; y++) {
    int up = (y * count) + (count - 1);
    int bottom = ((y + 1) * count) + (count - 1);
    edges.push_back(Edge { up, bottom, particle_dist });
  }

  //// Diagonal, top left -> bottom right.
  //for (int y = 0; y < count - 1; y++) {
  //  for (int x = 0; x < count - 1; x++) {
  //    edges.push_back({ x + (count * y), (x + 1) + (count * (y + 1)), particle_dist_diag });
  //  }
  //}

  //// Diagonal, top right -> bottom left.
  //for (int y = 0; y < count - 1; y++) {
  //  for (int x = 0; x < count - 1; x++) {
  //    edges.push_back({ (x + 1) + (count * y), x + (count * (y + 1)), particle_dist_diag });
  //  }
  //}

  // Main game loop
  while (!WindowShouldClose())
  {
    // Update
    //----------------------------------------------------------------------------------
    float delta_time = GetFrameTime();
    float sub_delta_time = delta_time / (float)substep_count;

    camera.zoom += ((float)GetMouseWheelMove() * 0.05f);

    if (camera.zoom > 3.0f) camera.zoom = 3.0f;
    else if (camera.zoom < 0.25f) camera.zoom = 0.25f;

    if (IsKeyPressed(KEY_R))
    {
      camera.zoom = 1.0f;
    }

    if (IsKeyPressed(KEY_SPACE)) {
      // Generate a random angle in radians
      float random_angle_deg = dist_angle(rng);
      float random_angle_rad = random_angle_deg * DEG2RAD;

      // Calculate new gravity components
      gravity.x = gravity_magnitude * cosf(random_angle_rad);

      TraceLog(LOG_INFO, "GRAVITY CHANGED: X=%.2f, Y=%.2f", gravity.x, gravity.y);
    }

    if (sub_delta_time > 0.0001f) {
      spatial_hash.hash_particles(particles, particle_diameter);
      for (int i = 0; i < substep_count; i++) {
        // Gravity.
        for (int p = 0; p < particles.size(); p++) {
          Particle* particle = &particles[p];
          particle->apply_gravity(gravity, sub_delta_time);
          particle->apply_velocity(sub_delta_time);
        }

        // Solve distance constraint.
        for (int e = 0; e < edges.size(); e++) {
          auto edge = edges[e];
          Particle* p0 = &particles[edge.p0];
          Particle* p1 = &particles[edge.p1];

          float w_sum = p0->inv_mass + p1->inv_mass;
          if (w_sum <= 0.0001f) { continue; }

          Vector2 diff = p0->position - p1->position;
          float dist = Vector2Length(diff);
        
          if (dist < 0.0001f) { continue; }
          Vector2 n = diff / dist;
          float corr = dist - edge.distance;
          p0->position -= n * corr * p0->inv_mass / w_sum * stiffness;
          p1->position += n * corr * p1->inv_mass / w_sum * stiffness;
        }

        // Solve collision constraint.
        std::vector<std::pair<int, int>> collision_pairs_this_substep;
        const int grid_offsets[9][2] = {
            {-1, -1}, {0, -1}, {1, -1},
            {-1,  0}, {0,  0}, {1,  0},
            {-1,  1}, {0,  1}, {1,  1}
        };

        for (int p_idx = 0; p_idx < particles.size(); p_idx++) {
          Particle* particle = &particles[p_idx];
          Vector2 p_i32_coord = get_i32_coord(particle->position, particle_diameter);

          for (int cell_offset = 0; cell_offset < 9; cell_offset++) {
            Vector2 neighbor_i32_coord = {
                p_i32_coord.x + grid_offsets[cell_offset][0],
                p_i32_coord.y + grid_offsets[cell_offset][1]
            };

            unsigned int neighbor_hash = spatial_hash.hash_coord(neighbor_i32_coord);

            const unsigned int* cell_entries = spatial_hash.get_cell_entries(neighbor_hash);
            size_t num_cell_entries = spatial_hash.get_cell_entries_count(neighbor_hash);

            for (int entry_idx = 0; entry_idx < num_cell_entries; ++entry_idx) {
              int q_idx = cell_entries[entry_idx];

              if (p_idx >= q_idx) {
                continue;
              }

              Particle* q = &particles[q_idx];

              Vector2 diff_pos = Vector2Subtract(particle->position, q->position);
              float dist_sq = Vector2LengthSqr(diff_pos);
              float min_dist_sq = (particle_diameter) * (particle_diameter);

              if (dist_sq < min_dist_sq) {
                collision_pairs_this_substep.push_back({ p_idx, q_idx });
              }
            }
          }
        }

        for (const auto& pair : collision_pairs_this_substep) {
          Particle* p0 = &particles[pair.first];
          Particle* p1 = &particles[pair.second];

          float w_sum = p0->inv_mass + p1->inv_mass;
          if (w_sum <= 0.0001f) { continue; }

          Vector2 diff = p0->position - p1->position;
          float dist = Vector2Length(diff);

          if (dist > particle_diameter) { continue; }
          Vector2 n = diff / dist;
          float corr = dist - particle_diameter;
          p0->position -= n * corr * p0->inv_mass / w_sum;
          p1->position += n * corr * p1->inv_mass / w_sum;
        }

        // Set velocity.
        for (int p = 0; p < particles.size(); p++) {
          Particle* particle = &particles[p];
          particle->set_velocity(sub_delta_time);
        }
      }
    }


    // Draw
    //----------------------------------------------------------------------------------
    BeginDrawing();

    ClearBackground(LIGHTGRAY);

    BeginMode2D(camera);

    for (int e = 0; e < edges.size(); e++) {
      auto edge = edges[e];
      DrawLineV(particles[edge.p0].position, particles[edge.p1].position, BLACK);
    }

    for (int p = 0; p < particles.size(); p++) {
      Particle particle = particles[p];
      particle.draw(particle_radius);
    }

    EndMode2D();

    EndDrawing();
    //----------------------------------------------------------------------------------
  }

  // De-Initialization
  //--------------------------------------------------------------------------------------
  CloseWindow();        // Close window and OpenGL context
  //--------------------------------------------------------------------------------------

  return 0;
}
