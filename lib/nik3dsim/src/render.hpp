#ifndef RENDERER_HPP
#define RENDERER_HPP

#include <SDL2/SDL.h>
#include "forward.hpp" // For RigidBody definitions

namespace nik3dsim {

struct Camera {
    float position[3];
    float target[3];
    float up[3];
    float fov;
    float aspectRatio;
    float nearPlane;
    float farPlane;
};

struct Renderer {
    SDL_Window* window;
    SDL_Renderer* sdl_renderer;
    int screenWidth;
    int screenHeight;
    Camera camera;
    float viewMatrix[16];
    float projectionMatrix[16];
};

// Initialization and cleanup
bool renderer_init(Renderer* renderer, int width, int height);
void renderer_cleanup(Renderer* renderer);

// Main rendering functions
void renderer_clear(Renderer* renderer);
void renderer_present(Renderer* renderer);

// Camera control
void renderer_set_camera(Renderer* renderer, Camera camera);

// Coordinate transformation
SDL_Point renderer_world_to_screen(Renderer* renderer, niknum point[3]);

// Drawing primitives
void renderer_draw_wireframe_line(Renderer* renderer, niknum start[3], niknum end[3]);
void renderer_draw_wireframe_box(Renderer* renderer, niknum pos[3], niknum size[3], niknum rot[4]);
void renderer_draw_wireframe_sphere(Renderer* renderer, niknum pos[3], float radius);

// High-level rendering
void renderer_draw_body(Renderer* renderer, RigidBody body);
void renderer_draw_simulation(Renderer* renderer, const RigidBodySimulator* sim);

// Window management
void renderer_resize(Renderer* renderer, int width, int height);

} // namespace nik3dsim

#endif // RENDERER_HPP