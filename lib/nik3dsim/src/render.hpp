#ifndef RENDERER_HPP
#define RENDERER_HPP

#include <SDL2/SDL.h>
#include "types.hpp" // For RigidBody definitions

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

struct MouseState {
    int lastX;
    int lastY;
    bool leftButtonDown;
    bool rightButtonDown;
    float dist;       // Distance from camera to target
    float azim;       // Azimuth angle in degrees
    float elev;       // Elevation angle in degrees
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

void handle_mouse_events(SDL_Event& event, Camera& camera, MouseState& mouseState);

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
void renderer_draw_wireframe_line(Renderer* renderer, const niknum start[3], const niknum end[3]);
void renderer_draw_wireframe_box(Renderer* renderer, niknum pos[3], niknum size[3], niknum rot[4]);
void renderer_draw_wireframe_sphere(Renderer* renderer, niknum pos[3], float radius);

// High-level rendering
void renderer_draw_body(Renderer* renderer, RigidBodyModel model, RigidBodyData data);
void renderer_draw_simulation(Renderer* renderer, const nikModel* model, const nikData* data);

// Window management
void renderer_resize(Renderer* renderer, int width, int height);

} // namespace nik3dsim

#endif // RENDERER_HPP