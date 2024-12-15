#include "nik3dsim.h"
#include <SDL2/SDL.h>
#include <SDL_stdinc.h>

using namespace nik3dsim;

int main() {
    // Create simulator with custom timestep and iterations
    nikModel m;
    nikData d;

    simulator_init(&m, (float[]){0, 0, -10}, 0.001f, 1, 0.0f);

    add_rigidbody(&m, BODY_BOX, (float[]){1.0f, 1.0f, 1.0f}, 0.0f, (float[]){0, 0, 0}, (float[]){0, 0, 0}, 0.0f, 0.01f, 0, 0);
    add_rigidbody(&m, BODY_BOX, (float[]){0.2f, 2.0f, 0.2f}, 1.0f, (float[]){0, 2, 0}, (float[]){0, 0, 0}, 0.0f, 0.01f, 0, 0);
    add_rigidbody(&m, BODY_BOX, (float[]){0.2f, 2.0f, 0.2f}, 1.0f, (float[]){0, 6, 0}, (float[]){0, 0, 0}, 0.0f, 0.01f, 0, 0);
    
    add_distance_constraint(&m, 0, 1, (float[]){0.0f, 0.0f, 0.0f}, (float[]){0.0f, -2.0f, 0.0f}, 0.0f, 0.0f);
    add_hinge_constraint(&m, 0, 1, (float[]){1.0f, 0.0f, 0.0f}, (float[]){1.0f, 0.0f, 0.0f}, 0.000000001f);
    add_distance_constraint(&m, 1, 2, (float[]){0.0f, 2.0f, 0.0f}, (float[]){0.0f, -2.0f, 0.0f}, 0.0f, 0.0f);

    data_init(&m, &d);
    
    // Initialize renderer
    Renderer renderer;
    if (!renderer_init(&renderer, 2000, 1200)) {
        printf("Failed to initialize renderer\n");
        return 1;
    }
    
    // Set up camera to view the scene
    Camera camera;
    vec3_copy(camera.target, (float[]){0.0f, 0.0f, 0.0f});
    vec3_copy(camera.up, (float[]){0.0f, 0.0f, 1.0f});
    
    camera.fov = 60.0f;
    camera.aspectRatio = 2000.0f / 1200.0f;
    camera.nearPlane = 0.1f;
    camera.farPlane = 100.0f;
    renderer_set_camera(&renderer, camera);
    
    // Initialize mouse state
    MouseState mouseState = {
        0,      // lastX
        0,      // lastY
        false,  // leftButtonDown
        false,  // rightButtonDown
        5.0f,   // dist - initial distance
        45.0f,  // azim - initial azimuth angle
        20.0f   // elev - initial elevation angle
    };

    update_camera_from_mouse_state(mouseState, renderer.camera);
    renderer_set_camera(&renderer, renderer.camera);
    
    // Main loop
    bool running = true;
    SDL_Event event;
    Uint32 lastTime = SDL_GetTicks();
    float accumulator = 0.0f;
    
    while (running) {
        // Handle events
        while (SDL_PollEvent(&event)) {
            switch (event.type) {
                case SDL_QUIT:
                    running = false;
                    break;
                case SDL_KEYDOWN:
                    if (event.key.keysym.sym == SDLK_ESCAPE) {
                        running = false;
                    }
                    break;
                case SDL_WINDOWEVENT:
                    if (event.window.event == SDL_WINDOWEVENT_RESIZED) {
                        renderer_resize(&renderer, event.window.data1, event.window.data2);
                    }
                    break;
                case SDL_MOUSEBUTTONDOWN:
                case SDL_MOUSEBUTTONUP:
                case SDL_MOUSEMOTION:
                case SDL_MOUSEWHEEL:
                    handle_mouse_events(event, renderer.camera, mouseState);
                    renderer_set_camera(&renderer, renderer.camera);
                    break;
            }
        }
        
        Uint32 currentTime = SDL_GetTicks();
        float deltaTime = (currentTime - lastTime) / 300.0f;
        lastTime = currentTime;
        accumulator += deltaTime;
        
        while (accumulator >= m.dt) {
            simulator_step(&m, &d);
            print_simulation_state(&m, &d);
            accumulator -= m.dt;
        }
        
        renderer_draw_simulation(&renderer, &m, &d);
        SDL_Delay(1);
    }
    
    renderer_cleanup(&renderer);
    return 0;
}