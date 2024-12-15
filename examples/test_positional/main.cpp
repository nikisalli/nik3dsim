#include "nik3dsim.h"
#include <SDL2/SDL.h>
#include <SDL_stdinc.h>

using namespace nik3dsim;

int main() {
    nikModel m;
    nikData d;
    simulator_init(&m, (float[]){0, 0, -10}, 0.01f, 1, 0.1f);

    add_rigidbody(&m, BODY_BOX, (float[]){1.0f, 1.0f, 1.0f}, 0.0f, (float[]){0, 0, 0}, (float[]){0, 0, 0}, 0.001f, 0.0f, 1, 1);
    add_rigidbody(&m, BODY_BOX, (float[]){1.0f, 1.0f, 1.0f}, 1.0f, (float[]){0, 0, -1}, (float[]){0, 0, 0}, 0.001f, 0.0f, 1, 1);
    add_rigidbody(&m, BODY_BOX, (float[]){1.0f, 1.0f, 1.0f}, 1.0f, (float[]){0, 0, -2}, (float[]){0, 0, 0}, 0.001f, 0.0f, 1, 1);

    float local_pos0[3] = {0, 0, 0};
    float local_pos1[3] = {0, 0, 0};
    float local_pos2[3] = {0.5, 0.5, 0.5};
    
    add_distance_constraint(&m, 0, 1, local_pos0, local_pos1, 0.0f, 2.0f);
    add_distance_constraint(&m, 1, 2, local_pos0, local_pos2, 0.0f, 2.0f);

    data_init(&m, &d);

    Renderer renderer;
    if (!renderer_init(&renderer, 800, 600)) {
        printf("Failed to initialize renderer\n");
        return 1;
    }

    Camera camera;
    vec3_copy(camera.target, (float[]){0.0f, 0.0f, 0.0f});
    vec3_copy(camera.up, (float[]){0.0f, 0.0f, 1.0f});
    camera.fov = 60.0f;
    camera.aspectRatio = 800.0f / 600.0f;
    camera.nearPlane = 0.1f;
    camera.farPlane = 100.0f;
    renderer_set_camera(&renderer, camera);

    MouseState mouseState = {0, 0, false, false, 10.0f, 45.0f, 30.0f};
    update_camera_from_mouse_state(mouseState, renderer.camera);
    renderer_set_camera(&renderer, renderer.camera);

    bool running = true;
    SDL_Event event;
    Uint32 lastTime = SDL_GetTicks();
    float accumulator = 0.0f;

    while (running) {
        while (SDL_PollEvent(&event)) {
            switch (event.type) {
                case SDL_QUIT:
                    running = false;
                    break;
                case SDL_KEYDOWN:
                    if (event.key.keysym.sym == SDLK_ESCAPE) {
                        running = false;
                    }
                    if (event.key.keysym.sym == SDLK_SPACE) {
                        d.bodies[1].vel[0] = 0.0f;
                        d.bodies[1].vel[1] = 2.0f;
                        d.bodies[1].vel[2] = 1.0f;
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
        float deltaTime = (currentTime - lastTime) / 100.0f;
        lastTime = currentTime;
        accumulator += deltaTime;

        while (accumulator >= m.dt) {
            simulator_step(&m, &d);
            accumulator -= m.dt;
        }

        renderer_draw_simulation(&renderer, &m, &d);
        SDL_Delay(1);
    }

    renderer_cleanup(&renderer);
    return 0;
}