#include "nik3dsim.h"
#include <SDL2/SDL.h>
#include <SDL_stdinc.h>

using namespace nik3dsim;

int main() {
    nikModel m;
    nikData d;
    simulator_init(&m, (float[]){0, 0, -1}, 0.01f, 1, 0.1f);

    add_rigidbody(&m, BODY_BOX, (float[]){0.5f, 0.5f, 0.5f}, 1.0f, (float[]){1, 0, 1.0f}, (float[]){0, M_PI / 2, 0}, 0.001f, 0.0f, 1, 1);
    add_staticbody(&m, BODY_PLANE, (float[]){0.0f, 0.0f, 0.0f}, (float[]){2, 0, 0}, (float[]){0, -M_PI * 0.4, 0}, 0.001f, 0.0f, 1, 1);
    add_staticbody(&m, BODY_PLANE, (float[]){0.0f, 0.0f, 0.0f}, (float[]){-2, 0, 0}, (float[]){0, M_PI * 0.4, 0}, 0.001f, 0.0f, 1, 1);

    data_init(&m, &d);

    Renderer renderer;
    if (!renderer_init(&renderer, 2000, 1200)) {
        printf("Failed to initialize renderer\n");
        return 1;
    }

    Camera camera;
    vec3_copy(camera.target, (float[]){0.0f, 0.0f, 0.0f});
    vec3_copy(camera.up, (float[]){0.0f, 0.0f, 1.0f});
    camera.fov = 60.0f;
    camera.aspectRatio = 2000.0f / 1200.0f;
    camera.nearPlane = 0.1f;
    camera.farPlane = 100.0f;
    renderer_set_camera(&renderer, camera);

    MouseState mouseState = {0, 0, false, false, 15.0f, 0.0f, 0.0f};
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
            accumulator -= m.dt;
        }

        for (int i = 0; i < d.contactCount; i++) {
            Contact* contact = &d.contacts[i];
            float dir[3], length;
            vec3_sub(dir, contact->pos1, contact->pos0);
            length = vec3_normalize(dir, dir);
            renderer_draw_wireframe_arrow(&renderer, contact->pos0, dir, 1.0f, 0.1f, 0.1f, 1.0f, 1.0f, 0.0f);
            printf("contact: pos0: %.2f %.2f %.2f pos1: %.2f %.2f %.2f depth: %.2f\n", 
                   contact->pos0[0], contact->pos0[1], contact->pos0[2], 
                   contact->pos1[0], contact->pos1[1], contact->pos1[2], 
                   contact->depth);
        }

        renderer_draw_simulation(&renderer, &m, &d);
        SDL_Delay(1);
    }

    renderer_cleanup(&renderer);
    return 0;
}