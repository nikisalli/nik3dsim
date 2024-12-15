#include "nik3dsim.h"
#include <SDL2/SDL.h>
#include <SDL_stdinc.h>

using namespace nik3dsim;

int main() {
    nikModel m;
    nikData d;
    simulator_init(&m, (float[]){0, 0, -9.81}, 0.001f, 2, 0.5f);

    const int width = 20;
    const int height = 20;
    const float spacing = 0.1f;
    const float radius = 0.1f;

    int anchor1_idx = m.rigidBodyCount;
    add_rigidbody(&m, BODY_SPHERE, (float[]){0.05f, 0.1f, 0.1f}, 0.0f, (float[]){-spacing * (width-1)/2, 0, 2}, (float[]){0, 0, 0}, 0.0f, 0.0f, 0, 0);

    int anchor2_idx = m.rigidBodyCount;
    add_rigidbody(&m, BODY_SPHERE, (float[]){0.05f, 0.1f, 0.1f}, 0.0f, (float[]){spacing * (width-1)/2, 0, 2}, (float[]){0, 0, 0}, 0.0f, 0.0f, 0, 0);

    add_staticbody(&m, BODY_SPHERE, (float[]){0.5f, 0.3f, 0.3f}, (float[]){0.3, 0, 0.5}, (float[]){0, 0, 0}, 0.01f, 0.0f, 1, 1);

    int particleIndices[width][height];
    for(int i = 0; i < width; i++) {
        for(int j = 0; j < height; j++) {
            float pos[3] = {
                spacing * (i - (width-1)/2.0f),
                spacing * j,
                2.0f
            };
            particleIndices[i][j] = m.rigidBodyCount;
            add_rigidbody(&m, BODY_SPHERE, (float[]){radius, radius, radius}, 0.1f, pos, (float[]){0, 0, 0}, 0.0f, 0.0f, 0, 1);
        }
    }

    float localPos[3] = {0, 0, 0};
    
    for(int i = 0; i < width; i++) {
        for(int j = 0; j < height; j++) {
            if(i < width-1) {
                add_distance_constraint(&m, particleIndices[i][j], particleIndices[i+1][j], localPos, localPos, 0.0f, spacing);
            }
            
            if(j < height-1) {
                add_distance_constraint(&m, particleIndices[i][j], particleIndices[i][j+1], localPos, localPos, 0.0f, spacing);
            }
        }
    }

    add_distance_constraint(&m, anchor1_idx, particleIndices[0][height-1], localPos, localPos, 0.0f, 0.001f);
    add_distance_constraint(&m, anchor2_idx, particleIndices[width-1][height-1], localPos, localPos, 0.0f, 0.001f);

    printf("pos constraints: %ld\n", m.positionalConstraintCount);

    data_init(&m, &d);

    Renderer renderer;
    if (!renderer_init(&renderer, 2000, 1500)) {
        printf("Failed to initialize renderer\n");
        return 1;
    }

    Camera camera;
    vec3_copy(camera.target, (float[]){0.0f, 0.0f, 2.0f});
    vec3_copy(camera.up, (float[]){0.0f, 0.0f, 1.0f});
    camera.fov = 60.0f;
    camera.aspectRatio = 2000.0f / 1500.0f;
    camera.nearPlane = 0.1f;
    camera.farPlane = 100.0f;
    renderer_set_camera(&renderer, camera);

    MouseState mouseState = {0, 0, false, false, 5.0f, 0.0f, 30.0f};
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
        float deltaTime = (currentTime - lastTime) / 1000.0f;
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