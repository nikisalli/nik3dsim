#include "nik3dsim.h"
#include <SDL2/SDL.h>
#include <SDL_stdinc.h>

using namespace nik3dsim;

int main() {
    nikModel m;
    nikData d;
    niknum gravity[3] = {0, 0, -1};
    simulator_init(&m, gravity, 0.01f, 1);
    m.damping = 0.1f;
    
    // Create several falling capsules
    const int NUM_CAPSULES = 100;
    for(int i = 0; i < NUM_CAPSULES; i++) {
        RigidBodyModel capsule;
        RigidBodyData capsuleData;
        // Randomize size slightly
        niknum size[3] = {0.2f + 0.1f*rand()/RAND_MAX, 1.5f + (float)rand()/(float)RAND_MAX, 0.0f};
        // Distribute in a circle above
        float angle = 2*M_PI*i/NUM_CAPSULES;
        float radius = 0.5f;
        niknum pos[3] = {radius*cosf(angle), radius*sinf(angle), 10.0f + i*1.0f};
        // Random rotation
        niknum angles[3] = {
            (float)M_PI*rand()/RAND_MAX, 
            (float)M_PI*rand()/RAND_MAX,
            (float)M_PI*rand()/RAND_MAX
        };
        
        capsule.conaffinity = 1;
        capsule.contype = 1;
        rigidbody_init(&capsule, &capsuleData, BODY_CAPSULE, size, 1.0f, pos, angles);
        capsule.frictionCoef = 0.5f;
        
        int idx = m.rigidBodyCount++;
        m.bodies[idx] = capsule;
        d.bodies[idx] = capsuleData;
    }

    // Create static capsule
    StaticBodyModel staticCapsule;
    staticCapsule.conaffinity = 1;
    staticCapsule.contype = 1;
    niknum staticSize[3] = {0.4f, 4.0f, 0.0f};
    niknum staticPos[3] = {0.0f, 0.0f, 0.0f};
    niknum staticAngles[3] = {0, M_PI * 0.25f, 0};
    static_init(&staticCapsule, BODY_CAPSULE, staticSize, staticPos, staticAngles);
    m.staticBodies[m.staticBodyCount++] = staticCapsule;
    
    // Visualization setup
    Renderer renderer;
    if (!renderer_init(&renderer, 2000, 1200)) {
        printf("Failed to initialize renderer\n");
        return 1;
    }
    
    Camera camera;
    niknum cam_pos[3] = {0.0f, 20.0f, 10.0f};
    niknum cam_target[3] = {0.0f, 0.0f, 0.0f};
    niknum cam_up[3] = {0.0f, 0.0f, 1.0f};
    
    for(int i = 0; i < 3; i++) {
        camera.position[i] = cam_pos[i];
        camera.target[i] = cam_target[i];
        camera.up[i] = cam_up[i];
    }
    
    camera.fov = 60.0f;
    camera.aspectRatio = 2000.0f / 1200.0f;
    camera.nearPlane = 0.1f;
    camera.farPlane = 100.0f;
    renderer_set_camera(&renderer, camera);
    
    MouseState mouseState = {0, 0, false, false, 20.0f, 0.0f, 0.0f};
    
    // Main loop
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
                    if (event.key.keysym.sym == SDLK_ESCAPE) running = false;
                    if (event.key.keysym.sym == SDLK_r) {
                        // Reset all dynamic capsules
                        for(int i = 0; i < NUM_CAPSULES; i++) {
                            float angle = 2*M_PI*i/NUM_CAPSULES;
                            d.bodies[i].pos[0] = 2.0f*cosf(angle);
                            d.bodies[i].pos[1] = 2.0f*sinf(angle);
                            d.bodies[i].pos[2] = 10.0f + i*0.5f;
                            d.bodies[i].vel[0] = d.bodies[i].vel[1] = d.bodies[i].vel[2] = 0;
                        }
                    }
                    break;
                case SDL_WINDOWEVENT:
                    if (event.window.event == SDL_WINDOWEVENT_RESIZED)
                        renderer_resize(&renderer, event.window.data1, event.window.data2);
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