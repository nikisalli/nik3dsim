#include "nik3dsim.h"
#include <SDL2/SDL.h>
using namespace nik3dsim;

int main() {
    // Create simulator with custom timestep and substeps for stability
    RigidBodySimulator sim;
    simulator_init(
        &sim,
        vec3_create(0, 0, -0.1), // No gravity for pure rotation demo
        0.00001f, // 1ms timestep for better stability
        1 // 1 position iteration (no constraints)
    );

    // Create a body with distinct principal moments of inertia
    RigidBody body;
    
    // Initialize with dummy size - we'll override the inertia
    rigidbody_init(
        &body,
        BODY_BOX,
        vec3_create(1.0f, 0.5f, 0.1f), // Size doesn't matter as we'll override inertia
        1.0f, // Density of 1 for simplicity
        vec3_create(0, 0, 0), // At origin
        vec3_create(0, 0, 0) // No initial rotation
    );    

    // Set initial angular velocity mostly around the middle (unstable) axis
    // with small perturbations on other axes to trigger the instability
    body.omega = vec3_create(0.0f, 2.0f, 0.01f);
    body.vel = vec3_create(0.0f, 0.0f, 3.0f);
    body.damping = 0;

    // Add body to simulator
    sim.rigidBodies[sim.rigidBodyCount++] = body;

    // Rest of your code remains the same...
    Renderer renderer;
    if (!renderer_init(&renderer, 800, 600)) {
        printf("Failed to initialize renderer\n");
        return 1;
    }

    Camera camera;
    camera.position = vec3_create(3.0f, 3.0f, 3.0f);
    camera.target = vec3_create(0.0f, 0.0f, 0.0f);
    camera.up = vec3_create(0.0f, 0.0f, 1.0f);
    camera.fov = 60.0f;
    camera.aspectRatio = 800.0f / 600.0f;
    camera.nearPlane = 0.1f;
    camera.farPlane = 100.0f;
    renderer_set_camera(&renderer, camera);

    bool running = true;
    SDL_Event event;
    Uint32 lastTime = SDL_GetTicks();
    
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
            }
        }

        Uint32 currentTime = SDL_GetTicks();
        float deltaTime = (currentTime - lastTime) / 1000.0f;
        lastTime = currentTime;

        static float accumulator = 0.0f;
        accumulator += deltaTime;
        
        while (accumulator >= sim.dt) {
            simulator_simulate(&sim);
            accumulator -= sim.dt;
        }

        renderer_draw_simulation(&renderer, &sim);
        SDL_Delay(1);
    }

    renderer_cleanup(&renderer);
    return 0;
}