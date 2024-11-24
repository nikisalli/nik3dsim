#include "nik3dsim.h"
#include <SDL2/SDL.h>
using namespace nik3dsim;

int main() {
    // Create simulator with custom timestep and substeps for stability
    nikModel m;
    nikData d;
    niknum gravity[3] = {0, 0, -9.81}; // No gravity for pure rotation demo
    simulator_init(
        &m,
        gravity,
        0.01f, // 1ms timestep for better stability
        1 // 1 position iteration (no constraints)
    );
    
    // Create a body with distinct principal moments of inertia
    RigidBodyModel bodymodel;
    RigidBodyData bodydata;
    // Initialize with dummy size - we'll override the inertia
    niknum size[3] = {1.0f, 0.5f, 0.1f}; // Size doesn't matter as we'll override inertia
    niknum pos[3] = {0, 0, 0}; // At origin
    niknum angles[3] = {0, 0, 0}; // No initial rotation
    rigidbody_init(
        &bodymodel,
        &bodydata,
        BODY_BOX,
        size,
        1.0f, // Density of 1 for simplicity
        pos,
        angles
    );
    
    printf("body inertia: %.2f, %.2f, %.2f\n",
           bodymodel.invInertia[0], bodymodel.invInertia[1], bodymodel.invInertia[2]);
    printf("body mass: %.2f\n", bodymodel.invMass);
    
    // Set initial angular velocity mostly around the middle (unstable) axis
    // with small perturbations on other axes to trigger the instability
    niknum init_omega[3] = {0.0f, 10.0f, 0.01f};
    niknum init_vel[3] = {0.0f, 0.0f, 6.0f};
    for(int i = 0; i < 3; i++) {
        bodydata.omega[i] = init_omega[i];
        bodydata.vel[i] = init_vel[i];
    }
    
    // Add body to simulator
    m.rigidBodies[m.rigidBodyCount++] = bodymodel;
    d.rigidBodies[m.rigidBodyCount - 1] = bodydata;
    
    // Initialize renderer
    Renderer renderer;
    if (!renderer_init(&renderer, 800, 600)) {
        printf("Failed to initialize renderer\n");
        return 1;
    }
    
    // Set up camera
    Camera camera;
    niknum cam_pos[3] = {3.0f, 3.0f, 3.0f};
    niknum cam_target[3] = {0.0f, 0.0f, 0.0f};
    niknum cam_up[3] = {0.0f, 0.0f, 1.0f};
    for(int i = 0; i < 3; i++) {
        camera.position[i] = cam_pos[i];
        camera.target[i] = cam_target[i];
        camera.up[i] = cam_up[i];
    }
    camera.fov = 60.0f;
    camera.aspectRatio = 800.0f / 600.0f;
    camera.nearPlane = 0.1f;
    camera.farPlane = 100.0f;
    renderer_set_camera(&renderer, camera);
    
    // Initialize mouse state
    MouseState mouseState = {
        0,      // lastX
        0,      // lastY
        false,  // leftButtonDown
        false,  // rightButtonDown
        10.0f,  // dist
        45.0f,  // azim
        30.0f   // elev
    };
    
    // Main loop
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
        static float accumulator = 0.0f;
        accumulator += deltaTime;
        while (accumulator >= m.dt) {
            simulator_simulate(&m, &d);
            accumulator -= m.dt;
        }
        
        renderer_draw_simulation(&renderer, &m, &d);
        SDL_Delay(1);
    }
    
    renderer_cleanup(&renderer);
    return 0;
}