#include "nik3dsim.h"
#include <SDL2/SDL.h>
#include <SDL_stdinc.h>

using namespace nik3dsim;

int main() {
    // Create simulator with custom timestep and iterations
    RigidBodySimulator sim;
    niknum gravity[3] = {0, 0, -10};
    simulator_init(
        &sim,
        gravity,
        0.01f,      // Small timestep for stability
        1          // More iterations for constraint stability
    );
    
    // Create first cube
    RigidBody body1;
    niknum size1[3] = {1.0f, 1.0f, 1.0f};     // Unit cube
    niknum pos1[3] = {0, 0, 0};           // Positioned left of origin
    niknum angles1[3] = {0, 0, 0};            // No initial rotation
    rigidbody_init(
        &body1,
        BODY_BOX,
        size1,  
        1.0f,   // Density
        pos1,   
        angles1 
    );
    
    // Create second cube
    RigidBody body2;
    niknum size2[3] = {1.0f, 1.0f, 1.0f};     // Unit cube
    niknum pos2[3] = {0, 0, -1};            // Positioned right of origin
    niknum angles2[3] = {0, 0, 0};            // No initial rotation
    rigidbody_init(
        &body2,
        BODY_BOX,
        size2,
        1.0f,   // Same density
        pos2,
        angles2
    );
    
    // Fix first body in place
    body1.invMass = 0.0f;
    body1.invInertia[0] = 0.0f;
    body1.invInertia[1] = 0.0f;
    body1.invInertia[2] = 0.0f;
    
    // Add bodies to simulator
    int body1_idx = sim.rigidBodyCount++;
    sim.rigidBodies[body1_idx] = body1;
    
    int body2_idx = sim.rigidBodyCount++;
    sim.rigidBodies[body2_idx] = body2;
    
    // Create positional constraint
    DistanceConstraint pos_constraint;
    
    // Set attachment points at the facing corners of the cubes
    niknum local_pos0[3] = {0, 0, 0};   // Right-top-front corner of body1
    niknum local_pos1[3] = {0, 0, 0};  // Left-top-front corner of body2
    
    // Initialize the constraint with some compliance for a slightly soft connection
    pos_constraint.compliance = 0.01f;
    pos_constraint.b0 = body1_idx;
    pos_constraint.b1 = body2_idx;
    vec3_copy(pos_constraint.r0, local_pos0);
    vec3_copy(pos_constraint.r1, local_pos1);
    pos_constraint.distance = 1.0f;
    
    // Add constraint to simulator
    sim.positionalConstraints[sim.positionalConstraintCount++] = pos_constraint;
    
    // Initialize renderer
    Renderer renderer;
    if (!renderer_init(&renderer, 800, 600)) {
        printf("Failed to initialize renderer\n");
        return 1;
    }
    
    // Set up camera to view the scene
    Camera camera;
    niknum cam_pos[3] = {2.0f, 5.0f, 3.0f};
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
                    // Add key to apply impulse to second body
                    if (event.key.keysym.sym == SDLK_SPACE) {
                        niknum impulse[3] = {0.0f, 2.0f, 1.0f};  // Push up and forward
                        for(int i = 0; i < 3; i++) {
                            sim.rigidBodies[body2_idx].vel[i] = impulse[i];
                        }
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

        // Uint32 currentTime = SDL_GetTicks();
        // for (int i = 0; i < 1e6; i++) {
        //     simulator_simulate(&sim);
        // }
        // Uint32 endTime = SDL_GetTicks();
        // printf("steps/s: %f\n", 1e9f / (endTime - currentTime));
        
        renderer_draw_simulation(&renderer, &sim);
        // print_simulation_state(&sim);
        SDL_Delay(100);
    }
    
    renderer_cleanup(&renderer);
    return 0;
}