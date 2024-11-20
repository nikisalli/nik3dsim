#include "nik3dsim.h"
#include <SDL2/SDL.h>

using namespace nik3dsim;

int main() {
    // Create simulator with custom timestep and iterations
    RigidBodySimulator sim;
    niknum gravity[3] = {0, 0, -10};
    simulator_init(
        &sim,
        gravity,    // No gravity
        0.01f,      // Small timestep for stability
        10          // More iterations for constraint stability
    );
    
    // Create first elongated cuboid
    RigidBody body1;
    niknum size1[3] = {2.0f, 0.2f, 0.2f};     // Elongated box
    niknum pos1[3] = {-1.0f, 0, 0};           // Positioned left of origin
    niknum angles1[3] = {0, 0, 0};            // No initial rotation
    rigidbody_init(
        &body1,
        BODY_BOX,
        size1,  
        0.1f,   // Density
        pos1,   
        angles1 
    );
    
    // Create second elongated cuboid
    RigidBody body2;
    niknum size2[3] = {2.0f, 0.2f, 0.2f};     // Same dimensions
    niknum pos2[3] = {1.0f, 0, 0};            // Positioned right of origin
    niknum angles2[3] = {0, 0, 0};            // No initial rotation
    rigidbody_init(
        &body2,
        BODY_BOX,
        size2,
        0.1f,   // Same density
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
    
    // Create hinge constraint
    // HingeConstraint hinge;
    // hinge.body0 = body1_idx;
    // hinge.body1 = body2_idx;
    
    // Set attachment points at the inner ends of the cuboids
    niknum local_pos0[3] = {1.0f, 0.0f, 0.0f};   // Right end of body1
    niknum local_pos1[3] = {-1.0f, 0.0f, 0.0f};  // Left end of body2
    niknum local_axis0[3] = {1.0f, 0.0f, 0.0f};  // Hinge axis aligned with Y axis
    niknum local_axis1[3] = {1.0f, 0.0f, 0.0f};
    
    // for(int i = 0; i < 3; i++) {
    //     hinge.local_pos0[i] = local_pos0[i];
    //     hinge.local_pos1[i] = local_pos1[i];
    //     hinge.local_axis0[i] = local_axis0[i];
    //     hinge.local_axis1[i] = local_axis1[i];
    // }
    
    // Add constraint to simulator
    // sim.hingeConstraints[sim.hingeConstraintCount++] = hinge;
    
    // Initialize renderer
    Renderer renderer;
    if (!renderer_init(&renderer, 800, 600)) {
        printf("Failed to initialize renderer\n");
        return 1;
    }
    
    // Set up camera to view the scene
    Camera camera;
    niknum cam_pos[3] = {5.0f, 5.0f, 3.0f};
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
                    // Add key to apply impulse to bodies
                    if (event.key.keysym.sym == SDLK_SPACE) {
                        niknum omega1[3] = {0.0f, 0.0f, 2.0f};
                        niknum omega2[3] = {0.0f, 0.0f, -1.0f};
                        for(int i = 0; i < 3; i++) {
                            sim.rigidBodies[body1_idx].omega[i] = omega1[i];
                            sim.rigidBodies[body2_idx].omega[i] = omega2[i];
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
        
        // Update physics
        simulator_simulate(&sim);
        print_simulation_state(&sim);
        
        // Render
        renderer_draw_simulation(&renderer, &sim);
        SDL_Delay(1);
    }
    
    renderer_cleanup(&renderer);
    return 0;
}