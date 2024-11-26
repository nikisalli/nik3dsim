#include "nik3dsim.h"
#include <SDL2/SDL.h>
#include <SDL_stdinc.h>

using namespace nik3dsim;

int main() {
    // Create simulator with custom timestep and iterations
    nikModel m;
    nikData d;
    niknum gravity[3] = {0, 0, -10};
    simulator_init(
        &m,
        gravity,
        0.01f,      // Small timestep for stability
        1          // More iterations for constraint stability
    );

    m.damping = 0.0f;
    
    // Create first cube
    RigidBodyModel body1model;
    RigidBodyData body1data;
    niknum size1[3] = {1.0f, 1.0f, 1.0f};     // Unit cube
    niknum pos1[3] = {0, 0, 0};           // Positioned left of origin
    niknum angles1[3] = {0, 0, 0};            // No initial rotation
    rigidbody_init(
        &body1model,
        &body1data,
        BODY_BOX,
        size1,  
        1.0f,   // Density
        pos1,   
        angles1 
    );

    // Create second cube
    RigidBodyModel body2model;
    RigidBodyData body2data;
    niknum size2[3] = {0.2f, 4.0f, 0.2f};     // Unit cube
    niknum pos2[3] = {0, 2, 0};            // Positioned right of origin
    niknum angles2[3] = {0, 0, 0};            // No initial rotation
    rigidbody_init(
        &body2model,
        &body2data,
        BODY_BOX,
        size2,
        1.0f,   // Same density
        pos2,
        angles2
    );

    // Create third cube
    RigidBodyModel body3model;
    RigidBodyData body3data;
    niknum size3[3] = {0.2f, 4.0f, 0.2f};     // Unit cube
    niknum pos3[3] = {0, 6, 0};            // Positioned right of origin
    niknum angles3[3] = {0, 0, 0};            // No initial rotation
    rigidbody_init(
        &body3model,
        &body3data,
        BODY_BOX,
        size3,
        1.0f,   // Same density
        pos3,
        angles3
    );
    
    // Fix first body in place
    body1model.invMass = 0.0f;
    body1model.invInertia[0] = 0.0f;
    body1model.invInertia[1] = 0.0f;
    body1model.invInertia[2] = 0.0f;
    
    // Add bodies to simulator
    int body1_idx = m.rigidBodyCount++;
    m.bodies[body1_idx] = body1model;
    d.bodies[body1_idx] = body1data;
    
    int body2_idx = m.rigidBodyCount++;
    m.bodies[body2_idx] = body2model;
    d.bodies[body2_idx] = body2data;

    int body3_idx = m.rigidBodyCount++;
    m.bodies[body3_idx] = body3model;
    d.bodies[body3_idx] = body3data;
    
    // Create positional constraint
    DistanceConstraint pos_constraint;
    pos_constraint.compliance = 0.0f;
    pos_constraint.b0 = body1_idx;
    pos_constraint.b1 = body2_idx;
    pos_constraint.r0[0] = 0.0f;
    pos_constraint.r0[1] = 0.0f;
    pos_constraint.r0[2] = 0.0f;
    pos_constraint.r1[0] = 0.0f;
    pos_constraint.r1[1] = -2.0f;
    pos_constraint.r1[2] = 0.0f;
    pos_constraint.distance = 0.0f;
    m.positionalConstraints[m.positionalConstraintCount++] = pos_constraint;

    // Create hinge constraint
    HingeConstraint hinge_constraint;
    hinge_constraint.b0 = body1_idx;
    hinge_constraint.b1 = body2_idx;
    hinge_constraint.a0[0] = 1.0f;
    hinge_constraint.a0[1] = 0.0f;
    hinge_constraint.a0[2] = 0.0f;
    hinge_constraint.a1[0] = 1.0f;
    hinge_constraint.a1[1] = 0.0f;
    hinge_constraint.a1[2] = 0.0f;
    hinge_constraint.compliance = 0.000000001f;
    m.hingeConstraints[m.hingeConstraintCount++] = hinge_constraint;

    // Create positional constraint
    DistanceConstraint pos_constraint2;
    pos_constraint2.compliance = 0.0f;
    pos_constraint2.b0 = body2_idx;
    pos_constraint2.b1 = body3_idx;
    pos_constraint2.r0[0] = 0.0f;
    pos_constraint2.r0[1] = 2.0f;
    pos_constraint2.r0[2] = 0.0f;
    pos_constraint2.r1[0] = 0.0f;
    pos_constraint2.r1[1] = -2.0f;
    pos_constraint2.r1[2] = 0.0f;
    pos_constraint2.distance = 0.0f;
    m.positionalConstraints[m.positionalConstraintCount++] = pos_constraint2;
    
    // Initialize renderer
    Renderer renderer;
    if (!renderer_init(&renderer, 800, 600)) {
        printf("Failed to initialize renderer\n");
        return 1;
    }
    
    // Set up camera to view the scene
    Camera camera;
    niknum cam_pos[3] = {-5.0f, 7.0f, 5.0f};
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
        5.0f,   // dist - initial distance
        45.0f,  // azim - initial azimuth angle
        20.0f   // elev - initial elevation angle
    };
    
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
        float deltaTime = (currentTime - lastTime) / 1000.0f;
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