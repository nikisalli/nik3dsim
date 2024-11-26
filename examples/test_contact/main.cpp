#include "nik3dsim.h"
#include <SDL2/SDL.h>
#include <SDL_stdinc.h>

using namespace nik3dsim;

int main() {
    // Create simulator with custom timestep and iterations
    nikModel m;
    nikData d;
    niknum gravity[3] = {0, 0, -1};
    simulator_init(
        &m,
        gravity,
        0.01f,      // Small timestep for stability
        1          // More iterations for constraint stability
    );

    m.damping = 0.1f;
    
    RigidBodyModel body1model;
    RigidBodyData body1data;
    niknum size1[3] = {0.4f, 1.0f, 1.0f};     // Unit cube
    niknum pos1[3] = {2, 0, 10.0f};           // Positioned left of origin
    niknum angles1[3] = {0, 0, 0};            // No initial rotation
    body1model.conaffinity = 1;
    body1model.contype = 1;
    rigidbody_init(
        &body1model,
        &body1data,
        nik3dsim::BODY_SPHERE,
        size1,  
        1.0f,   // Density
        pos1,   
        angles1 
    );

    RigidBodyModel body4model;
    RigidBodyData body4data;
    niknum size4[3] = {0.4f, 1.0f, 1.0f};     // Unit cube
    niknum pos4[3] = {1, 0.1, 10.0f};           // Positioned left of origin
    niknum angles4[3] = {0, 0, 0};            // No initial rotation
    body4model.conaffinity = 1;
    body4model.contype = 1;
    rigidbody_init(
        &body4model,
        &body4data,
        nik3dsim::BODY_SPHERE,
        size4,  
        1.0f,   // Density
        pos4,   
        angles4 
    );

    StaticBodyModel body2model;
    body2model.conaffinity = 1;
    body2model.contype = 1;
    niknum size2[3] = {0.0f, 0.0f, 0.0f};     // Unit cube
    niknum pos2[3] = {2, 0, 0};           // Positioned left of origin
    niknum angles2[3] = {0, -M_PI * 0.4, 0};            // No initial rotation
    static_init(
        &body2model,
        nik3dsim::BODY_PLANE,
        size2,
        pos2,
        angles2
    );

    StaticBodyModel body3model;
    body3model.conaffinity = 1;
    body3model.contype = 1;
    niknum size3[3] = {0.0f, 0.0f, 0.0f};     // Unit cube
    niknum pos3[3] = {-2, 0, 0};           // Positioned left of origin
    niknum angles3[3] = {0, M_PI * 0.4, 0};            // No initial rotation
    static_init(
        &body3model,
        nik3dsim::BODY_PLANE,
        size3,
        pos3,
        angles3
    );

    // Add bodies to simulator
    int body1_idx = m.rigidBodyCount++;
    m.bodies[body1_idx] = body1model;
    d.bodies[body1_idx] = body1data;

    int body4_idx = m.rigidBodyCount++;
    m.bodies[body4_idx] = body4model;
    d.bodies[body4_idx] = body4data;

    int body2_idx = m.staticBodyCount++;
    m.staticBodies[body2_idx] = body2model;

    int body3_idx = m.staticBodyCount++;
    m.staticBodies[body3_idx] = body3model;

    // Add positional constraint
    // Create positional constraint
    DistanceConstraint pos_constraint;
    
    // Set attachment points at the facing corners of the cubes
    niknum local_pos0[3] = {0, 0, 0};   // Right-top-front corner of body1
    niknum local_pos1[3] = {0, 0, 0};  // Left-top-front corner of body2
    
    // Initialize the constraint with some compliance for a slightly soft connection
    pos_constraint.compliance = 0.0f;
    pos_constraint.b0 = body1_idx;
    pos_constraint.b1 = body4_idx;
    vec3_copy(pos_constraint.r0, local_pos0);
    vec3_copy(pos_constraint.r1, local_pos1);
    pos_constraint.distance = 1.0f;

    // Add constraint to simulator
    m.positionalConstraints[m.positionalConstraintCount++] = pos_constraint;
    
    // Initialize renderer
    Renderer renderer;
    if (!renderer_init(&renderer, 2000, 1200)) {
        printf("Failed to initialize renderer\n");
        return 1;
    }
    
    // Set up camera to view the scene
    Camera camera;
    niknum cam_pos[3] = {0.0f, 15.0f, 0.0f};
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
    
    // Initialize mouse state
    MouseState mouseState = {
        0,      // lastX
        0,      // lastY
        false,  // leftButtonDown
        false,  // rightButtonDown
        15.0f,  // dist
        0.0f,  // azim
        0.0f   // elev
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
        float deltaTime = (currentTime - lastTime) / 300.0f;
        lastTime = currentTime;
        static float accumulator = 0.0f;
        accumulator += deltaTime;
        while (accumulator >= m.dt) {
            simulator_step(&m, &d);
            for (int i = 0; i < d.contactCount; i++) {
                Contact* contact = &d.contacts[i];
                renderer_draw_wireframe_arrow(&renderer, contact->pos, contact->n, 1.0f, 0.1f, 0.1f);
                // printf("contact: pos: %.2f %.2f %.2f n: %.2f %.2f %.2f depth: %.2f\n", contact->pos[0], contact->pos[1], contact->pos[2], contact->n[0], contact->n[1], contact->n[2], contact->depth);
            }
            accumulator -= m.dt;
        }

        
        renderer_draw_simulation(&renderer, &m, &d);
        SDL_Delay(1);
    }
    
    renderer_cleanup(&renderer);
    return 0;
}