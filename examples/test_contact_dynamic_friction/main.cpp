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

    m.damping = 0.1f;
    
    RigidBodyModel body1model;
    RigidBodyData body1data;
    niknum size1[3] = {0.5f, 0.5f, 0.5f};
    niknum pos1[3] = {1, 0, 0.5f};           // Positioned left of origin
    niknum angles1[3] = {0, M_PI / 2, 0};            // No initial rotation
    body1model.conaffinity = 1;
    body1model.contype = 1;
    rigidbody_init(
        &body1model,
        &body1data,
        nik3dsim::BODY_BOX,
        size1,  
        1.0f,   // Density
        pos1,   
        angles1 
    );
    body1model.contactCompliance = 0.001f;
    body1model.frictionCoef = 10.0f;

    RigidBodyModel body3model;
    RigidBodyData body3data;
    niknum size3[3] = {0.5f, 0.5f, 0.5f};
    niknum pos3[3] = {1, 3, 0.5f};           // Positioned left of origin
    niknum angles3[3] = {0, M_PI / 2, 0};            // No initial rotation
    body3model.conaffinity = 1;
    body3model.contype = 1;
    rigidbody_init(
        &body3model,
        &body3data,
        nik3dsim::BODY_BOX,
        size3,  
        1.0f,   // Density
        pos3,   
        angles3 
    );
    body3model.contactCompliance = 0.001f;
    body3model.frictionCoef = 0.2f;

    RigidBodyModel body4model;
    RigidBodyData body4data;
    niknum size4[3] = {0.5f, 0.5f, 0.5f};
    niknum pos4[3] = {1, 6, 0.5f};           // Positioned left of origin
    niknum angles4[3] = {0, M_PI / 2, 0};            // No initial rotation
    body4model.conaffinity = 1;
    body4model.contype = 1;
    rigidbody_init(
        &body4model,
        &body4data,
        nik3dsim::BODY_BOX,
        size4,  
        1.0f,   // Density
        pos4,   
        angles4 
    );
    body4model.contactCompliance = 0.001f;
    body4model.frictionCoef = 0.1f;

    RigidBodyModel body5model;
    RigidBodyData body5data;
    niknum size5[3] = {0.5f, 0.5f, 0.5f};
    niknum pos5[3] = {1, 9, 0.5f};           // Positioned left of origin
    niknum angles5[3] = {0, M_PI / 2, 0};            // No initial rotation
    body5model.conaffinity = 1;
    body5model.contype = 1;
    rigidbody_init(
        &body5model,
        &body5data,
        nik3dsim::BODY_BOX,
        size5,  
        1.0f,   // Density
        pos5,   
        angles5 
    );
    body5model.contactCompliance = 0.001f;
    body5model.frictionCoef = 0.0f;

    StaticBodyModel body2model;
    body2model.conaffinity = 1;
    body2model.contype = 1;
    niknum size2[3] = {0.0f, 0.0f, 0.0f};     // Unit cube
    niknum pos2[3] = {2, 0, 0};           // Positioned left of origin
    niknum angles2[3] = {0, -M_PI * 0.02, 0};            // No initial rotation
    static_init(
        &body2model,
        nik3dsim::BODY_PLANE,
        size2,
        pos2,
        angles2
    );
    body2model.contactCompliance = 0.001f;

    // Add bodies to simulator
    int body1_idx = m.rigidBodyCount++;
    m.bodies[body1_idx] = body1model;
    d.bodies[body1_idx] = body1data;

    int body3_idx = m.rigidBodyCount++;
    m.bodies[body3_idx] = body3model;
    d.bodies[body3_idx] = body3data;

    int body4_idx = m.rigidBodyCount++;
    m.bodies[body4_idx] = body4model;
    d.bodies[body4_idx] = body4data;

    int body5_idx = m.rigidBodyCount++;
    m.bodies[body5_idx] = body5model;
    d.bodies[body5_idx] = body5data;

    int body2_idx = m.staticBodyCount++;
    m.staticBodies[body2_idx] = body2model;
    
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
        float deltaTime = (currentTime - lastTime) / 1000.0f;
        lastTime = currentTime;
        static float accumulator = 0.0f;
        accumulator += deltaTime;
        while (accumulator >= m.dt) {
            simulator_step(&m, &d);
            accumulator -= m.dt;
        }

        for (int i = 0; i < d.contactCount; i++) {
            Contact* contact = &d.contacts[i];
            niknum dir[3], length;
            vec3_sub(dir, contact->pos1, contact->pos0);
            length = vec3_normalize(dir, dir);
            renderer_draw_wireframe_arrow(&renderer, contact->pos0, dir, 1.0f, 0.1f, 0.1f, 1.0f, 1.0f, 0.0f);
            // printf("contact: pos0: %.2f %.2f %.2f pos1: %.2f %.2f %.2f depth: %.2f\n", contact->pos0[0], contact->pos0[1], contact->pos0[2], contact->pos1[0], contact->pos1[1], contact->pos1[2], contact->depth);
        }
        
        renderer_draw_simulation(&renderer, &m, &d);
        SDL_Delay(1);
    }
    
    renderer_cleanup(&renderer);
    return 0;
}